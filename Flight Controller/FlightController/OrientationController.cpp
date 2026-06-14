#include "OrientationController.h"
#include <Arduino.h>
#include <Wire.h>

OrientationController::OrientationController() { }

bool OrientationController::Init() {
    _isInitialized = false;
    _isCalibrating = false;
    _isCalibrationComplete = false;

    Wire.begin();
    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x6B); // Power management register.
    Wire.write(0); // Wake command.
    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    _lastMeasurementTimeUs = micros();
    _isInitialized = true;
    return true;
}

void OrientationController::StartCalibration() {
    _isCalibrationComplete = false;
    _isCalibrating = _isInitialized;
    _gyroBiasX = 0.0f;
    _gyroBiasY = 0.0f;
    _gyroBiasZ = 0.0f;
    ResetCalibrationSamples();
}

bool OrientationController::IsCalibrating() const {
    return _isCalibrating;
}

bool OrientationController::IsCalibrationComplete() const {
    return _isCalibrationComplete;
}

uint16_t OrientationController::GetCalibrationSampleCount() const {
    return _calibrationSampleCount;
}

void OrientationController::ResetCalibrationSamples() {
    _calibrationSampleCount = 0;
    _gyroCalibrationSumX = 0.0f;
    _gyroCalibrationSumY = 0.0f;
    _gyroCalibrationSumZ = 0.0f;
}

void OrientationController::UpdateCalibration(const IMUData& data) {
    float gyroMagnitude = sqrt(
        data.GyroX * data.GyroX +
        data.GyroY * data.GyroY +
        data.GyroZ * data.GyroZ);
    float accelerationMagnitude = sqrt(
        data.AccelerationForceX * data.AccelerationForceX +
        data.AccelerationForceY * data.AccelerationForceY +
        data.AccelerationForceZ * data.AccelerationForceZ);

    bool isAccelerationStationary =
        abs(accelerationMagnitude - 1.0f) <=
        Config::GYRO_CALIBRATION_ACCEL_TOLERANCE_G;
    if (!isAccelerationStationary ||
        gyroMagnitude > Config::GYRO_CALIBRATION_MAX_RATE_DEG_S) {
        return;
    }

    if (_calibrationSampleCount > 0) {
        float sampleCount = static_cast<float>(_calibrationSampleCount);
        float averageGyroX = _gyroCalibrationSumX / sampleCount;
        float averageGyroY = _gyroCalibrationSumY / sampleCount;
        float averageGyroZ = _gyroCalibrationSumZ / sampleCount;
        float gyroDeviation = sqrt(
            (data.GyroX - averageGyroX) * (data.GyroX - averageGyroX) +
            (data.GyroY - averageGyroY) * (data.GyroY - averageGyroY) +
            (data.GyroZ - averageGyroZ) * (data.GyroZ - averageGyroZ));

        if (gyroDeviation >
            Config::GYRO_CALIBRATION_MAX_RATE_DEVIATION_DEG_S) {
            return;
        }
    }

    _gyroCalibrationSumX += data.GyroX;
    _gyroCalibrationSumY += data.GyroY;
    _gyroCalibrationSumZ += data.GyroZ;
    _calibrationSampleCount++;

    if (_calibrationSampleCount < Config::GYRO_CALIBRATION_SAMPLE_COUNT) {
        return;
    }

    float sampleCount = static_cast<float>(_calibrationSampleCount);
    _gyroBiasX = _gyroCalibrationSumX / sampleCount;
    _gyroBiasY = _gyroCalibrationSumY / sampleCount;
    _gyroBiasZ = _gyroCalibrationSumZ / sampleCount;
    _isCalibrating = false;
    _isCalibrationComplete = true;
    _lastOrientation = Orientation();
    _lastMeasurementTimeUs = micros();
}

IMUData OrientationController::GetIMUData() {
    IMUData data;

    if (!_isInitialized) return data;

    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x3B); // Accelerometer data starting register.
    if (Wire.endTransmission(false) != 0) {
        return data;
    }

    size_t bytesReceived = Wire.requestFrom(Config::MPU_ADDRESS, 14, true);
    if (bytesReceived != 14 || Wire.available() < 14) {
        while (Wire.available() > 0) {
            Wire.read();
        }
        return data;
    }

    int16_t rawAccelX = (Wire.read() << 8 | Wire.read());
    int16_t rawAccelY = (Wire.read() << 8 | Wire.read());
    int16_t rawAccelZ = (Wire.read() << 8 | Wire.read());
    Wire.read(); // Temperature high byte.
    Wire.read(); // Temperature low byte.
    int16_t rawGyroX = (Wire.read() << 8 | Wire.read());
    int16_t rawGyroY = (Wire.read() << 8 | Wire.read());
    int16_t rawGyroZ = (Wire.read() << 8 | Wire.read());

    // Convert to Gs.
    data.AccelerationForceX = rawAccelX / 16384.0f;
    data.AccelerationForceY = rawAccelY / 16384.0f;
    data.AccelerationForceZ = rawAccelZ / 16384.0f;

    // Convert to degrees per second.
    data.GyroX = rawGyroX / 131.0f;
    data.GyroY = rawGyroY / 131.0f;
    data.GyroZ = rawGyroZ / 131.0f;
    data.ReadSuccessful = true;

    return data;
}

Orientation OrientationController::GetOrientation() {
    IMUData rawData = GetIMUData();
    uint32_t currentTimeUs = micros();
    uint32_t elapsedTimeUs = currentTimeUs - _lastMeasurementTimeUs;
    _lastMeasurementTimeUs = currentTimeUs;

    if (!rawData.ReadSuccessful) {
        Orientation failedOrientation = _lastOrientation;
        failedOrientation.ReadSuccessful = false;
        return failedOrientation;
    }

    if (_isCalibrating) {
        UpdateCalibration(rawData);

        Orientation calibrationOrientation = _lastOrientation;
        calibrationOrientation.ReadSuccessful = true;
        return calibrationOrientation;
    }

    rawData.GyroX -= _gyroBiasX;
    rawData.GyroY -= _gyroBiasY;
    rawData.GyroZ -= _gyroBiasZ;

    float deltaTimeSeconds = elapsedTimeUs / 1000000.0f;
    
    // To get a more reliable orientation, we'll use a complementary filter with both the gyros and accelerometers.
    // Accelerometer orientation.
    float accelerometerRoll = atan2(rawData.AccelerationForceY, rawData.AccelerationForceZ) * RAD_TO_DEG;
    float accelerometerPitch = atan2(-rawData.AccelerationForceX, sqrt(rawData.AccelerationForceY * rawData.AccelerationForceY + rawData.AccelerationForceZ * rawData.AccelerationForceZ)) * RAD_TO_DEG;
    
    // Gyroscope orientation.
    float gyroscopeRoll = _lastOrientation.RollDeg + (rawData.GyroX * deltaTimeSeconds);
    float gyroscopePitch = _lastOrientation.PitchDeg + (rawData.GyroY * deltaTimeSeconds);
    float gyroscopeYaw = _lastOrientation.YawDeg + (rawData.GyroZ * deltaTimeSeconds);

    // Convert the filter time constant to a sample-rate-independent blend factor.
    float filterAlpha =
        Config::ORIENTATION_FILTER_TIME_CONSTANT_S /
        (Config::ORIENTATION_FILTER_TIME_CONSTANT_S + deltaTimeSeconds);
    float currentRoll = filterAlpha * gyroscopeRoll + (1.0f - filterAlpha) * accelerometerRoll;
    float currentPitch = filterAlpha * gyroscopePitch + (1.0f - filterAlpha) * accelerometerPitch;
    
    // For yaw we only have the gyro to go off of. If we don't want accumulating error
    // then we need to hook in a compass (not sure if a compass would be noisy or not so we may also want a filter here too).
    float currentYaw = gyroscopeYaw;

    Orientation currentOrientation = Orientation {currentRoll, currentPitch, currentYaw, true, rawData.GyroX, rawData.GyroY, rawData.GyroZ};
    
    _lastOrientation = currentOrientation;

    return currentOrientation;
}
