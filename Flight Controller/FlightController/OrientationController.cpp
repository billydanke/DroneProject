#include "OrientationController.h"
#include <Arduino.h>
#include <float.h>
#include <Preferences.h>
#include <Wire.h>

namespace {
    constexpr uint8_t QMC_DATA_REGISTER = 0x00;
    constexpr uint8_t QMC_STATUS_REGISTER = 0x06;
    constexpr uint8_t QMC_CONTROL_1_REGISTER = 0x09;
    constexpr uint8_t QMC_CONTROL_2_REGISTER = 0x0A;
    constexpr uint8_t QMC_SET_RESET_PERIOD_REGISTER = 0x0B;

    constexpr uint8_t QMC_STATUS_DATA_READY = 0x01;
    constexpr uint8_t QMC_STATUS_OVERFLOW = 0x02;
    constexpr uint8_t QMC_SOFT_RESET = 0x80;
    constexpr uint8_t QMC_RECOMMENDED_SET_RESET_PERIOD = 0x01;

    constexpr char MAGNETOMETER_PREFERENCES_NAMESPACE[] = "magcal";
    constexpr uint32_t MAGNETOMETER_CALIBRATION_MAGIC = 0x514D4331UL;
    constexpr uint16_t MAGNETOMETER_CALIBRATION_VERSION = 1;
}

OrientationController::OrientationController() { }

bool OrientationController::Init() {
    _isInitialized = false;
    _isOrientationCalibrating = false;
    _isOrientationCalibrationComplete = false;
    _isGyroCalibrationComplete = false;
    _isCompassInitialized = false;
    _isMagnetometerCalibrating = false;
    _isMagnetometerCalibrationComplete = false;
    _hasCompassYawReference = false;

    // Wake the MPU6050 (clears SLEEP bit in PWR_MGMT_1).
    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    // Set DLPF bandwidth (CONFIG register 0x1A). Also sets gyro output rate to 1kHz.
    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x1A);
    Wire.write(Config::MPU_DLPF_CFG);
    if (Wire.endTransmission(true) != 0) return false;

    // Set gyro full-scale range to +-250°/s (GYRO_CONFIG register 0x1B).
    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    // Set accel full-scale range to +-2g (ACCEL_CONFIG register 0x1C).
    Wire.beginTransmission(Config::MPU_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) return false;

    if (!InitCompass()) return false;
    if (LoadMagnetometerCalibration()) {
        Serial.println("Loaded saved compass calibration.");
    } else {
        Serial.println("No saved compass calibration found; compass calibration required.");
    }

    _lastMeasurementTimeUs = micros();
    _isInitialized = true;
    return true;
}

bool OrientationController::InitCompass() {
    if (!WriteCompassRegister(QMC_CONTROL_2_REGISTER, QMC_SOFT_RESET)) return false;
    delay(10);

    if (!WriteCompassRegister(QMC_SET_RESET_PERIOD_REGISTER, QMC_RECOMMENDED_SET_RESET_PERIOD)) return false;

    uint8_t controlValue = Config::QMC5883L_OVERSAMPLING_512 | Config::QMC5883L_RANGE_2G | Config::QMC5883L_OUTPUT_RATE_100_HZ | Config::QMC5883L_CONTINUOUS_MODE;
    if (!WriteCompassRegister(QMC_CONTROL_1_REGISTER, controlValue)) return false;

    _isCompassInitialized = true;
    return true;
}

bool OrientationController::WriteCompassRegister(uint8_t registerAddress, uint8_t value) {
    Wire.beginTransmission(Config::QMC5883L_ADDRESS);
    Wire.write(registerAddress);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool OrientationController::ReadCompassRegister(uint8_t registerAddress, uint8_t& value) {
    Wire.beginTransmission(Config::QMC5883L_ADDRESS);
    Wire.write(registerAddress);
    if (Wire.endTransmission(false) != 0) return false;

    size_t bytesReceived = Wire.requestFrom(Config::QMC5883L_ADDRESS, 1, true);
    if (bytesReceived != 1 || Wire.available() < 1) return false;

    value = Wire.read();
    return true;
}

float OrientationController::NormalizeAngleDeg(float angleDeg) {
    float normalized = fmodf(angleDeg, 360.0f);
    if (normalized < 0.0f) normalized += 360.0f;
    return normalized;
}

float OrientationController::WrapAngleErrorDeg(float targetDeg, float measuredDeg) {
    float errorDeg = fmodf(targetDeg - measuredDeg + 180.0f, 360.0f);
    if (errorDeg < 0.0f) errorDeg += 360.0f;

    return errorDeg - 180.0f;
}

void OrientationController::StartCalibration() {
    StartGyroscopeCalibration();
}

void OrientationController::StartGyroscopeCalibration() {
    _isOrientationCalibrationComplete = false;
    _isGyroCalibrationComplete = false;
    _isMagnetometerCalibrating = false;
    _hasCompassYawReference = false;
    _isOrientationCalibrating = _isInitialized;
    _gyroBiasX = 0.0f;
    _gyroBiasY = 0.0f;
    _gyroBiasZ = 0.0f;
    _lastCompassData = CompassData();
    ResetGyroCalibrationSamples();
    ResetMagnetometerCalibrationSamples();
}

void OrientationController::StartCompassCalibration() {
    if (!_isInitialized || !_isCompassInitialized) return;

    _isMagnetometerCalibrationComplete = false;
    _isMagnetometerCalibrating = true;
    _isOrientationCalibrating = true;
    _isOrientationCalibrationComplete = false;
    _hasCompassYawReference = false;
    _magnetometerOffsetX = 0.0f;
    _magnetometerOffsetY = 0.0f;
    _magnetometerOffsetZ = 0.0f;
    _magnetometerScaleX = 1.0f;
    _magnetometerScaleY = 1.0f;
    _magnetometerScaleZ = 1.0f;
    _lastCompassData = CompassData();
    ResetMagnetometerCalibrationSamples();
}

bool OrientationController::CalibrateGyroscope() {
    StartGyroscopeCalibration();
    if (!_isOrientationCalibrating) return false;

    uint32_t calibrationStartMs = millis();
    uint16_t lastReportedSampleCount = 0;

    while (IsGyroCalibrating()) {
        Orientation orientation = GetOrientation();
        if (!orientation.ReadSuccessful) {
            _isOrientationCalibrating = false;
            _isOrientationCalibrationComplete = false;
            return false;
        }

        uint16_t sampleCount = GetGyroCalibrationSampleCount();
        constexpr uint16_t calibrationReportInterval = 10;
        if (sampleCount / calibrationReportInterval != lastReportedSampleCount / calibrationReportInterval) {
            Serial.print("Gyroscope calibration samples: ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(Config::GYRO_CALIBRATION_SAMPLE_COUNT);
        }
        lastReportedSampleCount = sampleCount;

        if (millis() - calibrationStartMs > Config::GYRO_CALIBRATION_TIMEOUT_MS) {
            _isOrientationCalibrating = false;
            _isOrientationCalibrationComplete = false;
            return false;
        }

        delayMicroseconds(1000000UL / Config::LOOP_RATE_HZ);
    }

    _lastOrientation = Orientation();
    _lastMeasurementTimeUs = micros();
    return _isGyroCalibrationComplete;
}

bool OrientationController::CalibrateCompass() {
    StartCompassCalibration();
    if (!_isMagnetometerCalibrating) return false;

    uint32_t calibrationStartMs = millis();
    uint16_t lastReportedSampleCount = 0;

    while (IsCompassCalibrating()) {
        Orientation orientation = GetOrientation();
        if (!orientation.ReadSuccessful) {
            _isMagnetometerCalibrating = false;
            _isOrientationCalibrating = false;
            _isOrientationCalibrationComplete = _isGyroCalibrationComplete;
            return false;
        }

        uint16_t sampleCount = GetCompassCalibrationSampleCount();
        constexpr uint16_t calibrationReportInterval = 50;
        if (sampleCount / calibrationReportInterval != lastReportedSampleCount / calibrationReportInterval) {
            Serial.print("Compass calibration samples: ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(Config::MAGNETOMETER_CALIBRATION_SAMPLE_COUNT);
        }
        lastReportedSampleCount = sampleCount;

        if (millis() - calibrationStartMs > Config::MAGNETOMETER_CALIBRATION_TIMEOUT_MS) {
            _isMagnetometerCalibrating = false;
            _isOrientationCalibrating = false;
            _isOrientationCalibrationComplete = _isGyroCalibrationComplete;
            _hasCompassYawReference = false;
            return false;
        }

        delay(50);
    }

    _hasCompassYawReference = false;
    _lastMeasurementTimeUs = micros();
    return _isMagnetometerCalibrationComplete;
}

bool OrientationController::IsCalibrating() const {
    return _isOrientationCalibrating;
}

bool OrientationController::IsCalibrationComplete() const {
    return _isOrientationCalibrationComplete;
}

bool OrientationController::IsGyroCalibrating() const {
    return _isOrientationCalibrating && !_isGyroCalibrationComplete;
}

bool OrientationController::IsCompassCalibrating() const {
    return _isMagnetometerCalibrating;
}

bool OrientationController::HasValidCompassCalibration() const {
    return _isCompassInitialized && _isMagnetometerCalibrationComplete;
}

uint16_t OrientationController::GetGyroCalibrationSampleCount() const {
    return _gyroCalibrationSampleCount;
}

uint16_t OrientationController::GetCompassCalibrationSampleCount() const {
    return _magnetometerCalibrationSampleCount;
}

CompassData OrientationController::GetLastCompassData() const {
    return _lastCompassData;
}

void OrientationController::ResetGyroCalibrationSamples() {
    _gyroCalibrationSampleCount = 0;
    _gyroCalibrationSumX = 0.0f;
    _gyroCalibrationSumY = 0.0f;
    _gyroCalibrationSumZ = 0.0f;
}

void OrientationController::UpdateGyroCalibration(const IMUData& data) {
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

    if (_gyroCalibrationSampleCount > 0) {
        float sampleCount = static_cast<float>(_gyroCalibrationSampleCount);
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
    _gyroCalibrationSampleCount++;

    if (_gyroCalibrationSampleCount < Config::GYRO_CALIBRATION_SAMPLE_COUNT) {
        return;
    }

    float sampleCount = static_cast<float>(_gyroCalibrationSampleCount);
    _gyroBiasX = _gyroCalibrationSumX / sampleCount;
    _gyroBiasY = _gyroCalibrationSumY / sampleCount;
    _gyroBiasZ = _gyroCalibrationSumZ / sampleCount;
    _isGyroCalibrationComplete = true;
    _isMagnetometerCalibrating = false;
    _isOrientationCalibrationComplete = true;
    _isOrientationCalibrating = false;
    _lastOrientation = Orientation();
    _lastMeasurementTimeUs = micros();
}

void OrientationController::ResetMagnetometerCalibrationSamples() {
    _magnetometerCalibrationSampleCount = 0;

    _magnetometerMinX = FLT_MAX;
    _magnetometerMinY = FLT_MAX;
    _magnetometerMinZ = FLT_MAX;
    _magnetometerMaxX = -FLT_MAX;
    _magnetometerMaxY = -FLT_MAX;
    _magnetometerMaxZ = -FLT_MAX;
}

CompassData OrientationController::ApplyMagnetometerCalibration(const CompassData& data) const {
    CompassData calibratedData = data;

    calibratedData.CompassX = (data.RawX - _magnetometerOffsetX) * _magnetometerScaleX;
    calibratedData.CompassY = (data.RawY - _magnetometerOffsetY) * _magnetometerScaleY;
    calibratedData.CompassZ = (data.RawZ - _magnetometerOffsetZ) * _magnetometerScaleZ;

    calibratedData.MagneticFieldMagnitude = sqrt(calibratedData.CompassX * calibratedData.CompassX + calibratedData.CompassY * calibratedData.CompassY + calibratedData.CompassZ * calibratedData.CompassZ);

    calibratedData.IsCalibrated = _isMagnetometerCalibrationComplete;

    return calibratedData;
}

bool OrientationController::LoadMagnetometerCalibration() {
    Preferences preferences;
    if (!preferences.begin(MAGNETOMETER_PREFERENCES_NAMESPACE, true)) {
        return false;
    }

    uint32_t magic = preferences.getUInt("magic", 0);
    uint16_t version = preferences.getUShort("version", 0);
    if (magic != MAGNETOMETER_CALIBRATION_MAGIC || version != MAGNETOMETER_CALIBRATION_VERSION) {
        preferences.end();
        return false;
    }

    _magnetometerOffsetX = preferences.getFloat("offX", 0.0f);
    _magnetometerOffsetY = preferences.getFloat("offY", 0.0f);
    _magnetometerOffsetZ = preferences.getFloat("offZ", 0.0f);
    _magnetometerScaleX = preferences.getFloat("scaleX", 1.0f);
    _magnetometerScaleY = preferences.getFloat("scaleY", 1.0f);
    _magnetometerScaleZ = preferences.getFloat("scaleZ", 1.0f);
    preferences.end();

    if (!isfinite(_magnetometerOffsetX) || !isfinite(_magnetometerOffsetY) || !isfinite(_magnetometerOffsetZ) ||
        !isfinite(_magnetometerScaleX) || !isfinite(_magnetometerScaleY) || !isfinite(_magnetometerScaleZ) ||
        _magnetometerScaleX <= 0.0f || _magnetometerScaleY <= 0.0f || _magnetometerScaleZ <= 0.0f) {

        _magnetometerOffsetX = 0.0f;
        _magnetometerOffsetY = 0.0f;
        _magnetometerOffsetZ = 0.0f;
        _magnetometerScaleX = 1.0f;
        _magnetometerScaleY = 1.0f;
        _magnetometerScaleZ = 1.0f;
        _isMagnetometerCalibrationComplete = false;
        return false;
    }

    _isMagnetometerCalibrationComplete = true;
    return true;
}

bool OrientationController::SaveMagnetometerCalibration() {
    if (!_isMagnetometerCalibrationComplete) return false;

    Preferences preferences;
    if (!preferences.begin(MAGNETOMETER_PREFERENCES_NAMESPACE, false)) {
        return false;
    }

    preferences.putUInt("magic", MAGNETOMETER_CALIBRATION_MAGIC);
    preferences.putUShort("version", MAGNETOMETER_CALIBRATION_VERSION);
    preferences.putFloat("offX", _magnetometerOffsetX);
    preferences.putFloat("offY", _magnetometerOffsetY);
    preferences.putFloat("offZ", _magnetometerOffsetZ);
    preferences.putFloat("scaleX", _magnetometerScaleX);
    preferences.putFloat("scaleY", _magnetometerScaleY);
    preferences.putFloat("scaleZ", _magnetometerScaleZ);
    preferences.end();

    return true;
}

void OrientationController::UpdateMagnetometerCalibration(const CompassData& data) {
    if (!data.ReadSuccessful) return;

    _magnetometerMinX = min(_magnetometerMinX, data.RawX);
    _magnetometerMinY = min(_magnetometerMinY, data.RawY);
    _magnetometerMinZ = min(_magnetometerMinZ, data.RawZ);
    _magnetometerMaxX = max(_magnetometerMaxX, data.RawX);
    _magnetometerMaxY = max(_magnetometerMaxY, data.RawY);
    _magnetometerMaxZ = max(_magnetometerMaxZ, data.RawZ);
    _magnetometerCalibrationSampleCount++;

    if (_magnetometerCalibrationSampleCount < Config::MAGNETOMETER_CALIBRATION_SAMPLE_COUNT) {
        return;
    }

    float radiusX = (_magnetometerMaxX - _magnetometerMinX) * 0.5f;
    float radiusY = (_magnetometerMaxY - _magnetometerMinY) * 0.5f;
    float radiusZ = (_magnetometerMaxZ - _magnetometerMinZ) * 0.5f;

    if (radiusX < Config::MAGNETOMETER_MIN_CALIBRATION_RANGE || radiusY < Config::MAGNETOMETER_MIN_CALIBRATION_RANGE || radiusZ < Config::MAGNETOMETER_MIN_CALIBRATION_RANGE) {
        Serial.println("Compass calibration range too small; rotate through more roll, pitch, and yaw.");
        ResetMagnetometerCalibrationSamples();
        return;
    }

    _magnetometerOffsetX = (_magnetometerMaxX + _magnetometerMinX) * 0.5f;
    _magnetometerOffsetY = (_magnetometerMaxY + _magnetometerMinY) * 0.5f;
    _magnetometerOffsetZ = (_magnetometerMaxZ + _magnetometerMinZ) * 0.5f;

    float averageRadius = (radiusX + radiusY + radiusZ) / 3.0f;
    _magnetometerScaleX = averageRadius / radiusX;
    _magnetometerScaleY = averageRadius / radiusY;
    _magnetometerScaleZ = averageRadius / radiusZ;

    _isMagnetometerCalibrating = false;
    _isMagnetometerCalibrationComplete = true;
    _isOrientationCalibrating = false;
    _isOrientationCalibrationComplete = true;
    _hasCompassYawReference = false;
    if (SaveMagnetometerCalibration()) {
        Serial.println("Saved compass calibration.");
    } else {
        Serial.println("ERROR: Failed to save compass calibration.");
    }
    _lastCompassData = ApplyMagnetometerCalibration(data);
    _lastMeasurementTimeUs = micros();
}

CompassData OrientationController::GetCompassData() {
    CompassData data;

    if (!_isCompassInitialized) return data;

    uint8_t status = 0;
    if (!ReadCompassRegister(QMC_STATUS_REGISTER, status)) return data;
    if ((status & QMC_STATUS_OVERFLOW) != 0 || (status & QMC_STATUS_DATA_READY) == 0) {
        return data;
    }

    Wire.beginTransmission(Config::QMC5883L_ADDRESS);
    Wire.write(QMC_DATA_REGISTER);
    if (Wire.endTransmission(false) != 0) {
        return data;
    }

    size_t bytesReceived = Wire.requestFrom(Config::QMC5883L_ADDRESS, 6, true);
    if (bytesReceived != 6 || Wire.available() < 6) {
        while (Wire.available() > 0) {
            Wire.read();
        }
        return data;
    }

    int16_t rawX = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
    int16_t rawY = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
    int16_t rawZ = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
    data.RawX = static_cast<float>(rawX);
    data.RawY = static_cast<float>(rawY);
    data.RawZ = static_cast<float>(rawZ);

    data.CompassX = data.RawX;
    data.CompassY = data.RawY;
    data.CompassZ = data.RawZ;

    data.MagneticFieldMagnitude = sqrt(data.CompassX * data.CompassX + data.CompassY * data.CompassY + data.CompassZ * data.CompassZ);

    data.ReadSuccessful = data.MagneticFieldMagnitude >= Config::MAGNETOMETER_MIN_VALID_MAGNITUDE && data.MagneticFieldMagnitude <= Config::MAGNETOMETER_MAX_VALID_MAGNITUDE;

    if (data.ReadSuccessful && _isMagnetometerCalibrationComplete) {
        data = ApplyMagnetometerCalibration(data);
        data.IsCalibrated = true;
    }

    _lastCompassData = data;
    return data;
}

float OrientationController::GetTiltCompensatedHeadingDeg(const CompassData& data, float rollDeg, float pitchDeg) const {
    float rollRad = rollDeg * DEG_TO_RAD;
    float pitchRad = pitchDeg * DEG_TO_RAD;

    float cosRoll = cos(rollRad);
    float sinRoll = sin(rollRad);
    float cosPitch = cos(pitchRad);
    float sinPitch = sin(pitchRad);

    float horizontalX = data.CompassX * cosPitch + data.CompassZ * sinPitch;
    float horizontalY = data.CompassX * sinRoll * sinPitch + data.CompassY * cosRoll - data.CompassZ * sinRoll * cosPitch;

    return NormalizeAngleDeg(atan2(horizontalY, horizontalX) * RAD_TO_DEG);
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

    if (_isOrientationCalibrating) {
        if (!_isGyroCalibrationComplete) {
            UpdateGyroCalibration(rawData);
        }
        else if (_isMagnetometerCalibrating) {
            UpdateMagnetometerCalibration(GetCompassData());
        }

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
    float filterAlpha = Config::ORIENTATION_FILTER_TIME_CONSTANT_S / (Config::ORIENTATION_FILTER_TIME_CONSTANT_S + deltaTimeSeconds);
    float currentRoll = filterAlpha * gyroscopeRoll + (1.0f - filterAlpha) * accelerometerRoll;
    float currentPitch = filterAlpha * gyroscopePitch + (1.0f - filterAlpha) * accelerometerPitch;
    
    float currentYaw = gyroscopeYaw;
    CompassData compassData = GetCompassData();

    if (compassData.ReadSuccessful && compassData.IsCalibrated) {
        float compassHeading = GetTiltCompensatedHeadingDeg(compassData, currentRoll, currentPitch);
        compassData.HeadingDeg = compassHeading;
        _lastCompassData = compassData;

        if (!_hasCompassYawReference) {
            currentYaw = compassHeading;
            _hasCompassYawReference = true;
        }
        else {
            float yawErrorDeg = WrapAngleErrorDeg(compassHeading, gyroscopeYaw);
            float yawFilterAlpha = Config::MAGNETOMETER_YAW_FILTER_TIME_CONSTANT_S / (Config::MAGNETOMETER_YAW_FILTER_TIME_CONSTANT_S + deltaTimeSeconds);
            currentYaw = NormalizeAngleDeg(gyroscopeYaw + ((1.0f - yawFilterAlpha) * yawErrorDeg));
        }
    }
    else {
        currentYaw = NormalizeAngleDeg(currentYaw);
    }

    Orientation currentOrientation = Orientation {currentRoll, currentPitch, currentYaw, true, rawData.GyroX, rawData.GyroY, rawData.GyroZ};
    
    _lastOrientation = currentOrientation;

    return currentOrientation;
}
