#include "OrientationController.h"
#include <Arduino.h>
#include <Wire.h>

OrientationController::OrientationController() { }

bool OrientationController::Init() {
    _isInitialized = false;

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

    float deltaTimeSeconds = elapsedTimeUs / 1000000.0f;
    
    // To get a more reliable orientation, we'll use a complementary filter with both the gyros and accelerometers.
    // Accelerometer orientation.
    float accelerometerRoll = atan2(rawData.AccelerationForceY, rawData.AccelerationForceZ) * RAD_TO_DEG;
    float accelerometerPitch = atan2(-rawData.AccelerationForceX, sqrt(rawData.AccelerationForceY * rawData.AccelerationForceY + rawData.AccelerationForceZ * rawData.AccelerationForceZ)) * RAD_TO_DEG;
    
    // Gyroscope orientation.
    float gyroscopeRoll = _lastOrientation.RollDeg + (rawData.GyroX * deltaTimeSeconds);
    float gyroscopePitch = _lastOrientation.PitchDeg + (rawData.GyroY * deltaTimeSeconds);
    float gyroscopeYaw = _lastOrientation.YawDeg + (rawData.GyroZ * deltaTimeSeconds);

    // Then we blend them together.
    // Primarily trust the gyros for quick movement (98% trust), but the remaining 2% is based on the accelerometers.
    // This is to slowly adjust for accumulating drift from the gyros alone.
    float currentRoll = 0.98 * gyroscopeRoll + 0.02 * accelerometerRoll;
    float currentPitch = 0.98 * gyroscopePitch + 0.02 * accelerometerPitch;
    
    // For yaw we only have the gyro to go off of. If we don't want accumulating error
    // then we need to hook in a compass (not sure if a compass would be noisy or not so we may also want a filter here too).
    float currentYaw = gyroscopeYaw;

    Orientation currentOrientation = Orientation { currentRoll, currentPitch, currentYaw, true };
    
    _lastOrientation = currentOrientation;

    return currentOrientation;
}