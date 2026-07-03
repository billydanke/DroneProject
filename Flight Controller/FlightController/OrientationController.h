#pragma once
#include "CommonStructs.h"
#include "Config.h"

class OrientationController {
    private:
    bool _isInitialized = false;
    Orientation _lastOrientation = Orientation();
    uint32_t _lastMeasurementTimeUs = 0;

    bool _isOrientationCalibrating = false;
    bool _isOrientationCalibrationComplete = false;
    bool _isGyroCalibrationComplete = false;
    uint16_t _gyroCalibrationSampleCount = 0;
    float _gyroCalibrationSumX = 0.0f;
    float _gyroCalibrationSumY = 0.0f;
    float _gyroCalibrationSumZ = 0.0f;
    float _gyroBiasX = 0.0f;
    float _gyroBiasY = 0.0f;
    float _gyroBiasZ = 0.0f;

    bool _isCompassInitialized = false;
    bool _isMagnetometerCalibrating = false;
    bool _isMagnetometerCalibrationComplete = false;
    bool _hasCompassYawReference = false;
    uint16_t _magnetometerCalibrationSampleCount = 0;
    float _magnetometerMinX = 0.0f;
    float _magnetometerMinY = 0.0f;
    float _magnetometerMinZ = 0.0f;
    float _magnetometerMaxX = 0.0f;
    float _magnetometerMaxY = 0.0f;
    float _magnetometerMaxZ = 0.0f;
    float _magnetometerOffsetX = 0.0f;
    float _magnetometerOffsetY = 0.0f;
    float _magnetometerOffsetZ = 0.0f;
    float _magnetometerScaleX = 1.0f;
    float _magnetometerScaleY = 1.0f;
    float _magnetometerScaleZ = 1.0f;
    CompassData _lastCompassData = CompassData();

    bool InitCompass();
    bool WriteCompassRegister(uint8_t registerAddress, uint8_t value);
    bool ReadCompassRegister(uint8_t registerAddress, uint8_t& value);
    CompassData GetCompassData();
    CompassData ApplyMagnetometerCalibration(const CompassData& data) const;
    void UpdateMagnetometerCalibration(const CompassData& data);
    void ResetMagnetometerCalibrationSamples();
    float GetTiltCompensatedHeadingDeg(const CompassData& data, float rollDeg, float pitchDeg) const;
    IMUData GetIMUData();
    void UpdateGyroCalibration(const IMUData& data);
    void ResetGyroCalibrationSamples();
    static float NormalizeAngleDeg(float angleDeg);
    static float WrapAngleErrorDeg(float targetDeg, float measuredDeg);

    public:
    OrientationController();

    bool Init();

    void StartCalibration();
    bool IsCalibrating() const;
    bool IsCalibrationComplete() const;
    bool IsGyroCalibrating() const;
    bool IsCompassCalibrating() const;
    uint16_t GetGyroCalibrationSampleCount() const;
    uint16_t GetCompassCalibrationSampleCount() const;
    CompassData GetLastCompassData() const;

    Orientation GetOrientation();
};
