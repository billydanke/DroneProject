#pragma once
#include "CommonStructs.h"
#include "Config.h"

class OrientationController {
    private:
    bool _isInitialized = false;
    Orientation _lastOrientation = Orientation();
    uint32_t _lastMeasurementTimeUs = 0;

    bool _isCalibrating = false;
    bool _isCalibrationComplete = false;
    uint16_t _calibrationSampleCount = 0;
    float _gyroCalibrationSumX = 0.0f;
    float _gyroCalibrationSumY = 0.0f;
    float _gyroCalibrationSumZ = 0.0f;
    float _gyroBiasX = 0.0f;
    float _gyroBiasY = 0.0f;
    float _gyroBiasZ = 0.0f;

    IMUData GetIMUData();
    void UpdateCalibration(const IMUData& data);
    void ResetCalibrationSamples();

    public:
    OrientationController();

    bool Init();

    void StartCalibration();
    bool IsCalibrating() const;
    bool IsCalibrationComplete() const;
    uint16_t GetCalibrationSampleCount() const;

    Orientation GetOrientation();
};
