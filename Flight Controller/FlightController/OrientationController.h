#pragma once
#include "CommonStructs.h"
#include "Config.h"

class OrientationController {
    private:
    bool _isInitialized = false;
    Orientation _lastOrientation = Orientation();
    uint32_t _lastMeasurementTimeUs = 0;
    
    IMUData GetIMUData();

    public:
    OrientationController();

    bool Init();

    Orientation GetOrientation();
};