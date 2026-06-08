#pragma once
#include "CommonStructs.h"
#include "Config.h"

class OrientationController {
    private:
    bool _isInitialized = false;
    Orientation _lastOrientation = Orientation();
    unsigned long _lastMeasurementTime = 0;
    
    IMUData GetIMUData();

    public:
    OrientationController();

    void Init();

    Orientation GetOrientation();
};