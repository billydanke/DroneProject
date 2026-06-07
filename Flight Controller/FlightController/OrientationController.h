#pragma once
#include "CommonStructs.h"
#include "Config.h"

class OrientationController {
    private:
    IMUData GetIMUData();

    Orientation _lastOrientation = Orientation();
    unsigned long _lastMeasurementTime = 0;

    public:
    OrientationController();

    Orientation GetOrientation();
};