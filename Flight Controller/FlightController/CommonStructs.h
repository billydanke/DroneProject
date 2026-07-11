#pragma once

#include <stdint.h>

struct IMUData {
    float GyroX = 0.0f;
    float GyroY = 0.0f;
    float GyroZ = 0.0f;

    float AccelerationForceX = 0.0f;
    float AccelerationForceY = 0.0f;
    float AccelerationForceZ = 0.0f;

    bool ReadSuccessful = false;
};

struct BarometerData {
    float PressurePa = 0.0f;
    float PressureReadingKPa = 0.0f;
    float TemperatureC = 0.0f;
    float AltitudeMeters = 0.0f;
    float VerticalSpeedMetersPerSecond = 0.0f;
    bool ReadSuccessful = false;
};

struct CompassData {
    float CompassX = 0.0f;
    float CompassY = 0.0f;
    float CompassZ = 0.0f;
    float RawX = 0.0f;
    float RawY = 0.0f;
    float RawZ = 0.0f;
    float HeadingDeg = 0.0f;
    float MagneticFieldMagnitude = 0.0f;
    bool IsCalibrated = false;
    bool ReadSuccessful = false;
};

struct GPSData {
    double LatitudeDeg = 0.0f;
    double LongitudeDeg = 0.0f;
    float AltitudeMeters = 0.0f; // We probably don't want to rely on this as much as the barometer reading
    float GroundSpeedMetersPerSecond = 0.0f;
    float CourseDeg = 0.0f;
    float CourseRadians = 0.0f;
    float Hdop = 0.0f;
    uint32_t FixAgeMs = 0;
    int SatellitesConnectedCount = 0;
    bool IsLocationFixed = false;
    bool HasNewData = false;
    bool ReadSuccessful = false;
};

struct Orientation {
    float RollDeg = 0.0f;
    float PitchDeg = 0.0f;
    float YawDeg = 0.0f;

    bool ReadSuccessful = false;

    float RollRateDegS = 0.0f;
    float PitchRateDegS = 0.0f;
    float YawRateDegS = 0.0f;
};

struct MotorOutput {
    float Motor1Power = 0.0f;
    float Motor2Power = 0.0f;
    float Motor3Power = 0.0f;
    float Motor4Power = 0.0f;
};

struct PilotCommand {
    float ThrottlePercent = 0.0f;

    float RollDeg = 0.0f;
    float PitchDeg = 0.0f;
    float YawRateDegS = 0.0f;
};

struct FlightState {
    bool IsArmed = false;
};