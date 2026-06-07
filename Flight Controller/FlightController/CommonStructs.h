#pragma once

struct IMUData {
    float GyroX = 0.0f;
    float GyroY = 0.0f;
    float GyroZ = 0.0f;

    float AccelerationForceX = 0.0f;
    float AccelerationForceY = 0.0f;
    float AccelerationForceZ = 0.0f;
};

struct BarometerData {
    float PressureReadingKPa = 0.0f;
    float AltitudeMeters = 0.0f;
};

struct CompassData {
    float compassX = 0.0f;
    float compassY = 0.0f;
    float compassZ = 0.0f;
};

struct GPSData {
    double LatitudeDeg = 0.0f;
    double LongitudeDeg = 0.0f;
    float AltitudeMeters = 0.0f; // We probably don't want to rely on this as much as the barometer reading
    float GroundSpeedMetersPerSecond = 0.0f;
    float CourseRadians = 0.0f;
    int SatellitesConnectedCount = 0;
    bool IsLocationFixed = false;
};

struct Orientation {
    float RollDeg = 0.0f;
    float PitchDeg = 0.0f;
    float YawDeg = 0.0f;
};

struct MotorPWMOutput {
    int Motor1PwmUs = 1000;
    int Motor2PwmUs = 1000;
    int Motor3PwmUs = 1000;
    int Motor4PwmUs = 1000;
};

struct PilotCommand {
    float throttlePercent = 0.0f;

    float RollDeg = 0.0f;
    float PitchDeg = 0.0f;
    float YawDeg = 0.0f;

    bool DoEStop = false;
};