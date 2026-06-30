#pragma once

#include "CommonStructs.h"
#include "PIDController.h"
#include "Config.h"

class MotorController {
    private:
    PIDController _rollPID = PIDController(Config::ROLL_ANGLE_KP, Config::ROLL_ANGLE_KI, Config::ROLL_ANGLE_KD);
    PIDController _pitchPID = PIDController(Config::PITCH_ANGLE_KP, Config::PITCH_ANGLE_KI, Config::PITCH_ANGLE_KD);
    PIDController _yawPID = PIDController(Config::YAW_RATE_KP, Config::YAW_RATE_KI, Config::YAW_RATE_KD);

    FlightState& _flightState;
    MotorPWMOutput _currentMotorOutput = MotorPWMOutput {Config::PWM_MIN_US, Config::PWM_MIN_US, Config::PWM_MIN_US, Config::PWM_MIN_US };
    uint32_t _lastUpdateTimeUs = 0;
    bool _isInitialized = false;
    bool _mixerSaturatedLastUpdate = false;

    void ResetControllers();
    void WriteMinimumOutput();
    void WriteMotorPWM(const MotorPWMOutput& output);
    int PowerToPwmUs(float power) const;
    static float WrapAngleErrorDeg(float targetDeg, float measuredDeg);

    public:
    MotorController(FlightState& flightState);
    bool Init();

    bool Arm(float throttle);
    void Disarm();
    void EmergencyStop();
    bool IsArmed() const;

    bool UpdateMotorOutputs(float throttle, Orientation currentOrientation, Orientation targetOrientation);
    MotorPWMOutput GetCurrentMotorOutput() const;
};