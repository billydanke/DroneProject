#pragma once

#include <Servo.h>
#include "CommonStructs.h"
#include "PIDController.h"
#include "Config.h"

class MotorController {
    private:
    PIDController _rollPID = PIDController(Config::ROLL_ANGLE_KP, Config::ROLL_ANGLE_KI, Config::ROLL_ANGLE_KD);
    PIDController _pitchPID = PIDController(Config::PITCH_ANGLE_KP, Config::PITCH_ANGLE_KI, Config::PITCH_ANGLE_KD);
    PIDController _yawPID = PIDController(Config::YAW_ANGLE_KP, Config::YAW_ANGLE_KI, Config::YAW_ANGLE_KD);

    Servo _motor1ESC;
    Servo _motor2ESC;
    Servo _motor3ESC;
    Servo _motor4ESC;
    MotorPWMOutput _currentMotorOutput = MotorPWMOutput {1000, 1000, 1000, 1000};
    uint32_t _lastUpdateTimeUs = 0;

    public:
    MotorController();
    void Init();

    void UpdateMotorOutputs(float throttle, Orientation currentOriention, Orientation targetOrientation);
    void WriteMotorPWM(MotorPWMOutput output);
};