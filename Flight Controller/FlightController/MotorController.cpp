#include <Arduino.h>
#include "MotorController.h"

MotorController::MotorController() {
    
}

void MotorController::Init() {
    // Ensure the motor PWM pins are initialized.
    _motor1ESC.attach(Config::MOTOR_1_PIN, Config::PWM_MIN_US, Config::PWM_MAX_US);
    _motor2ESC.attach(Config::MOTOR_2_PIN, Config::PWM_MIN_US, Config::PWM_MAX_US);
    _motor3ESC.attach(Config::MOTOR_3_PIN, Config::PWM_MIN_US, Config::PWM_MAX_US);
    _motor4ESC.attach(Config::MOTOR_4_PIN, Config::PWM_MIN_US, Config::PWM_MAX_US);

    // Arm to minimum throttle.
    _motor1ESC.writeMicroseconds(Config::PWM_MIN_US);
    _motor2ESC.writeMicroseconds(Config::PWM_MIN_US);
    _motor3ESC.writeMicroseconds(Config::PWM_MIN_US);
    _motor4ESC.writeMicroseconds(Config::PWM_MIN_US);

    _lastUpdateTimeUs = micros();
}

void MotorController::WriteMotorPWM(MotorPWMOutput output) {
    _motor1ESC.writeMicroseconds(output.Motor1PwmUs);
    _motor2ESC.writeMicroseconds(output.Motor2PwmUs);
    _motor3ESC.writeMicroseconds(output.Motor3PwmUs);
    _motor4ESC.writeMicroseconds(output.Motor4PwmUs);
}

void MotorController::UpdateMotorOutputs(float throttle, Orientation currentOrientation, Orientation targetOrientation) {
    uint32_t currentTimeUs = micros();
    uint32_t elapsedTimeUs = currentTimeUs - _lastUpdateTimeUs;
    float elapsedSeconds = elapsedTimeUs / 1000000.0f;
    _lastUpdateTimeUs = currentTimeUs;

    // Pass the errors into their respective PIDs.
    float rollU = _rollPID.Update(targetOrientation.RollDeg, currentOrientation.RollDeg, elapsedSeconds);
    float pitchU = _pitchPID.Update(targetOrientation.PitchDeg, currentOrientation.PitchDeg, elapsedSeconds);
    float yawU = _yawPID.Update(targetOrientation.YawDeg, currentOrientation.YawDeg, elapsedSeconds);

    // Mix motor powers based on the current throttle + the U values.
    // This should output in the range of 0-1.
    float frontLeftRawPower = throttle + rollU - pitchU + yawU;
    float frontRightRawPower = throttle - rollU - pitchU - yawU;
    float backRightRawPower = throttle - rollU + pitchU + yawU;
    float backLeftRawPower = throttle + rollU + pitchU - yawU;

    // Convert the raw 0-1 power range into their PWM signals.
    int pwmRange = Config::PWM_MAX_US - Config::PWM_MIN_US;
    int frontLeftPWMPower = Config::PWM_MIN_US + (frontLeftRawPower * pwmRange);
    int frontRightPWMPower = Config::PWM_MIN_US + (frontRightRawPower * pwmRange);
    int backRightPWMPower = Config::PWM_MIN_US + (backRightRawPower * pwmRange);
    int backLeftPWMPower = Config::PWM_MIN_US + (backLeftRawPower * pwmRange);

    _currentMotorOutput = MotorPWMOutput {frontLeftPWMPower, frontRightPWMPower, backRightPWMPower, backLeftPWMPower};

    WriteMotorPWM(_currentMotorOutput);
}