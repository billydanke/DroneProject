#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController() { }

PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::SetGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::SetIntegratorLimit(float limit) {
    _integratorLimit = abs(limit);
}

void PIDController::SetOutputLimit(float limit) {
    _outputLimit = abs(limit);
}

float PIDController::Update(float targetValue, float measuredValue, float dt) {
    
    if (dt <= 0.0f) return 0.0f;

    float error = targetValue - measuredValue;

    _integrator += error * dt;
    _integrator = constrain(_integrator, -_integratorLimit, _integratorLimit);

    float derivative = (error - _previousError) / dt;
    _previousError = error;

    float output = (_kp * error) + (_ki * _integrator) + (_kd * derivative);
    output = constrain(output, -_outputLimit, _outputLimit);

    return output;
}

void PIDController::Reset() {
    _integrator = 0.0f;
    _previousError = 0.0f;
}