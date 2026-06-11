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

void PIDController::SetDerivativeFilterTimeConstant(float timeConstant) {
    _derivativeFilterTimeConstant = max(0.0f, timeConstant);
}

float PIDController::Update(float targetValue, float measuredValue, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = targetValue - measuredValue;

    float derivative = 0.0f;
    if (_hasPreviousMeasurement) {
        float rawDerivative = -(measuredValue - _previousMeasurement) / dt;
        float filterAlpha =
            _derivativeFilterTimeConstant /
            (_derivativeFilterTimeConstant + dt);
        _filteredDerivative =
            filterAlpha * _filteredDerivative +
            (1.0f - filterAlpha) * rawDerivative;
        derivative = _filteredDerivative;
    } else {
        _hasPreviousMeasurement = true;
    }
    _previousMeasurement = measuredValue;

    float candidateIntegrator = constrain(
        _integrator + error * dt,
        -_integratorLimit,
        _integratorLimit);
    float proportionalAndDerivative = (_kp * error) + (_kd * derivative);
    float candidateOutput =
        proportionalAndDerivative + (_ki * candidateIntegrator);
    float integratorOutputChange =
        _ki * (candidateIntegrator - _integrator);

    bool outputIsSaturatedHigh = candidateOutput > _outputLimit;
    bool outputIsSaturatedLow = candidateOutput < -_outputLimit;
    bool integratorReducesSaturation =
        (outputIsSaturatedHigh && integratorOutputChange < 0.0f) ||
        (outputIsSaturatedLow && integratorOutputChange > 0.0f);

    if ((!outputIsSaturatedHigh && !outputIsSaturatedLow) ||
        integratorReducesSaturation) {
        _integrator = candidateIntegrator;
    }

    float output = proportionalAndDerivative + (_ki * _integrator);
    output = constrain(output, -_outputLimit, _outputLimit);

    return output;
}

void PIDController::Reset() {
    _integrator = 0.0f;
    _previousMeasurement = 0.0f;
    _filteredDerivative = 0.0f;
    _hasPreviousMeasurement = false;
}
