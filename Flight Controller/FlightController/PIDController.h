#pragma once

class PIDController {
    private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integrator = 0.0f;
    float _previousError = 0.0f;

    float _integratorLimit = 0.5f;
    float _outputLimit = 1.0f;

    public:
    PIDController();
    PIDController(float kp, float ki, float kd);

    void SetGains(float kp, float ki, float kd);
    void SetIntegratorLimit(float limit);
    void SetOutputLimit(float limit);

    float Update(float targetValue, float measuredValue, float dt);
    void Reset();
};