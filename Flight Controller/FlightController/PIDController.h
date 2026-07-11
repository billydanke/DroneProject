#pragma once

class PIDController {
    private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integrator = 0.0f;
    float _previousMeasurement = 0.0f;
    float _filteredDerivative = 0.0f;
    bool _hasPreviousMeasurement = false;

    float _integratorLimit = 0.5f;
    float _outputLimit = 1.0f;
    float _derivativeFilterTimeConstant = 0.007f;

    public:
    PIDController();
    PIDController(float kp, float ki, float kd);

    void SetGains(float kp, float ki, float kd);
    void SetIntegratorLimit(float limit);
    void SetOutputLimit(float limit);
    void SetDerivativeFilterTimeConstant(float timeConstant);

    float Update(float targetValue, float measuredValue, float dt, bool allowIntegration = true);
    void Reset();
};
