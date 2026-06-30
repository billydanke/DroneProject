#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include "MotorController.h"

namespace {
    // These are the internal PWM channels, not to be confused with Config's motor pins.
    constexpr uint8_t MOTOR_1_PWM_CHANNEL = 0;
    constexpr uint8_t MOTOR_2_PWM_CHANNEL = 1;
    constexpr uint8_t MOTOR_3_PWM_CHANNEL = 2;
    constexpr uint8_t MOTOR_4_PWM_CHANNEL = 3;

    // Front-left, CW
    constexpr float MOTOR_1_ROLL_MIX = 1.0f;
    constexpr float MOTOR_1_PITCH_MIX = -1.0f;
    constexpr float MOTOR_1_YAW_MIX = 1.0f;

    // Front-right, CCW
    constexpr float MOTOR_2_ROLL_MIX = -1.0f;
    constexpr float MOTOR_2_PITCH_MIX = -1.0f;
    constexpr float MOTOR_2_YAW_MIX = -1.0f;

    // Back-right, CW
    constexpr float MOTOR_3_ROLL_MIX = -1.0f;
    constexpr float MOTOR_3_PITCH_MIX = 1.0f;
    constexpr float MOTOR_3_YAW_MIX = 1.0f;

    // Back-left, CCW
    constexpr float MOTOR_4_ROLL_MIX = 1.0f;
    constexpr float MOTOR_4_PITCH_MIX = 1.0f;
    constexpr float MOTOR_4_YAW_MIX = -1.0f;

    float GetMinimum(const float values[], size_t count) {
        float minimum = values[0];
        for (size_t index = 1; index < count; index++) {
            minimum = min(minimum, values[index]);
        }
        return minimum;
    }

    float GetMaximum(const float values[], size_t count) {
        float maximum = values[0];
        for (size_t index = 1; index < count; index++) {
            maximum = max(maximum, values[index]);
        }
        return maximum;
    }

    float GetSpan(const float values[], size_t count) {
        return GetMaximum(values, count) - GetMinimum(values, count);
    }

    // This is some bs but works.
    uint32_t PulseWidthUsToDuty(int pulseWidthUs) {
        constexpr uint32_t maxDuty = (1UL << Config::MOTOR_PWM_RESOLUTION_BITS) - 1UL;
        return static_cast<uint32_t>((static_cast<uint64_t>(pulseWidthUs) * Config::MOTOR_PWM_FREQUENCY_HZ * maxDuty + 500000ULL) / 1000000ULL);
    }

    // Links the physical pin to the internal LEDC PWM channel.
    bool AttachMotorPwm(uint8_t pin, uint8_t channel) {
        return ledcAttachChannel(pin, Config::MOTOR_PWM_FREQUENCY_HZ, Config::MOTOR_PWM_RESOLUTION_BITS, channel);
    }

    void WriteMotorDuty(uint8_t pin, uint8_t channel, uint32_t duty) {
        ledcWrite(pin, duty);
    }
}

MotorController::MotorController(FlightState& flightState) : _flightState(flightState) { }

bool MotorController::Init() {
    bool motor1Attached = AttachMotorPwm(Config::MOTOR_1_PIN, MOTOR_1_PWM_CHANNEL);
    bool motor2Attached = AttachMotorPwm(Config::MOTOR_2_PIN, MOTOR_2_PWM_CHANNEL);
    bool motor3Attached = AttachMotorPwm(Config::MOTOR_3_PIN, MOTOR_3_PWM_CHANNEL);
    bool motor4Attached = AttachMotorPwm(Config::MOTOR_4_PIN, MOTOR_4_PWM_CHANNEL);

    _rollPID.SetDerivativeFilterTimeConstant(Config::ROLL_ANGLE_DERIVATIVE_FILTER_TC_S);
    _pitchPID.SetDerivativeFilterTimeConstant(Config::PITCH_ANGLE_DERIVATIVE_FILTER_TC_S);
    _yawPID.SetDerivativeFilterTimeConstant(Config::YAW_RATE_DERIVATIVE_FILTER_TC_S);

    _isInitialized = motor1Attached && motor2Attached && motor3Attached && motor4Attached;
    _flightState.IsArmed = false;
    ResetControllers();
    WriteMinimumOutput();
    _lastUpdateTimeUs = micros();
    return _isInitialized;
}

bool MotorController::Arm(float throttle) {
    if (!_isInitialized || !isfinite(throttle) || throttle < 0.0f || throttle > Config::MOTOR_ARM_MAX_THROTTLE) {
        return false;
    }

    ResetControllers();
    WriteMinimumOutput();

    _lastUpdateTimeUs = micros();
    _flightState.IsArmed = true;

    return true;
}

void MotorController::Disarm() {
    _flightState.IsArmed = false;

    ResetControllers();
    WriteMinimumOutput();
}

void MotorController::EmergencyStop() {
    Disarm();
}

bool MotorController::IsArmed() const { return _flightState.IsArmed; }

void MotorController::ResetControllers() {
    _rollPID.Reset();
    _pitchPID.Reset();
    _yawPID.Reset();

    _mixerSaturatedLastUpdate = false;
}

void MotorController::WriteMinimumOutput() {
    MotorPWMOutput minimumOutput {Config::PWM_MIN_US, Config::PWM_MIN_US, Config::PWM_MIN_US, Config::PWM_MIN_US};
    WriteMotorPWM(minimumOutput);
}

float MotorController::WrapAngleErrorDeg(float targetDeg, float measuredDeg) {
    float errorDeg = fmodf(targetDeg - measuredDeg + 180.0f, 360.0f);
    if (errorDeg < 0.0f) errorDeg += 360.0f;

    return errorDeg - 180.0f;
}

bool MotorController::UpdateMotorOutputs(float throttle, Orientation currentOrientation, Orientation targetOrientation) {
    if (!_flightState.IsArmed) {
        WriteMinimumOutput();
        return false;
    }

    if (!currentOrientation.ReadSuccessful || !targetOrientation.ReadSuccessful ||
        !isfinite(throttle) || !isfinite(currentOrientation.RollDeg) ||
        !isfinite(currentOrientation.PitchDeg) ||
        !isfinite(currentOrientation.YawDeg) ||
        !isfinite(currentOrientation.YawRateDegS) ||
        !isfinite(targetOrientation.RollDeg) ||
        !isfinite(targetOrientation.PitchDeg) ||
        !isfinite(targetOrientation.YawDeg) ||
        !isfinite(targetOrientation.YawRateDegS)
    ) {
        Disarm();
        return false;
    }

    uint32_t currentTimeUs = micros();
    uint32_t elapsedTimeUs = currentTimeUs - _lastUpdateTimeUs;
    _lastUpdateTimeUs = currentTimeUs;
    float elapsedSeconds = elapsedTimeUs / 1000000.0f;
    if (elapsedSeconds <= 0.0f || elapsedSeconds > Config::MOTOR_CONTROLLER_MAX_DT_S) {
        Disarm();
        return false;
    }

    float requestedThrottle = constrain(throttle, 0.0f, 1.0f);
    if (requestedThrottle <= Config::MOTOR_CONTROL_MIN_THROTTLE) {
        ResetControllers();
        WriteMinimumOutput();
        return true;
    }

    bool allowIntegration = !_mixerSaturatedLastUpdate;
    float rollU = _rollPID.Update(targetOrientation.RollDeg, currentOrientation.RollDeg, elapsedSeconds, allowIntegration);
    float pitchU = _pitchPID.Update(targetOrientation.PitchDeg, currentOrientation.PitchDeg, elapsedSeconds, allowIntegration);
    
    float yawErrorDeg = WrapAngleErrorDeg(targetOrientation.YawDeg, currentOrientation.YawDeg);
    float targetYawRateDegS = constrain(targetOrientation.YawRateDegS, -Config::MAX_YAW_RATE_DEG_S, Config::MAX_YAW_RATE_DEG_S);
    
    if (abs(targetYawRateDegS) <= Config::YAW_RATE_COMMAND_DEADBAND_DEG_S) {
        targetYawRateDegS = constrain(yawErrorDeg * Config::YAW_HEADING_KP, -Config::MAX_YAW_RATE_DEG_S, Config::MAX_YAW_RATE_DEG_S);
    }

    float yawU = _yawPID.Update(targetYawRateDegS, currentOrientation.YawRateDegS, elapsedSeconds, allowIntegration);

    float rollPitchCorrections[] = {
        MOTOR_1_ROLL_MIX * rollU + MOTOR_1_PITCH_MIX * pitchU,
        MOTOR_2_ROLL_MIX * rollU + MOTOR_2_PITCH_MIX * pitchU,
        MOTOR_3_ROLL_MIX * rollU + MOTOR_3_PITCH_MIX * pitchU,
        MOTOR_4_ROLL_MIX * rollU + MOTOR_4_PITCH_MIX * pitchU
    };

    float rollPitchScale = 1.0f;
    float rollPitchSpan = GetSpan(rollPitchCorrections, 4);
    if (rollPitchSpan > 1.0f) {
        rollPitchScale = 1.0f / rollPitchSpan;
        for (float& correction : rollPitchCorrections) {
            correction *= rollPitchScale;
        }
    }

    // Preserve roll and pitch authority, then do yaw as much as possible.
    float yawScaleLow = 0.0f;
    float yawScaleHigh = 1.0f;
    float corrections[4];

    for (uint8_t iteration = 0; iteration < 12; iteration++) {
        float yawScale = (yawScaleLow + yawScaleHigh) * 0.5f;

        corrections[0] = rollPitchCorrections[0] + MOTOR_1_YAW_MIX * yawU * yawScale;
        corrections[1] = rollPitchCorrections[1] + MOTOR_2_YAW_MIX * yawU * yawScale;
        corrections[2] = rollPitchCorrections[2] + MOTOR_3_YAW_MIX * yawU * yawScale;
        corrections[3] = rollPitchCorrections[3] + MOTOR_4_YAW_MIX * yawU * yawScale;
        
        if (GetSpan(corrections, 4) <= 1.0f) {
            yawScaleLow = yawScale;
        } else {
            yawScaleHigh = yawScale;
        }
    }

    float yawScale = yawScaleLow;
    corrections[0] = rollPitchCorrections[0] + MOTOR_1_YAW_MIX * yawU * yawScale;
    corrections[1] = rollPitchCorrections[1] + MOTOR_2_YAW_MIX * yawU * yawScale;
    corrections[2] = rollPitchCorrections[2] + MOTOR_3_YAW_MIX * yawU * yawScale;
    corrections[3] = rollPitchCorrections[3] + MOTOR_4_YAW_MIX * yawU * yawScale;

    float minimumThrottle = -GetMinimum(corrections, 4);
    float maximumThrottle = 1.0f - GetMaximum(corrections, 4);
    float adjustedThrottle = constrain(requestedThrottle, minimumThrottle, maximumThrottle);

    float motorPowers[] = {
        constrain(adjustedThrottle + corrections[0], 0.0f, 1.0f),
        constrain(adjustedThrottle + corrections[1], 0.0f, 1.0f),
        constrain(adjustedThrottle + corrections[2], 0.0f, 1.0f),
        constrain(adjustedThrottle + corrections[3], 0.0f, 1.0f)
    };

    _mixerSaturatedLastUpdate = requestedThrottle != throttle || adjustedThrottle != requestedThrottle || rollPitchScale < 1.0f || yawScale < 0.999f;

    _currentMotorOutput = MotorPWMOutput {
        PowerToPwmUs(motorPowers[0]),
        PowerToPwmUs(motorPowers[1]),
        PowerToPwmUs(motorPowers[2]),
        PowerToPwmUs(motorPowers[3])
    };
    WriteMotorPWM(_currentMotorOutput);
    return true;
}

int MotorController::PowerToPwmUs(float power) const {
    int pwmRange = Config::PWM_MAX_US - Config::PWM_MIN_US;
    return Config::PWM_MIN_US + static_cast<int>(constrain(power, 0.0f, 1.0f) * pwmRange + 0.5f);
}

void MotorController::WriteMotorPWM(const MotorPWMOutput& output) {
    _currentMotorOutput = MotorPWMOutput {
        constrain(output.Motor1PwmUs, Config::PWM_MIN_US, Config::PWM_MAX_US),
        constrain(output.Motor2PwmUs, Config::PWM_MIN_US, Config::PWM_MAX_US),
        constrain(output.Motor3PwmUs, Config::PWM_MIN_US, Config::PWM_MAX_US),
        constrain(output.Motor4PwmUs, Config::PWM_MIN_US, Config::PWM_MAX_US)
    };

    WriteMotorDuty(Config::MOTOR_1_PIN, MOTOR_1_PWM_CHANNEL, PulseWidthUsToDuty(_currentMotorOutput.Motor1PwmUs));
    WriteMotorDuty(Config::MOTOR_2_PIN, MOTOR_2_PWM_CHANNEL, PulseWidthUsToDuty(_currentMotorOutput.Motor2PwmUs));
    WriteMotorDuty(Config::MOTOR_3_PIN, MOTOR_3_PWM_CHANNEL, PulseWidthUsToDuty(_currentMotorOutput.Motor3PwmUs));
    WriteMotorDuty(Config::MOTOR_4_PIN, MOTOR_4_PWM_CHANNEL, PulseWidthUsToDuty(_currentMotorOutput.Motor4PwmUs));
}

MotorPWMOutput MotorController::GetCurrentMotorOutput() const {
    return _currentMotorOutput;
}