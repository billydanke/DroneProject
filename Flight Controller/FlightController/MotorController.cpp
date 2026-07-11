#include <Arduino.h>
#include "MotorController.h"

namespace {
    // Front-left, CW
    constexpr float MOTOR_1_ROLL_MIX = 1.0f;
    constexpr float MOTOR_1_PITCH_MIX = -1.0f;
    constexpr float MOTOR_1_YAW_MIX = -1.0f;

    // Front-right, CCW
    constexpr float MOTOR_2_ROLL_MIX = -1.0f;
    constexpr float MOTOR_2_PITCH_MIX = -1.0f;
    constexpr float MOTOR_2_YAW_MIX = 1.0f;

    // Back-right, CW
    constexpr float MOTOR_3_ROLL_MIX = -1.0f;
    constexpr float MOTOR_3_PITCH_MIX = 1.0f;
    constexpr float MOTOR_3_YAW_MIX = -1.0f;

    // Back-left, CCW
    constexpr float MOTOR_4_ROLL_MIX = 1.0f;
    constexpr float MOTOR_4_PITCH_MIX = 1.0f;
    constexpr float MOTOR_4_YAW_MIX = 1.0f;

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
}

MotorController::MotorController(FlightState& flightState) : _flightState(flightState),
      _motor1(Config::MOTOR_1_PIN, DSHOT300, false),
      _motor2(Config::MOTOR_2_PIN, DSHOT300, false),
      _motor3(Config::MOTOR_3_PIN, DSHOT300, false),
      _motor4(Config::MOTOR_4_PIN, DSHOT300, false) { }

bool MotorController::Init() {
    dshot_result_t motor1Result = _motor1.begin();
    dshot_result_t motor2Result = _motor2.begin();
    dshot_result_t motor3Result = _motor3.begin();
    dshot_result_t motor4Result = _motor4.begin();

    bool motor1Ready = motor1Result.success;
    bool motor2Ready = motor2Result.success;
    bool motor3Ready = motor3Result.success;
    bool motor4Ready = motor4Result.success;

    _rollPID.SetDerivativeFilterTimeConstant(Config::ROLL_ANGLE_DERIVATIVE_FILTER_TC_S);
    _pitchPID.SetDerivativeFilterTimeConstant(Config::PITCH_ANGLE_DERIVATIVE_FILTER_TC_S);
    _yawPID.SetDerivativeFilterTimeConstant(Config::YAW_RATE_DERIVATIVE_FILTER_TC_S);

    _isInitialized = motor1Ready && motor2Ready && motor3Ready && motor4Ready;
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

bool MotorController::TestMotor(uint8_t motorNumber) {
    if (!_isInitialized || !_flightState.IsArmed || motorNumber < 1 || motorNumber > 4) {
        return false;
    }

    ResetControllers();
    WriteMinimumOutput();

    MotorOutput testOutput { };
    switch (motorNumber) {
        case 1: testOutput.Motor1Power = Config::MOTOR_TEST_THROTTLE; break;
        case 2: testOutput.Motor2Power = Config::MOTOR_TEST_THROTTLE; break;
        case 3: testOutput.Motor3Power = Config::MOTOR_TEST_THROTTLE; break;
        case 4: testOutput.Motor4Power = Config::MOTOR_TEST_THROTTLE; break;
        default: return false;
    }

    uint32_t testStartMs = millis();
    constexpr uint32_t outputPeriodUs = 1000000UL / Config::LOOP_RATE_HZ;
    while (millis() - testStartMs < Config::MOTOR_TEST_DURATION_MS) {
        WriteDShotOutput(testOutput);
        delayMicroseconds(outputPeriodUs);
    }

    Disarm();
    return true;
}

void MotorController::ResetControllers() {
    _rollPID.Reset();
    _pitchPID.Reset();
    _yawPID.Reset();

    _mixerSaturatedLastUpdate = false;
}

void MotorController::WriteMinimumOutput() {
    WriteDShotOutput(MotorOutput { });
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

    WriteDShotOutput(MotorOutput { motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3] });
    return true;
}

uint16_t MotorController::PowerToDShot(float power) const {
    if (power <= 0.0f) return 0;

    float scaled = Config::DSHOT_THROTTLE_MIN + constrain(power, 0.0f, 1.0f) * (Config::DSHOT_THROTTLE_MAX - Config::DSHOT_THROTTLE_MIN);
    return static_cast<uint16_t>(constrain(scaled + 0.5f, (float)Config::DSHOT_THROTTLE_MIN, (float)Config::DSHOT_THROTTLE_MAX));
}

void MotorController::WriteDShotOutput(const MotorOutput& output) {
    _currentMotorOutput = output;
    _motor1.sendThrottle(PowerToDShot(output.Motor1Power));
    _motor2.sendThrottle(PowerToDShot(output.Motor2Power));
    _motor3.sendThrottle(PowerToDShot(output.Motor3Power));
    _motor4.sendThrottle(PowerToDShot(output.Motor4Power));
}

MotorOutput MotorController::GetCurrentMotorOutput() const {
    return _currentMotorOutput;
}