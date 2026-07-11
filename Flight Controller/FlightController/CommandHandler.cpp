#include <Arduino.h>
#include "CommandHandler.h"

CommandHandler::CommandHandler() { }

void CommandHandler::Update() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (_bufferIndex > 0) {
                _buffer[_bufferIndex] = '\0';
                SubmitLine(_buffer);
                _bufferIndex = 0;
            }
            continue;
        }

        if (_bufferIndex < BUFFER_SIZE - 1) {
            _buffer[_bufferIndex++] = c;
        } else {
            // Line too long — discard and reset.
            _bufferIndex = 0;
        }
    }
}

bool CommandHandler::SubmitLine(const char* line) {
    return ParseCommand(line);
}

bool CommandHandler::ParseCommand(const char* line) {
    if (strcmp(line, "calibrate-compass") == 0) {
        _compassCalibrationRequested = true;
        return true;
    }

    if (strcmp(line, "calibrate-altitude") == 0) {
        _altitudeCalibrationRequested = true;
        return true;
    }

    if (strcmp(line, "calibrate-gyro") == 0) {
        _gyroscopeCalibrationRequested = true;
        return true;
    }

    if (strcmp(line, "enable-debug-serial") == 0) {
        _enableDebugSerialRequested = true;
        return true;
    }

    if (strcmp(line, "disable-debug-serial") == 0) {
        _disableDebugSerialRequested = true;
        return true;
    }

    if (strcmp(line, "arm") == 0) {
        _armRequested = true;
        return true;
    }

    if (strcmp(line, "disarm") == 0) {
        _disarmRequested = true;
        return true;
    }

    if (strcmp(line, "trigger-estop") == 0) {
        _emergencyStopRequested = true;
        return true;
    }

    if (strcmp(line, "release-estop") == 0) {
        _emergencyStopReleaseRequested = true;
        return true;
    }

    float throttle, roll, pitch, yawRate;
    char trailing;

    int parsed = sscanf(line, " %f , %f , %f , %f %c", &throttle, &roll, &pitch, &yawRate, &trailing);

    if (parsed != 4) return false;

    if (!isfinite(throttle) || !isfinite(roll) || !isfinite(pitch) || !isfinite(yawRate)) return false;
    if (throttle < 0.0f || throttle > 100.0f) return false;

    _currentCommand.ThrottlePercent = throttle;
    _currentCommand.RollDeg = roll;
    _currentCommand.PitchDeg = pitch;
    _currentCommand.YawRateDegS = yawRate;
    return true;
}

PilotCommand CommandHandler::GetCommand() const {
    return _currentCommand;
}

bool CommandHandler::ConsumeCompassCalibrationRequest() {
    bool requested = _compassCalibrationRequested;
    _compassCalibrationRequested = false;
    return requested;
}

bool CommandHandler::ConsumeAltitudeCalibrationRequest() {
    bool requested = _altitudeCalibrationRequested;
    _altitudeCalibrationRequested = false;
    return requested;
}

bool CommandHandler::ConsumeGyroscopeCalibrationRequest() {
    bool requested = _gyroscopeCalibrationRequested;
    _gyroscopeCalibrationRequested = false;
    return requested;
}

bool CommandHandler::ConsumeEnableDebugSerialRequest() {
    bool requested = _enableDebugSerialRequested;
    _enableDebugSerialRequested = false;
    return requested;
}

bool CommandHandler::ConsumeDisableDebugSerialRequest() {
    bool requested = _disableDebugSerialRequested;
    _disableDebugSerialRequested = false;
    return requested;
}

bool CommandHandler::ConsumeArmRequest() {
    bool requested = _armRequested;
    _armRequested = false;
    return requested;
}

bool CommandHandler::ConsumeDisarmRequest() {
    bool requested = _disarmRequested;
    _disarmRequested = false;
    return requested;
}

bool CommandHandler::ConsumeEmergencyStopRequest() {
    bool requested = _emergencyStopRequested;
    _emergencyStopRequested = false;
    return requested;
}

bool CommandHandler::ConsumeEmergencyStopReleaseRequest() {
    bool requested = _emergencyStopReleaseRequested;
    _emergencyStopReleaseRequested = false;
    return requested;
}
