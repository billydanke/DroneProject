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

    float throttle, roll, pitch, yawRate;
    int arm, estop;
    char trailing;

    int parsed = sscanf(line, " %f , %f , %f , %f , %d , %d %c", &throttle, &roll, &pitch, &yawRate, &arm, &estop, &trailing);

    if (parsed != 6) return false;

    if (!isfinite(throttle) || !isfinite(roll) || !isfinite(pitch) || !isfinite(yawRate)) return false;
    if (throttle < 0.0f || throttle > 100.0f) return false;
    if ((arm != 0 && arm != 1) || (estop != 0 && estop != 1)) return false;

    _currentCommand.ThrottlePercent = throttle;
    _currentCommand.RollDeg = roll;
    _currentCommand.PitchDeg = pitch;
    _currentCommand.YawRateDegS = yawRate;
    _currentCommand.DoArm = arm != 0;
    _currentCommand.DoEStop = estop != 0;

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
