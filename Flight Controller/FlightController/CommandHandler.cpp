#include <Arduino.h>
#include "CommandHandler.h"

CommandHandler::CommandHandler() { }

void CommandHandler::Update() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (_bufferIndex > 0) {
                _buffer[_bufferIndex] = '\0';
                ParseCommand(_buffer);
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

void CommandHandler::ParseCommand(const char* line) {
    float throttle, roll, pitch, yawRate;
    int arm, estop;

    int parsed = sscanf(line, "%f,%f,%f,%f,%d,%d",
        &throttle, &roll, &pitch, &yawRate, &arm, &estop);

    if (parsed != 6) return;

    _currentCommand.ThrottlePercent = throttle;
    _currentCommand.RollDeg        = roll;
    _currentCommand.PitchDeg       = pitch;
    _currentCommand.YawRateDegS    = yawRate;
    _currentCommand.DoArm          = arm != 0;
    _currentCommand.DoEStop        = estop != 0;
}

PilotCommand CommandHandler::GetCommand() const {
    return _currentCommand;
}
