#pragma once

#include "CommonStructs.h"

class CommandHandler {
    private:
    static constexpr uint8_t BUFFER_SIZE = 96;

    PilotCommand _currentCommand;
    char _buffer[BUFFER_SIZE];
    uint8_t _bufferIndex = 0;
    bool _compassCalibrationRequested = false;
    bool _altitudeCalibrationRequested = false;

    bool ParseCommand(const char* line);

    public:
    CommandHandler();

    void Update();
    bool SubmitLine(const char* line);
    PilotCommand GetCommand() const;
    bool ConsumeCompassCalibrationRequest();
    bool ConsumeAltitudeCalibrationRequest();
};
