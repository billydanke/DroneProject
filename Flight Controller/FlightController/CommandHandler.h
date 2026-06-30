#pragma once

#include "CommonStructs.h"

class CommandHandler {
    private:
    static constexpr uint8_t BUFFER_SIZE = 96;

    PilotCommand _currentCommand;
    char _buffer[BUFFER_SIZE];
    uint8_t _bufferIndex = 0;

    void ParseCommand(const char* line);

    public:
    CommandHandler();

    void Update();
    PilotCommand GetCommand() const;
};
