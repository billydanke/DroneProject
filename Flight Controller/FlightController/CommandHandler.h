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
    bool _gyroscopeCalibrationRequested = false;
    bool _enableDebugSerialRequested = false;
    bool _disableDebugSerialRequested = false;
    bool _armRequested = false;
    bool _disarmRequested = false;
    bool _emergencyStopRequested = false;
    bool _emergencyStopReleaseRequested = false;
    uint8_t _motorTestRequested = 0;

    bool ParseCommand(const char* line);

    public:
    CommandHandler();

    void Update();
    bool SubmitLine(const char* line);
    PilotCommand GetCommand() const;
    bool ConsumeCompassCalibrationRequest();
    bool ConsumeAltitudeCalibrationRequest();
    bool ConsumeGyroscopeCalibrationRequest();
    bool ConsumeEnableDebugSerialRequest();
    bool ConsumeDisableDebugSerialRequest();
    bool ConsumeArmRequest();
    bool ConsumeDisarmRequest();
    bool ConsumeEmergencyStopRequest();
    bool ConsumeEmergencyStopReleaseRequest();
    uint8_t ConsumeMotorTestRequest();
};
