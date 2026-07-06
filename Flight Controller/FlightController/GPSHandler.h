#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "CommonStructs.h"
#include "Config.h"

class GPSHandler {
    private:
    HardwareSerial& _serial;
    TinyGPSPlus _gps;
    GPSData _lastData = GPSData();
    bool _isInitialized = false;
    bool _hasDecodedSentence = false;
    uint32_t _lastDecodedSentenceTimeMs = 0;

    void ApplyParsedData(bool hasNewSentence, uint32_t nowMs);
    static float DegreesToRadians(float degrees);

    public:
    GPSHandler(HardwareSerial& serialPort = Serial2);

    bool Init();
    bool IsInitialized() const;
    void Update();
    GPSData GetGPSData();
};
