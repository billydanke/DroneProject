#pragma once

#include <Arduino.h>

#include "CommonStructs.h"
#include "Config.h"

class GPSHandler {
    private:
    enum ParserState {
        WaitingForSync1,
        WaitingForSync2,
        ReadingClass,
        ReadingId,
        ReadingLength1,
        ReadingLength2,
        ReadingPayload,
        SkippingPayload,
        ReadingChecksumA,
        ReadingChecksumB
    };

    static constexpr uint8_t UBX_SYNC_1 = 0xB5;
    static constexpr uint8_t UBX_SYNC_2 = 0x62;
    static constexpr uint8_t UBX_NAV_CLASS = 0x01;
    static constexpr uint8_t UBX_NAV_PVT_ID = 0x07;
    static constexpr uint16_t UBX_NAV_PVT_PAYLOAD_LENGTH = 92;

    HardwareSerial& _serial;
    GPSData _lastData = GPSData();
    bool _isInitialized = false;
    bool _hasPvtData = false;
    uint32_t _lastPvtTimeMs = 0;
    ParserState _parserState = WaitingForSync1;
    uint8_t _messageClass = 0;
    uint8_t _messageId = 0;
    uint16_t _payloadLength = 0;
    uint16_t _payloadIndex = 0;
    uint8_t _payload[UBX_NAV_PVT_PAYLOAD_LENGTH] = { };
    uint8_t _checksumA = 0;
    uint8_t _checksumB = 0;
    uint8_t _receivedChecksumA = 0;
    bool _isCurrentPacketNavPvt = false;

    void ApplyNavPvtData(uint32_t nowMs);
    void ProcessByte(uint8_t byte, uint32_t nowMs);
    void ProcessCurrentPacket(uint32_t nowMs);
    void ResetParser();
    void UpdateChecksum(uint8_t byte);
    void UpdateFixAge(uint32_t nowMs);
    static float DegreesToRadians(float degrees);
    static int32_t ReadInt32(const uint8_t* data, uint8_t offset);
    static uint16_t ReadUInt16(const uint8_t* data, uint8_t offset);

    public:
    GPSHandler(HardwareSerial& serialPort = Serial2);

    bool Init();
    bool IsInitialized() const;
    void Update();
    GPSData GetGPSData();
};
