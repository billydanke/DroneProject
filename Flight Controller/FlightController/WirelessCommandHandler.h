#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "CommandHandler.h"
#include "Config.h"

class WirelessCommandHandler {
    private:
    enum class ConnectionState : uint8_t {
        Disconnected,
        ReadingHandshake,
        Connected
    };

    enum class FrameReadState : uint8_t {
        Header,
        ExtendedLength,
        Mask,
        Payload
    };

    CommandHandler& _commandHandler;
    WiFiServer _server;
    WiFiClient _client;

    ConnectionState _connectionState = ConnectionState::Disconnected;
    FrameReadState _frameReadState = FrameReadState::Header;

    char _handshakeBuffer[Config::WEBSOCKET_HANDSHAKE_BUFFER_SIZE];
    uint16_t _handshakeLength = 0;
    uint32_t _handshakeStartMs = 0;

    uint8_t _frameHeader[2] = { 0, 0 };
    uint8_t _frameHeaderIndex = 0;
    uint8_t _extendedLengthBytes[2] = { 0, 0 };
    uint8_t _extendedLengthIndex = 0;
    uint8_t _mask[4] = { 0, 0, 0, 0 };
    uint8_t _maskIndex = 0;
    uint8_t _opcode = 0;
    uint16_t _payloadLength = 0;
    uint16_t _payloadIndex = 0;
    bool _frameMasked = false;
    bool _discardFrame = false;
    char _payload[Config::WEBSOCKET_MAX_TEXT_PAYLOAD_SIZE + 1];
    uint32_t _lastDiagnosticSendMs = 0;

    void AcceptClient();
    void ProcessHandshake(uint16_t& bytesBudget);
    void ProcessFrames(uint16_t& bytesBudget);
    void ProcessFrameByte(uint8_t value);
    void CompleteFrame();
    void ResetFrame();
    void DisconnectClient();
    bool IsHandshakeComplete() const;
    bool TryGetWebSocketKey(char* key, size_t keySize) const;
    bool SendHandshakeResponse(const char* webSocketKey);
    bool SendText(const char* text);
    bool SendFrame(uint8_t opcode, const uint8_t* payload, uint16_t payloadLength);
    void SendPendingDiagnostic();
    static void Base64Encode(const uint8_t* input, size_t inputLength, char* output, size_t outputSize);

    public:
    WirelessCommandHandler(CommandHandler& commandHandler);

    bool Init();
    void Update();
};
