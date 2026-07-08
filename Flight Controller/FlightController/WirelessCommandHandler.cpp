#include "WirelessCommandHandler.h"

#include <mbedtls/sha1.h>

namespace {
    constexpr char WEBSOCKET_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    constexpr uint8_t OPCODE_CLOSE = 0x8;
    constexpr uint8_t OPCODE_PING = 0x9;
    constexpr uint8_t OPCODE_PONG = 0xA;
    constexpr uint8_t OPCODE_TEXT = 0x1;

    bool HeaderHasValue(const char* headers, const char* headerName, const char* expectedValue) {
        const size_t headerNameLength = strlen(headerName);
        const size_t expectedValueLength = strlen(expectedValue);
        const char* cursor = headers;

        while ((cursor = strstr(cursor, headerName)) != nullptr) {
            cursor += headerNameLength;
            while (*cursor == ' ' || *cursor == '\t') cursor++;

            const char* lineEnd = strstr(cursor, "\r\n");
            if (lineEnd == nullptr) return false;

            for (const char* valueCursor = cursor; valueCursor + expectedValueLength <= lineEnd; valueCursor++) {
                if (strncasecmp(valueCursor, expectedValue, expectedValueLength) == 0) {
                    return true;
                }
            }
        }

        return false;
    }
}

WirelessCommandHandler::WirelessCommandHandler(CommandHandler& commandHandler) :
    _commandHandler(commandHandler),
    _server(Config::WEBSOCKET_SERVER_PORT) { }

bool WirelessCommandHandler::Init() {
    WiFi.mode(WIFI_AP);

    bool accessPointStarted = WiFi.softAP(
        Config::WIFI_ACCESS_POINT_SSID,
        Config::WIFI_ACCESS_POINT_PASSWORD,
        Config::WIFI_ACCESS_POINT_CHANNEL,
        false,
        Config::WIFI_ACCESS_POINT_MAX_CLIENTS
    );

    if (!accessPointStarted) {
        return false;
    }

    _server.begin();
    _server.setNoDelay(true);
    return true;
}

void WirelessCommandHandler::Update() {
    if (_connectionState == ConnectionState::Disconnected) {
        AcceptClient();
        return;
    }

    if (!_client || !_client.connected()) {
        DisconnectClient();
        return;
    }

    uint16_t bytesBudget = Config::WEBSOCKET_MAX_BYTES_PER_UPDATE;
    if (_connectionState == ConnectionState::ReadingHandshake) {
        ProcessHandshake(bytesBudget);
    }

    if (_connectionState == ConnectionState::Connected && bytesBudget > 0) {
        ProcessFrames(bytesBudget);
    }
}

void WirelessCommandHandler::AcceptClient() {
    WiFiClient nextClient = _server.available();
    if (!nextClient) return;

    _client = nextClient;
    _client.setNoDelay(true);
    _connectionState = ConnectionState::ReadingHandshake;
    _handshakeLength = 0;
    _handshakeStartMs = millis();
    ResetFrame();
}

void WirelessCommandHandler::ProcessHandshake(uint16_t& bytesBudget) {
    while (bytesBudget > 0 && _client.available() > 0) {
        char value = static_cast<char>(_client.read());
        bytesBudget--;

        if (_handshakeLength >= Config::WEBSOCKET_HANDSHAKE_BUFFER_SIZE - 1) {
            DisconnectClient();
            return;
        }

        _handshakeBuffer[_handshakeLength++] = value;
        _handshakeBuffer[_handshakeLength] = '\0';

        if (IsHandshakeComplete()) {
            char webSocketKey[64];
            if (!TryGetWebSocketKey(webSocketKey, sizeof(webSocketKey)) || !SendHandshakeResponse(webSocketKey)) {
                DisconnectClient();
                return;
            }

            _connectionState = ConnectionState::Connected;
            ResetFrame();
            SendText("ready");
            return;
        }
    }

    if (millis() - _handshakeStartMs > Config::WEBSOCKET_HANDSHAKE_TIMEOUT_MS) {
        DisconnectClient();
    }
}

void WirelessCommandHandler::ProcessFrames(uint16_t& bytesBudget) {
    while (bytesBudget > 0 && _client.available() > 0 && _connectionState == ConnectionState::Connected) {
        int value = _client.read();
        if (value < 0) return;

        bytesBudget--;
        ProcessFrameByte(static_cast<uint8_t>(value));
    }
}

void WirelessCommandHandler::ProcessFrameByte(uint8_t value) {
    switch (_frameReadState) {
        case FrameReadState::Header:
            _frameHeader[_frameHeaderIndex++] = value;
            if (_frameHeaderIndex < 2) return;

            _opcode = _frameHeader[0] & 0x0F;
            _frameMasked = (_frameHeader[1] & 0x80) != 0;
            _payloadLength = _frameHeader[1] & 0x7F;
            _payloadIndex = 0;
            _maskIndex = 0;
            _extendedLengthIndex = 0;
            _discardFrame = !_frameMasked;

            if ((_frameHeader[0] & 0x70) != 0) {
                _discardFrame = true;
            }

            if (_payloadLength == 126) {
                _payloadLength = 0;
                _frameReadState = FrameReadState::ExtendedLength;
            } else if (_payloadLength == 127) {
                DisconnectClient();
            } else {
                _discardFrame = _discardFrame || _payloadLength > Config::WEBSOCKET_MAX_TEXT_PAYLOAD_SIZE;
                _frameReadState = FrameReadState::Mask;
            }
            return;

        case FrameReadState::ExtendedLength:
            _extendedLengthBytes[_extendedLengthIndex++] = value;
            if (_extendedLengthIndex < 2) return;

            _payloadLength = (static_cast<uint16_t>(_extendedLengthBytes[0]) << 8) | _extendedLengthBytes[1];
            _discardFrame = _discardFrame || _payloadLength > Config::WEBSOCKET_MAX_TEXT_PAYLOAD_SIZE;
            _frameReadState = FrameReadState::Mask;
            return;

        case FrameReadState::Mask:
            _mask[_maskIndex++] = value;
            if (_maskIndex < 4) return;

            if (_payloadLength == 0) {
                CompleteFrame();
            } else {
                _frameReadState = FrameReadState::Payload;
            }
            return;

        case FrameReadState::Payload: {
            uint8_t unmasked = value ^ _mask[_payloadIndex % 4];

            if (!_discardFrame && _payloadIndex < Config::WEBSOCKET_MAX_TEXT_PAYLOAD_SIZE) {
                _payload[_payloadIndex] = static_cast<char>(unmasked);
            }

            _payloadIndex++;
            if (_payloadIndex >= _payloadLength) {
                CompleteFrame();
            }
            return;
        }
    }
}

void WirelessCommandHandler::CompleteFrame() {
    if (_opcode == OPCODE_CLOSE) {
        SendFrame(OPCODE_CLOSE, nullptr, 0);
        DisconnectClient();
        return;
    }

    if (_opcode == OPCODE_PING) {
        uint16_t pongLength = _discardFrame ? 0 : _payloadLength;
        SendFrame(OPCODE_PONG, reinterpret_cast<const uint8_t*>(_payload), pongLength);
        ResetFrame();
        return;
    }

    if (!_discardFrame && _opcode == OPCODE_TEXT) {
        _payload[_payloadLength] = '\0';
        if (_commandHandler.SubmitLine(_payload)) {
            SendText("ok");
        } else {
            SendText("err");
        }
    }

    ResetFrame();
}

void WirelessCommandHandler::ResetFrame() {
    _frameReadState = FrameReadState::Header;
    _frameHeaderIndex = 0;
    _extendedLengthIndex = 0;
    _maskIndex = 0;
    _opcode = 0;
    _payloadLength = 0;
    _payloadIndex = 0;
    _frameMasked = false;
    _discardFrame = false;
}

void WirelessCommandHandler::DisconnectClient() {
    if (_client) {
        _client.stop();
    }

    _connectionState = ConnectionState::Disconnected;
    _handshakeLength = 0;
    ResetFrame();
}

bool WirelessCommandHandler::IsHandshakeComplete() const {
    return strstr(_handshakeBuffer, "\r\n\r\n") != nullptr;
}

bool WirelessCommandHandler::TryGetWebSocketKey(char* key, size_t keySize) const {
    if (strncmp(_handshakeBuffer, "GET /commands ", 14) != 0) return false;
    if (!HeaderHasValue(_handshakeBuffer, "Upgrade:", "websocket")) return false;
    if (!HeaderHasValue(_handshakeBuffer, "Connection:", "Upgrade")) return false;

    const char* keyStart = strstr(_handshakeBuffer, "Sec-WebSocket-Key:");
    if (keyStart == nullptr) return false;

    keyStart += strlen("Sec-WebSocket-Key:");
    while (*keyStart == ' ' || *keyStart == '\t') keyStart++;

    const char* keyEnd = strstr(keyStart, "\r\n");
    if (keyEnd == nullptr) return false;

    size_t length = keyEnd - keyStart;
    if (length == 0 || length >= keySize) return false;

    memcpy(key, keyStart, length);
    key[length] = '\0';
    return true;
}

bool WirelessCommandHandler::SendHandshakeResponse(const char* webSocketKey) {
    char acceptSource[96];
    int sourceLength = snprintf(acceptSource, sizeof(acceptSource), "%s%s", webSocketKey, WEBSOCKET_GUID);
    if (sourceLength <= 0 || sourceLength >= static_cast<int>(sizeof(acceptSource))) return false;

    uint8_t sha1Hash[20];
    if (mbedtls_sha1(reinterpret_cast<const unsigned char*>(acceptSource), sourceLength, sha1Hash) != 0) {
        return false;
    }

    char acceptKey[32];
    Base64Encode(sha1Hash, sizeof(sha1Hash), acceptKey, sizeof(acceptKey));

    _client.print("HTTP/1.1 101 Switching Protocols\r\n");
    _client.print("Upgrade: websocket\r\n");
    _client.print("Connection: Upgrade\r\n");
    _client.print("Sec-WebSocket-Accept: ");
    _client.print(acceptKey);
    _client.print("\r\n\r\n");
    return true;
}

void WirelessCommandHandler::SendText(const char* text) {
    SendFrame(OPCODE_TEXT, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

void WirelessCommandHandler::SendFrame(uint8_t opcode, const uint8_t* payload, uint16_t payloadLength) {
    if (!_client || !_client.connected()) return;

    uint8_t header[4];
    uint8_t headerLength = 0;
    header[headerLength++] = 0x80 | (opcode & 0x0F);

    if (payloadLength <= 125) {
        header[headerLength++] = static_cast<uint8_t>(payloadLength);
    } else {
        header[headerLength++] = 126;
        header[headerLength++] = static_cast<uint8_t>((payloadLength >> 8) & 0xFF);
        header[headerLength++] = static_cast<uint8_t>(payloadLength & 0xFF);
    }

    _client.write(header, headerLength);
    if (payloadLength > 0 && payload != nullptr) {
        _client.write(payload, payloadLength);
    }
}

void WirelessCommandHandler::Base64Encode(const uint8_t* input, size_t inputLength, char* output, size_t outputSize) {
    static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t outputIndex = 0;

    for (size_t inputIndex = 0; inputIndex < inputLength; inputIndex += 3) {
        uint32_t value = static_cast<uint32_t>(input[inputIndex]) << 16;
        bool hasSecondByte = inputIndex + 1 < inputLength;
        bool hasThirdByte = inputIndex + 2 < inputLength;

        if (hasSecondByte) value |= static_cast<uint32_t>(input[inputIndex + 1]) << 8;
        if (hasThirdByte) value |= input[inputIndex + 2];

        if (outputIndex + 4 >= outputSize) break;

        output[outputIndex++] = alphabet[(value >> 18) & 0x3F];
        output[outputIndex++] = alphabet[(value >> 12) & 0x3F];
        output[outputIndex++] = hasSecondByte ? alphabet[(value >> 6) & 0x3F] : '=';
        output[outputIndex++] = hasThirdByte ? alphabet[value & 0x3F] : '=';
    }

    if (outputSize > 0) {
        output[min(outputIndex, outputSize - 1)] = '\0';
    }
}
