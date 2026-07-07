#include "GPSHandler.h"

GPSHandler::GPSHandler(HardwareSerial& serialPort) : _serial(serialPort) { }

bool GPSHandler::Init() {
    _lastData = GPSData();
    _hasPvtData = false;
    _lastPvtTimeMs = 0;
    ResetParser();

    _serial.end();
    delay(50);
    _serial.begin(Config::GPS_BAUD, SERIAL_8N1, Config::GPS_RX_PIN, Config::GPS_TX_PIN);
    _isInitialized = true;

    return _isInitialized;
}

bool GPSHandler::IsInitialized() const {
    return _isInitialized;
}

void GPSHandler::Update() {
    if (!_isInitialized) return;

    uint32_t nowMs = millis();
    uint16_t bytesProcessed = 0;

    while (_serial.available() > 0 && bytesProcessed < Config::GPS_MAX_BYTES_PER_UPDATE) {
        ProcessByte(static_cast<uint8_t>(_serial.read()), nowMs);
        bytesProcessed++;
    }

    UpdateFixAge(nowMs);
}

GPSData GPSHandler::GetGPSData() {
    GPSData data = _lastData;
    _lastData.HasNewData = false;
    return data;
}

void GPSHandler::ProcessByte(uint8_t byte, uint32_t nowMs) {
    switch (_parserState) {
        case WaitingForSync1:
            if (byte == UBX_SYNC_1) {
                _parserState = WaitingForSync2;
            }
            break;

        case WaitingForSync2:
            if (byte == UBX_SYNC_2) {
                _checksumA = 0;
                _checksumB = 0;
                _payloadIndex = 0;
                _payloadLength = 0;
                _isCurrentPacketNavPvt = false;
                _parserState = ReadingClass;
            } else {
                _parserState = byte == UBX_SYNC_1 ? WaitingForSync2 : WaitingForSync1;
            }
            break;

        case ReadingClass:
            _messageClass = byte;
            UpdateChecksum(byte);
            _parserState = ReadingId;
            break;

        case ReadingId:
            _messageId = byte;
            UpdateChecksum(byte);
            _parserState = ReadingLength1;
            break;

        case ReadingLength1:
            _payloadLength = byte;
            UpdateChecksum(byte);
            _parserState = ReadingLength2;
            break;

        case ReadingLength2:
            _payloadLength |= static_cast<uint16_t>(byte) << 8;
            UpdateChecksum(byte);
            _payloadIndex = 0;
            _isCurrentPacketNavPvt =
                _messageClass == UBX_NAV_CLASS &&
                _messageId == UBX_NAV_PVT_ID &&
                _payloadLength == UBX_NAV_PVT_PAYLOAD_LENGTH;

            if (_payloadLength == 0) {
                _parserState = ReadingChecksumA;
            } else {
                _parserState = _isCurrentPacketNavPvt ? ReadingPayload : SkippingPayload;
            }
            break;

        case ReadingPayload:
            _payload[_payloadIndex++] = byte;
            UpdateChecksum(byte);
            if (_payloadIndex >= _payloadLength) {
                _parserState = ReadingChecksumA;
            }
            break;

        case SkippingPayload:
            _payloadIndex++;
            UpdateChecksum(byte);
            if (_payloadIndex >= _payloadLength) {
                _parserState = ReadingChecksumA;
            }
            break;

        case ReadingChecksumA:
            _receivedChecksumA = byte;
            _parserState = ReadingChecksumB;
            break;

        case ReadingChecksumB:
            if (_receivedChecksumA == _checksumA && byte == _checksumB) {
                ProcessCurrentPacket(nowMs);
            }
            ResetParser();
            break;
    }
}

void GPSHandler::ProcessCurrentPacket(uint32_t nowMs) {
    if (!_isCurrentPacketNavPvt) return;

    ApplyNavPvtData(nowMs);
}

void GPSHandler::ApplyNavPvtData(uint32_t nowMs) {
    uint8_t fixType = _payload[20];
    uint8_t flags = _payload[21];
    bool hasValidFix = (flags & 0x01) != 0 && fixType >= 3;

    int32_t longitude = ReadInt32(_payload, 24);
    int32_t latitude = ReadInt32(_payload, 28);
    int32_t altitudeMslMm = ReadInt32(_payload, 36);
    int32_t groundSpeedMmS = ReadInt32(_payload, 60);
    int32_t headingMotion = ReadInt32(_payload, 64);
    uint16_t positionDop = ReadUInt16(_payload, 76);

    GPSData data = _lastData;
    data.LatitudeDeg = latitude / 10000000.0;
    data.LongitudeDeg = longitude / 10000000.0;
    data.AltitudeMeters = altitudeMslMm / 1000.0f;
    data.GroundSpeedMetersPerSecond = groundSpeedMmS / 1000.0f;
    data.CourseDeg = headingMotion / 100000.0f;
    data.CourseRadians = DegreesToRadians(data.CourseDeg);
    data.Hdop = positionDop / 100.0f;
    data.SatellitesConnectedCount = static_cast<int>(_payload[23]);
    data.FixAgeMs = 0;
    data.IsLocationFixed = hasValidFix;
    data.HasNewData = true;
    data.ReadSuccessful = true;

    _lastData = data;
    _hasPvtData = true;
    _lastPvtTimeMs = nowMs;
}

void GPSHandler::ResetParser() {
    _parserState = WaitingForSync1;
    _messageClass = 0;
    _messageId = 0;
    _payloadLength = 0;
    _payloadIndex = 0;
    _checksumA = 0;
    _checksumB = 0;
    _receivedChecksumA = 0;
    _isCurrentPacketNavPvt = false;
}

void GPSHandler::UpdateChecksum(uint8_t byte) {
    _checksumA += byte;
    _checksumB += _checksumA;
}

void GPSHandler::UpdateFixAge(uint32_t nowMs) {
    GPSData data = _lastData;
    data.FixAgeMs = _hasPvtData ? nowMs - _lastPvtTimeMs : 0;
    data.IsLocationFixed = _hasPvtData && data.IsLocationFixed && data.FixAgeMs <= Config::GPS_FIX_TIMEOUT_MS;
    data.ReadSuccessful = _hasPvtData;
    _lastData = data;
}

float GPSHandler::DegreesToRadians(float degrees) {
    return degrees * PI / 180.0f;
}

int32_t GPSHandler::ReadInt32(const uint8_t* data, uint8_t offset) {
    uint32_t value =
        static_cast<uint32_t>(data[offset]) |
        (static_cast<uint32_t>(data[offset + 1]) << 8) |
        (static_cast<uint32_t>(data[offset + 2]) << 16) |
        (static_cast<uint32_t>(data[offset + 3]) << 24);

    return static_cast<int32_t>(value);
}

uint16_t GPSHandler::ReadUInt16(const uint8_t* data, uint8_t offset) {
    return
        static_cast<uint16_t>(data[offset]) |
        (static_cast<uint16_t>(data[offset + 1]) << 8);
}
