#include "GPSHandler.h"

GPSHandler::GPSHandler(HardwareSerial& serialPort) : _serial(serialPort) { }

bool GPSHandler::Init() {
    _lastData = GPSData();
    _hasDecodedSentence = false;
    _lastDecodedSentenceTimeMs = 0;

    _serial.begin(Config::GPS_BAUD, SERIAL_8N1, Config::GPS_RX_PIN, Config::GPS_TX_PIN);
    _isInitialized = true;

    return _isInitialized;
}

bool GPSHandler::IsInitialized() const {
    return _isInitialized;
}

void GPSHandler::Update() {
    if (!_isInitialized) return;

    uint16_t bytesProcessed = 0;
    bool hasNewSentence = false;

    while (_serial.available() > 0 && bytesProcessed < Config::GPS_MAX_BYTES_PER_UPDATE) {

        char c = _serial.read();

        Serial.write(c);

        if (_gps.encode(c)) {
            hasNewSentence = true;
        }
        bytesProcessed++;
    }

    ApplyParsedData(hasNewSentence, millis());
}

GPSData GPSHandler::GetGPSData() {
    GPSData data = _lastData;
    _lastData.HasNewData = false;
    return data;
}

void GPSHandler::ApplyParsedData(bool hasNewSentence, uint32_t nowMs) {
    if (hasNewSentence) {
        _hasDecodedSentence = true;
        _lastDecodedSentenceTimeMs = nowMs;
    }

    GPSData data = _lastData;
    data.HasNewData = data.HasNewData || hasNewSentence;
    data.ReadSuccessful = _hasDecodedSentence;

    if (_gps.satellites.isValid()) {
        data.SatellitesConnectedCount = static_cast<int>(_gps.satellites.value());
    }

    if (_gps.hdop.isValid()) {
        data.Hdop = static_cast<float>(_gps.hdop.hdop());
    }

    if (_gps.altitude.isValid()) {
        data.AltitudeMeters = static_cast<float>(_gps.altitude.meters());
    }

    if (_gps.speed.isValid()) {
        data.GroundSpeedMetersPerSecond = static_cast<float>(_gps.speed.mps());
    }

    if (_gps.course.isValid()) {
        data.CourseDeg = static_cast<float>(_gps.course.deg());
        data.CourseRadians = DegreesToRadians(data.CourseDeg);
    }

    if (_gps.location.isValid()) {
        data.LatitudeDeg = _gps.location.lat();
        data.LongitudeDeg = _gps.location.lng();

        uint32_t locationAgeMs = _gps.location.age();
        data.FixAgeMs = locationAgeMs;
        data.IsLocationFixed = locationAgeMs <= Config::GPS_FIX_TIMEOUT_MS;
    } else {
        data.FixAgeMs = _hasDecodedSentence ? nowMs - _lastDecodedSentenceTimeMs : 0;
        data.IsLocationFixed = false;
    }

    _lastData = data;
}

float GPSHandler::DegreesToRadians(float degrees) {
    return degrees * PI / 180.0f;
}
