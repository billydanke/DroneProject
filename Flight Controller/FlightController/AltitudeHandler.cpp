#include "AltitudeHandler.h"
#include <Arduino.h>
#include <Wire.h>

namespace {
    constexpr uint8_t BMP180_CHIP_ID_REGISTER = 0xD0;
    constexpr uint8_t BMP180_EXPECTED_CHIP_ID = 0x55;
    constexpr uint8_t BMP180_CALIBRATION_START_REGISTER = 0xAA;
    constexpr uint8_t BMP180_CONTROL_REGISTER = 0xF4;
    constexpr uint8_t BMP180_DATA_REGISTER = 0xF6;
    constexpr uint8_t BMP180_TEMPERATURE_COMMAND = 0x2E;
    constexpr uint8_t BMP180_PRESSURE_COMMAND = 0x34;

    constexpr uint32_t TEMPERATURE_CONVERSION_DELAY_US = 5000;
    constexpr uint32_t PRESSURE_CONVERSION_DELAY_US[] = { 5000, 8000, 14000, 26000 };
}

AltitudeHandler::AltitudeHandler() { }

bool AltitudeHandler::Init() {
    _isInitialized = false;
    _measurementState = MeasurementIdle;
    _lastData = BarometerData();
    _isCalibrating = false;
    _isCalibrationComplete = false;
    _calibrationSampleCount = 0;
    _calibrationPressureSumPa = 0.0f;
    _groundPressurePa = Config::STANDARD_SEA_LEVEL_PRESSURE_PA;
    _hasFilteredAltitude = false;

    uint8_t chipId = 0;
    Wire.beginTransmission(Config::BMP180_ADDRESS);
    Wire.write(BMP180_CHIP_ID_REGISTER);
    if (Wire.endTransmission(false) != 0) return false;

    size_t bytesReceived = Wire.requestFrom(Config::BMP180_ADDRESS, 1, true);
    if (bytesReceived != 1 || Wire.available() < 1) return false;
    chipId = Wire.read();
    if (chipId != BMP180_EXPECTED_CHIP_ID) return false;

    if (!ReadCalibrationCoefficients()) return false;

    _lastUpdateTimeUs = micros();
    _lastSuccessfulMeasurementTimeUs = 0;
    _isInitialized = true;
    return true;
}

void AltitudeHandler::StartCalibration() {
    if (!_isInitialized) return;

    _isCalibrating = true;
    _isCalibrationComplete = false;
    _calibrationSampleCount = 0;
    _calibrationPressureSumPa = 0.0f;
    _groundPressurePa = Config::STANDARD_SEA_LEVEL_PRESSURE_PA;
    _hasFilteredAltitude = false;
}

bool AltitudeHandler::CalibrateBarometer() {
    StartCalibration();
    if (!_isCalibrating) return false;

    uint32_t calibrationStartMs = millis();
    uint16_t lastReportedSampleCount = 0;

    while (IsCalibrating()) {
        GetAltitude();

        uint16_t sampleCount = GetCalibrationSampleCount();
        constexpr uint16_t calibrationReportInterval = 10;
        if (sampleCount / calibrationReportInterval != lastReportedSampleCount / calibrationReportInterval) {
            Serial.print("Altitude calibration samples: ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(Config::BAROMETER_CALIBRATION_SAMPLE_COUNT);
        }
        lastReportedSampleCount = sampleCount;

        if (millis() - calibrationStartMs > Config::BAROMETER_CALIBRATION_TIMEOUT_MS) {
            _isCalibrating = false;
            _isCalibrationComplete = false;
            return false;
        }

        delayMicroseconds(1000000UL / Config::LOOP_RATE_HZ);
    }

    _lastUpdateTimeUs = micros();
    _lastSuccessfulMeasurementTimeUs = _lastUpdateTimeUs;
    return _isCalibrationComplete;
}

bool AltitudeHandler::IsCalibrating() const {
    return _isCalibrating;
}

bool AltitudeHandler::IsCalibrationComplete() const {
    return _isCalibrationComplete;
}

uint16_t AltitudeHandler::GetCalibrationSampleCount() const {
    return _calibrationSampleCount;
}

BarometerData AltitudeHandler::GetAltitude() {
    if (!_isInitialized) {
        BarometerData failedData = _lastData;
        failedData.ReadSuccessful = false;
        return failedData;
    }

    uint32_t now = micros();

    if (_measurementState == MeasurementIdle) {
        constexpr uint32_t updatePeriodUs = 1000000UL / Config::BAROMETER_UPDATE_RATE_HZ;
        if (now - _lastUpdateTimeUs < updatePeriodUs) {
            return _lastData;
        }

        if (!WriteControlCommand(BMP180_TEMPERATURE_COMMAND)) {
            MarkReadFailed();
            return _lastData;
        }

        _conversionStartTimeUs = now;
        _measurementState = WaitingForTemperature;
        return _lastData;
    }

    if (_measurementState == WaitingForTemperature) {
        if (now - _conversionStartTimeUs < TEMPERATURE_CONVERSION_DELAY_US) {
            return _lastData;
        }

        if (!ReadRawTemperature(_lastRawTemperature)) {
            MarkReadFailed();
            return _lastData;
        }

        uint8_t command = BMP180_PRESSURE_COMMAND | (Config::BMP180_OVERSAMPLING_SETTING << 6);
        if (!WriteControlCommand(command)) {
            MarkReadFailed();
            return _lastData;
        }

        _conversionStartTimeUs = now;
        _measurementState = WaitingForPressure;
        return _lastData;
    }

    if (now - _conversionStartTimeUs < GetPressureConversionDelayUs()) {
        return _lastData;
    }

    int32_t rawPressure = 0;
    if (!ReadRawPressure(rawPressure)) {
        MarkReadFailed();
        return _lastData;
    }

    float temperatureC = 0.0f;
    float pressurePa = 0.0f;
    if (!CompleteMeasurement(_lastRawTemperature, rawPressure, temperatureC, pressurePa)) {
        MarkReadFailed();
        return _lastData;
    }

    ApplyMeasurement(temperatureC, pressurePa, now);
    _measurementState = MeasurementIdle;
    _lastUpdateTimeUs = now;

    return _lastData;
}

bool AltitudeHandler::ReadCalibrationCoefficients() {
    return
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER, _ac1) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 2, _ac2) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 4, _ac3) &&
        ReadUInt16Register(BMP180_CALIBRATION_START_REGISTER + 6, _ac4) &&
        ReadUInt16Register(BMP180_CALIBRATION_START_REGISTER + 8, _ac5) &&
        ReadUInt16Register(BMP180_CALIBRATION_START_REGISTER + 10, _ac6) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 12, _b1) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 14, _b2) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 16, _mb) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 18, _mc) &&
        ReadInt16Register(BMP180_CALIBRATION_START_REGISTER + 20, _md);
}

bool AltitudeHandler::ReadInt16Register(uint8_t registerAddress, int16_t& value) {
    Wire.beginTransmission(Config::BMP180_ADDRESS);
    Wire.write(registerAddress);
    if (Wire.endTransmission(false) != 0) return false;

    size_t bytesReceived = Wire.requestFrom(Config::BMP180_ADDRESS, 2, true);
    if (bytesReceived != 2 || Wire.available() < 2) return false;

    value = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    return true;
}

bool AltitudeHandler::ReadUInt16Register(uint8_t registerAddress, uint16_t& value) {
    Wire.beginTransmission(Config::BMP180_ADDRESS);
    Wire.write(registerAddress);
    if (Wire.endTransmission(false) != 0) return false;

    size_t bytesReceived = Wire.requestFrom(Config::BMP180_ADDRESS, 2, true);
    if (bytesReceived != 2 || Wire.available() < 2) return false;

    value = static_cast<uint16_t>((Wire.read() << 8) | Wire.read());
    return true;
}

bool AltitudeHandler::WriteControlCommand(uint8_t command) {
    Wire.beginTransmission(Config::BMP180_ADDRESS);
    Wire.write(BMP180_CONTROL_REGISTER);
    Wire.write(command);
    return Wire.endTransmission(true) == 0;
}

bool AltitudeHandler::ReadRawTemperature(int32_t& rawTemperature) {
    uint16_t value = 0;
    if (!ReadUInt16Register(BMP180_DATA_REGISTER, value)) return false;

    rawTemperature = value;
    return true;
}

bool AltitudeHandler::ReadRawPressure(int32_t& rawPressure) {
    Wire.beginTransmission(Config::BMP180_ADDRESS);
    Wire.write(BMP180_DATA_REGISTER);
    if (Wire.endTransmission(false) != 0) return false;

    size_t bytesReceived = Wire.requestFrom(Config::BMP180_ADDRESS, 3, true);
    if (bytesReceived != 3 || Wire.available() < 3) return false;

    int32_t msb = Wire.read();
    int32_t lsb = Wire.read();
    int32_t xlsb = Wire.read();
    rawPressure = ((msb << 16) | (lsb << 8) | xlsb) >> (8 - Config::BMP180_OVERSAMPLING_SETTING);
    return true;
}

bool AltitudeHandler::CompleteMeasurement(int32_t rawTemperature, int32_t rawPressure, float& temperatureC, float& pressurePa) const {
    if (rawTemperature <= 0 || rawPressure <= 0) return false;

    int32_t x1 = ((rawTemperature - static_cast<int32_t>(_ac6)) * static_cast<int32_t>(_ac5)) >> 15;
    int32_t denominator = x1 + _md;
    if (denominator == 0) return false;

    int32_t x2 = (static_cast<int32_t>(_mc) << 11) / denominator;
    int32_t b5 = x1 + x2;
    temperatureC = static_cast<float>((b5 + 8) >> 4) / 10.0f;

    int32_t b6 = b5 - 4000;
    x1 = (static_cast<int32_t>(_b2) * ((b6 * b6) >> 12)) >> 11;
    x2 = (static_cast<int32_t>(_ac2) * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((static_cast<int32_t>(_ac1) * 4) + x3) << Config::BMP180_OVERSAMPLING_SETTING) + 2) >> 2;

    x1 = (static_cast<int32_t>(_ac3) * b6) >> 13;
    x2 = (static_cast<int32_t>(_b1) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (static_cast<uint32_t>(_ac4) * static_cast<uint32_t>(x3 + 32768)) >> 15;
    if (b4 == 0) return false;

    uint32_t b7 = (static_cast<uint32_t>(rawPressure - b3)) * (50000UL >> Config::BMP180_OVERSAMPLING_SETTING);
    int32_t pressure = 0;
    if (b7 < 0x80000000UL) {
        pressure = (b7 * 2) / b4;
    } else {
        pressure = (b7 / b4) * 2;
    }

    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;

    if (pressure <= 0) return false;

    pressurePa = static_cast<float>(pressure);
    return isfinite(temperatureC) && isfinite(pressurePa);
}

uint32_t AltitudeHandler::GetPressureConversionDelayUs() const {
    uint8_t oversampling = Config::BMP180_OVERSAMPLING_SETTING;
    if (oversampling > 3) oversampling = 3;
    return PRESSURE_CONVERSION_DELAY_US[oversampling];
}

void AltitudeHandler::MarkReadFailed() {
    _measurementState = MeasurementIdle;
    _lastUpdateTimeUs = micros();
    _lastData.ReadSuccessful = false;
}

void AltitudeHandler::ApplyMeasurement(float temperatureC, float pressurePa, uint32_t measurementTimeUs) {
    if (!isfinite(temperatureC) || !isfinite(pressurePa) || pressurePa <= 0.0f) {
        MarkReadFailed();
        return;
    }

    if (_isCalibrating) {
        UpdateCalibration(pressurePa);
    }

    float referencePressurePa = _isCalibrationComplete ? _groundPressurePa : Config::STANDARD_SEA_LEVEL_PRESSURE_PA;
    float rawAltitudeMeters = PressureToAltitudeMeters(pressurePa, referencePressurePa);
    float verticalSpeedMetersPerSecond = 0.0f;

    if (!_hasFilteredAltitude) {
        _filteredAltitudeMeters = rawAltitudeMeters;
        _hasFilteredAltitude = true;
    } else {
        float elapsedSeconds = 0.0f;
        if (_lastSuccessfulMeasurementTimeUs != 0) {
            elapsedSeconds = (measurementTimeUs - _lastSuccessfulMeasurementTimeUs) / 1000000.0f;
        }

        float previousAltitudeMeters = _filteredAltitudeMeters;
        if (elapsedSeconds > 0.0f) {
            float filterAlpha = Config::ALTITUDE_FILTER_TIME_CONSTANT_S / (Config::ALTITUDE_FILTER_TIME_CONSTANT_S + elapsedSeconds);
            _filteredAltitudeMeters = filterAlpha * _filteredAltitudeMeters + (1.0f - filterAlpha) * rawAltitudeMeters;
            verticalSpeedMetersPerSecond = (_filteredAltitudeMeters - previousAltitudeMeters) / elapsedSeconds;
        } else {
            _filteredAltitudeMeters = rawAltitudeMeters;
        }
    }

    _lastSuccessfulMeasurementTimeUs = measurementTimeUs;
    _lastData.PressurePa = pressurePa;
    _lastData.PressureReadingKPa = pressurePa / 1000.0f;
    _lastData.TemperatureC = temperatureC;
    _lastData.AltitudeMeters = _filteredAltitudeMeters;
    _lastData.VerticalSpeedMetersPerSecond = verticalSpeedMetersPerSecond;
    _lastData.ReadSuccessful = true;
}

void AltitudeHandler::UpdateCalibration(float pressurePa) {
    if (!isfinite(pressurePa) || pressurePa <= 0.0f) return;

    _calibrationPressureSumPa += pressurePa;
    _calibrationSampleCount++;

    if (_calibrationSampleCount < Config::BAROMETER_CALIBRATION_SAMPLE_COUNT) {
        return;
    }

    _groundPressurePa = _calibrationPressureSumPa / static_cast<float>(_calibrationSampleCount);
    _isCalibrating = false;
    _isCalibrationComplete = true;
    _hasFilteredAltitude = true;
    _filteredAltitudeMeters = 0.0f;
}

float AltitudeHandler::PressureToAltitudeMeters(float pressurePa, float referencePressurePa) {
    if (!isfinite(pressurePa) || !isfinite(referencePressurePa) || pressurePa <= 0.0f || referencePressurePa <= 0.0f) {
        return 0.0f;
    }

    return 44330.0f * (1.0f - pow(pressurePa / referencePressurePa, 0.19029495f));
}
