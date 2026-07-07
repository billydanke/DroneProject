#pragma once

#include "CommonStructs.h"
#include "Config.h"

class AltitudeHandler {
    private:

    enum MeasurementState {
        MeasurementIdle,
        WaitingForTemperature,
        WaitingForPressure
    };

    bool _isInitialized = false;
    MeasurementState _measurementState = MeasurementIdle;
    uint32_t _lastUpdateTimeUs = 0;
    uint32_t _conversionStartTimeUs = 0;
    uint32_t _lastSuccessfulMeasurementTimeUs = 0;

    int16_t _ac1 = 0;
    int16_t _ac2 = 0;
    int16_t _ac3 = 0;
    uint16_t _ac4 = 0;
    uint16_t _ac5 = 0;
    uint16_t _ac6 = 0;
    int16_t _b1 = 0;
    int16_t _b2 = 0;
    int16_t _mb = 0;
    int16_t _mc = 0;
    int16_t _md = 0;

    int32_t _lastRawTemperature = 0;
    BarometerData _lastData = BarometerData();

    bool _isCalibrating = false;
    bool _isCalibrationComplete = false;
    uint16_t _calibrationSampleCount = 0;
    float _calibrationPressureSumPa = 0.0f;
    float _groundPressurePa = Config::STANDARD_SEA_LEVEL_PRESSURE_PA;

    bool _hasFilteredAltitude = false;
    float _filteredAltitudeMeters = 0.0f;

    bool ReadCalibrationCoefficients();
    bool ReadInt16Register(uint8_t registerAddress, int16_t& value);
    bool ReadUInt16Register(uint8_t registerAddress, uint16_t& value);
    bool WriteControlCommand(uint8_t command);
    bool ReadRawTemperature(int32_t& rawTemperature);
    bool ReadRawPressure(int32_t& rawPressure);
    bool CompleteMeasurement(int32_t rawTemperature, int32_t rawPressure, float& temperatureC, float& pressurePa) const;
    uint32_t GetPressureConversionDelayUs() const;
    void MarkReadFailed();
    void ApplyMeasurement(float temperatureC, float pressurePa, uint32_t measurementTimeUs);
    void UpdateCalibration(float pressurePa);
    static float PressureToAltitudeMeters(float pressurePa, float referencePressurePa);

    public:
    
    AltitudeHandler();

    bool Init();
    void StartCalibration();
    bool CalibrateBarometer();
    bool IsCalibrating() const;
    bool IsCalibrationComplete() const;
    uint16_t GetCalibrationSampleCount() const;

    BarometerData GetAltitude();
};
