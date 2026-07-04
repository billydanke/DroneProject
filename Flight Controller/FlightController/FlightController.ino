#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "CommonStructs.h"
#include "AltitudeHandler.h"
#include "CommandHandler.h"
#include "MotorController.h"
#include "OrientationController.h"

FlightState flightState;
CommandHandler commandHandler;
AltitudeHandler altitudeHandler;
OrientationController orientationController;
MotorController motorController(flightState);
bool altitudeInitialized = false;

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);
    Serial.println("Initializing Flight Controller...");

    Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
    Wire.setClock(Config::I2C_CLOCK_HZ);

    // Initialize sensors, motors, etc.
    bool motorsInitialized = motorController.Init();
    bool orientationInitialized = orientationController.Init();
    altitudeInitialized = altitudeHandler.Init();

    if (!motorsInitialized) {
        Serial.println("ERROR: Failed to initialize DShot motor outputs.");
    }

    if (orientationInitialized) {
        orientationController.StartCalibration();
        Serial.println("Keep the drone still while the gyroscope calibrates.");
    } else {
        Serial.println("ERROR: Failed to initialize orientation sensors over I2C.");
    }

    if (altitudeInitialized) {
        Serial.println("BMP180 barometer initialized.");
    } else {
        Serial.println("ERROR: Failed to initialize BMP180 barometer over I2C.");
    }

    if (motorsInitialized && orientationInitialized) {
        Serial.println("Flight Controller initialized!");
    }
}

void loop() {

    commandHandler.Update();

    static bool orientationCalibrationCompleteReported = false;
    static bool altitudeCalibrationCompleteReported = false;
    static bool compassCalibrationPromptReported = false;
    static bool startupAltitudeCalibrationStarted = false;

    if (commandHandler.ConsumeCompassCalibrationRequest()) {
        if (flightState.IsArmed) {
            Serial.println("ERROR: Compass calibration request denied while armed.");
        } else if (altitudeHandler.IsCalibrating()) {
            Serial.println("ERROR: Compass calibration request denied while altitude is calibrating.");
        } else {
            orientationController.StartCompassCalibration();
            orientationCalibrationCompleteReported = false;
            compassCalibrationPromptReported = false;
            Serial.println("Rotate the drone through roll, pitch, and yaw while the compass calibrates.");
        }
    }

    if (commandHandler.ConsumeAltitudeCalibrationRequest()) {
        if (flightState.IsArmed) {
            Serial.println("ERROR: Altitude calibration request denied while armed.");
        } else if (orientationController.IsCalibrating()) {
            Serial.println("ERROR: Altitude calibration request denied while orientation is calibrating.");
        } else {
            altitudeHandler.StartCalibration();
            if (altitudeHandler.IsCalibrating()) {
                altitudeCalibrationCompleteReported = false;
                startupAltitudeCalibrationStarted = true;
                Serial.println("Keep the drone at takeoff altitude while the barometer calibrates.");
            } else {
                Serial.println("ERROR: Altitude calibration request failed; barometer is not initialized.");
            }
        }
    }

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();
    if (!orientation.ReadSuccessful) {
        motorController.EmergencyStop();
        Serial.println("ERROR: Failed to read IMU data.");
        return;
    }

    if (orientationController.IsCalibrating()) {
        motorController.Disarm();

        static uint16_t lastReportedGyroSampleCount = 0;
        static uint16_t lastReportedCompassSampleCount = 0;

        if (orientationController.IsGyroCalibrating()) {
            uint16_t sampleCount = orientationController.GetGyroCalibrationSampleCount();
            constexpr uint16_t calibrationReportInterval = 10;

            if (sampleCount / calibrationReportInterval != lastReportedGyroSampleCount / calibrationReportInterval) {
                Serial.print("Gyroscope calibration samples: ");
                Serial.print(sampleCount);
                Serial.print("/");
                Serial.println(Config::GYRO_CALIBRATION_SAMPLE_COUNT);
            }

            lastReportedGyroSampleCount = sampleCount;
        } else if (orientationController.IsCompassCalibrating()) {
            if (!compassCalibrationPromptReported) {
                Serial.println("Gyroscope calibration complete.");
                Serial.println("Rotate the drone through roll, pitch, and yaw while the compass calibrates.");
                compassCalibrationPromptReported = true;
            }

            uint16_t sampleCount = orientationController.GetCompassCalibrationSampleCount();
            constexpr uint16_t calibrationReportInterval = 50;

            if (sampleCount / calibrationReportInterval != lastReportedCompassSampleCount / calibrationReportInterval) {
                Serial.print("Compass calibration samples: ");
                Serial.print(sampleCount);
                Serial.print("/");
                Serial.println(Config::MAGNETOMETER_CALIBRATION_SAMPLE_COUNT);
            }

            lastReportedCompassSampleCount = sampleCount;
        }
        return;
    }

    if (orientationController.IsCalibrationComplete() && !orientationCalibrationCompleteReported) {
        Serial.println("Orientation calibration complete.");
        orientationCalibrationCompleteReported = true;
    }

    if (!startupAltitudeCalibrationStarted && altitudeInitialized && !altitudeHandler.IsCalibrationComplete()) {
        altitudeHandler.StartCalibration();
        startupAltitudeCalibrationStarted = true;
        altitudeCalibrationCompleteReported = false;
        Serial.println("Keep the drone at takeoff altitude while the barometer calibrates.");
    }

    BarometerData altitude = altitudeHandler.GetAltitude();

    if (altitudeHandler.IsCalibrating()) {
        motorController.Disarm();

        static uint16_t lastReportedAltitudeSampleCount = 0;
        uint16_t sampleCount = altitudeHandler.GetCalibrationSampleCount();
        constexpr uint16_t calibrationReportInterval = 10;

        if (sampleCount / calibrationReportInterval != lastReportedAltitudeSampleCount / calibrationReportInterval) {
            Serial.print("Altitude calibration samples: ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(Config::BAROMETER_CALIBRATION_SAMPLE_COUNT);
        }

        lastReportedAltitudeSampleCount = sampleCount;
        return;
    }

    if (altitudeHandler.IsCalibrationComplete() && !altitudeCalibrationCompleteReported) {
        Serial.println("Altitude calibration complete.");
        altitudeCalibrationCompleteReported = true;
    }

    Serial.print("Roll: ");
    Serial.print(orientation.RollDeg);
    Serial.print("\tPitch: ");
    Serial.print(orientation.PitchDeg);
    Serial.print("\tApprox Yaw: ");
    Serial.print(orientation.YawDeg);
    Serial.print("\tAltitude: ");
    if (altitude.ReadSuccessful) {
        Serial.print(altitude.AltitudeMeters);
        Serial.print(" m\tVSpeed: ");
        Serial.print(altitude.VerticalSpeedMetersPerSecond);
        Serial.println(" m/s");
    } else {
        Serial.println("unavailable");
    }

    PilotCommand pilotCommand = commandHandler.GetCommand();

    float throttle = constrain(pilotCommand.ThrottlePercent / 100.0f, 0.0f, 1.0f);

    if (pilotCommand.DoEStop) {
        motorController.EmergencyStop();
        return;
    }

    static float targetYawDeg = 0.0f;
    static bool targetYawCaptured = false;

    if (!flightState.IsArmed) {
        targetYawCaptured = false;

        if (!pilotCommand.DoArm) return;

        if (!motorController.Arm(throttle)) {
            Serial.println("ERROR: Motor arm request denied.");
            return;
        }

        targetYawDeg = orientation.YawDeg;
        targetYawCaptured = true;
        Serial.println("Motors armed.");

    } else if (!pilotCommand.DoArm) {
        motorController.Disarm();
        targetYawCaptured = false;
        Serial.println("Motors disarmed.");
        return;
    }

    if (!targetYawCaptured) {
        targetYawDeg = orientation.YawDeg;
        targetYawCaptured = true;
    }

    float targetYawRateDegS = constrain(pilotCommand.YawRateDegS, -Config::MAX_YAW_RATE_DEG_S, Config::MAX_YAW_RATE_DEG_S);
    if (abs(targetYawRateDegS) > Config::YAW_RATE_COMMAND_DEADBAND_DEG_S) {
        targetYawDeg = orientation.YawDeg;
    }

    Orientation targetOrientation {
        constrain(pilotCommand.RollDeg, -Config::MAX_ROLL_ANGLE_DEG, Config::MAX_ROLL_ANGLE_DEG),
        constrain(pilotCommand.PitchDeg, -Config::MAX_PITCH_ANGLE_DEG, Config::MAX_PITCH_ANGLE_DEG),
        targetYawDeg,
        true,
        0.0f,
        0.0f,
        targetYawRateDegS
    };

    if (!motorController.UpdateMotorOutputs(throttle, orientation, targetOrientation)) {
        Serial.println("ERROR: Motor update failed; motors disarmed.");
    }

    MotorOutput currentMotorOutput = motorController.GetCurrentMotorOutput();
    Serial.print("M1:");
    Serial.print(currentMotorOutput.Motor1Power);
    Serial.print("\tM2:");
    Serial.print(currentMotorOutput.Motor2Power);
    Serial.print("\tM3:");
    Serial.print(currentMotorOutput.Motor3Power);
    Serial.print("\tM4:");
    Serial.println(currentMotorOutput.Motor4Power);
    Serial.println();

    // Transmit whatever information is necessary back to user (battery, altitude, etc).

    // Hold loop timing. Serial output at 115200 baud will limit the achievable rate.
    // Serial prints need to be removed before this is "production-ready".
    static uint32_t nextLoopTimeUs = 0;
    constexpr uint32_t loopPeriodUs = 1000000UL / Config::LOOP_RATE_HZ;
    uint32_t now = micros();
    if (now >= nextLoopTimeUs + loopPeriodUs) {
        nextLoopTimeUs = now;
    }
    while (micros() < nextLoopTimeUs) {
        // Hold until next loop is ready.
    }
    nextLoopTimeUs += loopPeriodUs;
}
