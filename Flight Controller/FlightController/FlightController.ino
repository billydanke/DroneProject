#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "CommonStructs.h"
#include "AltitudeHandler.h"
#include "CommandHandler.h"
#include "GPSHandler.h"
#include "MotorController.h"
#include "OrientationController.h"

FlightState flightState;
CommandHandler commandHandler;
AltitudeHandler altitudeHandler;
GPSHandler gpsHandler;
OrientationController orientationController;
MotorController motorController(flightState);
bool altitudeInitialized = false;
bool gpsInitialized = false;

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
    gpsInitialized = gpsHandler.Init();
    bool startupCalibrationSuccessful = true;

    if (!motorsInitialized) {
        Serial.println("ERROR: Failed to initialize DShot motor outputs.");
        sleep(2);
    }

    if (orientationInitialized) {
        Serial.println("Keep the drone still while the gyroscope calibrates.");
        if (orientationController.CalibrateGyroscope()) {
            Serial.println("Gyroscope calibration complete.");
        } else {
            startupCalibrationSuccessful = false;
            Serial.println("ERROR: Gyroscope calibration failed.");
            sleep(2);
        }

        if (!orientationController.HasValidCompassCalibration()) {
            Serial.println("Compass calibration required before arming. Send 'calibrate-compass' while disarmed.");
        }
    } else {
        Serial.println("ERROR: Failed to initialize orientation sensors over I2C.");
        sleep(2);
    }

    if (altitudeInitialized) {
        Serial.println("BMP180 barometer initialized.");
        Serial.println("Keep the drone at takeoff altitude while the barometer calibrates.");
        if (altitudeHandler.CalibrateBarometer()) {
            Serial.println("Altitude calibration complete.");
        } else {
            startupCalibrationSuccessful = false;
            Serial.println("ERROR: Barometer calibration failed.");
            sleep(2);
        }
    } else {
        Serial.println("ERROR: Failed to initialize BMP180 barometer over I2C.");
        sleep(2);
    }

    if (gpsInitialized) {
        Serial.println("GPS UART initialized.");
    } else {
        Serial.println("ERROR: Failed to initialize GPS UART.");
        sleep(2);
    }

    if (motorsInitialized && orientationInitialized && altitudeInitialized && startupCalibrationSuccessful) {
        Serial.println("Flight Controller initialized!");
        sleep(2);
    }
}

void loop() {

    commandHandler.Update();
    gpsHandler.Update();

    if (commandHandler.ConsumeCompassCalibrationRequest()) {
        if (flightState.IsArmed) {
            Serial.println("ERROR: Compass calibration request denied while armed.");
        } else {
            motorController.Disarm();
            Serial.println("Rotate the drone through roll, pitch, and yaw while the compass calibrates.");
            if (orientationController.CalibrateCompass()) {
                Serial.println("Compass calibration complete.");
            } else {
                Serial.println("ERROR: Compass calibration failed.");
            }
        }
    }

    if (commandHandler.ConsumeAltitudeCalibrationRequest()) {
        if (flightState.IsArmed) {
            Serial.println("ERROR: Altitude calibration request denied while armed.");
        } else {
            motorController.Disarm();
            Serial.println("Keep the drone at takeoff altitude while the barometer calibrates.");
            if (altitudeHandler.CalibrateBarometer()) {
                Serial.println("Altitude calibration complete.");
            } else {
                Serial.println("ERROR: Altitude calibration failed.");
            }
        }
    }

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();
    if (!orientation.ReadSuccessful) {
        motorController.EmergencyStop();
        Serial.println("ERROR: Failed to read IMU data.");
        sleep(1);
        return;
    }

    BarometerData altitude = altitudeHandler.GetAltitude();
    GPSData gps = gpsHandler.GetGPSData();

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
        Serial.print(" m/s");
    } else {
        Serial.print("unavailable");
    }

    Serial.print("\tGPS: ");
    if (!gpsInitialized || !gps.ReadSuccessful) {
        Serial.print("no data");
    } else if (gps.IsLocationFixed) {
        Serial.print(gps.LatitudeDeg, 6);
        Serial.print(",");
        Serial.print(gps.LongitudeDeg, 6);
        Serial.print("\tGS:");
        Serial.print(gps.GroundSpeedMetersPerSecond);
        Serial.print(" m/s\tSats:");
        Serial.print(gps.SatellitesConnectedCount);
        Serial.print("\tHDOP:");
        Serial.print(gps.Hdop);
    } else {
        Serial.print("no fix\tSats:");
        Serial.print(gps.SatellitesConnectedCount);
    }
    Serial.println();

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

        if (!orientationController.IsCalibrationComplete()) {
            Serial.println("ERROR: Arm request denied; gyroscope calibration required.");
            return;
        }

        if (!orientationController.HasValidCompassCalibration()) {
            Serial.println("ERROR: Arm request denied; compass calibration required.");
            return;
        }

        if (altitudeInitialized && !altitudeHandler.IsCalibrationComplete()) {
            Serial.println("ERROR: Arm request denied; altitude calibration required.");
            return;
        }

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
