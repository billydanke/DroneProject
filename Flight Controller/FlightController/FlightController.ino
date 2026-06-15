#include <Arduino.h>

#include "Config.h"
#include "CommonStructs.h"
#include "MotorController.h"
#include "OrientationController.h"

OrientationController orientationController;
MotorController motorController;

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);
    Serial.println("Initializing Flight Controller...");

    // Initialize sensors, motors, etc.
    bool motorsInitialized = motorController.Init();
    bool orientationInitialized = orientationController.Init();

    if (!motorsInitialized) {
        Serial.println("ERROR: Failed to initialize motor PWM outputs.");
    }

    if (orientationInitialized) {
        orientationController.StartCalibration();
        Serial.println("Keep the drone still while the gyroscope calibrates.");
    } else {
        Serial.println("ERROR: Failed to initialize IMU over I2C.");
    }

    if (motorsInitialized && orientationInitialized) {
        Serial.println("Flight Controller initialized!");
    }
}

void loop() {

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();
    if (!orientation.ReadSuccessful) {
        motorController.EmergencyStop();
        Serial.println("ERROR: Failed to read IMU data.");
        return;
    }

    if (orientationController.IsCalibrating()) {
        motorController.Disarm();

        static uint16_t lastReportedSampleCount = 0;
        uint16_t sampleCount = orientationController.GetCalibrationSampleCount();
        constexpr uint16_t calibrationReportInterval = 10;

        if (sampleCount / calibrationReportInterval != lastReportedSampleCount / calibrationReportInterval) {
            Serial.print("Gyroscope calibration samples: ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(Config::GYRO_CALIBRATION_SAMPLE_COUNT);
        }

        lastReportedSampleCount = sampleCount;
        return;
    }

    static bool calibrationCompleteReported = false;
    if (orientationController.IsCalibrationComplete() && !calibrationCompleteReported) {
        Serial.println("Gyroscope calibration complete.");
        calibrationCompleteReported = true;
    }

    Serial.print("Roll: ");
    Serial.print(orientation.RollDeg);
    Serial.print("\tPitch: ");
    Serial.print(orientation.PitchDeg);
    Serial.print("\tApprox Yaw: ");
    Serial.println(orientation.YawDeg);

    // Read any RF commands to get the target orientation.
    // For now I will just force this to be a balanced target orientation.
    // Replace this with the latest command over RF.
    PilotCommand pilotCommand;
    float throttle = constrain(pilotCommand.throttlePercent / 100.0f, 0.0f, 1.0f);

    if (pilotCommand.DoEStop) {
        motorController.EmergencyStop();
        return;
    }

    static float targetYawDeg = 0.0f;
    static bool targetYawCaptured = false;

    if (!motorController.IsArmed()) {
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

    // Transmit whatever information is necessary back to user (battery, altitude, etc).
}
