#include <Arduino.h>

#include "Config.h"
#include "CommonStructs.h"
#include "OrientationController.h"

OrientationController orientationController;

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);
    Serial.println("Initializing Flight Controller...");

    // Initialize sensors, motors, etc.
    if (orientationController.Init()) {
        orientationController.StartCalibration();
        Serial.println("Flight Controller initialized!");
        Serial.println("Keep the drone still while the gyroscope calibrates.");
    } else {
        Serial.println("ERROR: Failed to initialize IMU over I2C.");
    }
}

void loop() {

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();
    if (!orientation.ReadSuccessful) {
        Serial.println("ERROR: Failed to read IMU data.");
        return;
    }

    if (orientationController.IsCalibrating()) {
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
    Orientation targetOrientation = Orientation(0,0,0, true);

    // Update PID controllers for roll, pitch, and yaw.

    // Transmit whatever information is necessary back to user (battery, altitude, etc).
}
