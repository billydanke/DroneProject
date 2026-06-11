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
        Serial.println("Flight Controller initialized!");
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

    Serial.print("Roll: ");
    Serial.print(orientation.RollDeg);
    Serial.print("\tPitch: ");
    Serial.print(orientation.PitchDeg);
    Serial.print("\tApprox Yaw: ");
    Serial.println(orientation.YawDeg);

    // Read any RF commands to get the target orientation.

    // Update PID controllers for roll, pitch, and yaw.

    // Transmit whatever information is necessary back to user (battery, altitude, etc).
}