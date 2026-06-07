#include <Arduino.h>

#include "Config.h"
#include "CommonStructs.h"
#include "OrientationController.h"

OrientationController orientationController;

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);

    // Initialize sensors, motors, etc.
    orientationController = OrientationController();
}

void loop() {

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();

    // Read any RF commands to get the target orientation.

    // Update PID controllers for roll, pitch, and yaw.

    // Transmit whatever information is necessary back to user (battery, altitude, etc).
}