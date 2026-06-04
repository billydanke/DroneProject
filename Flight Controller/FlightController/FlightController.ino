#include <Arduino.h>

#include "Config.h"
#include "CommonStructs.h"

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);
}

void loop() {

}