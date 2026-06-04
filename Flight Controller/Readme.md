# Flight Controller

This is the Arduino code (targeting ESP32) for the drone flight controller. It is responsible for controlling nearly all aspects of the drone, including the following tasks:

- Knowing its orientation via magnetometer and accelerometer/gyroscope.
- Knowing its GPS location.
- Knowing its altitude via barometer.
- Transmit its Remote ID.
- Receive commands and transmit metrics, messages, info, and logs over digital RF.
- Transmit video over analog RF.
- Controlling PWM power signals to the ESC(s).
- Balancing itself.
- Blending motor commands smoothly.
- Monitoring battery voltage.
- Handling connection dropout or GPS loss safely.