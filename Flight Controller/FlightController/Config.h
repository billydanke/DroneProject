#pragma once

#include <Arduino.h>

namespace Config {
    
    constexpr unsigned long SERIAL_BAUD = 115200;

    // ESC PWM Pins
    constexpr int MOTOR_1_PIN = 1; // TODO: Update this when we have wiring set up
    constexpr int MOTOR_2_PIN = 2; // TODO: Update this when we have wiring set up
    constexpr int MOTOR_3_PIN = 3; // TODO: Update this when we have wiring set up
    constexpr int MOTOR_4_PIN = 4; // TODO: Update this when we have wiring set up

    constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 400;
    constexpr int PWM_MIN_US = 1000;
    constexpr int PWM_MAX_US = 2000;

    // Angle Limits
    constexpr float MAX_ROLL_ANGLE_DEG = 25.0f;
    constexpr float MAX_PITCH_ANGLE_DEG = 25.0f;
    constexpr float MAX_YAW_RATE_DEG_S = 180.0f;

    // MPU Orientation
    constexpr int MPU_ADDRESS = 0x68;
    constexpr float ORIENTATION_FILTER_TIME_CONSTANT_S = 0.5f;

    // Gyroscope calibration
    constexpr uint16_t GYRO_CALIBRATION_SAMPLE_COUNT = 100;
    constexpr float GYRO_CALIBRATION_MAX_RATE_DEG_S = 20.0f;
    constexpr float GYRO_CALIBRATION_MAX_RATE_DEVIATION_DEG_S = 2.5f;
    constexpr float GYRO_CALIBRATION_ACCEL_TOLERANCE_G = 0.25f;

    // PID Gain Settings
    // We'll probably have PIDs for roll and pitch to hold the drone's attitude.
    // There will likely be more here but for now I'll add parameters for just the angles (we need to tune these).
    constexpr float ROLL_ANGLE_KP = 0.0f;
    constexpr float ROLL_ANGLE_KI = 0.0f;
    constexpr float ROLL_ANGLE_KD = 0.0f;

    constexpr float PITCH_ANGLE_KP = 0.0f;
    constexpr float PITCH_ANGLE_KI = 0.0f;
    constexpr float PITCH_ANGLE_KD = 0.0f;

    constexpr float YAW_ANGLE_KP = 0.0f;
    constexpr float YAW_ANGLE_KI = 0.0f;
    constexpr float YAW_ANGLE_KD = 0.0f;
}
