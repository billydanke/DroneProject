#pragma once

#include <Arduino.h>

namespace Config {
    
    constexpr unsigned long SERIAL_BAUD = 115200;
    constexpr uint32_t LOOP_RATE_HZ = 400;

    // Shared I2C Bus
    constexpr int I2C_SDA_PIN = 21;
    constexpr int I2C_SCL_PIN = 22;
    constexpr uint32_t I2C_CLOCK_HZ = 100000;

    // ESC PWM Pins
    constexpr int MOTOR_1_PIN = 27;
    constexpr int MOTOR_2_PIN = 26;
    constexpr int MOTOR_3_PIN = 25;
    constexpr int MOTOR_4_PIN = 33;

    constexpr uint16_t DSHOT_THROTTLE_MIN = 48;   // Values 1-47 are special ESC commands.
    constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;
    constexpr float MOTOR_CONTROLLER_MAX_DT_S = 0.05f;
    constexpr float MOTOR_ARM_MAX_THROTTLE = 0.05f;
    constexpr float MOTOR_CONTROL_MIN_THROTTLE = 0.05f;

    // Angle Limits
    constexpr float MAX_ROLL_ANGLE_DEG = 25.0f;
    constexpr float MAX_PITCH_ANGLE_DEG = 25.0f;
    constexpr float MAX_YAW_RATE_DEG_S = 180.0f;
    constexpr float YAW_RATE_COMMAND_DEADBAND_DEG_S = 1.0f;
    constexpr float YAW_HEADING_KP = 1.0f;

    // MPU Configuration
    constexpr int MPU_ADDRESS = 0x68;
    // DLPF_CFG=2: 94Hz gyro BW, 1kHz output rate. Reduces noise well below Nyquist at LOOP_RATE_HZ.
    constexpr uint8_t MPU_DLPF_CFG = 0x02;
    constexpr float ORIENTATION_FILTER_TIME_CONSTANT_S = 0.5f;

    // QMC5883L Magnetometer Configuration
    constexpr int QMC5883L_ADDRESS = 0x0D;
    constexpr uint8_t QMC5883L_OVERSAMPLING_512 = 0x00;
    constexpr uint8_t QMC5883L_RANGE_2G = 0x00;
    constexpr uint8_t QMC5883L_OUTPUT_RATE_100_HZ = 0x0C;
    constexpr uint8_t QMC5883L_CONTINUOUS_MODE = 0x01;
    constexpr float MAGNETOMETER_YAW_FILTER_TIME_CONSTANT_S = 2.0f;
    constexpr uint16_t MAGNETOMETER_CALIBRATION_SAMPLE_COUNT = 500;
    constexpr uint32_t MAGNETOMETER_CALIBRATION_TIMEOUT_MS = 60000;
    constexpr float MAGNETOMETER_MIN_VALID_MAGNITUDE = 50.0f;
    constexpr float MAGNETOMETER_MAX_VALID_MAGNITUDE = 30000.0f;
    constexpr float MAGNETOMETER_MIN_CALIBRATION_RANGE = 500.0f;

    // BMP180 Barometer Configuration
    constexpr int BMP180_ADDRESS = 0x77;
    constexpr uint8_t BMP180_OVERSAMPLING_SETTING = 3;
    constexpr uint16_t BAROMETER_UPDATE_RATE_HZ = 25;
    constexpr uint16_t BAROMETER_CALIBRATION_SAMPLE_COUNT = 100;
    constexpr uint32_t BAROMETER_CALIBRATION_TIMEOUT_MS = 15000;
    constexpr float STANDARD_SEA_LEVEL_PRESSURE_PA = 101325.0f;
    constexpr float ALTITUDE_FILTER_TIME_CONSTANT_S = 1.0f;

    // GPS Configuration
    constexpr unsigned long GPS_BAUD = 9600;
    constexpr int GPS_RX_PIN = 16;
    constexpr int GPS_TX_PIN = 17;
    constexpr uint32_t GPS_FIX_TIMEOUT_MS = 2000;
    constexpr uint16_t GPS_MAX_BYTES_PER_UPDATE = 128;

    // Gyroscope calibration
    constexpr uint16_t GYRO_CALIBRATION_SAMPLE_COUNT = 100;
    constexpr uint32_t GYRO_CALIBRATION_TIMEOUT_MS = 10000;
    constexpr float GYRO_CALIBRATION_MAX_RATE_DEG_S = 20.0f;
    constexpr float GYRO_CALIBRATION_MAX_RATE_DEVIATION_DEG_S = 2.5f;
    constexpr float GYRO_CALIBRATION_ACCEL_TOLERANCE_G = 0.25f;

    // PID Derivative Filter Time Constants
    constexpr float ROLL_ANGLE_DERIVATIVE_FILTER_TC_S = 0.005f;
    constexpr float PITCH_ANGLE_DERIVATIVE_FILTER_TC_S = 0.005f;
    constexpr float YAW_RATE_DERIVATIVE_FILTER_TC_S = 0.005f;

    // PID Gain Settings
    // We'll probably have PIDs for roll and pitch to hold the drone's attitude.
    // There will likely be more here but for now I'll add parameters for just the angles (we need to tune these).
    constexpr float ROLL_ANGLE_KP = 0.008f;
    constexpr float ROLL_ANGLE_KI = 0.0f;
    constexpr float ROLL_ANGLE_KD = 0.0015f;

    constexpr float PITCH_ANGLE_KP = 0.008f;
    constexpr float PITCH_ANGLE_KI = 0.0f;
    constexpr float PITCH_ANGLE_KD = 0.0015f;

    constexpr float YAW_RATE_KP = 0.002f;
    constexpr float YAW_RATE_KI = 0.0f;
    constexpr float YAW_RATE_KD = 0.0f;
}
