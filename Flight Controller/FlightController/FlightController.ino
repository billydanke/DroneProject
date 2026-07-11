#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "CommonStructs.h"
#include "AltitudeHandler.h"
#include "CommandHandler.h"
#include "DiagnosticLogger.h"
#include "GPSHandler.h"
#include "MotorController.h"
#include "OrientationController.h"
#include "WirelessCommandHandler.h"

FlightState flightState;
CommandHandler commandHandler;
WirelessCommandHandler wirelessCommandHandler(commandHandler);
AltitudeHandler altitudeHandler;
GPSHandler gpsHandler;
OrientationController orientationController;
MotorController motorController(flightState);
bool altitudeInitialized = false;
bool gpsInitialized = false;
bool emergencyStopActive = false;

void ServiceWirelessDiagnostics() {
    wirelessCommandHandler.Update();
}

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);
    DiagnosticLogger::Log("status:flight_controller_initializing");

    Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
    Wire.setClock(Config::I2C_CLOCK_HZ);

    // Initialize sensors, motors, etc.
    bool motorsInitialized = motorController.Init();
    bool wirelessInitialized = wirelessCommandHandler.Init();
    bool orientationInitialized = orientationController.Init();
    altitudeInitialized = altitudeHandler.Init();
    gpsInitialized = gpsHandler.Init();
    bool startupCalibrationSuccessful = true;

    sleep(5);

    if (!motorsInitialized) {
        DiagnosticLogger::Log("error:motor_output_initialization_failed");
        sleep(2);
    }

    if (wirelessInitialized) {
        DiagnosticLogger::Logf("status:wireless_ready:ws://%s/commands", WiFi.softAPIP().toString().c_str());
    } else {
        DiagnosticLogger::Log("error:wireless_initialization_failed");
        sleep(2);
    }

    if (orientationInitialized) {
        DiagnosticLogger::Log("status:gyro_calibration_started:keep_still");
        if (orientationController.CalibrateGyroscope(ServiceWirelessDiagnostics)) {
            DiagnosticLogger::Log("status:gyro_calibration_complete");
        } else {
            startupCalibrationSuccessful = false;
            DiagnosticLogger::Log("error:gyro_calibration_failed");
            sleep(2);
        }

        if (!orientationController.HasValidCompassCalibration()) {
            DiagnosticLogger::Log("warning:compass_calibration_required:send_calibrate-compass_while_disarmed");
        }
    } else {
        DiagnosticLogger::Log("error:orientation_sensor_initialization_failed");
        sleep(2);
    }

    if (altitudeInitialized) {
        DiagnosticLogger::Log("status:barometer_initialized");
        DiagnosticLogger::Log("status:altitude_calibration_started:hold_takeoff_altitude");
        if (altitudeHandler.CalibrateBarometer(ServiceWirelessDiagnostics)) {
            DiagnosticLogger::Log("status:altitude_calibration_complete");
        } else {
            startupCalibrationSuccessful = false;
            DiagnosticLogger::Log("error:altitude_calibration_failed");
            sleep(2);
        }
    } else {
        DiagnosticLogger::Log("error:barometer_initialization_failed");
        sleep(2);
    }

    if (gpsInitialized) {
        DiagnosticLogger::Log("status:gps_initialized");
    } else {
        DiagnosticLogger::Log("error:gps_initialization_failed");
        sleep(2);
    }

    if (motorsInitialized && orientationInitialized && altitudeInitialized && startupCalibrationSuccessful) {
        DiagnosticLogger::Log("status:flight_controller_initialized");
        sleep(2);
    }
}

void loop() {

    commandHandler.Update();
    wirelessCommandHandler.Update();
    gpsHandler.Update();

    if (commandHandler.ConsumeEnableDebugSerialRequest()) {
        DiagnosticLogger::SetSerialEnabled(true);
        DiagnosticLogger::Log("status:debug_serial_enabled");
    }

    if (commandHandler.ConsumeDisableDebugSerialRequest()) {
        DiagnosticLogger::Log("status:debug_serial_disabled");
        DiagnosticLogger::SetSerialEnabled(false);
    }

    bool emergencyStopRequested = commandHandler.ConsumeEmergencyStopRequest();
    bool emergencyStopReleaseRequested = commandHandler.ConsumeEmergencyStopReleaseRequest();
    bool disarmRequested = commandHandler.ConsumeDisarmRequest();
    bool armRequested = commandHandler.ConsumeArmRequest();

    if (emergencyStopRequested) {
        emergencyStopActive = true;
        motorController.EmergencyStop();
        DiagnosticLogger::Log("status:estop_triggered");
        return;
    }

    if (emergencyStopReleaseRequested) {
        emergencyStopActive = false;
        motorController.Disarm();
        DiagnosticLogger::Log("status:estop_released:motors_disarmed");
        return;
    }

    if (emergencyStopActive) {
        motorController.EmergencyStop();
        return;
    }

    if (disarmRequested) {
        motorController.Disarm();
        DiagnosticLogger::Log("status:motors_disarmed");
        return;
    }

    if (commandHandler.ConsumeGyroscopeCalibrationRequest()) {
        if (flightState.IsArmed) {
            DiagnosticLogger::Log("error:gyro_calibration_denied:armed");
            sleep(2);
        } else {
            motorController.Disarm();
            DiagnosticLogger::Log("status:gyro_calibration_started:keep_still");
            if (orientationController.CalibrateGyroscope(ServiceWirelessDiagnostics)) {
                DiagnosticLogger::Log("status:gyro_calibration_complete");
                sleep(2);
            } else {
                DiagnosticLogger::Log("error:gyro_calibration_failed");
                sleep(2);
            }
        }
    }

    if (commandHandler.ConsumeCompassCalibrationRequest()) {
        if (flightState.IsArmed) {
            DiagnosticLogger::Log("error:compass_calibration_denied:armed");
            sleep(2);
        } else {
            motorController.Disarm();
            DiagnosticLogger::Log("status:compass_calibration_started:rotate_roll_pitch_yaw");
            if (orientationController.CalibrateCompass(ServiceWirelessDiagnostics)) {
                DiagnosticLogger::Log("status:compass_calibration_complete");
                sleep(2);
            } else {
                DiagnosticLogger::Log("error:compass_calibration_failed");
                sleep(2);
            }
        }
    }

    if (commandHandler.ConsumeAltitudeCalibrationRequest()) {
        if (flightState.IsArmed) {
            DiagnosticLogger::Log("error:altitude_calibration_denied:armed");
            sleep(2);
        } else {
            motorController.Disarm();
            DiagnosticLogger::Log("status:altitude_calibration_started:hold_takeoff_altitude");
            if (altitudeHandler.CalibrateBarometer(ServiceWirelessDiagnostics)) {
                DiagnosticLogger::Log("status:altitude_calibration_complete");
                sleep(2);
            } else {
                DiagnosticLogger::Log("error:altitude_calibration_failed");
                sleep(2);
            }
        }
    }

    // Update sensor readings and determine current orientation.
    Orientation orientation = orientationController.GetOrientation();
    if (!orientation.ReadSuccessful) {
        motorController.EmergencyStop();
        DiagnosticLogger::Log("error:imu_read_failed");
        sleep(1);
        return;
    }

    BarometerData altitude = altitudeHandler.GetAltitude();
    GPSData gps = gpsHandler.GetGPSData();

    /*Serial.print("Roll: ");
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
    Serial.println();*/

    PilotCommand pilotCommand = commandHandler.GetCommand();

    float throttle = constrain(pilotCommand.ThrottlePercent / 100.0f, 0.0f, 1.0f);

    static float targetYawDeg = 0.0f;
    static bool targetYawCaptured = false;

    if (!flightState.IsArmed) {
        targetYawCaptured = false;

        if (!armRequested) return;

        if (!orientationController.IsCalibrationComplete()) {
            DiagnosticLogger::Log("error:arm_denied:gyro_calibration_required");
            return;
        }

        if (!orientationController.HasValidCompassCalibration()) {
            DiagnosticLogger::Log("error:arm_denied:compass_calibration_required");
            return;
        }

        if (altitudeInitialized && !altitudeHandler.IsCalibrationComplete()) {
            DiagnosticLogger::Log("error:arm_denied:altitude_calibration_required");
            return;
        }

        if (!motorController.Arm(throttle)) {
            DiagnosticLogger::Log("error:arm_denied:motor_controller");
            return;
        }

        targetYawDeg = orientation.YawDeg;
        targetYawCaptured = true;
        DiagnosticLogger::Log("status:motors_armed");

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
        DiagnosticLogger::Log("error:motor_update_failed:motors_disarmed");
    }

    MotorOutput currentMotorOutput = motorController.GetCurrentMotorOutput();
    /*Serial.print("M1:");
    Serial.print(currentMotorOutput.Motor1Power);
    Serial.print("\tM2:");
    Serial.print(currentMotorOutput.Motor2Power);
    Serial.print("\tM3:");
    Serial.print(currentMotorOutput.Motor3Power);
    Serial.print("\tM4:");
    Serial.println(currentMotorOutput.Motor4Power);
    Serial.println();*/

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
