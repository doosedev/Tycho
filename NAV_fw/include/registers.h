#pragma once
#include <Arduino.h>

// Nav Status
typedef uint8_t nav_status_t;

enum class NavStatus : nav_status_t
{
    NAV_STATUS_STARTUP = 0,
    NAV_STATUS_CALIBRATING,
    NAV_STATUS_CONVERGING,
    NAV_STATUS_OK_NO3D,
    NAV_STATUS_OK_3D,
    NAV_STATUS_NO_FIX,
    NAV_STATUS_ERROR
};

struct Flags1Data
{
    bool AHRS_EN                : 1;
    bool AHRS_COMP_FILTER_EN    : 1;
    bool AHRS_ZERO_ROLL_EN      : 1;
    bool AHRS_ZERO_ALL_EN       : 1;
    bool CALIBRATE_TRIG         : 1;
    bool RESET_TRIG             : 1;
    bool IN_CAL                 : 1;
    bool LED_EN                 : 1;
};
union Flags1
{
    Flags1Data fields;
    uint8_t raw;
};

struct Flags2Data
{
    bool MAG_EN                 : 1;
    bool GNSS_EN                : 1;
    bool DIST_EN                : 1;
    bool RESERVED               : 5;
};
union Flags2
{
    Flags2Data fields;
    uint8_t raw;
};

struct RegistersData
{
    // Chip Info
    uint8_t chipID;
    NavStatus chipStatus;

    // Flags
    Flags1 ctrlA;
    Flags2 ctrlB;

    // Barometric Data
    float temperature;
    float pressure;
    float barometricAltitude;

    // Accelerometer (Body Reference)
    float aXb;
    float aYb;
    float aZb;

    // Gyroscope (Body Reference)
    float gXb;
    float gYb;
    float gZb;

    // Magnetometer (Body Reference)
    float mXb;
    float mYb;
    float mZb;

    // Orientation Eulers
    float yaw;
    float pitch;
    float roll;

    // Velocity (Inertial Reference)
    float vXi;
    float vYi;
    float vZi;

    // Position (Inertial Reference)
    float pXi;
    float pYi;
    float pZi;

    // Voltages
    float batteryVoltage;
    float pyroVoltage;
};
union Registers
{
    RegistersData fields;
    uint8_t raw[sizeof(RegistersData)];
};