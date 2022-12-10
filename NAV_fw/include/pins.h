#pragma once
#include <Arduino.h>
#include <Wire.h>

// Pins
typedef uint8_t pin_t;

constexpr pin_t ACCELEROMETER_INTERRUPT_PIN = 9;
constexpr pin_t BAROMETER_INTERRUPT_PIN = 8;
constexpr pin_t GYROSCOPE_INTERRUPT_PIN = A3;
constexpr pin_t MAGNETOMETER_INTERRUPT_PIN = 3;

constexpr pin_t NEOPIXEL_OUTPUT_PIN = 5;

constexpr pin_t PERIPHERAL_SDA_PIN = 11;
constexpr pin_t PERIPHERAL_SCL_PIN = 13;

constexpr pin_t BATTERY_DIVIDER_INPUT_PIN = A5;
constexpr pin_t PYRO_DIVIDER_INPUT_PIN = A4;

// Analog pin scales
constexpr int ANALOG_READ_RESOLUTION = 12;
constexpr int ANALOG_READ_MAX_VALUE = (1 << ANALOG_READ_RESOLUTION);

constexpr float ANALOG_READ_REFERENCE = 3.3f;
constexpr float ANALOG_READ_SCALE = (ANALOG_READ_REFERENCE / ANALOG_READ_MAX_VALUE);

// Voltage divider scales
constexpr int BATTERY_DIVIDER_HIGH = 86600;
constexpr int BATTERY_DIVIDER_LOW = 30100;
constexpr float BATTERY_DIVIDER_SCALE = ((float)(BATTERY_DIVIDER_HIGH + BATTERY_DIVIDER_LOW)) / BATTERY_DIVIDER_LOW;

constexpr int PYRO_DIVIDER_HIGH = 86600;
constexpr int PYRO_DIVIDER_LOW = 30100;
constexpr float PYRO_DIVIDER_SCALE = ((float)(PYRO_DIVIDER_HIGH + PYRO_DIVIDER_LOW)) / PYRO_DIVIDER_LOW;

// Sensor addresses
typedef uint8_t addr_t;

constexpr addr_t ACCELEROMETER_ADDRESS = 0x18;
constexpr addr_t BAROMETER_ADDRESS = 0x76;
constexpr addr_t GYROSCOPE_ADDRESS = 0x68;
constexpr addr_t MAGNETOMETER_ADDRESS = 0x1C;

// Communication buses
typedef TwoWire wire_t;

wire_t& TWISensor = Wire;
wire_t TWIPeripheral(&sercom1, PERIPHERAL_SDA_PIN, PERIPHERAL_SCL_PIN);