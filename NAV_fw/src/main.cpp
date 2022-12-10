#include <Arduino.h>

#include "pins.h"
#include "registers.h"
#include "colors.h"

#include <Wire.h>
#include "wiring_private.h"

#include <BMP388_DEV.h>
#include <BMI088.h>
#include <LIS3MDL.h>

#include <Orientation.h>

#include <Adafruit_NeoPixel_ZeroDMA.h>

#define NO_MAG
#define NO_GNSS
#define NO_DIST

//#define DEBUG

// Chip peripheral registers
Registers registers;
uint8_t registerIndex = 0x0;

// NAV peripheral bus
uint8_t NAV_ADDRESS = 0x30;

void receiveEvent(int n);
void requestEvent();

// Sensors
BMP388_DEV barometer;

Bmi088Accel accelerometer(TWISensor, ACCELEROMETER_ADDRESS);
Bmi088Gyro  gyroscope(TWISensor, GYROSCOPE_ADDRESS);

LIS3MDL magnetometer;

// Interrupt flags
volatile bool accelerometerInterruptFlag = false;
volatile bool barometerInterruptFlag = false;
volatile bool gyroscopeInterruptFlag = false;
volatile bool magnetometerInterruptFlag = false;

// Interrupts
void accelerometerInterrupt() { accelerometerInterruptFlag = true; }
void barometerInterrupt() { barometerInterruptFlag = true; }
void gyroscopeInterrupt() { gyroscopeInterruptFlag = true; }
void magnetometerInterrupt() { magnetometerInterruptFlag = true; }

// State estimator
Orientation ahrs;
EulerAngles ahrsOutput;

void calibrate();

// Calibration constants
float gyroXBias;
float gyroYBias;
float gyroZBias;

float surfaceAltitude;

// Filter constants
///TODO: filter constants

// Visual indicators
Adafruit_NeoPixel_ZeroDMA led(1, NEOPIXEL_OUTPUT_PIN, NEO_GRB);

void fail(uint8_t flags);

// Timing
time_t thisLoopMicros;

time_t lastAHRSUpdateMicros;
time_t lastLEDUpdateMicros;

time_t LEDUpdateInterval = 1000000 / 20; // 50,000 uS



void setup() {
#ifdef DEBUG
  SERIAL_PORT_MONITOR.begin(56700);
  while(!SERIAL_PORT_MONITOR) { delay(10); }

  SERIAL_PORT_MONITOR.println("Tycho NAV Startup");
#endif

  // Register default values
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing registers...");
#endif

  registers.fields.chipID = NAV_ADDRESS;
  registers.fields.chipStatus = NavStatus::NAV_STATUS_STARTUP;

  registers.fields.ctrlA.raw = 0;
  registers.fields.ctrlB.raw = 0;

  registers.fields.ctrlA.fields.LED_EN = true;

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing LED...");
#endif

  // NeoPixel Setup
  if(registers.fields.ctrlA.fields.LED_EN)
  {
    led.begin();
    led.setBrightness(64);
    led.setPixelColor(0, Color::BLUE);
    led.show();
  }

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

  // Peripheral bus setup
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing peripheral bus...");
#endif

  TWIPeripheral.begin(NAV_ADDRESS);

  pinPeripheral(PERIPHERAL_SDA_PIN, PIO_SERCOM);
  pinPeripheral(PERIPHERAL_SCL_PIN, PIO_SERCOM);

  //TWIPeripheral.onReceive(receiveEvent);
  //TWIPeripheral.onRequest(requestEvent);

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

  // Sensor bus setup
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing sensor bus...");
#endif

  TWISensor.begin();
  TWISensor.setClock(400000);

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

  // Pins setup
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing pins...");
#endif

  analogReadResolution(ANALOG_READ_RESOLUTION);

  pinMode(BATTERY_DIVIDER_INPUT_PIN, INPUT);
  pinMode(PYRO_DIVIDER_INPUT_PIN, INPUT);

  pinMode(ACCELEROMETER_INTERRUPT_PIN, INPUT);
  pinMode(BAROMETER_INTERRUPT_PIN, INPUT);
  pinMode(GYROSCOPE_INTERRUPT_PIN, INPUT);
  pinMode(MAGNETOMETER_INTERRUPT_PIN, INPUT);

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

  // Interrupt handlers
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing interrupt handlers...");
#endif

  attachInterrupt(digitalPinToInterrupt(ACCELEROMETER_INTERRUPT_PIN),
                  accelerometerInterrupt,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(BAROMETER_INTERRUPT_PIN),
                  barometerInterrupt,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(GYROSCOPE_INTERRUPT_PIN),
                  gyroscopeInterrupt,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(MAGNETOMETER_INTERRUPT_PIN),
                  magnetometerInterrupt,
                  RISING);

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Dome");
#endif

  // Sensors initialization
#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Initializing sensors...");
#endif

  uint8_t failFlags = 0x0;

  // Barometer setup
  if(!barometer.begin(BAROMETER_ADDRESS))
  {
    failFlags |= (1 << 0);
  }
  else
  {
    barometer.enableInterrupt();
    barometer.setTimeStandby(TIME_STANDBY_10MS);
    barometer.setTempOversampling(OVERSAMPLING_X2);
    barometer.setPresOversampling(OVERSAMPLING_X2);
    barometer.startNormalConversion();
  }

  // Accelerometer
  if(accelerometer.begin() < 0)
  {
    failFlags |= (1 << 1);
  }
  else
  {
    accelerometer.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
    accelerometer.pinModeInt1(Bmi088Accel::PUSH_PULL, Bmi088Accel::ACTIVE_HIGH);
    accelerometer.mapDrdyInt1(true);
  }

  // Gyroscope
  if(gyroscope.begin() < 0)
  {
    failFlags |= (1 << 2);
  }
  else
  {
    gyroscope.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    gyroscope.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    gyroscope.mapDrdyInt3(true);
  }

#ifndef NO_MAG
  // Magnetometer
  if(!magnetometer.init())
  {
    failFlags |= (1 << 3);
  }
  else
  {
    magnetometer.enableDefault();
    magnetometer.writeReg(LIS3MDL::CTRL_REG1, 0b01111100); // 80Hz ODR
  }
#endif

#ifndef NO_GNSS
  // GNSS
#endif

#ifndef NO_DIST
  // Ground proximity distance
#endif

  // Failure indication
  if(failFlags)
  {
    fail(failFlags);
  }

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Done");
#endif

  // Startup complete
  registers.fields.chipStatus = NavStatus::NAV_STATUS_CALIBRATING;

#ifdef DEBUG
  SERIAL_PORT_MONITOR.println("Startup complete!");
#endif

  // Timing initialization
  thisLoopMicros = 
  lastAHRSUpdateMicros = 
  lastLEDUpdateMicros = micros();
}

void loop() {
  thisLoopMicros = micros();

  // Update indicator LED
  if(registers.fields.ctrlA.fields.LED_EN)
  {
    if(thisLoopMicros >= lastLEDUpdateMicros + LEDUpdateInterval)
    {
      if(registers.fields.ctrlA.fields.AHRS_EN)
      {
        if(registers.fields.ctrlA.fields.IN_CAL)
        {
          led.setPixelColor(0, Color::GREEN);
        }
        else
        {
          led.setPixelColor(0, Color::CYAN);
        }
      }
      else
      {
        led.setPixelColor(0, Color::YELLOW);
      }

      int rawBrightness = ((int)((thisLoopMicros / 4000u) % 500) - 250) >> 1;
      led.setBrightness(abs(rawBrightness));
      led.show();
      
      lastLEDUpdateMicros += LEDUpdateInterval;
    }
  }
}

// Helper functions
void fail(uint8_t failFlags)
{
#ifdef DEBUG
  SERIAL_PORT_MONITOR.print("Startup failure: ");
  SERIAL_PORT_MONITOR.println(failFlags, HEX);
#endif

  registers.fields.chipStatus = NavStatus::NAV_STATUS_ERROR;
  led.setBrightness(128);

  while(true)
  {
    for(uint8_t i = 0; i < 8; i++)
    {
      if(failFlags & (1 << i))
      {
        for(uint8_t n = 0; n < i + 1; n++)
        {
          led.setPixelColor(0, Color::RED);
          led.show();
          delay(200);
          led.setPixelColor(0, Color::OFF);
          led.show();
          delay(300);
        }

        delay(2000);
      }
    }
  }
}