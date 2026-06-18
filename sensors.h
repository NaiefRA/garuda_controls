#pragma once
#include <Wire.h>
#include <Adafruit_DPS310.h>

#define BNO_ADDR 0x28

// Sensor objects
extern Adafruit_DPS310 dps;

// Raw sensor data
extern float accx, accy, accz;
extern float gyrox, gyroy, gyroz;
extern float raw_altitude;
extern float raw_pressure;     // hPa, from DPS310
extern float raw_temperature;  // °C, from DPS310
extern float P0;

// =========================================================================
//                         HARDWARE DRIVER SETUPS
// =========================================================================

void setupBNO();
void setupDPS310();
void getAccel();
void getGyro();
bool getBaroAltitude();


