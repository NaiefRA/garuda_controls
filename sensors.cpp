#include "sensors.h"

Adafruit_DPS310 dps;

float accx, accy, accz;
float gyrox, gyroy, gyroz;
float raw_altitude;
float P0;

/////////////////////////
// SETUP FUNCTIONS
/////////////////////////

void setupBNO(){
  Serial.println("[BNO] Initializing...");
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x3E); 
  Wire.write(0x00); 
  Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x3D); 
  Wire.write(0x08); 
  Wire.endTransmission();
  delay(10);
  Serial.println("[BNO] OK");
};

void setupDPS310() {
    Serial.println("[DPS310] Initializing...");

    if (!dps.begin_I2C()) {
        Serial.println("[DPS310] FAILED - using default P0 = 1013.25 hPa");
        P0 = 1013.25f;
        return;
    }

    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    Serial.println("[DPS310] Waiting for first reading...");

    uint32_t sensor_timeout = millis();
    while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
        if (millis() - sensor_timeout > 1500) {
            Serial.println("[DPS310] Timeout - using default P0");
            P0 = 1013.25f;
            return;
        }
        delay(5);
    }

    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    P0 = pressure_event.pressure;

    Serial.print("[DPS310] OK - P0 = ");
    Serial.print(P0);
    Serial.println(" hPa");
}


/////////////////////////////
// DATA
/////////////////////////////

void getAccel(){
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x08); 
  Wire.endTransmission();
  Wire.requestFrom(BNO_ADDR, 6);
  int16_t AccelX = Wire.read() | (Wire.read() << 8);
  int16_t AccelY = Wire.read() | (Wire.read() << 8);
  int16_t AccelZ = Wire.read() | (Wire.read() << 8);
  
  accx = (float)AccelX / 100.0f;
  accy = (float)AccelY / 100.0f;
  accz = (float)AccelZ / 100.0f;
};

void getGyro(){
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x14); 
  Wire.endTransmission();
  Wire.requestFrom(BNO_ADDR, 6);
  int16_t GyroX = Wire.read() | (Wire.read() << 8);
  int16_t GyroY = Wire.read() | (Wire.read() << 8);
  int16_t GyroZ = Wire.read() | (Wire.read() << 8);
  
  gyrox = ((float)GyroX / 16.0f) * 0.01745329f;
  gyroy = ((float)GyroY / 16.0f) * 0.01745329f;
  gyroz = ((float)GyroZ / 16.0f) * 0.01745329f;
};

bool getBaroAltitude() {
    if (!dps.pressureAvailable() || !dps.temperatureAvailable()) return false;
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    raw_altitude = 44330.0f * (1.0f - powf(pressure_event.pressure / P0, 0.1902949f));
    return true;
}