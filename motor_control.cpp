#include <Arduino.h>
#include "motor_control.h"

// --- PID PARAMETERS ---
float Kp = 6.0;   
float Ki = 0.04;  
float Kd = 0.2;  

static float lastError = 0, integral = 0;


// --- VARIABLES ---
float rawangle = 0, zeroangle = 0, realangle = 0, prev_raw = 0;
float filteredAngle = 0, homeOffset = 0;
int turns = 0;
int targetLeadAngle = 0;
bool motorTimedOut = false;
unsigned long moveStartTime = 0;


void setupMotor(){  
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    analogWriteFrequency(PWM_PIN, 400); 

    delay(500);
    calculateAngle();
    zeroangle = rawangle; 
  
    for(int i=0; i<50; i++) { 
        calculateAngle(); 
        delay(10);
    }
    homeOffset = realangle; 
    Serial.print("[Motor] Setup Complete - Zero Angle: ");
    Serial.print(zeroangle);
    Serial.print(" Home Offset: ");
    Serial.println(homeOffset);
}

void calculateAngle(){
    Wire.beginTransmission(0x36);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 2);
    if (Wire.available() == 2) {
        rawangle = (Wire.read() << 8 | Wire.read()) * GEAR_FACTOR;
    }
    if (prev_raw > 280 && rawangle < 80) turns++;
    if (prev_raw < 80 && rawangle > 280) turns--;
    prev_raw = rawangle;
    float magnetTotal = (turns * 360.0) + rawangle - zeroangle;
    realangle = magnetTotal / OURRATIO;
}

void updateMotor() {
    calculateAngle();
    float currentPos = realangle - homeOffset;
    filteredAngle = (filteredAngle * 0.8f) + (currentPos * 0.2f);
    bool lowLimitHit = (filteredAngle < LIMIT_MIN);
    float error = targetLeadAngle - filteredAngle;

    if (millis() - moveStartTime > MAX_MOVE_TIME) motorTimedOut = true;

    if (abs(error) < DEADBAND || motorTimedOut || lowLimitHit) {
        analogWrite(PWM_PIN, 0); 
        integral = 0;
    } else {
        integral += error;
        integral = constrain(integral, -500, 500);
        float derivative = error - lastError;
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        digitalWrite(DIR_PIN, (output > 0) ? LOW : HIGH);

        int speed = constrain(abs(output), 0, 255);
        if (speed > 0 && speed < 65) speed = 65; 
        analogWrite(PWM_PIN, speed);
    }
}


void setTargetDeg(int deg) {
    int clamped = constrain(deg, 0, 80);
    targetLeadAngle = motorTable[clamped];
    integral = 0;
    motorTimedOut = false;
    moveStartTime = millis();
}

void retractAirbrakes() {
    Serial.print("[Motor] RETRACTING AIRBRAKES. Current filtered angle: ");
    Serial.println(filteredAngle);
    setTargetDeg(0);
}