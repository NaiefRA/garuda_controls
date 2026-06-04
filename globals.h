#pragma once
#include <Arduino.h>

// Flight States
extern float pitch, roll, yaw;
extern float raw_altitude, filtered_altitude, vertical_velocity;
extern float airbrake_theta, predicted_apogee;
extern float current_inclination;
extern uint32_t loop_time;

// Configuration Constants
const float TARGET_APOGEE = 3048.0f;
const float SMC_C = 0.4f;
const float SMC_K = 1.0f;
const float SMC_PHI = 25.0f;
const float RATE_LIMIT = 0.013f;
