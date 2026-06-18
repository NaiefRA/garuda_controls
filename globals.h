#pragma once
#include <Arduino.h>

// Flight States
extern float pitch, roll, yaw;
extern float raw_altitude, filtered_altitude, vertical_velocity;
extern float airbrake_theta, predicted_apogee;
extern float current_inclination;
extern uint32_t loop_time;

// Loop diagnostics
extern bool     baro_fresh;       // true when baro updated this loop
extern float    loop_dt;          // integration timestep (seconds)
extern uint32_t t_sensors_us;     // sensor read time
extern uint32_t t_ekf_us;         // EKF time
extern uint32_t t_fsm_us;         // FSM time
extern uint32_t t_exp_us;         // experiments time
extern uint32_t t_airbrakes_us;   // airbrake control time

// Configuration Constants
const float TARGET_APOGEE = 3048.0f;
const float SMC_C = 0.4f;
const float SMC_K = 1.0f;
const float SMC_PHI = 25.0f;
const float RATE_LIMIT = 0.013f;
