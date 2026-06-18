#pragma once
#include <Arduino.h>

enum FlightPhase {
    IDLE,
    LAUNCHED,
    AIRBRAKES_ACTIVE,
    APOGEE,
    DESCENDED
};

extern FlightPhase flight_phase;

const float LAUNCH_ACCEL_THRESHOLD   = 20.0f;   // m/s^2, must be sustained
const float LAUNCH_ACCEL_CONFIRM_MS  = 150.0f;  // ms accel must be above threshold
const float LAUNCH_VEL_THRESHOLD     = 10.0f;   // m/s, vertical velocity at launch confirm

const float APOGEE_VEL_THRESHOLD     = -1.0f;   // m/s

const float AIRBRAKES_MIN_ALT        = 2000.0f; // m — ensures burnout complete and no early SMC deployment
const float AIRBRAKES_MAX_PITCH      = 20.0f;   // degrees — inhibit above this
const float AIRBRAKES_RESUME_PITCH   = 15.0f;   // degrees — resume below this (hysteresis)
// const float AIRBRAKES_MAX_ROLL_RATE  = 30.0f;   // deg/s

extern bool safety_inclination_failed;
extern bool safety_rollrate_ok;
extern bool airbrakes_inhibited;     // true => cancel deployment

extern float launch_time_ms;

void update_flight_state();   // handles transitions
bool airbrakes_allowed();     // single clean gate for airbrakes logic to check
const char* phase_name();     // for logging