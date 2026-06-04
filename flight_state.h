#pragma once
#include <Arduino.h>

enum FlightPhase {
    IDLE,
    LAUNCHED,
    BURNOUT,
    COASTING,
    AIRBRAKES_ACTIVE,
    APOGEE,
    DESCENDED
};

extern FlightPhase flight_phase;

const float LAUNCH_ACCEL_THRESHOLD   = 20.0f;   // m/s^2 
const float BURNOUT_ACCEL_THRESHOLD  = 10.0f;   // m/s^2
const float BURNOUT_CONFIRM_MS       = 200.0f;  
const float COAST_DELAY_MS           = 1500.0f; // wait after burnout
const float APOGEE_VEL_THRESHOLD     = -1.0f;   // m/s

const float AIRBRAKES_MIN_ALT        = 500.0f;  // m
const float AIRBRAKES_MAX_PITCH      = 20.0f;   // degrees
// const float AIRBRAKES_MAX_ROLL_RATE  = 30.0f;   // deg/s

extern bool safety_inclination_ok;
extern bool safety_inclination_failed;
extern bool safety_rollrate_ok;
extern bool airbrakes_inhibited;     // true => cancel deployment

extern float burnout_time_ms;
extern float launch_time_ms;

void update_flight_state();   // handles transitions
bool airbrakes_allowed();     // single clean gate for airbrakes logic to check
const char* phase_name();     // for logging