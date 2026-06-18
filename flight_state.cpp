#include "flight_state.h"
#include "globals.h"
#include "sensors.h"

FlightPhase flight_phase = IDLE;

bool safety_inclination_failed = false;
bool safety_rollrate_ok        = true;
bool airbrakes_inhibited       = false;

float launch_time_ms = 0;

// Launch detection internal state
static float   launch_candidate_time    = 0;
static bool    launch_candidate_started = false;

// ─────────────────────────────────────────────────────────────────────────────
//  SAFETY CHECKS
// ─────────────────────────────────────────────────────────────────────────────
static void update_safety_checks() {
    // Hysteresis: inhibit above MAX_PITCH, resume below RESUME_PITCH
    if (current_inclination > AIRBRAKES_MAX_PITCH) {
        safety_inclination_failed = true;
    } else if (current_inclination < AIRBRAKES_RESUME_PITCH) {
        safety_inclination_failed = false;
    }
    // If between RESUME and MAX, keep the previous state (hysteresis band)

    airbrakes_inhibited = safety_inclination_failed;
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAUNCH DETECTION  (time-sustained accel AND velocity gate)
// ─────────────────────────────────────────────────────────────────────────────
static void check_launch() {
    float accel_mag = sqrtf(accx*accx + accy*accy + accz*accz);

    if (accel_mag > LAUNCH_ACCEL_THRESHOLD) {
        if (!launch_candidate_started) {
            launch_candidate_time    = millis();
            launch_candidate_started = true;
        }

        // Confirm: accel has been high for long enough AND velocity is building
        if ((millis() - launch_candidate_time > LAUNCH_ACCEL_CONFIRM_MS) &&
            (vertical_velocity > LAUNCH_VEL_THRESHOLD)) {
            flight_phase   = LAUNCHED;
            launch_time_ms = millis();
            Serial.println("[FSM] LAUNCHED");
        }
    } else {
        // Accel dropped — reset the timer, not a real launch event
        launch_candidate_started = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  COAST → AIRBRAKES  (triggered once altitude gate is cleared)
// ─────────────────────────────────────────────────────────────────────────────
static void check_coast_to_airbrakes() {
    if (filtered_altitude < AIRBRAKES_MIN_ALT) return;  // below 2000 m — not ready
    if (airbrakes_inhibited)                   return;  // safety inhibit

    flight_phase = AIRBRAKES_ACTIVE;
    Serial.println("[FSM] AIRBRAKES ACTIVE");
}

// ─────────────────────────────────────────────────────────────────────────────
//  APOGEE DETECTION
// ─────────────────────────────────────────────────────────────────────────────
static void check_apogee() {
    if (vertical_velocity < APOGEE_VEL_THRESHOLD) {
        flight_phase = APOGEE;
        Serial.println("[FSM] APOGEE REACHED");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN FSM UPDATE — call every loop
// ─────────────────────────────────────────────────────────────────────────────
void update_flight_state() {
    if (flight_phase >= LAUNCHED) {
        update_safety_checks();
    }

    switch (flight_phase) {
        case IDLE:
            check_launch();
            break;

        case LAUNCHED:
            check_coast_to_airbrakes();
            break;

        case AIRBRAKES_ACTIVE:
            check_apogee();
            break;

        case APOGEE:
        case DESCENDED:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────────────────────
bool airbrakes_allowed() {
    return (flight_phase == AIRBRAKES_ACTIVE) && !airbrakes_inhibited;
}

const char* phase_name() {
    switch (flight_phase) {
        case IDLE:             return "IDLE";
        case LAUNCHED:         return "LAUNCHED";
        case AIRBRAKES_ACTIVE: return "AIRBRAKES_ACTIVE";
        case APOGEE:           return "APOGEE";
        case DESCENDED:        return "DESCENDED";
        default:               return "UNKNOWN";
    }
}