#include "flight_state.h"
#include "globals.h"
#include "sensors.h"

FlightPhase flight_phase = IDLE;

// bool safety_inclination_ok = true;
bool safety_inclination_failed = false;
bool safety_rollrate_ok    = true;
bool airbrakes_inhibited   = false;

float burnout_time_ms = 0;
float launch_time_ms  = 0;

// internal
static float burnout_candidate_time = 0;
static bool  burnout_candidate_started = false;

//  SAFETY CHECKS 
static void update_safety_checks() {

     if (current_inclination > AIRBRAKES_MAX_PITCH) {
        safety_inclination_failed = true;
     } else {
        safety_inclination_failed = false;
     }
    // float roll_rate_deg = abs(gyroz) * 57.29578f;
    // safety_rollrate_ok = (roll_rate_deg < AIRBRAKES_MAX_ROLL_RATE);

    airbrakes_inhibited = safety_inclination_failed;
}


static void check_launch() {
    float accel_mag = sqrtf(accx*accx + accy*accy + accz*accz);
    if (accel_mag > LAUNCH_ACCEL_THRESHOLD) {
        flight_phase = LAUNCHED;
        launch_time_ms = millis();
        Serial.println("[FSM] LAUNCHED");
    }
}

static void check_burnout() {
    float accel_mag = sqrtf(accx*accx + accy*accy + accz*accz);

    if (accel_mag < BURNOUT_ACCEL_THRESHOLD) {
        if (!burnout_candidate_started) {
            burnout_candidate_time    = millis();
            burnout_candidate_started = true;
        }
        if (millis() - burnout_candidate_time > BURNOUT_CONFIRM_MS) {
            flight_phase  = BURNOUT;
            burnout_time_ms = millis();
            Serial.println("[FSM] BURNOUT");
        }
    } else {
        burnout_candidate_started = false;
    }
}

static void check_coast_to_airbrakes() {
    float time_since_burnout = millis() - burnout_time_ms;

    if (time_since_burnout < COAST_DELAY_MS)   return;  // still waiting
    // if (filtered_altitude < AIRBRAKES_MIN_ALT) return;  // too low
    if (airbrakes_inhibited)                   return; 

    flight_phase = AIRBRAKES_ACTIVE;
    Serial.println("[FSM] AIRBRAKES ACTIVE");
}

static void check_apogee() {
    if (vertical_velocity < APOGEE_VEL_THRESHOLD) {
        // flight_phase = APOGEE;
        Serial.println("[FSM] APOGEE REACHED");
    }
}


void update_flight_state() {
    if (flight_phase >= LAUNCHED) {
        update_safety_checks();
    }

    switch (flight_phase) {
        case IDLE:             check_launch();              break;
        case LAUNCHED:         check_burnout();             break;
        case BURNOUT:          check_coast_to_airbrakes();  break;
        case COASTING:                                      break;

        case AIRBRAKES_ACTIVE:
            if (airbrakes_inhibited) {
                Serial.println("[FSM] Safety inhibit — retracting");
            }
            check_apogee();
            break;

        case APOGEE:
        case DESCENDED:
            break;
    }
}


bool airbrakes_allowed() {
    return (flight_phase == AIRBRAKES_ACTIVE) && !airbrakes_inhibited;
}

const char* phase_name() {
    switch (flight_phase) {
        case IDLE:             return "IDLE";
        case LAUNCHED:         return "LAUNCHED";
        case BURNOUT:          return "BURNOUT";
        case COASTING:         return "COASTING";
        case AIRBRAKES_ACTIVE: return "AIRBRAKES_ACTIVE";
        case APOGEE:           return "APOGEE";
        case DESCENDED:        return "DESCENDED";
        default:               return "UNKNOWN";
    }
}