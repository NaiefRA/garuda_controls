#include "experiments.h"
#include "globals.h"
#include "sensors.h"
#include "flight_state.h"

ExperimentData exp_data;

// ─── Internal state for phantom detectors ───

// Phantom launch sustained
static float   ph_launch_cand_time    = 0;
static bool    ph_launch_cand_started = false;
static bool    ph_launch_sustained_latched = false;

// Phantom burnout sustained
static float   ph_burnout_cand_time    = 0;
static bool    ph_burnout_cand_started = false;
static bool    ph_burnout_sustained_latched = false;

// Phantom burnout jerk
static float   prev_accel_mag  = 0;
static uint32_t prev_jerk_time = 0;
static bool    ph_burnout_jerk_latched = false;

// Phantom apogee baro
static float   baro_peak_alt = -9999.0f;
static bool    ph_apogee_baro_latched = false;

// Phantom apogee EKF strict
static bool    ph_apogee_ekf_strict_latched = false;

// ─── Thresholds (match or shadow the FSM thresholds) ───
static const float PH_LAUNCH_ACCEL    = 20.0f;   // m/s^2
static const float PH_LAUNCH_HOLD_MS  = 150.0f;  // ms
static const float PH_LAUNCH_VEL      = 10.0f;   // m/s
static const float PH_BURNOUT_ACCEL   = 10.0f;   // m/s^2
static const float PH_BURNOUT_HOLD_MS = 200.0f;  // ms
static const float PH_BURNOUT_JERK    = -100.0f;  // m/s^3 (large negative = sudden decel)
static const float PH_APOGEE_DROP     = 5.0f;    // m below peak
static const float PH_APOGEE_VEL      = -1.0f;   // m/s

// ─── Constants ───
static const float R_SPECIFIC = 287.058f;  // J/(kg·K) for dry air
static const float GAMMA_AIR  = 1.4f;


void update_experiments() {
    // ── Derived quantities ──
    float ax = accx, ay = accy, az = accz;
    exp_data.accel_mag = sqrtf(ax*ax + ay*ay + az*az);

    // Air density from ideal gas law: rho = P / (R_specific * T_kelvin)
    float T_kelvin = raw_temperature + 273.15f;
    if (T_kelvin < 200.0f) T_kelvin = 288.15f;  // sanity clamp
    float P_pascal = raw_pressure * 100.0f;      // hPa to Pa
    exp_data.air_density = P_pascal / (R_SPECIFIC * T_kelvin);

    // Mach number: v / sqrt(gamma * R * T)
    float speed_of_sound = sqrtf(GAMMA_AIR * R_SPECIFIC * T_kelvin);
    float v_mag = fabsf(vertical_velocity);
    exp_data.mach_number = (speed_of_sound > 0.0f) ? (v_mag / speed_of_sound) : 0.0f;

    // Jerk: d(accel_mag)/dt
    uint32_t now = micros();
    if (prev_jerk_time > 0) {
        float jdt = (float)(now - prev_jerk_time) / 1000000.0f;
        if (jdt > 0.0f && jdt < 0.5f) {
            exp_data.jerk = (exp_data.accel_mag - prev_accel_mag) / jdt;
        }
    } else {
        exp_data.jerk = 0.0f;
    }
    prev_accel_mag  = exp_data.accel_mag;
    prev_jerk_time  = now;

    // ── Phantom launch detectors ──

    // Instantaneous accel threshold
    exp_data.ph_launch_accel = (exp_data.accel_mag > PH_LAUNCH_ACCEL);

    // Sustained accel (latched)
    if (!ph_launch_sustained_latched) {
        if (exp_data.accel_mag > PH_LAUNCH_ACCEL) {
            if (!ph_launch_cand_started) {
                ph_launch_cand_time    = millis();
                ph_launch_cand_started = true;
            }
            if (millis() - ph_launch_cand_time > PH_LAUNCH_HOLD_MS) {
                ph_launch_sustained_latched = true;
            }
        } else {
            ph_launch_cand_started = false;
        }
    }
    exp_data.ph_launch_sustained = ph_launch_sustained_latched;

    // Instantaneous velocity threshold
    exp_data.ph_launch_vel = (vertical_velocity > PH_LAUNCH_VEL);

    // ── Phantom burnout detectors ──

    // Instantaneous low-accel
    exp_data.ph_burnout_simple = (exp_data.accel_mag < PH_BURNOUT_ACCEL);

    // Sustained low-accel (latched)
    if (!ph_burnout_sustained_latched) {
        if (exp_data.accel_mag < PH_BURNOUT_ACCEL) {
            if (!ph_burnout_cand_started) {
                ph_burnout_cand_time    = millis();
                ph_burnout_cand_started = true;
            }
            if (millis() - ph_burnout_cand_time > PH_BURNOUT_HOLD_MS) {
                ph_burnout_sustained_latched = true;
            }
        } else {
            ph_burnout_cand_started = false;
        }
    }
    exp_data.ph_burnout_sustained = ph_burnout_sustained_latched;

    // Jerk-based burnout (latched) — triggers on large negative jerk
    if (!ph_burnout_jerk_latched) {
        if (exp_data.jerk < PH_BURNOUT_JERK) {
            ph_burnout_jerk_latched = true;
        }
    }
    exp_data.ph_burnout_jerk = ph_burnout_jerk_latched;

    // ── Phantom apogee detectors ──

    // Track baro peak
    if (raw_altitude > baro_peak_alt) {
        baro_peak_alt = raw_altitude;
    }

    // Baro peak drop (latched)
    if (!ph_apogee_baro_latched) {
        if (raw_altitude < (baro_peak_alt - PH_APOGEE_DROP)) {
            ph_apogee_baro_latched = true;
        }
    }
    exp_data.ph_apogee_baro = ph_apogee_baro_latched;

    // Instantaneous velocity < 0
    exp_data.ph_apogee_ekf_vel = (vertical_velocity < 0.0f);

    // Strict velocity (latched)
    if (!ph_apogee_ekf_strict_latched) {
        if (vertical_velocity < PH_APOGEE_VEL) {
            ph_apogee_ekf_strict_latched = true;
        }
    }
    exp_data.ph_apogee_ekf_strict = ph_apogee_ekf_strict_latched;
}
