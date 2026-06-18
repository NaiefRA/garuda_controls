#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  EXPERIMENTAL / PHANTOM DETECTORS
//  These run in shadow mode — they log what WOULD have triggered,
//  without affecting the real flight state machine.
// ─────────────────────────────────────────────────────────────────────────────

struct ExperimentData {
    // Derived quantities
    float accel_mag;       // magnitude of raw acceleration (m/s^2)
    float air_density;     // kg/m^3, from pressure + temperature
    float mach_number;     // v / local speed of sound
    float jerk;            // d(accel_mag)/dt  (m/s^3)

    // Phantom launch detectors
    bool ph_launch_accel;      // instantaneous: accel_mag > threshold
    bool ph_launch_sustained;  // latched: accel above threshold for 150ms+
    bool ph_launch_vel;        // instantaneous: vertical_velocity > threshold

    // Phantom burnout detectors
    bool ph_burnout_simple;    // instantaneous: accel_mag < threshold
    bool ph_burnout_sustained; // latched: accel below threshold for 200ms
    bool ph_burnout_jerk;      // latched: large negative jerk detected

    // Phantom apogee detectors
    bool ph_apogee_baro;       // latched: raw altitude drops 5m below peak
    bool ph_apogee_ekf_vel;    // instantaneous: vertical_velocity < 0
    bool ph_apogee_ekf_strict; // latched: vertical_velocity < -1 m/s
};

extern ExperimentData exp_data;

void update_experiments();
