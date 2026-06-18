#include "logger.h"
#include "globals.h"
#include "sensors.h"
#include "flight_state.h"
#include "motor_control.h"
#include "ekf.h"
#include "experiments.h"

bool sd_working = false;
File logFile;

static uint32_t last_flush_time = 0;

// ─────────────────────────────────────────────
//  HELPERS — write to Serial AND SD together
// ─────────────────────────────────────────────
void logPrintln(const char* msg) {
    Serial.println(msg);
    if (sd_working && logFile) logFile.println(msg);
}

void logPrintf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    if (sd_working && logFile) logFile.print(buf);
}

// ─────────────────────────────────────────────
//  LED SETUP
// ─────────────────────────────────────────────
void setupLED() {
    pinMode(LED_PIN, OUTPUT);
    // rapid blink on power-on
    for (int i = 0; i < 6; i++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(50);
    }
}

// ─────────────────────────────────────────────
//  LED STATE INDICATOR — non-blocking
//
//  What you'll see on the pad:
//    Setup:          rapid blink (from setupLED)
//    SD fail:        10 rapid blinks then OFF (from setupSD)
//    Setup complete: solid ON (from setupSD setting HIGH)
//    IDLE + running: heartbeat — brief OFF flash every 1.5s
//                    confirms the loop is alive. If solid
//                    ON with no pulse, code is frozen.
//    Post-launch:    solid ON (can't see it anyway)
// ─────────────────────────────────────────────
void updateLED() {
    static uint32_t last_pulse = 0;
    uint32_t now = millis();

    if (flight_phase == IDLE) {
        // Heartbeat: LED is normally ON, briefly turns OFF for 100ms every 1500ms
        if (now - last_pulse >= 1500) {
            digitalWrite(LED_PIN, LOW);
            last_pulse = now;
        } else if (now - last_pulse >= 100) {
            digitalWrite(LED_PIN, HIGH);
        }
    } else {
        // Post-launch: just stay ON, can't see it in flight
        digitalWrite(LED_PIN, HIGH);
    }
}

// ─────────────────────────────────────────────
//  SD CARD SETUP
// ─────────────────────────────────────────────
void setupSD() {
    Serial.println("[SD] Initializing...");

    uint8_t attempts = 0;
    while (!SD.begin(BUILTIN_SDCARD)) {
        attempts++;
        delay(50);
        if (attempts > 30) break;
    }

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("[SD] FAILED");
        // rapid blink to signal failure
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        return;
    }

    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        Serial.println("[SD] File open FAILED");
        sd_working = false;
        return;
    }

    sd_working = true;
    digitalWrite(LED_PIN, HIGH);  // solid on = SD good

    logFile.println("\n=============================================");
    logFile.println("========= NEW POWER-ON CYCLE STARTED =========");
    logFile.println("=============================================");
    logFile.flush();

    Serial.println("[SD] OK - logging to " LOG_FILENAME);
}

// ─────────────────────────────────────────────
//  INIT LOG — call after ALL setup is complete
// ─────────────────────────────────────────────
void logInit() {
    logPrintln("\n--- SYSTEM INIT COMPLETE ---");
    logPrintf("  P0 (ground pressure): %.2f hPa\n", P0);
    logPrintf("  Target apogee:        %.1f m\n",   TARGET_APOGEE);
    logPrintf("  SMC C / K / PHI:      %.2f / %.2f / %.2f\n", SMC_C, SMC_K, SMC_PHI);
    logPrintf("  Rate limit:           %.4f /loop\n", RATE_LIMIT);
    logPrintf("  Motor home offset:    %.2f\n",      homeOffset);
    logPrintln("----------------------------");

    // CSV header — 43 columns
    logPrintln(
        // Section 1: Core (4)
        "T_us,dt_us,Phase,Inhibited,"
        // Section 2: Raw sensors (10)
        "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,RawPress,RawTemp,RawAlt,BaroFresh,"
        // Section 3: EKF filtered (7)
        "FiltAlt,VertVel,Pitch,Roll,Yaw,Inclination,EKF_Innov,"
        // Section 4: Airbrake & motor (4)
        "AirbrakeDeg,MotorAngle,MotorTarget,PredApogee,"
        // Section 5: Derived (4)
        "AccelMag,AirDensity,Mach,Jerk,"
        // Section 6: Phantom detectors (9)
        "PH_LA,PH_LS,PH_LV,PH_BS,PH_BSust,PH_BJ,PH_ABaro,PH_AVel,PH_AStrict,"
        // Section 7: Timing (5)
        "T_Sens,T_EKF,T_FSM,T_Exp,T_Total"
    );

    if (sd_working) logFile.flush();
}

// ─────────────────────────────────────────────
//  FLIGHT DATA LOG — call every loop
//
//  Uses sectional snprintf for readability.
//  Total line length ~300 chars, buffer is 512.
// ─────────────────────────────────────────────
void logData() {
    char buf[512];
    int pos = 0;

    // Section 1: Core
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "%lu,%lu,%s,%d",
        (unsigned long)micros(),
        (unsigned long)(loop_dt * 1000000.0f),
        phase_name(),
        airbrakes_inhibited);

    // Section 2: Raw sensors
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d",
        accx, accy, accz,
        gyrox, gyroy, gyroz,
        raw_pressure, raw_temperature, raw_altitude,
        baro_fresh);

    // Section 3: EKF filtered
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f",
        filtered_altitude, vertical_velocity,
        pitch, roll, yaw,
        current_inclination,
        filter.last_innovation);

    // Section 4: Airbrake & motor
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%.2f,%.1f,%d,%.1f",
        airbrake_theta, filteredAngle, targetLeadAngle,
        predicted_apogee);

    // Section 5: Derived
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%.2f,%.4f,%.4f,%.1f",
        exp_data.accel_mag, exp_data.air_density,
        exp_data.mach_number, exp_data.jerk);

    // Section 6: Phantom detectors
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
        exp_data.ph_launch_accel, exp_data.ph_launch_sustained, exp_data.ph_launch_vel,
        exp_data.ph_burnout_simple, exp_data.ph_burnout_sustained, exp_data.ph_burnout_jerk,
        exp_data.ph_apogee_baro, exp_data.ph_apogee_ekf_vel, exp_data.ph_apogee_ekf_strict);

    // Section 7: Timing
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        ",%lu,%lu,%lu,%lu,%lu",
        (unsigned long)t_sensors_us, (unsigned long)t_ekf_us,
        (unsigned long)t_fsm_us, (unsigned long)t_exp_us,
        (unsigned long)loop_time);

    Serial.println(buf);
    if (sd_working && logFile) logFile.println(buf);
}

// ─────────────────────────────────────────────
//  FLUSH — call every ~1s, not every loop
// ─────────────────────────────────────────────
void flushLog() {
    if (!sd_working || !logFile) return;
    if (millis() - last_flush_time > 1000) {
        logFile.flush();
        last_flush_time = millis();
    }
}