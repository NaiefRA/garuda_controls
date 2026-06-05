#include "logger.h"
#include "globals.h"
#include "sensors.h"
#include "flight_state.h"
#include "motor_control.h"

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
//  SETUP
// ─────────────────────────────────────────────
void setupLED() {
    pinMode(LED_PIN, OUTPUT);
    // quick blink on power-on
    for (int i = 0; i < 6; i++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(50);
    }
}

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

    // CSV header
    logPrintln("Time_ms,LoopTime,Inclination,Pitch,Yaw,Roll,AccX,AccY,AccZ,"
            "RawAlt,FiltAlt,VertVel,"
            "AirbrakeTargetDeg,MotorAngle,PredApogee,"
            "Phase,Inhibited");

    if (sd_working) logFile.flush();
}

// ─────────────────────────────────────────────
//  FLIGHT DATA LOG — call every loop
// ─────────────────────────────────────────────
void logData() {
    char buf[192];
    snprintf(buf, sizeof(buf),
        "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%d",
        millis(), loop_time, current_inclination,
        accz,
        filtered_altitude, vertical_velocity,
        airbrake_theta, filteredAngle, predicted_apogee,
        phase_name(),
        airbrakes_inhibited
    );

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