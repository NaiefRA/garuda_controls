#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <SD.h>
#include <SPI.h>
#include "ekf.h"
#include "sensors.h"
#include "motor_control.h"
#include "logger.h"
#include "globals.h"
#include "flight_state.h"
#include "smc.h"
#include "experiments.h"

// ── Global flight variables ───────────────────
float pitch = 0, roll = 0, yaw = 0;
float filtered_altitude = 0, vertical_velocity = 0;
float airbrake_theta = 0, predicted_apogee = TARGET_APOGEE;
float current_inclination = 0.0f;
uint32_t loop_time = 0;

// ── Loop diagnostics ──────────────────────────
bool     baro_fresh     = false;
float    loop_dt        = 0.0f;
uint32_t t_sensors_us   = 0;
uint32_t t_ekf_us       = 0;
uint32_t t_fsm_us       = 0;
uint32_t t_exp_us       = 0;
uint32_t t_airbrakes_us = 0;

// ── Internal timing ───────────────────────────
uint32_t last_time = 0;
unsigned long last_loop_time = 0;

// ─────────────────────────────────────────────
//  FLAP TEST STATE MACHINE
//
//  Two independent triggers, each runs 3 flaps (extend/retract × 3):
//    1. Apogee detection (flight_phase transitions to APOGEE)
//    2. Altitude drops below FLAP_ALT_M on descent
//
//  Motor is NOT commanded at any other point during flight.
// ─────────────────────────────────────────────
enum FlapState { FLAP_IDLE, FLAP_ACTIVE, FLAP_DONE };

static FlapState flap_state           = FLAP_IDLE;
static int       flap_step            = 0;      // 0-5: 3 extend + 3 retract steps
static uint32_t  flap_step_time       = 0;
static bool      flap_apogee_done     = false;  // true once apogee sequence fired
static bool      flap_alt_done        = false;  // true once altitude sequence fired

const int      FLAP_ANGLE_DEG = 45;    // degrees of extension per flap
const uint32_t FLAP_HOLD_MS   = 400;   // ms to hold each extend/retract
const float    FLAP_ALT_M     = 2750.0f; // m AGL — second trigger on descent

static void startFlapSequence(const char* reason) {
    flap_state     = FLAP_ACTIVE;
    flap_step      = 0;
    flap_step_time = millis();
    setTargetDeg(FLAP_ANGLE_DEG);  // kick off first extend immediately
    Serial.print("[FLAP] Starting — ");
    Serial.println(reason);
}

static void updateFlapTest() {
    // ── Trigger 1: apogee ──
    if (!flap_apogee_done && flight_phase == APOGEE &&
        filtered_altitude > 2500.0f) {
        flap_apogee_done = true;
        startFlapSequence("apogee");
        return;  // don't check alt trigger same loop
    }

    // ── Trigger 2: altitude below FLAP_ALT_M on descent ──
    // Only fires once apogee sequence is fully done (avoids overlap)
    if (!flap_alt_done && flap_apogee_done &&
        flight_phase == APOGEE &&
        filtered_altitude < FLAP_ALT_M && filtered_altitude > 0.0f) {
        flap_alt_done = true;
        if (flap_state == FLAP_DONE || flap_state == FLAP_IDLE) {
            startFlapSequence("2850m altitude");
        }
    }

    // ── Step through the flap sequence ──
    if (flap_state == FLAP_ACTIVE) {
        if (millis() - flap_step_time >= FLAP_HOLD_MS) {
            flap_step++;
            flap_step_time = millis();

            if (flap_step >= 6) {
                // 3 full flaps done — retract and finish
                setTargetDeg(0);
                flap_state = FLAP_DONE;
                Serial.println("[FLAP] Sequence complete — retracted");
            } else {
                // even steps → extend, odd steps → retract
                setTargetDeg((flap_step % 2 == 0) ? FLAP_ANGLE_DEG : 0);
            }
        }
    }
}


void setup() {
    setupLED();
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);

    // Enable ARM cycle counter
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CYCCNT = 0;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    uint32_t serial_timeout = millis();
    while (!Serial) {
        if (millis() - serial_timeout > 1500) break;
        delay(10);
    }

    setupBNO();
    setupDPS310();
    setupMotor();
    setupSD();
    logInit();

    filter.init();
    last_time = micros();
    last_loop_time = millis();

    Serial.println("Setup COMPLETE. Flight loop running...");
}

void loop() {
    updateMotor();

    // ── Serial test command (bench testing only) ──
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'b') {
            Serial.println("\n[TEST] Extending airbrakes to 45 degrees...");
            setTargetDeg(45);

            uint32_t test_timer = millis();
            while (millis() - test_timer < 2000) {
                updateMotor();
                delay(2);
            }

            Serial.println("[TEST] Retracting...");
            retractAirbrakes();

            test_timer = millis();
            while (millis() - test_timer < 2000) {
                updateMotor();
                delay(2);
            }
            Serial.println("[TEST] Sequence complete.\n");
        }
    }

    // ── Rate limit: ~66 Hz main loop ──
    if (millis() - last_loop_time < 15) return;
    last_loop_time = millis();

    uint32_t t_total_start = micros();

    // ── dt calculation ──
    uint32_t current_time = micros();
    float dt = (float)(current_time - last_time) / 1000000.0f;
    last_time = current_time;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
    loop_dt = dt;

    // ── Sensors ──
    uint32_t t0 = micros();
    getAccel();
    getGyro();
    baro_fresh = getBaroAltitude();
    t_sensors_us = micros() - t0;

    // ── EKF ──
    t0 = micros();
    filter.predict(accx, accy, -accz, gyrox, gyroy, gyroz, dt);
    if (baro_fresh) {
        filter.updateAltimeter(raw_altitude);
    }
    filter.getEulerAngles(yaw, pitch, roll);
    filtered_altitude = -filter.x[0];
    vertical_velocity = -filter.x[3];

    float cos_incl = 1.0f - 2.0f * (filter.x[5]*filter.x[5] + filter.x[6]*filter.x[6]);
    cos_incl = constrain(cos_incl, -1.0f, 1.0f);
    current_inclination = acosf(cos_incl) * 57.29578f;
    t_ekf_us = micros() - t0;

    // ── Flight state / safety checks ──
    t0 = micros();
    update_flight_state();
    t_fsm_us = micros() - t0;

    // ── Experiments (phantom detectors — always run) ──
    t0 = micros();
    update_experiments();
    t_exp_us = micros() - t0;

    // ── SMC: shadow mode — compute but do NOT command the motor ──
    // airbrake_theta logs what SMC would have commanded.
    // Motor is never driven during the boost/coast/airbrakes window.
    t0 = micros();
    if (flight_phase == AIRBRAKES_ACTIVE) {
        float extension = runSMC(filtered_altitude, vertical_velocity, current_inclination);
        airbrake_theta = extension * 60.0f;  // degrees — logged for post-flight analysis
        // setTargetDeg intentionally NOT called here
    } else {
        airbrake_theta = 0.0f;
    }

    // ── Flap test: only actual motor actuation ──
    updateFlapTest();
    t_airbrakes_us = micros() - t0;

    // ── Logging ──
    logData();
    flushLog();

    // ── LED state indicator ──
    updateLED();

    // ── Total loop time ──
    loop_time = micros() - t_total_start;
}