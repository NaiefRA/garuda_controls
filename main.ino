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

            Serial.println("[TEST] Executing retractAirbrakes()...");
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

    // ── Experiments (phantom detectors) ──
    t0 = micros();
    update_experiments();
    t_exp_us = micros() - t0;

    // ── Airbrakes ──
    t0 = micros();
    {
        static bool retract_sent = false;

        if (airbrakes_allowed()) {
            retract_sent = false;  // reset flag — ready to retract again if needed
            float extension = runSMC(filtered_altitude, vertical_velocity, current_inclination);
            airbrake_theta = extension * 60.0f;
            setTargetDeg((int)airbrake_theta);
        } else if ((flight_phase == AIRBRAKES_ACTIVE && airbrakes_inhibited) ||
                   (flight_phase == APOGEE || flight_phase == DESCENDED)) {
            if (!retract_sent) {
                retractAirbrakes();
                airbrake_theta = 0;
                retract_sent = true;
            }
        }
    }
    t_airbrakes_us = micros() - t0;

    // ── Logging ──
    logData();
    flushLog();

    // ── LED state indicator ──
    updateLED();

    // ── Total loop time ──
    loop_time = micros() - t_total_start;
}