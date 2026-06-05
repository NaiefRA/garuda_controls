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

// ── Global flight variables ───────────────────
float pitch = 0, roll = 0, yaw = 0;
float filtered_altitude = 0, vertical_velocity = 0;
float airbrake_theta = 0, predicted_apogee = TARGET_APOGEE;
float current_inclination = 0.0f;
uint32_t loop_time = 0;

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
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'b') {
            Serial.println("\n[TEST] Extending airbrakes to 45 degrees...");
            setTargetDeg(45);
            
            uint32_t test_timer = millis();
            while (millis() - test_timer < 2000) { // Hold for 2 seconds
                updateMotor();
                delay(2);
            }
            
            Serial.println("[TEST] Executing retractAirbrakes()...");
            retractAirbrakes();
            
            test_timer = millis();
            while (millis() - test_timer < 2000) { // Allow 2 seconds to retract
                updateMotor();
                delay(2);
            }
            Serial.println("[TEST] Sequence complete.\n");
            
        }
    }
    if (millis() - last_loop_time < 15) return;
    last_loop_time = millis();

    uint32_t t_total_start = micros();
    uint32_t t_total_start_millis = millis();
    uint32_t c_start_net = ARM_DWT_CYCCNT;


    uint32_t current_time = micros();
    float dt = (float)(current_time - last_time) / 1000000.0f;
    last_time = current_time;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    // sensors
    uint32_t t_sensors = micros();
    getAccel();
    getGyro();
    getBaroAltitude();
    t_sensors = micros() - t_sensors;

    // EKF
    uint32_t t_ekf = micros();
    filter.predict(accx, accy, -accz, gyrox, gyroy, gyroz, dt);
    filter.updateAltimeter(raw_altitude);
    filter.getEulerAngles(yaw, pitch, roll);
    filtered_altitude = -filter.x[0];
    vertical_velocity = -filter.x[3];

    float cos_incl = 1.0f - 2.0f * (filter.x[5]*filter.x[5] + filter.x[6]*filter.x[6]);
    cos_incl = constrain(cos_incl, -1.0f, 1.0f);
    current_inclination = acosf(cos_incl) * 57.29578f;
    // Serial.println(current_inclination);
    t_ekf = micros() - t_ekf;


    // Flight state / safety checks
    uint32_t t_fsm = micros();
    update_flight_state();
    t_fsm = micros() - t_fsm;

    // Airbrakes
    uint32_t t_airbrakes = micros();
    if (airbrakes_allowed()) {
        float extension = runSMC(filtered_altitude, vertical_velocity, current_inclination);
        airbrake_theta = extension * 60.0f;
        // setTargetDeg((int)airbrake_theta);
        setTargetDeg(45);

        Serial.println("EXTENSION");
    } else if (phase_name() == "IDLE") {
        Serial.println("IDLE");
    } else if (phase_name() == "AIRBRAKES_ACTIVE") {
        Serial.println("RETRACT");
        retractAirbrakes();
        airbrake_theta = 0;
    } else {
        Serial.println("ELSE");
    }
    
    // retractAirbrakes();
    t_airbrakes = micros() - t_airbrakes;

    // Logging
    uint32_t t_log = micros();
    logData();
    flushLog();
    t_log = micros() - t_log;

    loop_time = micros() - t_total_start;
    uint32_t t_total_millis = millis() - t_total_start_millis;
    uint32_t c_net = ARM_DWT_CYCCNT - c_start_net;
    uint32_t c_net_time = c_net * 1.667;
}