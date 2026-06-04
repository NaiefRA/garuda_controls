#pragma once
#include <Wire.h>

/* --- 4R-SLIDER MOTION CONTROL (Bottom-Limit Protected) ---
 * Pins: DIR 37, PWM 36
 * Protection: Timeout + Strict 0-Limit (No high-end limit)
 */

#define DIR_PIN 37
#define PWM_PIN 36

// --- CONSTANTS ---
const float OURRATIO = 290871.0f / 97240.0f;
const float GEAR_FACTOR = 0.087890625f;
const int DEADBAND = 3;
const long MAX_MOVE_TIME = 2500;
const int LIMIT_MIN = -10;    // Cut power if it goes significantly below home

const int motorTable[81] = {
    0, 17, 34, 52, 69, 87, 104, 122, 139, 156,
    173, 191, 208, 226, 243, 261, 278, 296, 313, 331,
    348, 366, 383, 401, 419, 436, 454, 472, 489, 507,
    524, 542, 560, 578, 596, 614, 631, 649, 667, 686,
    704, 722, 740, 759, 777, 796, 814, 833, 852, 871,
    889, 908, 927, 946, 966, 985, 1005, 1024, 1044, 1064, 1083,
    1104, 1125, 1146, 1167, 1188, 1210, 1232, 1255, 1277, 1300,
    1324, 1347, 1372, 1396, 1421, 1447, 1472, 1499, 1525, 1552
};

// Motor State
extern float rawangle, zeroangle, realangle, prev_raw;
extern float filteredAngle, homeOffset;
extern int turns;
extern int targetLeadAngle;
extern bool motorTimedOut;
extern unsigned long moveStartTime;

// Functions
void setupMotor();
void calculateAngle();     // read encoder, update realangle
void updateMotor();        // run PID, drive motor
void setTargetDeg(int deg); // takes 0-80 degrees, looks up motorTable, sets targetLeadAngle CHANGE
void retractAirbrakes();  // setTargetDeg(0)