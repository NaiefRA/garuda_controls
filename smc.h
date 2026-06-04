#pragma once
#include <Arduino.h>

// --- SMC FLIGHT CONTROLLER API ---
// Takes current altitude, velocity magnitude, and inclination (degrees from vertical)
// Returns target extension command from 0.0 (retracted) to 1.0 (fully deployed)
float runSMC(float h, float vMag, float inclDeg);
