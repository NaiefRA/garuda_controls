#pragma once
#include <Arduino.h> // Required for the PROGMEM macro

// Grid: 43 alts x 41 machs x 21 defls x 11 incls
const int N_ALT  = 43;
const int N_MACH = 41;
const int N_DEF  = 21;
const int N_INCL = 11;

const float ALT_MIN  = 1000.0f, ALT_STEP  = 50.0f;
const float MACH_MIN = 0.00f,   MACH_STEP = 0.02f;
const float DEF_MIN  = 0.0f,    DEF_STEP  = 3.0f;
const float INCL_MIN = 0.0f,    INCL_STEP = 1.0f;

// Add PROGMEM here
extern const float PROGMEM APOGEE_DELTA[43][41][21][11];