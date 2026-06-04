#pragma once
#include <SD.h>
#include <Arduino.h>

#define SDCARD_CS BUILTIN_SDCARD
#define LED_PIN 13
#define LOG_FILENAME "FLIGHT.TXT"

// SD state
extern bool sd_working;
extern File logFile;

// Setup
void setupSD();
void setupLED();

// Logging
void logData();        // logs all flight data every loop
void logInit();        // called once after all setup, logs config constants
void flushLog();       // call periodically, not every loop

// Print + Log helpers (write to both Serial and SD)
void logPrintln(const char* msg);
void logPrintf(const char* fmt, ...);