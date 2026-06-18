#include "smc.h"
#include "globals.h"
#include "apogee_interp.h"
#include <math.h>

static uint32_t lastSMCTime = 0;
static float lastSMCError = 0.0f;
static float currentExtension = 0.0f;

// --- SMC CONTROL STEP ---
float runSMC(float h, float vMag, float inclDeg) {
    float mach = vMag / 343.2f;

    // SMC reference angle of 30 degrees
    float deltaH = interpolate_apogee(h, mach, 30.0f, inclDeg);
    predicted_apogee = h + deltaH;
    float error = predicted_apogee - TARGET_APOGEE;
    
    uint32_t currentTime = micros();
    if (lastSMCTime == 0) {
        lastSMCTime = currentTime;
        lastSMCError = error;
        return 0.0f;
    }

    float dt = (currentTime - lastSMCTime) / 1000000.0f; // micros to seconds
    lastSMCTime = currentTime;
    if (dt <= 0.0f) return currentExtension;

    float error_dot = (error - lastSMCError) / dt;
    lastSMCError = error;

    float S = SMC_C * error + error_dot;
    
    float output = SMC_K * tanh(S / SMC_PHI);

    // Rate limit
    float extError = output - currentExtension;
    if (extError > RATE_LIMIT) output = currentExtension + RATE_LIMIT;
    else if (extError < -RATE_LIMIT) output = currentExtension - RATE_LIMIT;

    currentExtension = constrain(output, 0.0f, 1.0f);
    return currentExtension;
}
