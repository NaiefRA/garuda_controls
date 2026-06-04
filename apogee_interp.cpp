#include "apogee_interp.h"
#include "apogee_table.h"
#include <algorithm>

float interpolate_apogee(float alt, float mach, float defl, float incl) {
    // Clamp to table bounds
    alt  = std::max(ALT_MIN,  std::min(alt,  ALT_MIN  + (N_ALT  - 1) * ALT_STEP));
    mach = std::max(MACH_MIN, std::min(mach, MACH_MIN + (N_MACH - 1) * MACH_STEP));
    defl = std::max(DEF_MIN,  std::min(defl, DEF_MIN  + (N_DEF  - 1) * DEF_STEP));
    incl = std::max(INCL_MIN, std::min(incl, INCL_MIN + (N_INCL - 1) * INCL_STEP));

    // Lower indices
    int ia = std::min((int)((alt  - ALT_MIN)  / ALT_STEP),  N_ALT  - 2);
    int im = std::min((int)((mach - MACH_MIN) / MACH_STEP), N_MACH - 2);
    int id = std::min((int)((defl - DEF_MIN)  / DEF_STEP),  N_DEF  - 2);
    int ii = std::min((int)((incl - INCL_MIN) / INCL_STEP), N_INCL - 2);

    // Fractional positions
    float ta = (alt  - (ALT_MIN  + ia * ALT_STEP))  / ALT_STEP;
    float tm = (mach - (MACH_MIN + im * MACH_STEP)) / MACH_STEP;
    float td = (defl - (DEF_MIN  + id * DEF_STEP))  / DEF_STEP;
    float ti = (incl - (INCL_MIN + ii * INCL_STEP)) / INCL_STEP;

    // 16 surrounding corners - Interpolate along inclination first
    float c000 = APOGEE_DELTA[ia  ][im  ][id  ][ii] + ti * (APOGEE_DELTA[ia  ][im  ][id  ][ii+1] - APOGEE_DELTA[ia  ][im  ][id  ][ii]);
    float c100 = APOGEE_DELTA[ia+1][im  ][id  ][ii] + ti * (APOGEE_DELTA[ia+1][im  ][id  ][ii+1] - APOGEE_DELTA[ia+1][im  ][id  ][ii]);
    float c010 = APOGEE_DELTA[ia  ][im+1][id  ][ii] + ti * (APOGEE_DELTA[ia  ][im+1][id  ][ii+1] - APOGEE_DELTA[ia  ][im+1][id  ][ii]);
    float c110 = APOGEE_DELTA[ia+1][im+1][id  ][ii] + ti * (APOGEE_DELTA[ia+1][im+1][id  ][ii+1] - APOGEE_DELTA[ia+1][im+1][id  ][ii]);
    float c001 = APOGEE_DELTA[ia  ][im  ][id+1][ii] + ti * (APOGEE_DELTA[ia  ][im  ][id+1][ii+1] - APOGEE_DELTA[ia  ][im  ][id+1][ii]);
    float c101 = APOGEE_DELTA[ia+1][im  ][id+1][ii] + ti * (APOGEE_DELTA[ia+1][im  ][id+1][ii+1] - APOGEE_DELTA[ia+1][im  ][id+1][ii]);
    float c011 = APOGEE_DELTA[ia  ][im+1][id+1][ii] + ti * (APOGEE_DELTA[ia  ][im+1][id+1][ii+1] - APOGEE_DELTA[ia  ][im+1][id+1][ii]);
    float c111 = APOGEE_DELTA[ia+1][im+1][id+1][ii] + ti * (APOGEE_DELTA[ia+1][im+1][id+1][ii+1] - APOGEE_DELTA[ia+1][im+1][id+1][ii]);

    // Interpolate along deflection
    float c00 = c000 + td * (c001 - c000);
    float c10 = c100 + td * (c101 - c100);
    float c01 = c010 + td * (c011 - c010);
    float c11 = c110 + td * (c111 - c110);

    // Interpolate along mach
    float c0 = c00 + tm * (c01 - c00);
    float c1 = c10 + tm * (c11 - c10);

    // Interpolate along altitude
    return c0 + ta * (c1 - c0);
}