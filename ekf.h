#pragma once
#include <Arduino.h>


// =========================================================================
//                           ROCKET EKF DEFINITION
// =========================================================================
struct RocketEKF {
  float x[8];
  float P[8][8];
  float Q[8][8];
  float R_alt;
  float last_innovation;  // baro innovation (z_measured - z_predicted), for logging

  void init();

  // IMU prediction
  void predict(float ax, float ay, float az, float gyro_x, float gyro_y, float gyro_z, float dt);
  
  // Update based on Baro reading
  void updateAltimeter(float measured_altitude);
  
  void getEulerAngles(float &roll, float &pitch, float &yaw);

};

// Extern
extern RocketEKF filter;