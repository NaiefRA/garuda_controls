#include "ekf.h"

// =========================================================================
//                           ROCKET EKF DEFINITION
// =========================================================================

RocketEKF filter;

void RocketEKF::init() {
    x[0] = 0.0f;  x[1] = 0.0f;  x[2] = 0.0f;  x[3] = 0.0f;
    x[4] = 1.0f;  x[5] = 0.0f;  x[6] = 0.0f;  x[7] = 0.0f;

    // Initialize state covariance with larger uncertainty to allow rapid startup convergence
    for(int i = 0; i < 8; i++) {
      for(int j = 0; j < 8; j++) {
        P[i][j] = (i == j) ? 10.0f : 0.0f;
      }
    }
    P[4][4] = 0.5f; P[5][5] = 0.5f; P[6][6] = 0.5f; P[7][7] = 0.5f;

    for(int i = 0; i < 8; i++) {
      for(int j = 0; j < 8; j++) {
        Q[i][j] = 0.0f;
      }
    }
    Q[1][1] = 0.05f; Q[2][2] = 0.05f; Q[3][3] = 0.05f; 
    Q[4][4] = 0.001f; Q[5][5] = 0.001f; Q[6][6] = 0.001f; Q[7][7] = 0.001f;

    R_alt = 0.5f; 
}

void RocketEKF::predict(float ax, float ay, float az, float gyro_x, float gyro_y, float gyro_z, float dt) {
    float pz = x[0];  float vx = x[1];  float vy = x[2];  float vz = x[3];
    float qw = x[4];  float qx = x[5];  float qy = x[6];  float qz = x[7];

    float ax_ned = (1.0f - 2.0f*(qy*qy + qz*qz))*ax + 2.0f*(qx*qy - qw*qz)*ay         + 2.0f*(qx*qz + qw*qy)*az;
    float ay_ned = 2.0f*(qx*qy + qw*qz)*ax         + (1.0f - 2.0f*(qx*qx + qz*qz))*ay + 2.0f*(qy*qz - qw*qx)*az;
    float az_ned = 2.0f*(qx*qz - qw*qy)*ax         + 2.0f*(qy*qz + qw*qx)*ay         + (1.0f - 2.0f*(qx*qx + qy*qy))*az;

    az_ned += 9.80665f;

    x[0] = pz + vz * dt;
    x[1] = vx + ax_ned * dt;
    x[2] = vy + ay_ned * dt;
    x[3] = vz + az_ned * dt;

    float qw_dot = 0.5f * (-qx*gyro_x - qy*gyro_y - qz*gyro_z);
    float qx_dot = 0.5f * ( qw*gyro_x - qz*gyro_y + qy*gyro_z);
    float qy_dot = 0.5f * ( qz*gyro_x + qw*gyro_y - qx*gyro_z);
    float qz_dot = 0.5f * (-qy*gyro_x + qx*gyro_y + qw*gyro_z);

    x[4] = qw + qw_dot * dt;
    x[5] = qx + qx_dot * dt;
    x[6] = qy + qy_dot * dt;
    x[7] = qz + qz_dot * dt;

    float norm = sqrtf(x[4]*x[4] + x[5]*x[5] + x[6]*x[6] + x[7]*x[7]);
    x[4] /= norm; x[5] /= norm; x[6] /= norm; x[7] /= norm;

    float F[8][8] = {0};
    for(int i = 0; i < 8; i++) F[i][i] = 1.0f;

    F[0][3] = dt;

    F[4][4] = 1.0f;              F[4][5] = -gyro_x * 0.5f * dt;   F[4][6] = -gyro_y * 0.5f * dt;   F[4][7] = -gyro_z * 0.5f * dt;
    F[5][4] = gyro_x * 0.5f * dt;    F[5][5] = 1.0f;              F[5][6] =  gyro_z * 0.5f * dt;   F[5][7] = -gyro_y * 0.5f * dt;
    F[6][4] = gyro_y * 0.5f * dt;    F[6][5] = -gyro_z * 0.5f * dt;   F[6][6] = 1.0f;              F[6][7] =  gyro_x * 0.5f * dt;
    F[7][4] = gyro_z * 0.5f * dt;    F[7][5] =  gyro_y * 0.5f * dt;   F[7][6] = -gyro_x * 0.5f * dt;   F[7][7] = 1.0f;

    // Corrected Jacobians for velocity states w.r.t quaternions
    F[1][4] = 2.0f * (qy * az - qz * ay) * dt;
    F[1][5] = 2.0f * (qy * ay + qz * az) * dt;
    F[1][6] = 2.0f * (qx * ay + qw * az - 2.0f * qy * ax) * dt;
    F[1][7] = 2.0f * (qx * az - qw * ay - 2.0f * qz * ax) * dt;

    F[2][4] = 2.0f * (qz * ax - qx * az) * dt;
    F[2][5] = 2.0f * (qy * ax - qw * az - 2.0f * qx * ay) * dt;
    F[2][6] = 2.0f * (qx * ax + qz * az) * dt;
    F[2][7] = 2.0f * (qw * ax + qy * az - 2.0f * qz * ay) * dt;

    F[3][4] = 2.0f * (qx * ay - qy * ax) * dt;
    F[3][5] = 2.0f * (qz * ax + qw * ay - 2.0f * qx * az) * dt;
    F[3][6] = 2.0f * (qz * ay - qw * ax - 2.0f * qy * az) * dt;
    F[3][7] = 2.0f * (qx * ax + qy * ay) * dt;

    float FP[8][8] = {0};
    for(int i=0; i<8; i++) {
      for(int j=0; j<8; j++) {
        for(int k=0; k<8; k++) {
          FP[i][j] += F[i][k] * P[k][j];
        }
      }
    }

    float FPFt[8][8] = {0};
    for(int i=0; i<8; i++) {
      for(int j=0; j<8; j++) {
        for(int k=0; k<8; k++) {
          FPFt[i][j] += FP[i][k] * F[j][k]; 
        }
        P[i][j] = FPFt[i][j] + Q[i][j];
      }
    }
}

void RocketEKF::updateAltimeter(float measured_altitude) {
    float z_obs = -measured_altitude;
    float z_pred = x[0]; 
    float innovation = z_obs - z_pred;
    float S = P[0][0] + R_alt;

    float K[8];
    for(int i = 0; i < 8; i++) {
      K[i] = P[i][0] / S;
    }

    for(int i = 0; i < 8; i++) {
      x[i] += K[i] * innovation;
    }

    // Normalize quaternion to keep it unit length and prevent divergence
    float q_norm = sqrtf(x[4]*x[4] + x[5]*x[5] + x[6]*x[6] + x[7]*x[7]);
    if (q_norm > 0.0f) {
      x[4] /= q_norm; x[5] /= q_norm; x[6] /= q_norm; x[7] /= q_norm;
    }

    float KH[8][8] = {0};
    for(int i = 0; i < 8; i++) {
      KH[i][0] = K[i]; 
    }

    float new_P[8][8] = {0};
    for(int i = 0; i < 8; i++) {
      for(int j = 0; j < 8; j++) {
        float sum = 0.0f;
        for(int k = 0; k < 8; k++) {
          sum += KH[i][k] * P[k][j];
        }
        new_P[i][j] = P[i][j] - sum;
      }
    }

    for(int i = 0; i < 8; i++) {
      for(int j = 0; j < 8; j++) {
        P[i][j] = new_P[i][j];
      }
    }
}

void RocketEKF::getEulerAngles(float &roll, float &pitch, float &yaw) {
    float qw = x[4]; float qx = x[5]; float qy = x[6]; float qz = x[7];
    roll  = atan2f(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy)) * 57.29578f;
    pitch = asinf(2.0f * (qw*qy - qz*qx)) * 57.29578f;
    yaw   = atan2f(2.0f * (qw*qz + qx*qy), 1.0f - 2.0f * (qy*qy + qz*qz)) * 57.29578f; 
};