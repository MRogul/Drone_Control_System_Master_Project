/*
 * imu_fusion.c
 *
 *  Created on: Jul 24, 2025
 *      Author: lolme
 */
#include "imu_fusion.h"

#define RAD_TO_DEG 57.2958f
#define ALPHA 0.98f // Współczynnik komplementarnego filtra

void IMU_Fusion_Update(IMU_Angles *angles,
                       float ax, float ay, float az,
                       float gx, float gy,
                       float dt)
{
    // 1. Kąty z akcelerometru (pitch i roll)
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;

    // 2. Kąty z żyroskopu (integracja)
    float gyro_pitch = angles->pitch + gx * dt; // gx w stopniach/s
    float gyro_roll  = angles->roll  + gy * dt;

    // 3. Filtr komplementarny
    angles->pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * acc_pitch;
    angles->roll  = ALPHA * gyro_roll  + (1.0f - ALPHA) * acc_roll;
}


