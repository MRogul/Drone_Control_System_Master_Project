/*
 * imu_fusion.c
 *
 *  Created on: Jul 24, 2025
 *      Author: lolme
 */
#include "imu_fusion.h"

#define ALPHA 0.98f // Współczynnik filtru
#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.0174533f

void IMU_Fusion_Update(IMU_Angles *angles,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt)
{
    // Przybliżenie kątów z akcelerometru [stopnie]
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    // Zamiana istniejących kątów na radiany do przekształcenia
    float roll_rad  = angles->roll  * DEG_TO_RAD;
    float pitch_rad = angles->pitch * DEG_TO_RAD;

    // KONWERSJA GYRO: °/s → rad/s
    float p = gx * DEG_TO_RAD;
    float q = gy * DEG_TO_RAD;
    float r = gz * DEG_TO_RAD;
    // Transformacja z body rates do Euler rates (na radianach)
    float euler_roll_rate = p + tanf(pitch_rad) * (sinf(roll_rad) * q + cosf(roll_rad) * r);
    float euler_pitch_rate = cosf(roll_rad) * q - sinf(roll_rad) * r;

    // Integracja – kąty w radianach
    roll_rad  += euler_roll_rate * dt;
    pitch_rad += euler_pitch_rate * dt;

    // Konwersja z powrotem na stopnie
    float gyro_roll  = roll_rad * RAD_TO_DEG;
    float gyro_pitch = pitch_rad * RAD_TO_DEG;

    // Filtr komplementarny
    angles->roll  = ALPHA * gyro_roll  + (1.0f - ALPHA) * acc_roll;
    angles->pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * acc_pitch;
}


