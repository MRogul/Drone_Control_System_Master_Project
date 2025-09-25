/*
 * imu_fusion.c
 *
 *  Created on: Jul 24, 2025
 *      Author: lolme
 */
#include "imu_fusion.h"
#include <math.h>

#define ALPHA 0.98f // Współczynnik filtru
#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.0174533f

void IMU_Fusion_Update(IMU_Angles *angles,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt)
{
    // 1. Filtracja wejścia
    static float smooth_ax = 0, smooth_ay = 0, smooth_az = 0;
    static float smooth_gx = 0, smooth_gy = 0, smooth_gz = 0;

    smooth_ax = 0.8f * ax + 0.2f * smooth_ax;
    smooth_ay = 0.8f * ay + 0.2f * smooth_ay;
    smooth_az = 0.8f * az + 0.2f * smooth_az;

    smooth_gx = 0.7f * gx + 0.3f * smooth_gx;
    smooth_gy = 0.7f * gy + 0.3f * smooth_gy;
    smooth_gz = 0.7f * gz + 0.3f * smooth_gz;

//    // 2. Przybliżenie kątów z akcelerometru [stopnie]
//    float acc_roll  = atan2f(smooth_ay, smooth_az) * RAD_TO_DEG;
//    float acc_pitch = atan2f(-smooth_ax, sqrtf(smooth_ay * smooth_ay + smooth_az * smooth_az)) * RAD_TO_DEG;

    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    // 3. Zamiana istniejących kątów na radiany do przekształcenia
    float roll_rad  = angles->roll  * DEG_TO_RAD;
    float pitch_rad = angles->pitch * DEG_TO_RAD;

    // 4. KONWERSJA GYRO: °/s → rad/s
    float p = smooth_gx * DEG_TO_RAD;
    float q = smooth_gy * DEG_TO_RAD;
    float r = smooth_gz * DEG_TO_RAD;

    // 5. Transformacja z body rates do Euler rates (na radianach)
    float euler_roll_rate = p + tanf(pitch_rad) * (sinf(roll_rad) * q + cosf(roll_rad) * r);
    float euler_pitch_rate = cosf(roll_rad) * q - sinf(roll_rad) * r;

    // 6. Integracja – kąty w radianach
    roll_rad  += euler_roll_rate * dt;
    pitch_rad += euler_pitch_rate * dt;

    // 7. Konwersja z powrotem na stopnie
    float gyro_roll  = roll_rad * RAD_TO_DEG;
    float gyro_pitch = pitch_rad * RAD_TO_DEG;

    // 8. FILTR KOMPLEMENTARNY (najważniejsza część!)
    float fused_roll = ALPHA * gyro_roll + (1.0f - ALPHA) * acc_roll;
    float fused_pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * acc_pitch;

    // 9. Filtracja wyjścia (opcjonalna, dla dodatkowego wygładzenia)
    static float output_roll = 0, output_pitch = 0;
    output_roll = 0.9f * fused_roll + 0.1f * output_roll;
    output_pitch = 0.9f * fused_pitch + 0.1f * output_pitch;

    angles->roll = output_roll;
    angles->pitch = output_pitch;
}
