#include "IMU_kalman.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define dt 0.01f

// STRUCTURES
typedef struct {
    float value;
    float alpha;
} Filter;


// LŻEJSZE filtry wyjściowe
static Filter filter_roll_out = {0, 0.6f};
static Filter filter_pitch_out = {0, 0.6f};

float apply_filter(float input, Filter* filter) {
    filter->value = filter->alpha * filter->value + (1.0f - filter->alpha) * input;
    return filter->value;
}

void Kalman_Init(KalmanFilter* kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    // PRZYWRÓĆ ORYGINALNE, SPRAWDZONE USTAWIENIA
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.5f;
}

void Kalman_Update(volatile float *Roll, volatile float *Pitch, KalmanFilter *kf_roll,
                   KalmanFilter *kf_pitch,
                   float ax, float ay, float az,
                   float gx, float gy, float gz)
{

    // Akcelerometr
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

    // 2. PREDYKCJA
    float rate_roll = gx - kf_roll->bias;
    kf_roll->angle += dt * rate_roll;

    float rate_pitch = gy - kf_pitch->bias;
    kf_pitch->angle += dt * rate_pitch;

    // 3. UPDATE MACIERZY KOWARIANCJI
    kf_roll->P[0][0] += dt * (dt*kf_roll->P[1][1] - kf_roll->P[0][1] - kf_roll->P[1][0] + kf_roll->Q_angle);
    kf_roll->P[0][1] -= dt * kf_roll->P[1][1];
    kf_roll->P[1][0] -= dt * kf_roll->P[1][1];
    kf_roll->P[1][1] += kf_roll->Q_bias * dt;

    kf_pitch->P[0][0] += dt * (dt*kf_pitch->P[1][1] - kf_pitch->P[0][1] - kf_pitch->P[1][0] + kf_pitch->Q_angle);
    kf_pitch->P[0][1] -= dt * kf_pitch->P[1][1];
    kf_pitch->P[1][0] -= dt * kf_pitch->P[1][1];
    kf_pitch->P[1][1] += kf_pitch->Q_bias * dt;

    // 4. KALMAN GAIN
    float S_r = kf_roll->P[0][0] + kf_roll->R_measure;
    float K0_r = kf_roll->P[0][0] / S_r;
    float K1_r = kf_roll->P[1][0] / S_r;

    float S_p = kf_pitch->P[0][0] + kf_pitch->R_measure;
    float K0_p = kf_pitch->P[0][0] / S_p;
    float K1_p = kf_pitch->P[1][0] / S_p;

    // 5. KOREKCJA
    kf_roll->angle += K0_r * (acc_roll - kf_roll->angle);
    kf_roll->bias  += K1_r * (acc_roll - kf_roll->angle);

    kf_pitch->angle += K0_p * (acc_pitch - kf_pitch->angle);
    kf_pitch->bias  += K1_p * (acc_pitch - kf_pitch->angle);

    // 6. UPDATE MACIERZY P
    float P00_temp_r = kf_roll->P[0][0];
    float P01_temp_r = kf_roll->P[0][1];
    kf_roll->P[0][0] -= K0_r * P00_temp_r;
    kf_roll->P[0][1] -= K0_r * P01_temp_r;
    kf_roll->P[1][0] -= K1_r * P00_temp_r;
    kf_roll->P[1][1] -= K1_r * P01_temp_r;

    float P00_temp_p = kf_pitch->P[0][0];
    float P01_temp_p = kf_pitch->P[0][1];
    kf_pitch->P[0][0] -= K0_p * P00_temp_p;
    kf_pitch->P[0][1] -= K0_p * P01_temp_p;
    kf_pitch->P[1][0] -= K1_p * P00_temp_p;
    kf_pitch->P[1][1] -= K1_p * P01_temp_p;

    // 7. BARDZO LEKKA FILTRACJA WYJŚCIA
    float raw_roll = -kf_roll->angle;
    float raw_pitch = -kf_pitch->angle;

    *Roll = apply_filter(raw_roll, &filter_roll_out);
    *Pitch = apply_filter(raw_pitch, &filter_pitch_out);
}
