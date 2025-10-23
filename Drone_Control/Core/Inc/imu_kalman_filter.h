
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float angle;      // estymowany kąt
    float bias;       // estymowany bias żyroskopu
    float P[2][2];    // macierz kowariancji błędu
    float Q_angle;    // szum procesu (kąt)
    float Q_bias;     // szum procesu (bias)
    float R_measure;  // szum pomiaru (akcelerometr)
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf, float Qa, float Qb, float R);
void Kalman_Update(volatile float *Roll, volatile float *Pitch, KalmanFilter *kf_roll,
        KalmanFilter *kf_pitch,
        float ax, float ay, float az,
        float gx, float gy, float gz);

#endif // KALMAN_H
