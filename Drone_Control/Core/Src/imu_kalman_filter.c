#include <imu_kalman_filter.h>
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define dt 0.01f


// STRUCTURES
typedef struct {
    float value;
    float alpha;
} Filter;


// LŻEJSZE filtry wyjściowe
//static Filter filter_roll_out = {0, 0.6f};
//static Filter filter_pitch_out = {0, 0.6f};

static Filter filter_ax = {0, 0.7f};
static Filter filter_ay = {0, 0.7f};
static Filter filter_az = {0, 0.7f};

float apply_filter(float input, Filter* filter) {
    filter->value = filter->alpha * filter->value + (1.0f - filter->alpha) * input;
    return filter->value;
}

void Kalman_Init(KalmanFilter* kf, float Qa, float Qb, float R) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    // PRZYWRÓĆ ORYGINALNE, SPRAWDZONE USTAWIENIA
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    kf->Q_angle = Qa;//*0.3
    kf->Q_bias = Qb;//*2.0
    kf->R_measure = R*0.8f;//*10.0f
}

/*
#define Qa_roll 0.00708008f
#define Qa_pitch 0.005146486f

#define Qb_roll  0.0000000069556853f
#define Qb_pitch  0.000000012531481f

#define R_roll 0.00708008f
#define R_pitch 0.005146486f
*/
void Kalman_Update(volatile float *Roll, volatile float *Pitch, KalmanFilter *kf_roll,
                   KalmanFilter *kf_pitch,
                   float ax, float ay, float az,
                   float gx, float gy, float gz)
{

    // Akcelerometr
	float ax_f = apply_filter(ax, &filter_ax);
	float ay_f = apply_filter(ay, &filter_ay);
	float az_f = apply_filter(az, &filter_az);

    float acc_roll  = atan2f(ay_f, az_f) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * RAD_TO_DEG;

//    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
//    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;


    // 2. PREDYKCJA
    float rate_roll = gx - kf_roll->bias;
    kf_roll->angle += dt * rate_roll;

    float rate_pitch = gy - kf_pitch->bias;
    kf_pitch->angle += dt * rate_pitch;

    // 3. UPDATE MACIERZY KOWARIANCJI
    kf_roll->P[0][0] += dt * (dt*kf_roll->P[1][1] - kf_roll->P[0][1] - kf_roll->P[1][0] + kf_roll->Q_angle*dt);
    kf_roll->P[0][1] -= dt * kf_roll->P[1][1];
    kf_roll->P[1][0] -= dt * kf_roll->P[1][1];
    kf_roll->P[1][1] += kf_roll->Q_bias * dt;

    kf_pitch->P[0][0] += dt * (dt*kf_pitch->P[1][1] - kf_pitch->P[0][1] - kf_pitch->P[1][0] + kf_pitch->Q_angle*dt);
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

    float y_r = acc_roll - kf_roll->angle;
    float y_p = acc_pitch - kf_pitch->angle;

    kf_roll->angle += K0_r * y_r;
    kf_roll->bias  += K1_r * y_r;

    kf_pitch->angle += K0_p * y_p;
    kf_pitch->bias  += K1_p * y_p;

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

    *Roll = -kf_roll->angle;
    *Pitch = -kf_pitch->angle;

}
