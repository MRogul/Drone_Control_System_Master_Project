#include "esp32_cmd.h"

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* --- CRC8 SAE J1850: init=0xFF, poly=0x1D, bez ref/refout; OK gdy wynik==0 --- */
static uint8_t compute_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;
    uint8_t poly = 0x1D;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static float map_0_100(uint8_t perc, float vmin, float vmax)
{
    uint8_t p = (uint8_t)CLAMP(perc, 0, 100);
    return vmin + (vmax - vmin) * ((float)p) / 100.0f;
}

int ESP32_ProcessPacket(const uint8_t *rx, uint8_t len, ESP32_CmdCtx *ctx)
{
    if (!rx || !ctx)                          return -1;   // null
    if (len != ESP32_CMD_FRAME_LEN)           return -2;   // zła długość (oczekiwane 6 B)
    if (compute_crc8(rx, len) != 0)           return -3;   // CRC niepoprawne

    uint8_t cmd  = rx[0];
    uint8_t perc = rx[1];

    switch (cmd)
    {
        /* Wzmocnienia wspólne dla pitch/roll */
        case CMD_KP:
            if (!ctx->kp) return -10;
            *ctx->kp = map_0_100(perc, ctx->kp_min, ctx->kp_max);
            break;
        case CMD_KI:
            if (!ctx->ki) return -11;
            *ctx->ki = map_0_100(perc, ctx->ki_min, ctx->ki_max);
            break;
        case CMD_KD:
            if (!ctx->kd) return -12;
            *ctx->kd = map_0_100(perc, ctx->kd_min, ctx->kd_max);
            break;
        case CMD_TAU:
            if (!ctx->tau) return -13;
            *ctx->tau = map_0_100(perc, ctx->tau_min, ctx->tau_max);
            break;

        /* Referencje */
        case CMD_ROLL_REF:
            if (!ctx->ref_roll) return -20;
            *ctx->ref_roll = map_0_100(perc, ctx->roll_min, ctx->roll_max);
            break;
        case CMD_PITCH_REF:
            if (!ctx->ref_pitch) return -21;
            *ctx->ref_pitch = map_0_100(perc, ctx->pitch_min, ctx->pitch_max);
            break;
        case CMD_YAW_REF:
            if (!ctx->ref_yaw) return -22;
            *ctx->ref_yaw = map_0_100(perc, ctx->yaw_min, ctx->yaw_max);
            break;
        case CMD_Z_REF:
            if (!ctx->ref_z) return -23;
            *ctx->ref_z = map_0_100(perc, ctx->z_min, ctx->z_max);
            break;

        default:
            return -30; // nieznana komenda
    }

    /* Aktualizacje PID – tak jak miałeś w main */
    if (ctx->pid_pitch && ctx->kp && ctx->ki && ctx->kd && ctx->tau)
        PID_Controller_Update_Gains(ctx->pid_pitch, *ctx->kp, *ctx->ki, *ctx->kd, *ctx->tau);

    if (ctx->pid_roll && ctx->kp && ctx->ki && ctx->kd && ctx->tau)
        PID_Controller_Update_Gains(ctx->pid_roll,  *ctx->kp, *ctx->ki, *ctx->kd, *ctx->tau);

    if (ctx->pid_yaw && ctx->kp_y && ctx->ki_y && ctx->kd_y && ctx->tau_y)
        PID_Controller_Update_Gains(ctx->pid_yaw,   *ctx->kp_y, *ctx->ki_y, *ctx->kd_y, *ctx->tau_y);

    if (ctx->pid_z && ctx->kp_z && ctx->ki_z && ctx->kd_z && ctx->tau_z)
        PID_Controller_Update_Gains(ctx->pid_z,     *ctx->kp_z, *ctx->ki_z, *ctx->kd_z, *ctx->tau_z);

    return 0;
}

int ESP32_PollAndProcess(volatile uint8_t *flag, const uint8_t *rx, uint8_t len, ESP32_CmdCtx *ctx)
{
    if (!flag) return -100;
    if (*flag == 0) return 0;          // nic do zrobienia

    *flag = 0;                          // wyczyść flagę jak w Twoim kodzie
    int rc = ESP32_ProcessPacket(rx, len, ctx);
    return (rc == 0) ? 1 : rc;          // 1 = przetworzono OK; <0 = błąd
}
