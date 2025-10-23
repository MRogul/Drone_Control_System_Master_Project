#ifndef ESP32_CMD_H
#define ESP32_CMD_H

#include <stdint.h>
#include <stddef.h>

/* Użyj istniejącego typu i API PID */
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_CMD_FRAME_LEN  (6U)

/* Identyfikatory komend z ESP32 */
typedef enum {
    CMD_KP        = 0x01,
    CMD_KI        = 0x02,
    CMD_KD        = 0x03,
    CMD_TAU       = 0x04,
    CMD_ROLL_REF  = 0x05,
    CMD_PITCH_REF = 0x06,
    CMD_YAW_REF   = 0x07,
    CMD_Z_REF     = 0x08
} Esp32CmdId;

/* Kontekst – wskaźniki na obiekty/zmienne i zakresy mapowania */
typedef struct {
    /* PIDy do aktualizacji */
    PID_t *pid_pitch;
    PID_t *pid_roll;
    PID_t *pid_yaw;
    PID_t *pid_z;

    /* Bieżące wzmocnienia (pitch/roll wspólne) */
    float *kp, *ki, *kd, *tau;

    /* Oddzielne wzmocnienia dla yaw i z – tylko do Update_Gains */
    float *kp_y, *ki_y, *kd_y, *tau_y;
    float *kp_z, *ki_z, *kd_z, *tau_z;

    /* Referencje */
    float *ref_roll, *ref_pitch, *ref_yaw, *ref_z;

    /* Zakresy mapowania 0..100 → min..max */
    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;
    float tau_min, tau_max;

    float roll_min,  roll_max;
    float pitch_min, pitch_max;
    float yaw_min,   yaw_max;
    float z_min,     z_max;
} ESP32_CmdCtx;

/* Niskopoziomowe przetworzenie ramki (CRC, switch, update PID) */
int ESP32_ProcessPacket(const uint8_t *rx, uint8_t len, ESP32_CmdCtx *ctx);

/* Poller do pętli głównej – minimalizuje kod w main:
   Zwraca:
     1  – była flaga, pakiet OK i przetworzony,
     0  – brak flagi (nic do roboty),
    <0  – była flaga, ale błąd (kod ujemny). */
int ESP32_PollAndProcess(volatile uint8_t *flag, const uint8_t *rx, uint8_t len, ESP32_CmdCtx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_CMD_H */
