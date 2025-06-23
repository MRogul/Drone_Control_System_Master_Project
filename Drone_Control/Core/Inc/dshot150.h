/*
 * dshot150.h
 *
 *  Created on: Sep 1, 2023
 *      Author: user
 *  Modified on: May 8, 2025
 */

#ifndef INC_DSHOT150_H_
#define INC_DSHOT150_H_

#include "main.h"

#define DSHOT150_TIM_ARR       266-1
#define DSHOT150_BIT_0         100
#define DSHOT150_BIT_1         200
#define DSHOT_TELEMETRY        0 // usually 0 unless ESC supports telemetry
#define SPEED_MIN              48
#define SPEED_MAX              2047

#define DSHOT_FRAME_LENGTH     17
#define DSHOT_NUM_MOTORS       4

typedef enum {
    DSHOT_SPIN_DIRECTION_NORMAL  = 0x00U,
    DSHOT_SPIN_DIRECTION_REVERSE = 0x01U
} DShot_SpinDirectionTypeDef;

typedef enum {
    DSHOT_BLUE_LED  = 0x00U,
    DSHOT_GREEN_LED = 0x01U,
    DSHOT_RED_LED   = 0x02U
} DShot_LedColorTypeDef;

// Low-level core
uint16_t dshot_prepare_packet(uint16_t command);
void dshot_prepare_dmabuffer(uint32_t *buffer, uint16_t value);

// Single-motor control
void dshot_send_ref_speed(uint8_t motor, uint16_t speed);
void dshot_arm_motor(uint8_t motor);
void dshot_led_on(uint8_t motor, DShot_LedColorTypeDef color);
void dshot_led_off(uint8_t motor, DShot_LedColorTypeDef color);
void dshot_set_spin_direction(uint8_t motor, DShot_SpinDirectionTypeDef dir);
uint8_t check_dshot_ref_speed(uint16_t speed);

// Multi-motor control
void dshot_send_all_ref_speeds(uint16_t speeds[DSHOT_NUM_MOTORS]);
void dshot_arm_all_esc(void);
// Można później dodać funkcje zbiorcze do LED i kierunku jeśli będą potrzebne

#endif /* INC_DSHOT150_H_ */
