#include "dshot150.h"
#include "tim.h"

#define NUM_MOTORS 4
#define DSHOT_DMA_BUFFER_SIZE 17

uint32_t dshot_dmabuffer_ccr[NUM_MOTORS][DSHOT_DMA_BUFFER_SIZE];
volatile uint8_t dma_done_flags[NUM_MOTORS] = {0};

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        switch (htim->Channel)
        {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            dma_done_flags[0] = 1;
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            dma_done_flags[1] = 1;
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            dma_done_flags[2] = 1;
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            dma_done_flags[3] = 1;
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
            break;
        default:
            break;
        }
    }
}

uint16_t dshot_prepare_packet(uint16_t _command)
{
    uint16_t packet = (_command << 1) | (DSHOT_TELEMETRY ? 1 : 0);
    uint8_t csum = 0;
    uint16_t csum_data = packet;

    for (uint8_t i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    csum &= 0xf;
    return (packet << 4) | csum;
}

void dshot_prepare_dmabuffer(uint32_t *_dshot_dmabuffer_ccr, uint16_t _value)
{
    uint16_t packet = dshot_prepare_packet(_value);
    for (int i = 0; i < 16; i++)
    {
        _dshot_dmabuffer_ccr[i] = (packet & 0x8000) ? DSHOT150_BIT_1 : DSHOT150_BIT_0;
        packet <<= 1;
    }
    _dshot_dmabuffer_ccr[16] = 0;
}

void dshot_send_motor(uint8_t motor_index, uint16_t value)
{
    if (motor_index >= NUM_MOTORS) return;

    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr[motor_index], value);
    dma_done_flags[motor_index] = 0;

    uint32_t* buffer_ptr = (uint32_t*) dshot_dmabuffer_ccr[motor_index];
    switch (motor_index)
    {
    case 0:
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, buffer_ptr, DSHOT_DMA_BUFFER_SIZE);
        break;
    case 1:
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, buffer_ptr, DSHOT_DMA_BUFFER_SIZE);
        break;
    case 2:
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, buffer_ptr, DSHOT_DMA_BUFFER_SIZE);
        break;
    case 3:
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, buffer_ptr, DSHOT_DMA_BUFFER_SIZE);
        break;
    }

    while (!dma_done_flags[motor_index]) {}
}

void dshot_arm_motor(uint8_t motor_index)
{
    for (int i = 0; i < 1000; i++)
    {
        dshot_send_motor(motor_index, 0);
    }
}

void dshot_arm_all_esc(void)
{
    for (int i = 0; i < 1000; i++)
    {
        for (uint8_t motor = 0; motor < NUM_MOTORS; motor++)
        {
            dshot_send_motor(motor, 0);
        }
        HAL_Delay(5);
    }

}

void dshot_send_all_ref_speeds(uint16_t speeds[NUM_MOTORS])
{
	for (uint8_t i = 0; i < NUM_MOTORS; i++) {
	    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr[i], speeds[i]);
	    dma_done_flags[i] = 0;
	}
	// Startuj wszystkie DMA naraz
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)dshot_dmabuffer_ccr[0], DSHOT_DMA_BUFFER_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)dshot_dmabuffer_ccr[1], DSHOT_DMA_BUFFER_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dshot_dmabuffer_ccr[2], DSHOT_DMA_BUFFER_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t*)dshot_dmabuffer_ccr[3], DSHOT_DMA_BUFFER_SIZE);

}

void dshot_led_on(uint8_t motor, DShot_LedColorTypeDef color)
{
    dshot_send_motor(motor, 22 + color);
}

void dshot_led_off(uint8_t motor, DShot_LedColorTypeDef color)
{
    dshot_send_motor(motor, 26 + color);
}

void dshot_set_spin_direction(uint8_t motor, DShot_SpinDirectionTypeDef spin_dir)
{
    uint16_t value = 0;
    if (spin_dir == DSHOT_SPIN_DIRECTION_NORMAL)
        value = 20;
    else if (spin_dir == DSHOT_SPIN_DIRECTION_REVERSE)
        value = 21;

    for (int i = 0; i < 10; i++)
    {
        dshot_send_motor(motor, value);
    }
}

void dshot_send_ref_speed(uint8_t motor, uint16_t speed)
{
    dshot_send_motor(motor, speed);
}

uint8_t check_dshot_ref_speed(uint16_t speed)
{
    return (speed < 48 || speed > 2047) ? 1 : 0;
}
