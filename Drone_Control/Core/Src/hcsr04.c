#include "hcsr04.h"

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* Szybkie makra na timer */
#define HCSR04_CNT_RESET(s)     (__HAL_TIM_SET_COUNTER((s)->htim_cnt, 0))
#define HCSR04_CNT_READ(s)      (__HAL_TIM_GET_COUNTER((s)->htim_cnt))
#define HCSR04_TIM_START(s)     (HAL_TIM_Base_Start((s)->htim_cnt))
#define HCSR04_TIM_STOP(s)      (HAL_TIM_Base_Stop((s)->htim_cnt))

void HCSR04_Init(HCSR04_t *s,
                 TIM_HandleTypeDef *htim_counter,
                 uint32_t timer_hz,
                 GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 uint16_t echo_pin)
{
    s->htim_cnt  = htim_counter;
    s->trig_port = trig_port;
    s->trig_pin  = trig_pin;
    s->echo_pin  = echo_pin;

    s->timer_hz  = (timer_hz == 0) ? 1000000u : timer_hz; /* domyślnie 1 MHz */
    s->cm_per_tick = 17150.0f / (float)s->timer_hz;       /* 34300/2/timer_hz */

    s->trig_pulse_cycles = 400;  /* ~50 µs, jak w Twoim kodzie */

    s->echo_active = 0;
    s->echo_ticks  = 0;
    s->distance_cm = 0.0f;
    s->new_sample  = 0;

    s->period_divider = 6;  /* domyślnie co 6 wywołań OnPeriodic() */
    s->period_cnt     = 0;
}

void HCSR04_Trigger(HCSR04_t *s)
{
    /* Impuls TRIG: minimum ~10 µs; zostawiamy ~50 µs dla pewności */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < s->trig_pulse_cycles; i++) {
        __NOP();
    }
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    /* Oczekuj na narastające zbocze ECHO */
    s->echo_active = 0;
}

void HCSR04_ProcessExti(HCSR04_t *s, uint16_t gpio_pin)
{
    if (gpio_pin != s->echo_pin) return;

    if (s->echo_active == 0u) {
        /* Narastające zbocze – start pomiaru */
        s->echo_active = 1u;
        HCSR04_CNT_RESET(s);
        HCSR04_TIM_START(s);
    } else {
        /* Opadające zbocze – koniec pomiaru */
        s->echo_ticks = HCSR04_CNT_READ(s);
        HCSR04_TIM_STOP(s);
        s->echo_active = 0u;

        /* Przeliczenie czasu na odległość:
           odległość [cm] = ticks * (34300 cm/s) / (2 * timer_hz) = ticks * (17150 / timer_hz)
         */
        s->distance_cm = (float)s->echo_ticks * s->cm_per_tick;
        s->new_sample  = 1u;
    }
}

int HCSR04_OnPeriodic(HCSR04_t *s)
{
    s->period_cnt++;
    if (s->period_cnt >= s->period_divider) {
        s->period_cnt = 0;
        HCSR04_Trigger(s);
        return 1;
    }
    return 0;
}

float HCSR04_GetDistanceCm(HCSR04_t *s)
{
    s->new_sample = 0u;
    return s->distance_cm;
}
