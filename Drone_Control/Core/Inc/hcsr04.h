#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Struktura sterująca czujnikiem HC-SR04 */
typedef struct {
    TIM_HandleTypeDef *htim_cnt;     /* timer do pomiaru ECHO (Base Timer) */
    GPIO_TypeDef      *trig_port;    /* port pinu TRIG */
    uint16_t           trig_pin;     /* pin TRIG (np. TRIG_PIN_Pin) */
    uint16_t           echo_pin;     /* pin ECHO (do porównania w EXTI callback) */

    /* Parametry czasowe */
    uint32_t           timer_hz;     /* częstotliwość licznika timera [Hz] */
    float              cm_per_tick;  /* przelicznik (17150.0 / timer_hz) */
    uint16_t           trig_pulse_cycles; /* ile cykli __NOP() dla impulsu TRIG (~50us = ok. 400 Twoich pętli) */

    /* Stan pomiaru */
    volatile uint8_t   echo_active;  /* 0 = oczekiwanie na zbocze narastające, 1 = w trakcie pomiaru */
    volatile uint32_t  echo_ticks;   /* zmierzony czas trwania ECHO (w tikach timera) */
    volatile float     distance_cm;  /* przeliczony dystans [cm] */
    volatile uint8_t   new_sample;   /* 1 = dostępna nowa próbka */

    /* Periodyczne wyzwalanie (opcjonalne) */
    uint32_t           period_divider; /* co ile "tików" wołać trigger (np. 6) */
    uint32_t           period_cnt;     /* licznik do periodycznego wyzwalania */
} HCSR04_t;

/* --- API --- */

/* Inicjalizacja struktury i przelicznika. Nie włącza przerwań – to robisz w CubeMX. */
void HCSR04_Init(HCSR04_t *s,
                 TIM_HandleTypeDef *htim_counter,
                 uint32_t timer_hz,
                 GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 uint16_t echo_pin);

/* Ustawia długość impulsu TRIG jako liczbę pętli __NOP() (domyślnie ~400 ≈ 50us) */
static inline void HCSR04_SetTrigPulse(HCSR04_t *s, uint16_t nop_cycles) {
    s->trig_pulse_cycles = nop_cycles;
}

/* Ustawia dzielnik periodycznego wyzwalania (np. 6) */
static inline void HCSR04_SetPeriodicDivider(HCSR04_t *s, uint32_t divider) {
    s->period_divider = (divider == 0) ? 1 : divider;
}

/* Ręczne wyzwolenie pomiaru (impuls TRIG) */
void HCSR04_Trigger(HCSR04_t *s);

/* Wywołaj w EXTI callback: przekaż numer pinu z przerwania */
void HCSR04_ProcessExti(HCSR04_t *s, uint16_t gpio_pin);

/* Wygoda: wywołuj np. w swoim ISR od TIM15 – co wywołanie zwiększa licznik
   i co 'period_divider' robi trigger. Zwraca 1, jeśli wyzwolono trigger. */
int HCSR04_OnPeriodic(HCSR04_t *s);

/* Czy pojawiła się nowa próbka? (nie czyści flagi) */
static inline bool HCSR04_HasNew(const HCSR04_t *s) {
    return s->new_sample != 0u;
}

/* Pobiera najnowszą odległość [cm] i czyści flagę new_sample */
float HCSR04_GetDistanceCm(HCSR04_t *s);

/* Dostęp do surowych tików timera */
static inline uint32_t HCSR04_GetEchoTicks(const HCSR04_t *s) {
    return s->echo_ticks;
}

#ifdef __cplusplus
}
#endif

#endif /* HCSR04_H */
