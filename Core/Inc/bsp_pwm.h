/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   6-channel 50Hz sequential-slot servo PWM on TIM3 (1us resolution).
  *
  *          KPZ-STM32 hardware pin map (candidate PB7/PB4/PB6/PB9/PB8/PB3):
  *            ch0 -> PB7
  *            ch1 -> PB4   (released via SWJ_NOJTAG)
  *            ch2 -> PB6
  *            ch3 -> PB9
  *            ch4 -> PB8
  *            ch5 -> PB3   (released via SWJ_NOJTAG)
  *
  *          All channels start relaxed (0 us, continuous LOW, zero torque) until
  *          BSP_PWM_SetPulse() is called with a pulse in [500, 2500] us.
  ******************************************************************************
  */
#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BSP_PWM_CHANNELS   6u

/* 1MHz counter -> 20000 ticks = 20ms = 50Hz (6 x 3000us slots + 2000us tail) */
#define BSP_PWM_PERIOD     20000u
#define BSP_PWM_SLOT_US     3000u
#define BSP_PWM_MIN_PULSE    500u   /* 0.5ms */
#define BSP_PWM_MAX_PULSE   2500u   /* 2.5ms */
#define BSP_PWM_MID_PULSE   1500u   /* 1.5ms */

void        BSP_PWM_Init(void);

/* pulse_us == 0 turns channel OFF (continuous LOW, zero servo torque).
   Non-zero is clamped to [MIN, MAX]. Returns the value actually applied. */
uint16_t    BSP_PWM_SetPulse(uint8_t ch, uint16_t pulse_us);

/* Turn all 6 servo outputs OFF immediately (continuous LOW, zero torque). */
void        BSP_PWM_AllOff(void);

/* Last applied pulse width in us (0 = relaxed / off). */
uint16_t    BSP_PWM_GetPulse(uint8_t ch);

/* Pin label ("PB7".."PB3") for diagnostics, or "?" on out-of-range. */
const char *BSP_PWM_PinName(uint8_t ch);

/* Called from TIM3_IRQHandler in stm32f1xx_it.c. */
void        BSP_PWM_TIM3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
