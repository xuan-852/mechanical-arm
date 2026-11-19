/**
  ******************************************************************************
  * @file    tim.h
  * @brief   TIM3 1MHz (20ms / 50Hz) timebase for 6-channel servo slot PWM.
  ******************************************************************************
  */
#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

void MX_TIM3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */
