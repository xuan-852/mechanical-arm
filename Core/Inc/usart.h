/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART1 @ 115200 8N1 - host link / debug console.
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void);

/* Non-blocking helpers used by the protocol layer. */
void UART_WriteByte(uint8_t byte);
void UART_WriteBuf(const uint8_t *buf, uint16_t len);

/* RX ring buffer access (filled by HAL_UART_RxCpltCallback). */
int  UART_RxAvailable(void);
int  UART_ReadByte(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
