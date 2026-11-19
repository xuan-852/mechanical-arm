/* Host-side double for Core/Inc/usart.h -- only the link primitives that
   protocol.c depends on. */
#ifndef __FAKE_USART_H
#define __FAKE_USART_H

#include <stdint.h>

void UART_WriteByte(uint8_t byte);
void UART_WriteBuf(const uint8_t *buf, uint16_t len);
int  UART_RxAvailable(void);
int  UART_ReadByte(uint8_t *out);

#endif /* __FAKE_USART_H */
