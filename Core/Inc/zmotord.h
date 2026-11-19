/**
  ******************************************************************************
  * @file    zmotord.h
  * @brief   ZMotorD single-bus dual motor driver module (USART3 PB10/PB11).
  *
  *          Hardware-verified chassis wiring & polarities (@ 115200 8N1):
  *            Bus UART : USART3 (PB10 TX / PB11 RX; shared with OpenMV RX)
  *            Time unit: milliseconds (0000 = continuous, 0001..9999 ms)
  *            #006     : Front-Right (FR), forward = 1500 - speed
  *            #007     : Front-Left  (FL), forward = 1500 - speed
  *            #008     : Rear-Right  (RR), forward = 1500 + speed
  *            #009     : Rear-Left   (RL), forward = 1500 + speed
  ******************************************************************************
  */
#ifndef __ZMOTORD_H
#define __ZMOTORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ZM_DEFAULT_BAUD    115200u
#define ZM_BROADCAST_ID    255u
#define ZM_PWM_MIN         500u
#define ZM_PWM_STOP        1500u
#define ZM_PWM_MAX         2500u
#define ZM_SPEED_MAX       1000

/* Hardware-verified wheel IDs on the mecanum chassis. */
#define ZM_ID_FR           6u
#define ZM_ID_FL           7u
#define ZM_ID_RR           8u
#define ZM_ID_RL           9u

typedef enum
{
  ZM_BUS_DUPLEX = 0,     /* PB10=USART3_TX (AF_PP), PB11=USART3_RX */
  ZM_BUS_HALFDUPLEX = 1  /* PB10=USART3_TX single-wire HDSEL (AF_OD) */
} zm_bus_mode_t;

void     ZM_Init(zm_bus_mode_t mode, uint32_t baud);
void     ZM_SendRaw(const uint8_t *buf, uint16_t len);
int      ZM_ReadByte(uint8_t *out);
void     ZM_FlushRx(void);

void     ZM_QueryId(uint8_t id);
void     ZM_QueryVer(uint8_t id);
void     ZM_SetId(uint8_t old_id, uint8_t new_id);
void     ZM_SetMotor(uint8_t id, uint16_t pwm, uint16_t time_ms);

/* Pure kinematics & formatting helpers (also exercised on host unit tests). */
void     ZM_ComputeWheels(int16_t vx, int16_t vy, int16_t wz, int16_t out_spd[4]);
uint16_t ZM_FormatWheelsPacket(uint8_t out[60], int16_t fl, int16_t fr,
                               int16_t rl, int16_t rr, uint16_t time_ms);

/* High-level chassis control with verified wheel IDs and polarities. */
void     ZM_SetWheels(int16_t fl, int16_t fr, int16_t rl, int16_t rr,
                      uint16_t time_ms);
void     ZM_DriveMecanum(int16_t vx, int16_t vy, int16_t wz, uint16_t time_ms);
void     ZM_StopAll(void);

/* Called from USART3_IRQHandler in stm32f1xx_it.c. */
void     ZM_UART_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __ZMOTORD_H */
