/**
  ******************************************************************************
  * @file    zmotord.c
  * @brief   ZMotorD single-bus motor driver (USART3 PB10/PB11) + kinematics.
  ******************************************************************************
  */
#include "zmotord.h"

#ifndef ZM_HOST_TEST
#include "main.h"
#include "usart.h"
#endif

#include <stddef.h>
#include <string.h>

/* --- formatting & kinematics helpers (hardware independent) -------------- */

static void fmt_u3(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)('0' + ((v / 100u) % 10u));
  dst[1] = (uint8_t)('0' + ((v / 10u) % 10u));
  dst[2] = (uint8_t)('0' + (v % 10u));
}

static void fmt_u4(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)('0' + ((v / 1000u) % 10u));
  dst[1] = (uint8_t)('0' + ((v / 100u) % 10u));
  dst[2] = (uint8_t)('0' + ((v / 10u) % 10u));
  dst[3] = (uint8_t)('0' + (v % 10u));
}

static void fmt_motor_cmd(uint8_t *dst15, uint8_t id, uint16_t pwm, uint16_t time_ms)
{
  static const uint8_t tmpl[16] = "#000P1500T0000!";

  if (pwm < ZM_PWM_MIN) { pwm = ZM_PWM_MIN; }
  if (pwm > ZM_PWM_MAX) { pwm = ZM_PWM_MAX; }
  if (time_ms > 9999u)  { time_ms = 9999u; }

  memcpy(dst15, tmpl, 15u);
  fmt_u3(&dst15[1], id);
  fmt_u4(&dst15[5], pwm);
  fmt_u4(&dst15[10], time_ms);
}

static int16_t clamp_speed(int32_t v)
{
  if (v > (int32_t)ZM_SPEED_MAX)  { return (int16_t)ZM_SPEED_MAX; }
  if (v < -(int32_t)ZM_SPEED_MAX) { return (int16_t)-(int32_t)ZM_SPEED_MAX; }
  return (int16_t)v;
}

#ifndef ZM_HOST_TEST

#define ZM_RX_SIZE  256u

static UART_HandleTypeDef s_huart3;
static zm_bus_mode_t      s_mode = ZM_BUS_DUPLEX;

static volatile uint8_t   s_rx_buf[ZM_RX_SIZE];
static volatile uint16_t  s_rx_head;
static volatile uint16_t  s_rx_tail;

/* --- USART3 (PB10 TX / PB11 RX) low-level driver ------------------------- */

void ZM_Init(zm_bus_mode_t mode, uint32_t baud)
{
  GPIO_InitTypeDef gi = {0};

  s_mode = mode;

  HAL_NVIC_DisableIRQ(USART3_IRQn);
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  if (mode == ZM_BUS_HALFDUPLEX)
  {
    /* Single-wire half-duplex on PB10: open-drain + external pull-up */
    gi.Pin   = GPIO_PIN_10;
    gi.Mode  = GPIO_MODE_AF_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);

    gi.Pin   = GPIO_PIN_11;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gi);
  }
  else
  {
    /* Full-duplex on PB10 TX / PB11 RX (chassis ZMotorD + OpenMV RX) */
    gi.Pin   = GPIO_PIN_10;
    gi.Mode  = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);

    gi.Pin   = GPIO_PIN_11;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gi);
  }

  USART3->CR1 &= ~USART_CR1_UE;

  s_huart3.Instance          = USART3;
  s_huart3.Init.BaudRate     = baud;
  s_huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  s_huart3.Init.StopBits     = UART_STOPBITS_1;
  s_huart3.Init.Parity       = UART_PARITY_NONE;
  s_huart3.Init.Mode         = UART_MODE_TX_RX;
  s_huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  s_huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (mode == ZM_BUS_HALFDUPLEX)
  {
    (void)HAL_HalfDuplex_Init(&s_huart3);
    /* Start in receive mode until ZM_SendRaw() runs */
    USART3->CR1 &= ~USART_CR1_TE;
    USART3->CR1 |= USART_CR1_RE;
  }
  else
  {
    USART3->CR3 &= ~USART_CR3_HDSEL;
    (void)HAL_UART_Init(&s_huart3);
  }

  (void)USART3->SR;
  (void)USART3->DR;

  s_rx_head = 0u;
  s_rx_tail = 0u;

  USART3->CR1 |= USART_CR1_RXNEIE;
  HAL_NVIC_SetPriority(USART3_IRQn, 2u, 0u);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void ZM_UART_IRQHandler(void)
{
  uint32_t sr = USART3->SR;

  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE |
             USART_SR_NE | USART_SR_PE)) != 0u)
  {
    uint8_t b = (uint8_t)(USART3->DR & 0xFFu);

    if ((sr & USART_SR_RXNE) != 0u)
    {
      uint16_t next = (uint16_t)((s_rx_head + 1u) % ZM_RX_SIZE);
      if (next != s_rx_tail)
      {
        s_rx_buf[s_rx_head] = b;
        s_rx_head = next;
      }
    }
  }
}

int ZM_ReadByte(uint8_t *out)
{
  if ((out == NULL) || (s_rx_head == s_rx_tail))
  {
    return -1;
  }

  *out = s_rx_buf[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) % ZM_RX_SIZE);
  return 0;
}

void ZM_FlushRx(void)
{
  s_rx_tail = s_rx_head;
}

void ZM_SendRaw(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0u))
  {
    return;
  }

  if (s_mode == ZM_BUS_HALFDUPLEX)
  {
    USART3->CR1 &= ~USART_CR1_RE;
    USART3->CR1 |= USART_CR1_TE;
  }

  for (uint16_t i = 0u; i < len; i++)
  {
    while ((USART3->SR & USART_SR_TXE) == 0u) { }
    USART3->DR = buf[i];
  }

  while ((USART3->SR & USART_SR_TC) == 0u) { }

  if (s_mode == ZM_BUS_HALFDUPLEX)
  {
    USART3->CR1 &= ~USART_CR1_TE;
    (void)USART3->SR;
    (void)USART3->DR;
    USART3->CR1 |= USART_CR1_RE;
  }
}

#endif /* !ZM_HOST_TEST */

/* --- ZMotorD commands ---------------------------------------------------- */

void ZM_QueryId(uint8_t id)
{
  uint8_t cmd[9] = "#000PID!";
  fmt_u3(&cmd[1], id);
  ZM_SendRaw(cmd, 8u);
}

void ZM_QueryVer(uint8_t id)
{
  uint8_t cmd[10] = "#000PVER!";
  fmt_u3(&cmd[1], id);
  ZM_SendRaw(cmd, 9u);
}

void ZM_SetId(uint8_t old_id, uint8_t new_id)
{
  uint8_t cmd[13] = "#000PID000!";
  fmt_u3(&cmd[1], old_id);
  fmt_u3(&cmd[7], new_id);
  ZM_SendRaw(cmd, 11u);
}

void ZM_SetMotor(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
  uint8_t cmd[15];

  fmt_motor_cmd(cmd, id, pwm, time_ms);
  ZM_SendRaw(cmd, 15u);
}

void ZM_ComputeWheels(int16_t vx, int16_t vy, int16_t wz, int16_t out_spd[4])
{
  int32_t w[4];
  int32_t peak = 0;

  if (out_spd == NULL) { return; }

  /* Mecanum kinematics matching Carbot(C5):
   *   +vx = forward, +vy = strafe left, +wz = rotate CCW (left). */
  w[0] = (int32_t)vx - (int32_t)vy - (int32_t)wz; /* FL */
  w[1] = (int32_t)vx + (int32_t)vy + (int32_t)wz; /* FR */
  w[2] = (int32_t)vx + (int32_t)vy - (int32_t)wz; /* RL */
  w[3] = (int32_t)vx - (int32_t)vy + (int32_t)wz; /* RR */

  for (uint8_t i = 0u; i < 4u; i++)
  {
    int32_t a = (w[i] < 0) ? -w[i] : w[i];
    if (a > peak) { peak = a; }
  }

  if (peak > (int32_t)ZM_SPEED_MAX)
  {
    for (uint8_t i = 0u; i < 4u; i++)
    {
      w[i] = (w[i] * (int32_t)ZM_SPEED_MAX) / peak;
    }
  }

  for (uint8_t i = 0u; i < 4u; i++)
  {
    out_spd[i] = clamp_speed(w[i]);
  }
}

uint16_t ZM_FormatWheelsPacket(uint8_t out[60], int16_t fl, int16_t fr,
                               int16_t rl, int16_t rr, uint16_t time_ms)
{
  /* Hardware-verified per-wheel ID + direction table on STM32 chassis:
   *   #006 (FR): 1500 - fr    #007 (FL): 1500 - fl
   *   #008 (RR): 1500 + rr    #009 (RL): 1500 + rl */
  uint16_t p_fr = (uint16_t)((int32_t)ZM_PWM_STOP - (int32_t)clamp_speed(fr));
  uint16_t p_fl = (uint16_t)((int32_t)ZM_PWM_STOP - (int32_t)clamp_speed(fl));
  uint16_t p_rr = (uint16_t)((int32_t)ZM_PWM_STOP + (int32_t)clamp_speed(rr));
  uint16_t p_rl = (uint16_t)((int32_t)ZM_PWM_STOP + (int32_t)clamp_speed(rl));

  fmt_motor_cmd(&out[0],  ZM_ID_FR, p_fr, time_ms);
  fmt_motor_cmd(&out[15], ZM_ID_FL, p_fl, time_ms);
  fmt_motor_cmd(&out[30], ZM_ID_RR, p_rr, time_ms);
  fmt_motor_cmd(&out[45], ZM_ID_RL, p_rl, time_ms);
  return 60u;
}

void ZM_SetWheels(int16_t fl, int16_t fr, int16_t rl, int16_t rr,
                  uint16_t time_ms)
{
  uint8_t pkt[60];

  (void)ZM_FormatWheelsPacket(pkt, fl, fr, rl, rr, time_ms);
  ZM_SendRaw(pkt, 60u);
}

void ZM_DriveMecanum(int16_t vx, int16_t vy, int16_t wz, uint16_t time_ms)
{
  int16_t spd[4];

  ZM_ComputeWheels(vx, vy, wz, spd);
  ZM_SetWheels(spd[0], spd[1], spd[2], spd[3], time_ms);
}

void ZM_StopAll(void)
{
  ZM_SetWheels(0, 0, 0, 0, 0u);
  ZM_SetMotor(ZM_BROADCAST_ID, ZM_PWM_STOP, 0u);
}
