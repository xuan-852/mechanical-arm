/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @brief   6-channel 50Hz sequential-slot servo PWM on TIM3 (1us resolution).
  *
  *          Each 20,000 us frame is split into six non-overlapping 3,000 us
  *          windows (ch0..ch5) plus a 2,000 us sync tail. Only one GPIO toggles
  *          per edge at NVIC priority 0, giving <1us jitter on arbitrary pins
  *          (including PA12/PA13/PA14 which have no hardware timer OC channels).
  ******************************************************************************
  */
#include "bsp_pwm.h"
#include "tim.h"
#include "main.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  const char   *name;
} bsp_pwm_pin_t;

static const bsp_pwm_pin_t s_pins[BSP_PWM_CHANNELS] =
{
  { GPIOB, GPIO_PIN_7, "PB7" }, /* ch0 */
  { GPIOB, GPIO_PIN_4, "PB4" }, /* ch1 */
  { GPIOB, GPIO_PIN_6, "PB6" }, /* ch2 */
  { GPIOB, GPIO_PIN_9, "PB9" }, /* ch3 */
  { GPIOB, GPIO_PIN_8, "PB8" }, /* ch4 */
  { GPIOB, GPIO_PIN_3, "PB3" }, /* ch5 */
};

static volatile uint16_t s_pulse[BSP_PWM_CHANNELS];
static volatile uint8_t  s_slot;

static void all_pins_low(void)
{
  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++)
  {
    s_pins[i].port->BRR = s_pins[i].pin;
  }
}

static void slot_begin(uint8_t slot)
{
  uint16_t w    = s_pulse[slot];
  uint16_t base = (uint16_t)slot * BSP_PWM_SLOT_US;

  s_slot     = slot;
  TIM3->CCR2 = (uint16_t)(base + BSP_PWM_SLOT_US);

  if (w != 0u)
  {
    TIM3->CCR1 = (uint16_t)(base + w);
    TIM3->SR   = (uint16_t)~TIM_SR_CC1IF;
    TIM3->DIER |= TIM_DIER_CC1IE;
    s_pins[slot].port->BSRR = s_pins[slot].pin;
  }
  else
  {
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    s_pins[slot].port->BRR = s_pins[slot].pin;
  }
}

void BSP_PWM_Init(void)
{
  GPIO_InitTypeDef gi = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Release PB3 (JTDO) and PB4 (NJTRST) to plain push-pull GPIO while keeping
     SWD (PA13/PA14) enabled. */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++)
  {
    s_pulse[i]          = 0u;
    s_pins[i].port->BRR = s_pins[i].pin;

    gi.Pin   = s_pins[i].pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(s_pins[i].port, &gi);
  }
  all_pins_low();

  MX_TIM3_Init();

  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  TIM3->CR1  &= ~TIM_CR1_CEN;
  TIM3->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE);
  s_slot      = BSP_PWM_CHANNELS;
  TIM3->CNT   = (uint16_t)(BSP_PWM_PERIOD - 10u); /* triggers slot 0 in 10us */
  TIM3->SR    = 0u;
  TIM3->DIER |= TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM3_IRQn, 0u, 0u);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  TIM3->CR1 |= TIM_CR1_CEN;
}

uint16_t BSP_PWM_SetPulse(uint8_t ch, uint16_t pulse_us)
{
  uint32_t primask;

  if (ch >= BSP_PWM_CHANNELS)
  {
    return 0u;
  }

  if (pulse_us == 0u)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    s_pulse[ch]          = 0u;
    s_pins[ch].port->BRR = s_pins[ch].pin;
    if (s_slot == ch)
    {
      TIM3->DIER &= ~TIM_DIER_CC1IE;
    }
    __set_PRIMASK(primask);
    return 0u;
  }

  if (pulse_us < BSP_PWM_MIN_PULSE) { pulse_us = BSP_PWM_MIN_PULSE; }
  if (pulse_us > BSP_PWM_MAX_PULSE) { pulse_us = BSP_PWM_MAX_PULSE; }

  s_pulse[ch] = pulse_us;
  return pulse_us;
}

void BSP_PWM_AllOff(void)
{
  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++)
  {
    (void)BSP_PWM_SetPulse(i, 0u);
  }
}

uint16_t BSP_PWM_GetPulse(uint8_t ch)
{
  return (ch < BSP_PWM_CHANNELS) ? s_pulse[ch] : 0u;
}

const char *BSP_PWM_PinName(uint8_t ch)
{
  return (ch < BSP_PWM_CHANNELS) ? s_pins[ch].name : "?";
}

void BSP_PWM_TIM3_IRQHandler(void)
{
  uint32_t sr   = TIM3->SR;
  uint32_t dier = TIM3->DIER;

  if (((sr & TIM_SR_CC1IF) != 0u) && ((dier & TIM_DIER_CC1IE) != 0u))
  {
    if (s_slot < BSP_PWM_CHANNELS)
    {
      s_pins[s_slot].port->BRR = s_pins[s_slot].pin;
    }
    TIM3->SR    = (uint16_t)~TIM_SR_CC1IF;
    TIM3->DIER &= ~TIM_DIER_CC1IE;
  }

  if (((sr & TIM_SR_UIF) != 0u) && ((dier & TIM_DIER_UIE) != 0u))
  {
    TIM3->SR = (uint16_t)~TIM_SR_UIF;
    all_pins_low();
    TIM3->SR   = (uint16_t)~TIM_SR_CC2IF;
    TIM3->DIER |= TIM_DIER_CC2IE;
    slot_begin(0u);
  }
  else if (((sr & TIM_SR_CC2IF) != 0u) && ((dier & TIM_DIER_CC2IE) != 0u))
  {
    TIM3->SR = (uint16_t)~TIM_SR_CC2IF;
    if (s_slot < BSP_PWM_CHANNELS)
    {
      s_pins[s_slot].port->BRR = s_pins[s_slot].pin;
    }

    if ((uint8_t)(s_slot + 1u) < BSP_PWM_CHANNELS)
    {
      slot_begin((uint8_t)(s_slot + 1u));
    }
    else
    {
      /* Sync tail 18000..20000 us until UIF restarts slot 0. */
      s_slot = BSP_PWM_CHANNELS;
      TIM3->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    }
  }
}
