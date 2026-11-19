/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART1 driver: interrupt-driven RX ring buffer + blocking TX.
  ******************************************************************************
  */
#include "usart.h"
#include "main.h"

UART_HandleTypeDef huart1;

/* --- RX ring buffer ------------------------------------------------------ *
 * Bounded so a stuck host can never overflow RAM; when full we drop bytes
 * (the CRC will reject the truncated frame). 128 bytes = ~11ms at 115200. */
#define UART_RX_SIZE   256u

static volatile uint8_t  s_rx_buf[UART_RX_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t           s_rx_byte;

void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  s_rx_head = 0u;
  s_rx_tail = 0u;

  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance != USART1)
  {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA9  -> USART1_TX (AF push-pull)
     PA10 -> USART1_RX (floating input) */
  GPIO_InitStruct.Pin   = GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* NVIC gate. Without this the RXNE interrupt stays masked no matter what
     HAL_UART_Receive_IT arms in CR1, so the receive callback never runs and
     PROTO_Poll() never sees a single byte (the board is deaf).

     Priority 1 preempts the planner tick (SysTick, priority 15) so a slow
     ROBOT_Tick cannot cause RX overrun at 115200 baud. */
  HAL_NVIC_SetPriority(USART1_IRQn, 1u, 0u);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void UART_WriteByte(uint8_t byte)
{
  (void)HAL_UART_Transmit(&huart1, &byte, 1u, 10u);
}

void UART_WriteBuf(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0u))
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100u);
}

/* --- RX plumbing --------------------------------------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next = (uint16_t)((s_rx_head + 1u) % UART_RX_SIZE);

    if (next != s_rx_tail)   /* drop on overflow rather than overwrite */
    {
      s_rx_buf[s_rx_head] = s_rx_byte;
      s_rx_head = next;
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
  }
}

int UART_RxAvailable(void)
{
  return (s_rx_head != s_rx_tail) ? 1 : 0;
}

int UART_ReadByte(uint8_t *out)
{
  if ((out == NULL) || (s_rx_head == s_rx_tail))
  {
    return -1;
  }

  *out = s_rx_buf[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) % UART_RX_SIZE);

  return 0;
}
