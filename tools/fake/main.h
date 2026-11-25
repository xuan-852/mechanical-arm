/* Host-side doubles for the firmware unit test.
   Placed first on the include path so the real main.h / usart.h are shadowed
   and no STM32 HAL or CMSIS is required. */
#ifndef __FAKE_MAIN_H
#define __FAKE_MAIN_H

#include <stdint.h>

#define HAL_OK                      0u
#define FLASH_TYPEERASE_PAGES       0u
#define FLASH_TYPEPROGRAM_HALFWORD  0u

#define GPIO_PIN_8                  ((uint16_t)0x0100u)
#define GPIO_PIN_RESET              0u
#define GPIO_PIN_SET                1u

typedef struct
{
  uint8_t dummy;
} GPIO_TypeDef;

extern GPIO_TypeDef g_fake_gpioa;
#define GPIOA (&g_fake_gpioa)

uint8_t  HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

typedef struct
{
  uint32_t TypeErase;
  uint32_t PageAddress;
  uint32_t NbPages;
} FLASH_EraseInitTypeDef;

void     HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
uint32_t HAL_FLASH_Unlock(void);
uint32_t HAL_FLASH_Lock(void);
uint32_t HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
uint32_t HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

const uint8_t *cfg_fake_flash_ptr(uint32_t offset);
#define CFG_FLASH_PTR(off)  cfg_fake_flash_ptr(off)

/* CMSIS PRIMASK intrinsics, stubbed: the host build has no interrupt gate. */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void     __disable_irq(void) { }
static inline void     __set_PRIMASK(uint32_t primask) { (void)primask; }

#endif /* __FAKE_MAIN_H */
