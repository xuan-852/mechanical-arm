/**
  ******************************************************************************
  * @file    cfg_store.c
  * @brief   Per-joint calibration persistence in the last flash page.
  ******************************************************************************
  */
#include "cfg_store.h"
#include "bsp_pwm.h"
#include "protocol.h"
#include "main.h"

#include <string.h>

#ifndef CFG_FLASH_PTR
#define CFG_FLASH_PTR(off)  ((const uint8_t *)(uintptr_t)(CFG_PAGE_ADDR + (off)))
#endif

/* --- little-endian helpers ----------------------------------------------- */

static void put_i16(uint8_t *p, int16_t v)
{
  uint16_t u = (uint16_t)v;

  p[0] = (uint8_t)(u & 0xFFu);
  p[1] = (uint8_t)((u >> 8) & 0xFFu);
}

static int16_t get_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* --- validation ---------------------------------------------------------- */

uint8_t CFG_IsValid(const servo_cfg_t *c)
{
  if (c == NULL) { return 0u; }

  if (c->min_deg < 0)                          { return 0u; }
  if (c->max_deg > 1800)                       { return 0u; }
  if (c->min_deg >= c->max_deg)                { return 0u; }
  if (c->home_deg < c->min_deg)                { return 0u; }
  if (c->home_deg > c->max_deg)                { return 0u; }
  if (c->pulse_min_us < (int16_t)BSP_PWM_MIN_PULSE) { return 0u; }
  if (c->pulse_max_us > (int16_t)BSP_PWM_MAX_PULSE) { return 0u; }
  if (c->pulse_min_us >= c->pulse_max_us)      { return 0u; }
  if (c->invert > 1u)                          { return 0u; }

  return 1u;
}

/* --- serialisation ------------------------------------------------------- */

uint8_t CFG_Encode(uint8_t *buf, const servo_cfg_t cfg[SERVO_NUM])
{
  uint8_t n = 0u;

  buf[n++] = (uint8_t)'R';
  buf[n++] = (uint8_t)'B';
  buf[n++] = (uint8_t)'O';
  buf[n++] = (uint8_t)'T';

  buf[n++] = (uint8_t)(CFG_VERSION & 0xFFu);
  buf[n++] = (uint8_t)((CFG_VERSION >> 8) & 0xFFu);

  buf[n++] = (uint8_t)SERVO_NUM;

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    put_i16(&buf[n], cfg[i].min_deg);       n = (uint8_t)(n + 2u);
    put_i16(&buf[n], cfg[i].max_deg);       n = (uint8_t)(n + 2u);
    put_i16(&buf[n], cfg[i].home_deg);      n = (uint8_t)(n + 2u);
    put_i16(&buf[n], cfg[i].pulse_min_us);  n = (uint8_t)(n + 2u);
    put_i16(&buf[n], cfg[i].pulse_max_us);  n = (uint8_t)(n + 2u);
    buf[n++] = cfg[i].invert;
  }

  buf[n] = PROTO_Crc8(buf, n);
  n = (uint8_t)(n + 1u);

  return n;
}

uint8_t CFG_Decode(const uint8_t *buf, servo_cfg_t cfg[SERVO_NUM])
{
  uint8_t n = 0u;

  if (memcmp(buf, "RBOT", 4u) != 0) { return 1u; }
  n = 4u;

  if ((uint16_t)(buf[n] | ((uint16_t)buf[n + 1u] << 8)) != CFG_VERSION)
  {
    return 2u;
  }
  n = (uint8_t)(n + 2u);

  if (buf[n] != (uint8_t)SERVO_NUM) { return 3u; }
  n = (uint8_t)(n + 1u);

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    servo_cfg_t tmp;

    tmp.min_deg      = get_i16(&buf[n]); n = (uint8_t)(n + 2u);
    tmp.max_deg      = get_i16(&buf[n]); n = (uint8_t)(n + 2u);
    tmp.home_deg     = get_i16(&buf[n]); n = (uint8_t)(n + 2u);
    tmp.pulse_min_us = get_i16(&buf[n]); n = (uint8_t)(n + 2u);
    tmp.pulse_max_us = get_i16(&buf[n]); n = (uint8_t)(n + 2u);
    tmp.invert       = buf[n];           n = (uint8_t)(n + 1u);

    if (CFG_IsValid(&tmp) == 0u) { return 4u; }

    cfg[i] = tmp;
  }

  if (PROTO_Crc8(buf, n) != buf[n]) { return 5u; }

  return 0u;
}

/* --- flash I/O ----------------------------------------------------------- */

uint8_t CFG_Save(const servo_cfg_t cfg[SERVO_NUM])
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0u;
  uint8_t  buf[CFG_BLOB_BYTES];
  uint8_t  len;
  uint8_t  i;

  for (i = 0u; i < SERVO_NUM; i++)
  {
    if (CFG_IsValid(&cfg[i]) == 0u) { return 1u; }
  }

  len = CFG_Encode(buf, cfg);

  /* HAL_FLASH_Program works in half-words, so the blob must be even sized.
     It is 74 bytes by construction; this guards the future. */
  if (((len & 1u) != 0u) || (len > CFG_BLOB_BYTES)) { return 2u; }

  if (HAL_FLASH_Unlock() != HAL_OK) { return 3u; }

  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = CFG_PAGE_ADDR;
  erase.NbPages     = 1u;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return 4u;
  }

  for (i = 0u; i < len; i = (uint8_t)(i + 2u))
  {
    uint16_t halfword = (uint16_t)((uint16_t)buf[i] |
                                   ((uint16_t)buf[i + 1u] << 8));

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          CFG_PAGE_ADDR + i,
                          (uint64_t)halfword) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return 5u;
    }
  }

  (void)HAL_FLASH_Lock();

  /* Read back and compare. A marginal supply fails here, not silently at the
     next boot, which is where a bad calibration would actually hurt. */
  for (i = 0u; i < len; i++)
  {
    if (*CFG_FLASH_PTR(i) != buf[i]) { return 6u; }
  }

  return 0u;
}

uint8_t CFG_Load(servo_cfg_t cfg[SERVO_NUM])
{
  return CFG_Decode(CFG_FLASH_PTR(0u), cfg);
}

uint8_t CFG_Erase(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0u;

  if (HAL_FLASH_Unlock() != HAL_OK) { return 3u; }

  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = CFG_PAGE_ADDR;
  erase.NbPages     = 1u;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return 4u;
  }

  (void)HAL_FLASH_Lock();

  return 0u;
}
