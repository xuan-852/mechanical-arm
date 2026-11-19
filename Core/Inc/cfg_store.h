/**
  ******************************************************************************
  * @file    cfg_store.h
  * @brief   Per-joint calibration persistence in the last flash page.
  *
  *          Why a separate page: the calibration blob must survive power cycles
  *          but must never sit in the same page as code, otherwise erasing it
  *          would take the firmware with it. STM32F103C8 has 64 KiB of flash in
  *          1 KiB pages; the last page starts at 0x0800FC00.
  *
  *          Blob layout (74 bytes, CRC8 over everything before the CRC):
  *            [0..3]   magic   'R','B','O','T'
  *            [4..5]   version
  *            [6]      joint count
  *            [7..72]  6 x 11 bytes, per joint:
  *                       min(i16) max(i16) home(i16)
  *                       pulse_min(i16) pulse_max(i16) invert(u8)
  *            [73]     CRC8
  *
  *          All multi-byte fields are little endian. The blob is serialised
  *          byte by byte rather than memcpy'd from a struct, so compiler
  *          padding can never change the on-flash format.
  ******************************************************************************
  */
#ifndef __CFG_STORE_H
#define __CFG_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "servo.h"

#define CFG_PAGE_ADDR   0x0800FC00u
#define CFG_PAGE_SIZE   1024u
#define CFG_BLOB_BYTES  74u
#define CFG_VERSION     1u

/* Serialise / deserialise. Pure functions, no hardware -- used by the tests as
   well as by Save/Load. CFG_Encode returns the number of bytes written. */
uint8_t CFG_Encode(uint8_t *buf, const servo_cfg_t cfg[SERVO_NUM]);
uint8_t CFG_Decode(const uint8_t *buf, servo_cfg_t cfg[SERVO_NUM]);

/* A calibration is only accepted if it is mechanically plausible. */
uint8_t CFG_IsValid(const servo_cfg_t *c);

/* Flash I/O. Return 0 on success. */
uint8_t CFG_Save(const servo_cfg_t cfg[SERVO_NUM]);
uint8_t CFG_Load(servo_cfg_t cfg[SERVO_NUM]);
uint8_t CFG_Erase(void);

#ifdef __cplusplus
}
#endif

#endif /* __CFG_STORE_H */
