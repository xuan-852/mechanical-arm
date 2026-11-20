/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   Host link protocol: framing, CRC and command dispatch.
  *
  *          Frame layout (binary)
  *          ----------------------
  *            offset  size  field
  *              0      1    0xAA  (header 1)
  *              1      1    0x55  (header 2)
  *              2      1    LEN   = payload length (0..20)
  *              3      1    CMD
  *              4     LEN   PAYLOAD
  *            4+LEN    1    CRC8 (poly 0x31, init 0xFF) over CMD..PAYLOAD
  *
  *          Every command except telemetry is answered with ACK (0x80) or
  *          NAK (0x82) carrying the original CMD and a status code.
  ******************************************************************************
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "servo.h"

#define PROTO_HEAD1        0xAAu
#define PROTO_HEAD2        0x55u
#define PROTO_MAX_PAYLOAD  20u

/* Host -> robot */
#define CMD_SET_ANGLE      0x01u   /* payload: 6 x int16 (0.1 deg)        */
#define CMD_SET_TIME       0x02u   /* payload: uint16 ms (0 = auto)       */
#define CMD_READ_POSE      0x03u   /* no payload                          */
#define CMD_HOME           0x04u   /* no payload                          */
#define CMD_ESTOP          0x05u   /* no payload                          */
#define CMD_SET_SPEED      0x06u   /* payload: uint8 percent              */
#define CMD_CLEAR_ESTOP    0x07u   /* no payload                          */
#define CMD_SELFTEST       0x08u   /* no payload: sweep each joint in turn */
#define CMD_STOP           0x09u   /* no payload: hold position, cancel segment */
#define CMD_SET_JOINT_CFG  0x0Au   /* joint, min, max, home, pmin, pmax, invert */
#define CMD_GET_JOINT_CFG  0x0Bu   /* payload: joint -> CMD_JOINT_CFG          */
#define CMD_SAVE_CFG       0x0Cu   /* no payload: persist calibration to flash */
#define CMD_RESET_CFG      0x0Du   /* no payload: erase stored calibration     */
#define CMD_DRIVE_BASE     0x0Eu   /* payload: vx, vy, wz (i16), time_ms (u16) */
#define CMD_DRIVE_WHEELS   0x0Fu   /* payload: fl, fr, rl, rr (i16), time (u16)*/
#define CMD_TRACK_MODE     0x10u   /* payload: mode(u8), dry_run(u8)           */
#define CMD_TRACK_CFG      0x11u   /* payload: 7 x u16/i16 track tuning (14B)  */
#define CMD_READ_TRACK     0x12u   /* no payload -> CMD_TRACK_STAT             */
#define CMD_PICK_MODE      0x13u   /* payload: mode(u8), dry_run(u8)           */
#define CMD_READ_PICK      0x14u   /* no payload -> CMD_PICK_STAT              */

/* Robot -> host */
#define CMD_ACK            0x80u   /* payload: cmd, status                */
#define CMD_POSE           0x81u   /* payload: 6 x int16 (0.1 deg)        */
#define CMD_NAK            0x82u   /* payload: cmd, status                */
#define CMD_JOINT_CFG      0x83u   /* payload: joint + cfg (12 bytes)     */
#define CMD_TRACK_STAT     0x84u   /* payload: track_status_t (16 bytes)  */
#define CMD_PICK_STAT      0x85u   /* payload: pick_status_t (18 bytes)   */

/* Status codes */
#define ST_OK              0x00u
#define ST_BAD_CRC         0x01u
#define ST_BAD_LEN         0x02u
#define ST_BAD_CMD         0x03u
#define ST_BUSY            0x04u
#define ST_ESTOP           0x05u
#define ST_LIMIT           0x06u
#define ST_FLASH           0x07u

void     PROTO_Init(void);

/* Drain the UART and execute whatever complete frames are available.
   Call from the main loop, never from an ISR. */
void     PROTO_Poll(void);

uint8_t  PROTO_Crc8(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
