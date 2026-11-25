/**
  ******************************************************************************
  * @file    line_track.h
  * @brief   OpenMV vision line-tracking receiver (USART3 @L frames) and
  *          mecanum chassis PD follower over ZMotorD.
  *
  *          UART3 RX line packet from OpenMV (115200 8N1, never starts with '#'):
  *            @L,<err>,<ang>,<valid>,<arm>*<XX>\n
  *              err   : -1000..1000 (<0 line left, >0 line right)
  *              ang   : -90..90 deg lookahead curve slope
  *              valid : 0=lost, 1=locked, 2=junction/wide cross-bar
  *              arm   : 0=bench/dry telemetry, 1=autonomous run enabled
  *              XX    : 2-digit hex XOR checksum over bytes between '@' and '*'
  ******************************************************************************
  */
#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRACK_RX_TIMEOUT_MS    350u
#define TRACK_CTRL_PERIOD_MS    40u
#define TRACK_SEARCH_MS        350u
#define TRACK_MOTOR_HOLD_MS    250u

typedef enum
{
  TRACK_MODE_OFF    = 0u,
  TRACK_MODE_ACTIVE = 1u,
  TRACK_MODE_AUTO   = 2u
} track_mode_t;

typedef enum
{
  TRACK_ST_IDLE = 0u,
  TRACK_ST_FOLLOW,
  TRACK_ST_JUNCTION,
  TRACK_ST_SEARCH,
  TRACK_ST_LOST
} track_state_t;

typedef struct
{
  int16_t  vx_cruise;
  int16_t  vx_min;
  uint16_t kp;
  uint16_t kd;
  uint16_t k_ang;
  uint16_t wz_max;
  uint16_t search_wz;
} track_cfg_t;

typedef struct
{
  uint8_t  mode;
  uint8_t  dry_run;
  uint8_t  state;
  uint8_t  valid;
  int16_t  err;
  int16_t  ang;
  int16_t  vx_cmd;
  int16_t  wz_cmd;
  uint16_t age_ms;
  uint16_t rx_frames;
} track_status_t;

void               TRACK_Init(void);
void               TRACK_Poll(void);

/* Called by shared UART3 parser after '@L,' and XOR checksum succeed. */
void               TRACK_OnFrame(const char *fields, uint32_t now_ms);

int                TRACK_SetMode(uint8_t mode, uint8_t dry_run);
void               TRACK_ForceOff(void);
int                TRACK_SetConfig(const track_cfg_t *cfg);
const track_cfg_t *TRACK_GetConfig(void);
void               TRACK_GetStatus(track_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACK_H */
