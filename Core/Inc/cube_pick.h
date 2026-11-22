/**
  ******************************************************************************
  * @file    cube_pick.h
  * @brief   OpenMV @O cube detection receiver, mecanum pulse-jog aligner, and
  *          non-blocking 7-step arm pick-up controller.
  *
  *          UART3 RX cube packet from OpenMV (115200 8N1, never starts with '#'):
  *            @O,<dx>,<dy>,<w>,<valid>,<arm>*<XX>\n
  *              dx    : -1000..1000 (<0 cube left of gripper, >0 right)
  *              dy    : -1000..1000 (>0 cube too far ahead, <0 too close)
  *              w     : 0..320 pixel width estimate
  *              valid : 0=lost, 1=dark cube blob, 2=AprilTag verified
  *              arm   : 0=bench/dry telemetry, 1=autonomous find+pick enabled
  *              XX    : 2-digit hex XOR checksum over bytes between '@' and '*'
  ******************************************************************************
  */
#ifndef __CUBE_PICK_H
#define __CUBE_PICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PICK_RX_TIMEOUT_MS       450u
#define PICK_SETTLE_MS           170u
#define PICK_SEARCH_TIMEOUT_MS 18000u
#define PICK_ACTIVE_TIMEOUT_MS 30000u
#define PICK_TOL_DX               65
#define PICK_TOL_DY               45
#define PICK_APPROACH_DY         200
#define PICK_MIN_GRAB_W           24
#define PICK_CONFIRM_FRAMES        3u

typedef enum
{
  PICK_MODE_OFF    = 0u,
  PICK_MODE_ACTIVE = 1u,
  PICK_MODE_AUTO   = 2u,
  PICK_MODE_RESET  = 3u
} pick_mode_t;

typedef enum
{
  PICK_ST_IDLE = 0u,
  PICK_ST_SEARCH,
  PICK_ST_APPROACH,
  PICK_ST_ALIGN,
  PICK_ST_ARM_WAKE,
  PICK_ST_ARM_PRE,
  PICK_ST_ARM_ABOVE,
  PICK_ST_ARM_GRAB,
  PICK_ST_ARM_CLAMP,
  PICK_ST_ARM_LIFT,
  PICK_ST_ARM_RETRACT,
  PICK_ST_ARM_HOME,
  PICK_ST_HOLDING,
  PICK_ST_FAIL
} pick_state_t;

typedef struct
{
  uint8_t  mode;
  uint8_t  dry_run;
  uint8_t  state;
  uint8_t  valid;
  int16_t  dx;
  int16_t  dy;
  int16_t  cube_w;
  int16_t  vx_cmd;
  int16_t  wy_cmd;   /* vy during ALIGN strafe, wz during SEARCH/APPROACH */
  uint16_t age_ms;
  uint16_t rx_frames;
} pick_status_t;

void    PICK_Init(void);
void    PICK_Poll(void);

/* Called by UART3 parser after '@'..'O,' and XOR checksum succeed. */
void    PICK_OnFrame(const char *fields, uint32_t now_ms);

int     PICK_SetMode(uint8_t mode, uint8_t dry_run);
void    PICK_ForceOff(void);
uint8_t PICK_IsBusy(void);
void    PICK_GetStatus(pick_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CUBE_PICK_H */
