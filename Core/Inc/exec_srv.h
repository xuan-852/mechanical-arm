/**
  ******************************************************************************
  * @file    exec_srv.h
  * @brief   Competition-frozen STM32 execution service (UART3 OpenMV bridge,
  *          PA8 Key1 one-button launch/abort, safe chassis watchdog, and
  *          parameterized non-blocking arm sequences).
  *
  *          UART3 RX commands from OpenMV (115200 8N1, '@<TAG>,<args>*<XX>\n'):
  *            @V,<vx>,<vy>,<wz>[,<ms>]*XX\n
  *              Drive mecanum chassis. ms=0 uses streaming 280ms watchdog;
  *              ms>0 runs timed pulse (50..3000ms) with clean brake.
  *            @A,<op>[,<j0>,<j1>,<j5>]*XX\n
  *              Parameterized arm sequence (-1 keeps default/current):
  *                op=0 : return to upright open HOME (j5=90.0 deg)
  *                op=1 : 7-step floor PICK (custom j0_aim, j1_grab, j5_clamp)
  *                op=2 : 7-step target PLACE (custom j0_aim, j1_place, j5_open)
  *                op=3 : return to upright HOME preserving/setting j5
  *                op=4 : move gripper j5 only at current pose
  *            @J,<j0>,<j1>,<j2>,<j3>,<j4>,<j5>,<dur_ms>*XX\n
  *              Move all 6 joints directly (-1 keeps joint current angle),
  *              subject to SERVO_Clamp limits and speed ceiling.
  *            @S,<subcmd>[,<val>]*XX\n
  *              sub=0 : stop wheels and current arm motion
  *              sub=1 : emit immediate $E telemetry reply
  *              sub=2 : set global arm speed override (val=15..100%)
  *
  *          Legacy frames handled by central UART3 parser when explicitly armed:
  *            @L,...*XX\n -> TRACK_OnFrame (isolated unless TRACK_SetMode != OFF)
  *            @O,...*XX\n -> PICK_OnFrame  (isolated unless PICK_SetMode != OFF)
  *
  *          UART3 TX telemetry to OpenMV (never uses '#' or '!' so ZMotorD ignores):
  *            $E,<armed>,<arm_st>,<arm_busy>,<holding>,<robot_st>,<arm_seq>*<XX>\n
  ******************************************************************************
  */
#ifndef __EXEC_SRV_H
#define __EXEC_SRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define EXEC_KEY_DEBOUNCE_MS    40u
#define EXEC_DRIVE_WDG_MS      280u
#define EXEC_DRIVE_HOLD_MS     260u
#define EXEC_BUS_MIN_PERIOD_MS  35u
#define EXEC_TELEM_PERIOD_MS   100u
#define EXEC_MAX_SPEED         950

typedef enum
{
  EXEC_MODE_OFF    = 0u,
  EXEC_MODE_ACTIVE = 1u
} exec_mode_t;

typedef enum
{
  EXEC_ARM_IDLE = 0u,
  EXEC_ARM_WAKE,
  EXEC_ARM_PICK_PRE,
  EXEC_ARM_PICK_ABOVE,
  EXEC_ARM_PICK_GRAB,
  EXEC_ARM_PICK_CLAMP,
  EXEC_ARM_PICK_LIFT,
  EXEC_ARM_PICK_RETRACT,
  EXEC_ARM_PICK_HOME,
  EXEC_ARM_PLACE_PRE,
  EXEC_ARM_PLACE_ABOVE,
  EXEC_ARM_PLACE_LOWER,
  EXEC_ARM_PLACE_RELEASE,
  EXEC_ARM_PLACE_LIFT,
  EXEC_ARM_PLACE_RETRACT,
  EXEC_ARM_PLACE_HOME,
  EXEC_ARM_CUSTOM
} exec_arm_st_t;

typedef enum
{
  EXEC_OP_HOME_OPEN = 0u,
  EXEC_OP_PICK      = 1u,
  EXEC_OP_PLACE     = 2u,
  EXEC_OP_HOLD_HOME = 3u,
  EXEC_OP_GRIP_ONLY = 4u
} exec_arm_op_t;

typedef struct
{
  uint8_t  armed;
  uint8_t  dry_run;
  uint8_t  arm_state;
  uint8_t  arm_busy;
  uint8_t  holding;
  uint8_t  btn1_down;
  int16_t  vx_cmd;
  int16_t  vy_cmd;
  int16_t  wz_cmd;
  uint16_t arm_seq;
  uint16_t age_ms;
  uint16_t rx_frames;
  uint16_t rx_bad;
} exec_status_t;

void    EXEC_Init(void);
void    EXEC_Poll(void);

/* Drain any pending UART3 RX bytes into shared frame router (@V/@A/@J/@S/@L/@O). */
void    EXEC_PumpUartRx(uint32_t now_ms);

int     EXEC_SetMode(uint8_t mode, uint8_t dry_run);
void    EXEC_ForceOff(void);
uint8_t EXEC_IsArmed(void);
void    EXEC_GetStatus(exec_status_t *out);

/* Shared fixed-point / UART ASCII helpers reused by line_track & cube_pick. */
int32_t EXEC_Abs32(int32_t v);
int16_t EXEC_ClampS16(int32_t v, int32_t lo, int32_t hi);
int     EXEC_ParseI32(const char **p, int32_t *out);
int     EXEC_ParseCsvI32(const char *s, int32_t *vals, uint8_t min_cnt,
                         uint8_t max_cnt, uint8_t *out_cnt);

#ifdef __cplusplus
}
#endif

#endif /* __EXEC_SRV_H */
