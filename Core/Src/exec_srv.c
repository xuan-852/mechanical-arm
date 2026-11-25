/**
  ******************************************************************************
  * @file    exec_srv.c
  * @brief   Frozen STM32 execution service: UART3 @V/@A/@J/@S router,
  *          PA8 Key1 one-button launch/abort, chassis RX watchdog, and
  *          data-driven non-blocking arm pick/place engine.
  ******************************************************************************
  */
#include "exec_srv.h"
#include "line_track.h"
#include "cube_pick.h"
#include "zmotord.h"
#include "robot.h"
#include "servo.h"
#include "bsp_pwm.h"
#include "main.h"

#include <stddef.h>

#define EXEC_LINE_BUF_LEN   64u
#define EXEC_DRIVE_MIN_MS   50u
#define EXEC_DRIVE_MAX_MS 3000u

typedef enum
{
  POSE_HOME = 0u,
  POSE_PRE,
  POSE_ABOVE,
  POSE_GRAB
} pose_id_t;

typedef enum
{
  J0_FIXED_450 = 0u,
  J0_ARG
} j0_sel_t;

typedef enum
{
  J1_POSE = 0u,
  J1_ARG
} j1_sel_t;

typedef enum
{
  J5_OPEN_900 = 0u,
  J5_ARG,
  J5_CARRY
} j5_sel_t;

typedef struct
{
  uint8_t  pose_id;
  uint8_t  j0_sel;
  uint8_t  j1_sel;
  uint8_t  j5_sel;
  uint16_t dur_ms;
  uint16_t dwell_ms;
  uint8_t  arm_st;
} arm_step_t;

/* Hardware-verified reference poses (0.1 deg, see tools/grab_poses.json). */
static const int16_t s_poses[4][SERVO_NUM] =
{
  { 450,  918, 900,  990, 1035, 900 }, /* POSE_HOME  */
  { 450, 1120, 820, 1120, 1800, 900 }, /* POSE_PRE   */
  { 450, 1520, 759, 1130, 1800, 900 }, /* POSE_ABOVE */
  { 450, 1716, 759, 1130, 1800, 900 }  /* POSE_GRAB  */
};

static const arm_step_t s_pick_steps[7] =
{
  { POSE_PRE,   J0_ARG,       J1_POSE, J5_OPEN_900, 750u,  50u, EXEC_ARM_PICK_PRE     },
  { POSE_ABOVE, J0_ARG,       J1_POSE, J5_OPEN_900, 700u,  50u, EXEC_ARM_PICK_ABOVE   },
  { POSE_GRAB,  J0_ARG,       J1_ARG,  J5_OPEN_900, 650u,  50u, EXEC_ARM_PICK_GRAB    },
  { POSE_GRAB,  J0_ARG,       J1_ARG,  J5_ARG,      550u, 250u, EXEC_ARM_PICK_CLAMP   },
  { POSE_ABOVE, J0_ARG,       J1_POSE, J5_ARG,      650u,  50u, EXEC_ARM_PICK_LIFT    },
  { POSE_PRE,   J0_FIXED_450, J1_POSE, J5_ARG,      750u,  50u, EXEC_ARM_PICK_RETRACT },
  { POSE_HOME,  J0_FIXED_450, J1_POSE, J5_ARG,      800u,  40u, EXEC_ARM_PICK_HOME    }
};

static const arm_step_t s_place_steps[7] =
{
  { POSE_PRE,   J0_ARG,       J1_POSE, J5_CARRY,    750u,  50u, EXEC_ARM_PLACE_PRE     },
  { POSE_ABOVE, J0_ARG,       J1_POSE, J5_CARRY,    700u,  50u, EXEC_ARM_PLACE_ABOVE   },
  { POSE_GRAB,  J0_ARG,       J1_ARG,  J5_CARRY,    650u, 100u, EXEC_ARM_PLACE_LOWER   },
  { POSE_GRAB,  J0_ARG,       J1_ARG,  J5_ARG,      500u, 150u, EXEC_ARM_PLACE_RELEASE },
  { POSE_ABOVE, J0_ARG,       J1_POSE, J5_ARG,      650u,  50u, EXEC_ARM_PLACE_LIFT    },
  { POSE_PRE,   J0_FIXED_450, J1_POSE, J5_ARG,      750u,  50u, EXEC_ARM_PLACE_RETRACT },
  { POSE_HOME,  J0_FIXED_450, J1_POSE, J5_ARG,      800u,  40u, EXEC_ARM_PLACE_HOME    }
};

/* UART3 ASCII RX frame state */
static char     s_line[EXEC_LINE_BUF_LEN];
static uint8_t  s_line_len;
static uint8_t  s_in_frame;

/* Key1 (PA8) debounced input state */
static uint8_t  s_key_raw;
static uint8_t  s_key_stable;
static uint32_t s_key_change_ms;

/* Execution state */
static uint8_t  s_armed;
static uint8_t  s_dry_run;
static uint8_t  s_holding;
static int16_t  s_hold_j5;
static uint16_t s_arm_seq;
static uint16_t s_rx_frames;
static uint16_t s_rx_bad;
static uint32_t s_last_rx_ms;

/* Chassis drive watchdog & bus throttle */
static int16_t  s_vx_cmd;
static int16_t  s_vy_cmd;
static int16_t  s_wz_cmd;
static int16_t  s_last_tx_vx;
static int16_t  s_last_tx_vy;
static int16_t  s_last_tx_wz;
static uint8_t  s_driving;
static uint8_t  s_was_driving;
static uint32_t s_drive_deadline_ms;
static uint32_t s_last_zm_ms;

/* Non-blocking arm step engine */
static uint8_t           s_arm_st;
static const arm_step_t *s_seq_tbl;
static uint8_t           s_seq_len;
static uint8_t           s_seq_idx;
static uint8_t           s_seq_is_pick;
static uint8_t           s_seq_updates_hold;
static uint8_t           s_arm_dwell;
static uint32_t          s_arm_dwell_ms;
static int16_t           s_arg_j0;
static int16_t           s_arg_j1;
static int16_t           s_arg_j5;
static int16_t           s_carry_j5;
static int16_t           s_wake_tgt[SERVO_NUM];
static uint16_t          s_wake_dur;
static uint8_t           s_wake_resume_seq;
static uint8_t           s_wake_next_st;

/* Telemetry state */
static uint8_t  s_telem_dirty;
static uint32_t s_last_telem_ms;
static uint8_t  s_prev_armed;
static uint8_t  s_prev_busy;
static uint8_t  s_prev_hold;
static uint16_t s_prev_seq;

/* --- shared helpers ------------------------------------------------------ */

int32_t EXEC_Abs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

int16_t EXEC_ClampS16(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) { return (int16_t)lo; }
  if (v > hi) { return (int16_t)hi; }
  return (int16_t)v;
}

static int hex_nibble(char c)
{
  if ((c >= '0') && (c <= '9')) { return (int)(c - '0'); }
  if ((c >= 'A') && (c <= 'F')) { return (int)(c - 'A' + 10); }
  if ((c >= 'a') && (c <= 'f')) { return (int)(c - 'a' + 10); }
  return -1;
}

int EXEC_ParseI32(const char **p, int32_t *out)
{
  const char *s;
  int32_t sign = 1;
  int32_t val = 0;
  uint8_t digits = 0u;

  if ((p == NULL) || (*p == NULL) || (out == NULL)) { return -1; }
  s = *p;

  while (*s == ' ') { s++; }

  if (*s == '-')
  {
    sign = -1;
    s++;
  }
  else if (*s == '+')
  {
    s++;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    if (val < 100000)
    {
      val = (val * 10) + (int32_t)(*s - '0');
    }
    digits++;
    s++;
  }

  if (digits == 0u) { return -1; }

  *out = val * sign;
  *p = s;
  return 0;
}

int EXEC_ParseCsvI32(const char *s, int32_t *vals, uint8_t min_cnt,
                     uint8_t max_cnt, uint8_t *out_cnt)
{
  uint8_t n = 0u;

  if ((s == NULL) || (vals == NULL) || (min_cnt == 0u) || (max_cnt < min_cnt))
  {
    return -1;
  }

  while (n < max_cnt)
  {
    if (EXEC_ParseI32(&s, &vals[n]) != 0)
    {
      return -1;
    }
    n++;

    while (*s == ' ') { s++; }
    if (*s == '\0')
    {
      break;
    }
    if (*s != ',')
    {
      return -1;
    }
    s++;
  }

  if ((*s != '\0') || (n < min_cnt))
  {
    return -1;
  }

  if (out_cnt != NULL)
  {
    *out_cnt = n;
  }
  return 0;
}

/* --- low-level actuator helpers ------------------------------------------ */

static void stop_wheels(void)
{
  if (s_was_driving != 0u)
  {
    ZM_StopAll();
    s_was_driving = 0u;
  }
  s_driving    = 0u;
  s_vx_cmd     = 0;
  s_vy_cmd     = 0;
  s_wz_cmd     = 0;
  s_last_tx_vx = 0;
  s_last_tx_vy = 0;
  s_last_tx_wz = 0;
}

static void cancel_arm_seq(void)
{
  s_arm_st          = (uint8_t)EXEC_ARM_IDLE;
  s_seq_tbl         = NULL;
  s_seq_len         = 0u;
  s_seq_idx         = 0u;
  s_arm_dwell       = 0u;
  s_wake_resume_seq = 0u;
  s_wake_next_st    = (uint8_t)EXEC_ARM_IDLE;
}

static uint8_t is_arm_busy_now(void)
{
  if ((s_arm_st != (uint8_t)EXEC_ARM_IDLE) || (ROBOT_IsBusy() != 0u))
  {
    return 1u;
  }
  return 0u;
}

static uint8_t any_servo_relaxed(void)
{
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    if (BSP_PWM_GetPulse(SERVO_ChannelOf(i)) == 0u)
    {
      return 1u;
    }
  }
  return 0u;
}

/* Staggered wake so relaxed 0us servos engage without a 6-servo surge. */
static void wake_servos_staggered(int16_t j5_deg)
{
  static const uint8_t wake_order[SERVO_NUM] = { 1u, 2u, 3u, 0u, 4u, 5u };

  if (s_dry_run != 0u)
  {
    return;
  }

  for (uint8_t k = 0u; k < SERVO_NUM; k++)
  {
    uint8_t j = wake_order[k];
    if (BSP_PWM_GetPulse(SERVO_ChannelOf(j)) == 0u)
    {
      int16_t deg = ((j == 5u) && (j5_deg >= 0)) ? j5_deg : SERVO_GetConfig(j)->home_deg;
      SERVO_WriteAngle(j, deg);
      HAL_Delay(60u);
    }
  }
}

static void launch_arm_target(const int16_t tgt[SERVO_NUM], uint16_t dur_ms,
                              uint8_t resume_seq, uint8_t target_st)
{
  if (s_dry_run != 0u)
  {
    s_arm_st = (uint8_t)EXEC_ARM_IDLE;
    if ((s_seq_tbl != NULL) && (s_seq_updates_hold != 0u))
    {
      s_holding = s_seq_is_pick;
      s_hold_j5 = s_arg_j5;
    }
    s_seq_tbl = NULL;
    return;
  }

  if (any_servo_relaxed() != 0u)
  {
    for (uint8_t i = 0u; i < SERVO_NUM; i++)
    {
      s_wake_tgt[i] = tgt[i];
    }
    s_wake_dur        = dur_ms;
    s_wake_resume_seq = resume_seq;
    s_wake_next_st    = target_st;
    s_arm_st          = (uint8_t)EXEC_ARM_WAKE;
    wake_servos_staggered(tgt[5]);
    {
      int16_t home[SERVO_NUM];
      for (uint8_t i = 0u; i < SERVO_NUM; i++)
      {
        home[i] = SERVO_GetConfig(i)->home_deg;
      }
      if (tgt[5] >= 900)
      {
        home[5] = tgt[5];
      }
      (void)ROBOT_MoveToAuto(home, 0u);
    }
    return;
  }

  s_arm_st = (target_st == (uint8_t)EXEC_ARM_IDLE) ? (uint8_t)EXEC_ARM_CUSTOM : target_st;
  (void)ROBOT_MoveToAuto(tgt, dur_ms);
}

static void start_seq_step(uint8_t idx)
{
  const arm_step_t *st = &s_seq_tbl[idx];
  int16_t tgt[SERVO_NUM];

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    tgt[i] = s_poses[st->pose_id][i];
  }

  if (st->j0_sel == (uint8_t)J0_ARG)
  {
    tgt[0] = s_arg_j0;
  }
  if (st->j1_sel == (uint8_t)J1_ARG)
  {
    tgt[1] = s_arg_j1;
  }
  if (st->j5_sel == (uint8_t)J5_ARG)
  {
    tgt[5] = s_arg_j5;
  }
  else if (st->j5_sel == (uint8_t)J5_CARRY)
  {
    tgt[5] = s_carry_j5;
  }

  s_arm_dwell = 0u;
  launch_arm_target(tgt, st->dur_ms, 1u, st->arm_st);
}

static void step_arm_engine(uint32_t now_ms)
{
  if (s_arm_st == (uint8_t)EXEC_ARM_IDLE)
  {
    return;
  }

  if (ROBOT_IsBusy() != 0u)
  {
    return;
  }

  if (s_arm_st == (uint8_t)EXEC_ARM_WAKE)
  {
    if (s_wake_resume_seq != 0u)
    {
      s_wake_resume_seq = 0u;
      start_seq_step(s_seq_idx);
    }
    else if (s_wake_next_st != (uint8_t)EXEC_ARM_IDLE)
    {
      s_wake_next_st = (uint8_t)EXEC_ARM_IDLE;
      s_arm_st       = (uint8_t)EXEC_ARM_CUSTOM;
      (void)ROBOT_MoveToAuto(s_wake_tgt, s_wake_dur);
    }
    else
    {
      s_arm_st = (uint8_t)EXEC_ARM_IDLE;
    }
    return;
  }

  if (s_arm_st == (uint8_t)EXEC_ARM_CUSTOM)
  {
    s_arm_st = (uint8_t)EXEC_ARM_IDLE;
    return;
  }

  if (s_seq_tbl == NULL)
  {
    s_arm_st = (uint8_t)EXEC_ARM_IDLE;
    return;
  }

  if (s_arm_dwell == 0u)
  {
    s_arm_dwell    = 1u;
    s_arm_dwell_ms = now_ms;
    return;
  }

  if ((now_ms - s_arm_dwell_ms) < (uint32_t)s_seq_tbl[s_seq_idx].dwell_ms)
  {
    return;
  }

  s_arm_dwell = 0u;
  s_seq_idx++;

  if (s_seq_idx < s_seq_len)
  {
    start_seq_step(s_seq_idx);
    return;
  }

  if (s_seq_updates_hold != 0u)
  {
    s_holding = s_seq_is_pick;
    s_hold_j5 = s_arg_j5;
  }
  cancel_arm_seq();
}

static void move_home_safe(int16_t j5_deg, uint8_t allow_when_disarmed)
{
  if ((allow_when_disarmed == 0u) && (s_armed == 0u))
  {
    return;
  }

  /* If disarmed while arm is extended low (>130 deg), retract via PRE before
     folding upright to avoid sweeping across the OpenMV lens. */
  if ((allow_when_disarmed != 0u) &&
      (s_dry_run == 0u) &&
      (any_servo_relaxed() == 0u) &&
      (SERVO_GetActual(1u) > 1300))
  {
    s_seq_tbl          = s_place_steps;
    s_seq_len          = 7u;
    s_seq_idx          = 5u; /* PLACE_RETRACT -> PLACE_HOME */
    s_seq_is_pick      = 0u;
    s_seq_updates_hold = 0u;
    s_arg_j0           = 450;
    s_arg_j1           = 1700;
    s_arg_j5           = EXEC_ClampS16(j5_deg, 900, 1485);
    s_carry_j5         = s_arg_j5;
    s_arm_seq++;
    start_seq_step(s_seq_idx);
    return;
  }

  cancel_arm_seq();
  {
    int16_t tgt[SERVO_NUM];
    for (uint8_t i = 0u; i < SERVO_NUM; i++)
    {
      tgt[i] = SERVO_GetConfig(i)->home_deg;
    }
    if (j5_deg >= 0)
    {
      tgt[5] = EXEC_ClampS16(j5_deg, 900, 1485);
    }
    s_arm_seq++;
    launch_arm_target(tgt, 0u, 0u, (uint8_t)EXEC_ARM_IDLE);
  }
}

static void enter_armed(void)
{
  if (ROBOT_GetState() == ROBOT_ESTOP)
  {
    return;
  }

  TRACK_ForceOff();
  PICK_ForceOff();
  stop_wheels();

  s_armed = 1u;
  if (any_servo_relaxed() != 0u)
  {
    move_home_safe((s_holding != 0u) ? s_hold_j5 : 900, 0u);
  }
  s_telem_dirty = 1u;
}

static void enter_disarmed(void)
{
  s_armed = 0u;
  stop_wheels();

  if (ROBOT_GetState() != ROBOT_ESTOP)
  {
    int16_t keep_j5 = (any_servo_relaxed() == 0u)
                    ? SERVO_GetActual(5u)
                    : ((s_holding != 0u) ? s_hold_j5 : 900);
    if ((any_servo_relaxed() == 0u) || (is_arm_busy_now() != 0u))
    {
      move_home_safe(keep_j5, 1u);
    }
    else
    {
      cancel_arm_seq();
    }
  }
  else
  {
    cancel_arm_seq();
  }
  s_telem_dirty = 1u;
}

/* --- telemetry formatter ($E,...*XX\n) ----------------------------------- */

static char *put_u32(char *dst, uint32_t v)
{
  char tmp[12];
  uint8_t n = 0u;

  if (v == 0u)
  {
    *dst++ = '0';
    return dst;
  }

  while ((v > 0u) && (n < (uint8_t)sizeof(tmp)))
  {
    tmp[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  }
  while (n > 0u)
  {
    *dst++ = tmp[--n];
  }
  return dst;
}

static void send_telemetry(uint32_t now_ms)
{
  static const char hex[] = "0123456789ABCDEF";
  char buf[48];
  char *p = buf;
  uint8_t xor_sum = 0u;
  uint8_t busy = is_arm_busy_now();
  uint8_t r_st = (uint8_t)ROBOT_GetState();

  *p++ = '$';
  *p++ = 'E';
  *p++ = ','; p = put_u32(p, s_armed);
  *p++ = ','; p = put_u32(p, s_arm_st);
  *p++ = ','; p = put_u32(p, busy);
  *p++ = ','; p = put_u32(p, s_holding);
  *p++ = ','; p = put_u32(p, r_st);
  *p++ = ','; p = put_u32(p, s_arm_seq);

  for (const char *c = &buf[1]; c < p; c++)
  {
    xor_sum ^= (uint8_t)(*c);
  }

  *p++ = '*';
  *p++ = hex[(xor_sum >> 4) & 0x0Fu];
  *p++ = hex[xor_sum & 0x0Fu];
  *p++ = '\n';

  ZM_SendRaw((const uint8_t *)buf, (uint16_t)(p - buf));

  s_last_telem_ms = now_ms;
  s_telem_dirty   = 0u;
  s_prev_armed    = s_armed;
  s_prev_busy     = busy;
  s_prev_hold     = s_holding;
  s_prev_seq      = s_arm_seq;
}

/* --- UART3 frame dispatcher ---------------------------------------------- */

static void handle_vel_cmd(const char *fields, uint32_t now_ms)
{
  int32_t v[4];
  uint8_t cnt = 0u;
  int16_t vx, vy, wz;
  uint16_t hold_ms;

  if (EXEC_ParseCsvI32(fields, v, 3u, 4u, &cnt) != 0)
  {
    s_rx_bad++;
    return;
  }

  s_last_rx_ms = now_ms;
  s_rx_frames++;

  if ((s_armed == 0u) || (ROBOT_GetState() == ROBOT_ESTOP))
  {
    return;
  }

  vx = EXEC_ClampS16(v[0], -EXEC_MAX_SPEED, EXEC_MAX_SPEED);
  vy = EXEC_ClampS16(v[1], -EXEC_MAX_SPEED, EXEC_MAX_SPEED);
  wz = EXEC_ClampS16(v[2], -EXEC_MAX_SPEED, EXEC_MAX_SPEED);

  /* Hold chassis wheels stopped while a floor pick/place arm sequence runs. */
  if (s_seq_tbl != NULL)
  {
    stop_wheels();
    return;
  }

  if ((vx == 0) && (vy == 0) && (wz == 0))
  {
    stop_wheels();
    return;
  }

  s_vx_cmd = vx;
  s_vy_cmd = vy;
  s_wz_cmd = wz;

  if ((cnt == 4u) && (v[3] > 0))
  {
    hold_ms = (uint16_t)EXEC_ClampS16(v[3], (int32_t)EXEC_DRIVE_MIN_MS,
                                       (int32_t)EXEC_DRIVE_MAX_MS);
    s_driving           = 1u;
    s_drive_deadline_ms = now_ms + (uint32_t)hold_ms;

    if (s_dry_run == 0u)
    {
      ZM_DriveMecanum(vx, vy, wz, hold_ms);
      s_was_driving = 1u;
      s_last_tx_vx  = vx;
      s_last_tx_vy  = vy;
      s_last_tx_wz  = wz;
      s_last_zm_ms  = now_ms;
    }
    return;
  }

  /* Streaming mode: keep ZMotorD refreshed without saturating the 115200 bus. */
  s_driving           = 1u;
  s_drive_deadline_ms = now_ms + EXEC_DRIVE_WDG_MS;

  if (s_dry_run == 0u)
  {
    uint8_t send_now = (s_was_driving == 0u) ? 1u : 0u;
    if ((now_ms - s_last_zm_ms) >= EXEC_BUS_MIN_PERIOD_MS)
    {
      send_now = 1u;
    }
    else if ((EXEC_Abs32((int32_t)vx - (int32_t)s_last_tx_vx) > 40) ||
             (EXEC_Abs32((int32_t)vy - (int32_t)s_last_tx_vy) > 40) ||
             (EXEC_Abs32((int32_t)wz - (int32_t)s_last_tx_wz) > 40))
    {
      send_now = 1u;
    }

    if (send_now != 0u)
    {
      ZM_DriveMecanum(vx, vy, wz, EXEC_DRIVE_HOLD_MS);
      s_was_driving = 1u;
      s_last_tx_vx  = vx;
      s_last_tx_vy  = vy;
      s_last_tx_wz  = wz;
      s_last_zm_ms  = now_ms;
    }
  }
}

static void handle_arm_cmd(const char *fields, uint32_t now_ms)
{
  int32_t a[4] = { 0, -1, -1, -1 };
  uint8_t cnt = 0u;

  if (EXEC_ParseCsvI32(fields, a, 1u, 4u, &cnt) != 0)
  {
    s_rx_bad++;
    return;
  }

  s_last_rx_ms = now_ms;
  s_rx_frames++;

  if ((s_armed == 0u) || (ROBOT_GetState() == ROBOT_ESTOP))
  {
    return;
  }

  if ((a[0] < 0) || (a[0] > (int32_t)EXEC_OP_GRIP_ONLY))
  {
    s_rx_bad++;
    return;
  }

  stop_wheels();

  switch ((exec_arm_op_t)a[0])
  {
    case EXEC_OP_HOME_OPEN:
      s_holding = 0u;
      s_hold_j5 = 900;
      move_home_safe(900, 0u);
      break;

    case EXEC_OP_PICK:
      s_seq_tbl          = s_pick_steps;
      s_seq_len          = 7u;
      s_seq_idx          = 0u;
      s_seq_is_pick      = 1u;
      s_seq_updates_hold = 1u;
      s_arg_j0           = (a[1] < 0) ? 450  : EXEC_ClampS16(a[1], 0, 1800);
      s_arg_j1           = (a[2] < 0) ? 1716 : EXEC_ClampS16(a[2], 630, 1800);
      s_arg_j5           = (a[3] < 0) ? 1319 : EXEC_ClampS16(a[3], 900, 1485);
      s_arm_seq++;
      start_seq_step(0u);
      break;

    case EXEC_OP_PLACE:
      s_seq_tbl          = s_place_steps;
      s_seq_len          = 7u;
      s_seq_idx          = 0u;
      s_seq_is_pick      = 0u;
      s_seq_updates_hold = 1u;
      s_arg_j0           = (a[1] < 0) ? 450  : EXEC_ClampS16(a[1], 0, 1800);
      s_arg_j1           = (a[2] < 0) ? 1700 : EXEC_ClampS16(a[2], 630, 1800);
      s_arg_j5           = (a[3] < 0) ? 900  : EXEC_ClampS16(a[3], 900, 1485);
      s_carry_j5         = (any_servo_relaxed() == 0u)
                         ? SERVO_GetActual(5u)
                         : ((s_holding != 0u) ? s_hold_j5 : 1319);
      s_arm_seq++;
      start_seq_step(0u);
      break;

    case EXEC_OP_HOLD_HOME:
    {
      int16_t j5 = (a[3] >= 900) ? EXEC_ClampS16(a[3], 900, 1485)
                                 : SERVO_GetActual(5u);
      if (a[3] >= 900)
      {
        s_hold_j5 = j5;
      }
      move_home_safe(j5, 0u);
      break;
    }

    case EXEC_OP_GRIP_ONLY:
    {
      int16_t tgt[SERVO_NUM];
      int16_t j5 = (a[3] < 0) ? 900 : EXEC_ClampS16(a[3], 900, 1485);
      cancel_arm_seq();
      for (uint8_t i = 0u; i < SERVO_NUM; i++)
      {
        tgt[i] = SERVO_GetActual(i);
      }
      tgt[5] = j5;
      s_arm_seq++;
      launch_arm_target(tgt, 450u, 0u, (uint8_t)EXEC_ARM_CUSTOM);
      break;
    }

    default:
      break;
  }

  s_telem_dirty = 1u;
}

static void handle_joints_cmd(const char *fields, uint32_t now_ms)
{
  int32_t j[8] = { -1, -1, -1, -1, -1, -1, -1, 0 };
  int16_t tgt[SERVO_NUM];
  uint8_t cnt = 0u;
  uint16_t dur_ms;

  if (EXEC_ParseCsvI32(fields, j, 7u, 8u, &cnt) != 0)
  {
    s_rx_bad++;
    return;
  }

  s_last_rx_ms = now_ms;
  s_rx_frames++;

  if ((s_armed == 0u) || (ROBOT_GetState() == ROBOT_ESTOP))
  {
    return;
  }

  cancel_arm_seq();
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    tgt[i] = (j[i] < 0) ? SERVO_GetActual(i)
                        : SERVO_Clamp(i, EXEC_ClampS16(j[i], 0, 1800));
  }
  dur_ms = (cnt == 8u && j[7] > 0)
         ? (uint16_t)EXEC_ClampS16(j[7], 100, (int32_t)ROBOT_MAX_TIME_MS)
         : 0u;

  s_arm_seq++;
  launch_arm_target(tgt, dur_ms, 0u, (uint8_t)EXEC_ARM_CUSTOM);
  s_telem_dirty = 1u;
}

static void handle_sys_cmd(const char *fields, uint32_t now_ms)
{
  int32_t s[2] = { 0, 0 };
  uint8_t cnt = 0u;

  if (EXEC_ParseCsvI32(fields, s, 1u, 2u, &cnt) != 0)
  {
    s_rx_bad++;
    return;
  }

  s_last_rx_ms = now_ms;
  s_rx_frames++;

  if (s[0] == 0)
  {
    /* Soft stop wheels + cancel arm motion immediately without full ESTOP. */
    stop_wheels();
    if (s_dry_run == 0u)
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      cancel_arm_seq();
      __set_PRIMASK(primask);
      if (ROBOT_GetState() == ROBOT_MOVING)
      {
        int16_t cur[SERVO_NUM];
        ROBOT_GetPose(cur);
        (void)ROBOT_MoveToAuto(cur, 100u);
        s_arm_st = (uint8_t)EXEC_ARM_CUSTOM;
      }
    }
    else
    {
      cancel_arm_seq();
    }
    s_telem_dirty = 1u;
  }
  else if (s[0] == 1)
  {
    s_telem_dirty = 1u;
  }
  else if (s[0] == 2)
  {
    if ((cnt == 2u) && (s_armed != 0u) && (s_dry_run == 0u))
    {
      ROBOT_SetSpeed((uint8_t)EXEC_ClampS16(s[1], 15, 100));
    }
  }
  else
  {
    s_rx_bad++;
  }
}

static void parse_line(uint32_t now_ms)
{
  uint8_t star = 0u;
  uint8_t xor_calc = 0u;
  int hi, lo;
  char tag;
  const char *fields;

  if ((s_line_len < 6u) || (s_line[1] != ','))
  {
    if ((s_line_len >= 1u) &&
        ((s_line[0] == 'V') || (s_line[0] == 'A') ||
         (s_line[0] == 'J') || (s_line[0] == 'S')))
    {
      s_rx_bad++;
    }
    return;
  }

  for (uint8_t i = 2u; i < s_line_len; i++)
  {
    if (s_line[i] == '*')
    {
      star = i;
      break;
    }
  }

  if ((star == 0u) || ((uint8_t)(star + 3u) != s_line_len))
  {
    s_rx_bad++;
    return;
  }

  for (uint8_t i = 0u; i < star; i++)
  {
    xor_calc ^= (uint8_t)s_line[i];
  }

  hi = hex_nibble(s_line[star + 1u]);
  lo = hex_nibble(s_line[star + 2u]);
  if ((hi < 0) || (lo < 0) || (((uint8_t)((hi << 4) | lo)) != xor_calc))
  {
    s_rx_bad++;
    return;
  }

  s_line[star] = '\0';
  tag    = s_line[0];
  fields = &s_line[2];

  switch (tag)
  {
    case 'L':
      TRACK_OnFrame(fields, now_ms);
      break;

    case 'O':
      PICK_OnFrame(fields, now_ms);
      break;

    case 'V':
      handle_vel_cmd(fields, now_ms);
      break;

    case 'A':
      handle_arm_cmd(fields, now_ms);
      break;

    case 'J':
      handle_joints_cmd(fields, now_ms);
      break;

    case 'S':
      handle_sys_cmd(fields, now_ms);
      break;

    default:
      s_rx_bad++;
      break;
  }
}

void EXEC_PumpUartRx(uint32_t now_ms)
{
  uint8_t b;

  while (ZM_ReadByte(&b) == 0)
  {
    if (b == (uint8_t)'@')
    {
      s_in_frame = 1u;
      s_line_len = 0u;
    }
    else if (s_in_frame != 0u)
    {
      if (b == (uint8_t)'\n')
      {
        parse_line(now_ms);
        s_in_frame = 0u;
        s_line_len = 0u;
      }
      else if (b == (uint8_t)'\r')
      {
        /* ignore CR */
      }
      else if (s_line_len < (uint8_t)(EXEC_LINE_BUF_LEN - 1u))
      {
        s_line[s_line_len++] = (char)b;
      }
      else
      {
        s_in_frame = 0u;
        s_line_len = 0u;
        s_rx_bad++;
      }
    }
  }
}

static void poll_key1(uint32_t now_ms)
{
  uint8_t raw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) ? 1u : 0u;

  if (raw != s_key_raw)
  {
    s_key_raw       = raw;
    s_key_change_ms = now_ms;
  }

  if ((raw != s_key_stable) && ((now_ms - s_key_change_ms) >= EXEC_KEY_DEBOUNCE_MS))
  {
    s_key_stable = raw;
    if (s_key_stable != 0u)
    {
      /* Edge-triggered press: launch from IDLE, or safe-abort & return HOME. */
      if (s_armed == 0u)
      {
        s_dry_run = 0u;
        enter_armed();
      }
      else
      {
        enter_disarmed();
      }
    }
  }
}

/* --- public API ---------------------------------------------------------- */

void EXEC_Init(void)
{
  uint32_t now_ms = HAL_GetTick();

  s_in_frame          = 0u;
  s_line_len          = 0u;
  s_key_raw           = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) ? 1u : 0u;
  s_key_stable        = s_key_raw;
  s_key_change_ms     = now_ms;
  s_armed             = 0u;
  s_dry_run           = 0u;
  s_holding           = 0u;
  s_hold_j5           = 900;
  s_arm_seq           = 0u;
  s_rx_frames         = 0u;
  s_rx_bad            = 0u;
  s_last_rx_ms        = 0u;
  s_vx_cmd            = 0;
  s_vy_cmd            = 0;
  s_wz_cmd            = 0;
  s_last_tx_vx        = 0;
  s_last_tx_vy        = 0;
  s_last_tx_wz        = 0;
  s_driving           = 0u;
  s_was_driving       = 0u;
  s_drive_deadline_ms = now_ms;
  s_last_zm_ms        = now_ms;
  cancel_arm_seq();
  s_telem_dirty       = 1u;
  s_last_telem_ms     = now_ms;
  s_prev_armed        = 0u;
  s_prev_busy         = 0u;
  s_prev_hold         = 0u;
  s_prev_seq          = 0u;
}

int EXEC_SetMode(uint8_t mode, uint8_t dry_run)
{
  if (mode > (uint8_t)EXEC_MODE_ACTIVE)
  {
    return -1;
  }
  if ((ROBOT_GetState() == ROBOT_ESTOP) &&
      (mode != (uint8_t)EXEC_MODE_OFF) && (dry_run == 0u))
  {
    return -1;
  }

  s_dry_run = (dry_run != 0u) ? 1u : 0u;

  if (mode == (uint8_t)EXEC_MODE_OFF)
  {
    enter_disarmed();
  }
  else
  {
    if (s_dry_run != 0u)
    {
      TRACK_ForceOff();
      PICK_ForceOff();
      stop_wheels();
      s_armed       = 1u;
      s_telem_dirty = 1u;
    }
    else
    {
      enter_armed();
    }
  }
  return 0;
}

void EXEC_ForceOff(void)
{
  s_armed       = 0u;
  s_was_driving = 0u;
  s_driving     = 0u;
  s_vx_cmd      = 0;
  s_vy_cmd      = 0;
  s_wz_cmd      = 0;
  cancel_arm_seq();
  s_telem_dirty = 1u;
}

uint8_t EXEC_IsArmed(void)
{
  return s_armed;
}

void EXEC_GetStatus(exec_status_t *out)
{
  uint32_t now_ms;
  uint32_t age;

  if (out == NULL) { return; }

  now_ms = HAL_GetTick();
  if (s_rx_frames == 0u)
  {
    age = 0xFFFFu;
  }
  else
  {
    age = now_ms - s_last_rx_ms;
    if (age > 0xFFFFu) { age = 0xFFFFu; }
  }

  out->armed     = s_armed;
  out->dry_run   = s_dry_run;
  out->arm_state = s_arm_st;
  out->arm_busy  = is_arm_busy_now();
  out->holding   = s_holding;
  out->btn1_down = s_key_stable;
  out->vx_cmd    = s_vx_cmd;
  out->vy_cmd    = s_vy_cmd;
  out->wz_cmd    = s_wz_cmd;
  out->arm_seq   = s_arm_seq;
  out->age_ms    = (uint16_t)age;
  out->rx_frames = s_rx_frames;
  out->rx_bad    = s_rx_bad;
}

void EXEC_Poll(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t  busy_now;

  poll_key1(now_ms);
  EXEC_PumpUartRx(now_ms);

  if (ROBOT_GetState() == ROBOT_ESTOP)
  {
    if (s_armed != 0u)
    {
      s_armed       = 0u;
      s_telem_dirty = 1u;
    }
    stop_wheels();
    cancel_arm_seq();
  }
  else
  {
    if ((s_armed != 0u) && (s_seq_tbl == NULL) &&
        (s_driving != 0u) && (now_ms >= s_drive_deadline_ms))
    {
      stop_wheels();
    }

    if (s_armed != 0u || s_arm_st != (uint8_t)EXEC_ARM_IDLE)
    {
      step_arm_engine(now_ms);
    }
  }

  busy_now = is_arm_busy_now();
  if ((s_armed != s_prev_armed) || (busy_now != s_prev_busy) ||
      (s_holding != s_prev_hold) || (s_arm_seq != s_prev_seq))
  {
    s_telem_dirty = 1u;
  }

  if ((s_telem_dirty != 0u) || ((now_ms - s_last_telem_ms) >= EXEC_TELEM_PERIOD_MS))
  {
    send_telemetry(now_ms);
  }
}
