/**
  ******************************************************************************
  * @file    cube_pick.c
  * @brief   OpenMV cube search, mecanum micro-jog alignment, and 7-step arm
  *          pickup state machine over calibrated KPZ-STM32 hardware poses.
  ******************************************************************************
  */
#include "cube_pick.h"
#include "line_track.h"
#include "zmotord.h"
#include "robot.h"
#include "servo.h"
#include "bsp_pwm.h"
#include "main.h"

#include <stddef.h>

/* Verified 7-step floor pickup trajectory in 0.1 deg units (from grab_poses.json):
 *   pre   : high clearance above OpenMV with horizontal wrist (j4 = 180.0 deg)
 *   above : open horizontal jaws hovering ~5cm above cube
 *   grab  : floor grab pose with open jaws
 *   clamp : closed grip (131.9 deg ~= 1966 us)
 *   home  : upright safe carry pose (holds j5 clamped) */
static const int16_t s_pose_pre[SERVO_NUM]   = { 450, 1120,  820, 1120, 1800,  900 };
static const int16_t s_pose_above[SERVO_NUM] = { 450, 1520,  759, 1130, 1800,  900 };
static const int16_t s_pose_grab[SERVO_NUM]  = { 450, 1716,  759, 1130, 1800,  900 };
static const int16_t s_pose_clamp[SERVO_NUM] = { 450, 1716,  759, 1130, 1800, 1319 };
static const int16_t s_pose_home[SERVO_NUM]  = { 450,  918,  900,  990, 1035, 1319 };

static uint8_t  s_mode;
static uint8_t  s_dry_run;
static uint8_t  s_state;
static uint8_t  s_valid;
static uint8_t  s_arm_flag;
static int16_t  s_dx;
static int16_t  s_dy;
static int16_t  s_cube_w;
static int16_t  s_vx_cmd;
static int16_t  s_wy_cmd;
static uint16_t s_rx_frames;
static uint32_t s_last_rx_ms;

static uint8_t  s_active_started;
static uint8_t  s_seen_ever;
static uint8_t  s_was_driving;
static uint8_t  s_pulsing;
static uint8_t  s_settle_ok;
static uint8_t  s_arm_dwell;
static int16_t  s_j0_aim;

static uint32_t s_start_ms;
static uint32_t s_pulse_end_ms;
static uint32_t s_settle_start_ms;
static uint32_t s_last_ctrl_ms;
static uint32_t s_arm_dwell_ms;

static int32_t iabs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static int16_t clamp_s16(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) { return (int16_t)lo; }
  if (v > hi) { return (int16_t)hi; }
  return (int16_t)v;
}

static int parse_i32_field(const char **p, int32_t *out)
{
  const char *s = *p;
  int32_t sign = 1;
  int32_t val = 0;
  uint8_t digits = 0u;

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

static void stop_wheels(void)
{
  if (s_was_driving != 0u)
  {
    ZM_StopAll();
    s_was_driving = 0u;
  }
  s_pulsing = 0u;
  s_vx_cmd  = 0;
  s_wy_cmd  = 0;
}

static void reset_cycle_runtime(uint32_t now_ms)
{
  s_active_started = 0u;
  s_seen_ever      = 0u;
  s_pulsing        = 0u;
  s_settle_ok      = 0u;
  s_arm_dwell      = 0u;
  s_j0_aim         = 450;
  s_start_ms       = now_ms;
  s_pulse_end_ms   = now_ms;
  s_settle_start_ms = now_ms;
  s_last_ctrl_ms   = now_ms;
}

static void start_arm_move(const int16_t base_pose[SERVO_NUM], int16_t j0,
                           int16_t j5_override, uint16_t dur_ms)
{
  int16_t tgt[SERVO_NUM];

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    tgt[i] = base_pose[i];
  }
  tgt[0] = j0;
  if (j5_override >= 0)
  {
    tgt[5] = j5_override;
  }
  (void)ROBOT_MoveToAuto(tgt, dur_ms);
}

static void fire_pulse(uint32_t now_ms, int16_t vx, int16_t vy, int16_t wz,
                       uint16_t pulse_ms)
{
  s_vx_cmd       = vx;
  s_wy_cmd       = (vy != 0) ? vy : wz;
  s_pulsing      = 1u;
  s_pulse_end_ms = now_ms + (uint32_t)pulse_ms;

  if (s_dry_run == 0u)
  {
    ZM_DriveMecanum(vx, vy, wz, pulse_ms);
    s_was_driving = 1u;
  }
}

static void begin_arm_sequence(uint32_t now_ms)
{
  stop_wheels();
  s_j0_aim = clamp_s16(450 - (((int32_t)s_dx * 3) / 10), 320, 580);

  if (s_dry_run != 0u)
  {
    s_state = (uint8_t)PICK_ST_HOLDING;
    return;
  }

  s_arm_dwell = 0u;
  if (BSP_PWM_GetPulse(SERVO_ChannelOf(1u)) == 0u)
  {
    s_state = (uint8_t)PICK_ST_ARM_WAKE;
    ROBOT_GoHome();
  }
  else
  {
    s_state = (uint8_t)PICK_ST_ARM_PRE;
    start_arm_move(s_pose_pre, s_j0_aim, -1, 800u);
  }
  s_arm_dwell_ms = now_ms;
}

static void step_arm_sequence(uint32_t now_ms)
{
  if (ROBOT_IsBusy() != 0u)
  {
    return;
  }

  if (s_arm_dwell == 0u)
  {
    s_arm_dwell    = 1u;
    s_arm_dwell_ms = now_ms;
    return;
  }

  /* Extra dwell after closing jaws so rubber/teeth settle before lifting. */
  {
    uint32_t need_ms = (s_state == (uint8_t)PICK_ST_ARM_CLAMP) ? 250u : 60u;
    if ((now_ms - s_arm_dwell_ms) < need_ms)
    {
      return;
    }
  }

  s_arm_dwell = 0u;

  switch ((pick_state_t)s_state)
  {
    case PICK_ST_ARM_WAKE:
      s_state = (uint8_t)PICK_ST_ARM_PRE;
      start_arm_move(s_pose_pre, s_j0_aim, -1, 800u);
      break;

    case PICK_ST_ARM_PRE:
      s_state = (uint8_t)PICK_ST_ARM_ABOVE;
      start_arm_move(s_pose_above, s_j0_aim, -1, 750u);
      break;

    case PICK_ST_ARM_ABOVE:
      s_state = (uint8_t)PICK_ST_ARM_GRAB;
      start_arm_move(s_pose_grab, s_j0_aim, -1, 700u);
      break;

    case PICK_ST_ARM_GRAB:
      s_state = (uint8_t)PICK_ST_ARM_CLAMP;
      start_arm_move(s_pose_clamp, s_j0_aim, -1, 600u);
      break;

    case PICK_ST_ARM_CLAMP:
      s_state = (uint8_t)PICK_ST_ARM_LIFT;
      start_arm_move(s_pose_above, s_j0_aim, s_pose_clamp[5], 700u);
      break;

    case PICK_ST_ARM_LIFT:
      s_state = (uint8_t)PICK_ST_ARM_RETRACT;
      start_arm_move(s_pose_pre, 450, s_pose_clamp[5], 800u);
      break;

    case PICK_ST_ARM_RETRACT:
      s_state = (uint8_t)PICK_ST_ARM_HOME;
      start_arm_move(s_pose_home, 450, s_pose_clamp[5], 850u);
      break;

    case PICK_ST_ARM_HOME:
      s_state = (uint8_t)PICK_ST_HOLDING;
      break;

    default:
      break;
  }
}

void PICK_OnFrame(const char *fields, uint32_t now_ms)
{
  const char *p = fields;
  int32_t dx_v, dy_v, w_v, val_v, arm_v;

  if (fields == NULL) { return; }

  if (parse_i32_field(&p, &dx_v)  != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &dy_v)  != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &w_v)   != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &val_v) != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &arm_v) != 0 || *p != '\0') { return; }

  s_dx       = clamp_s16(dx_v, -1000, 1000);
  s_dy       = clamp_s16(dy_v, -1000, 1000);
  s_cube_w   = clamp_s16(w_v, 0, 320);
  s_valid    = (uint8_t)clamp_s16(val_v, 0, 2);
  s_arm_flag = (arm_v != 0) ? 1u : 0u;
  s_last_rx_ms = now_ms;
  s_rx_frames++;
}

void PICK_Init(void)
{
  s_mode       = (uint8_t)PICK_MODE_AUTO;
  s_dry_run    = 0u;
  s_state      = (uint8_t)PICK_ST_IDLE;
  s_valid      = 0u;
  s_arm_flag   = 0u;
  s_dx         = 0;
  s_dy         = 0;
  s_cube_w     = 0;
  s_vx_cmd     = 0;
  s_wy_cmd     = 0;
  s_rx_frames  = 0u;
  s_last_rx_ms = 0u;
  s_was_driving = 0u;
  reset_cycle_runtime(0u);
}

int PICK_SetMode(uint8_t mode, uint8_t dry_run)
{
  uint32_t now_ms = HAL_GetTick();

  if (mode > (uint8_t)PICK_MODE_RESET) { return -1; }
  if ((ROBOT_GetState() == ROBOT_ESTOP) &&
      (mode != (uint8_t)PICK_MODE_OFF) && (dry_run == 0u))
  {
    return -1;
  }

  stop_wheels();
  reset_cycle_runtime(now_ms);
  s_dry_run = (dry_run != 0u) ? 1u : 0u;

  if (mode == (uint8_t)PICK_MODE_RESET)
  {
    s_mode  = (uint8_t)PICK_MODE_AUTO;
    s_state = (uint8_t)PICK_ST_IDLE;
    return 0;
  }

  s_mode  = mode;
  s_state = (uint8_t)PICK_ST_IDLE;
  return 0;
}

void PICK_ForceOff(void)
{
  uint32_t now_ms = HAL_GetTick();

  s_mode        = (uint8_t)PICK_MODE_OFF;
  s_state       = (uint8_t)PICK_ST_IDLE;
  s_was_driving = 0u;
  s_vx_cmd      = 0;
  s_wy_cmd      = 0;
  reset_cycle_runtime(now_ms);
}

uint8_t PICK_IsBusy(void)
{
  return ((s_state != (uint8_t)PICK_ST_IDLE) &&
          (s_state != (uint8_t)PICK_ST_HOLDING) &&
          (s_state != (uint8_t)PICK_ST_FAIL)) ? 1u : 0u;
}

void PICK_GetStatus(pick_status_t *out)
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

  out->mode      = s_mode;
  out->dry_run   = s_dry_run;
  out->state     = s_state;
  out->valid     = s_valid;
  out->dx        = s_dx;
  out->dy        = s_dy;
  out->cube_w    = s_cube_w;
  out->vx_cmd    = s_vx_cmd;
  out->wy_cmd    = s_wy_cmd;
  out->age_ms    = (uint16_t)age;
  out->rx_frames = s_rx_frames;
}

void PICK_Poll(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t  fresh;
  uint8_t  armed = 0u;

  if (ROBOT_GetState() == ROBOT_ESTOP)
  {
    if (s_mode == (uint8_t)PICK_MODE_ACTIVE)
    {
      s_mode = (uint8_t)PICK_MODE_OFF;
    }
    stop_wheels();
    s_state = (uint8_t)PICK_ST_IDLE;
    reset_cycle_runtime(now_ms);
    return;
  }

  /* Latch HOLDING / FAIL until host resets or restarts mode. */
  if ((s_state == (uint8_t)PICK_ST_HOLDING) ||
      (s_state == (uint8_t)PICK_ST_FAIL))
  {
    stop_wheels();
    return;
  }

  /* 7-step arm sequence completes deterministically regardless of vision occlusion. */
  if (s_state >= (uint8_t)PICK_ST_ARM_WAKE)
  {
    step_arm_sequence(now_ms);
    return;
  }

  fresh = ((s_rx_frames > 0u) &&
           ((now_ms - s_last_rx_ms) <= PICK_RX_TIMEOUT_MS)) ? 1u : 0u;

  if ((s_mode == (uint8_t)PICK_MODE_ACTIVE) && (fresh != 0u))
  {
    armed = 1u;
  }
  else if ((s_mode == (uint8_t)PICK_MODE_AUTO) && (fresh != 0u) && (s_arm_flag != 0u))
  {
    armed = 1u;
  }

  if (armed == 0u)
  {
    stop_wheels();
    s_state = (uint8_t)PICK_ST_IDLE;
    reset_cycle_runtime(now_ms);
    return;
  }

  if (s_active_started == 0u)
  {
    TRACK_ForceOff();
    reset_cycle_runtime(now_ms);
    s_active_started = 1u;
  }

  if ((s_seen_ever == 0u) && ((now_ms - s_start_ms) > PICK_SEARCH_TIMEOUT_MS))
  {
    stop_wheels();
    s_state = (uint8_t)PICK_ST_FAIL;
    return;
  }

  if ((now_ms - s_start_ms) > PICK_ACTIVE_TIMEOUT_MS)
  {
    stop_wheels();
    s_state = (uint8_t)PICK_ST_FAIL;
    return;
  }

  /* Complete active pulse-jog, brake cleanly, and enter stationary camera settle. */
  if (s_pulsing != 0u)
  {
    if (now_ms >= s_pulse_end_ms)
    {
      stop_wheels();
      s_settle_start_ms = now_ms;
    }
    return;
  }

  if (s_valid == 0u)
  {
    s_state     = (uint8_t)PICK_ST_SEARCH;
    s_settle_ok = 0u;
    if ((now_ms - s_settle_start_ms) < 240u)
    {
      return;
    }
    /* Crisp ~15 deg pulsed scan so OpenMV captures clear frames between turns. */
    fire_pulse(now_ms, 0, 0, 360, 120u);
    return;
  }

  s_seen_ever = 1u;

  /* Coarse approach while cube is well ahead of grab zone. */
  if ((s_state != (uint8_t)PICK_ST_ALIGN) && (s_dy > PICK_APPROACH_DY))
  {
    int16_t vx;
    int16_t wz;

    s_state     = (uint8_t)PICK_ST_APPROACH;
    s_settle_ok = 0u;

    if ((now_ms - s_last_ctrl_ms) < 40u)
    {
      return;
    }
    s_last_ctrl_ms = now_ms;

    wz = clamp_s16(-(((int32_t)s_dx * 35) / 100), -240, 240);
    vx = (iabs32((int32_t)s_dx) > 350) ? 260 : 480;
    s_vx_cmd = vx;
    s_wy_cmd = wz;

    if (s_dry_run == 0u)
    {
      ZM_DriveMecanum(vx, 0, wz, 220u);
      s_was_driving = 1u;
    }
    return;
  }

  /* Braking handoff when entering close-range pulse-jog alignment. */
  if (s_state != (uint8_t)PICK_ST_ALIGN)
  {
    stop_wheels();
    s_state           = (uint8_t)PICK_ST_ALIGN;
    s_settle_ok       = 0u;
    s_settle_start_ms = now_ms;
    return;
  }

  if ((now_ms - s_settle_start_ms) < PICK_SETTLE_MS)
  {
    return;
  }

  /* Wait for a fresh frame captured after braking before judging alignment. */
  if ((s_last_rx_ms < s_settle_start_ms) && ((now_ms - s_last_rx_ms) > 120u))
  {
    return;
  }

  if ((iabs32((int32_t)s_dx) <= PICK_TOL_DX) &&
      (iabs32((int32_t)s_dy) <= PICK_TOL_DY))
  {
    s_settle_ok++;
    if (s_settle_ok >= PICK_CONFIRM_FRAMES)
    {
      begin_arm_sequence(now_ms);
    }
    return;
  }

  s_settle_ok = 0u;

  if (iabs32((int32_t)s_dx) > 240)
  {
    int16_t wz = (s_dx < 0) ? 360 : -360;
    fire_pulse(now_ms, 0, 0, wz, 95u);
  }
  else if (iabs32((int32_t)s_dx) > PICK_TOL_DX)
  {
    int16_t vy = (s_dx < 0) ? 480 : -480;
    fire_pulse(now_ms, 0, vy, 0, 90u);
  }
  else
  {
    int16_t  vx       = (s_dy > 0) ? 420 : -420;
    uint16_t pulse_ms = (iabs32((int32_t)s_dy) > 130) ? 105u : 80u;
    fire_pulse(now_ms, vx, 0, 0, pulse_ms);
  }
}
