/**
  ******************************************************************************
  * @file    robot.c
  * @brief   Multi-axis synchronous motion planner (time-normalised).
  ******************************************************************************
  */
#include "robot.h"
#include "servo.h"
#include "bsp_pwm.h"
#include "zmotord.h"
#include "exec_srv.h"
#include "line_track.h"
#include "cube_pick.h"
#include "main.h"
#include <stddef.h>

typedef struct
{
  int16_t  start[SERVO_NUM];
  int16_t  target[SERVO_NUM];
  uint32_t duration_ticks;   /* in ROBOT_TICK_MS units */
  uint32_t elapsed_ticks;
} segment_t;

static volatile robot_state_t s_state;
static segment_t              s_seg;
static volatile uint16_t      s_progress;   /* 0..1000 */
static uint8_t                s_speed_pct;

static int32_t iabs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static int16_t interp(int16_t from, int16_t to, uint32_t elapsed, uint32_t total)
{
  int32_t delta = (int32_t)to - (int32_t)from;

  /* Integer-only: avoids pulling soft-float into the 200Hz path. */
  return (int16_t)((int32_t)from +
                   (int32_t)((delta * (int32_t)elapsed) / (int32_t)total));
}

void ROBOT_Init(void)
{
  SERVO_Init();

  s_state        = ROBOT_IDLE;
  s_progress     = 0u;
  s_speed_pct    = ROBOT_DEFAULT_SPEED_PCT;
  s_seg.elapsed_ticks  = 0u;
  s_seg.duration_ticks = 0u;
}

static uint16_t auto_duration_ms(const int16_t target_deg[SERVO_NUM])
{
  int32_t  longest_step   = 0;
  int32_t  ms;
  uint16_t max_step_deg;

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    int32_t d = iabs32((int32_t)target_deg[i] - (int32_t)SERVO_GetActual(i));
    if (d > longest_step) { longest_step = d; }
  }

  if (longest_step == 0)
  {
    return ROBOT_MIN_TIME_MS;
  }

  /* Slowest allowed joint motion defines the minimum feasible duration.
     max_step_deg is in 0.1deg units per planner tick, so
     ms = units_total / (units_per_tick / tick_ms) = units_total * tick_ms / units_per_tick. */
  max_step_deg = (uint16_t)SERVO_MaxStep(0u);
  ms = (int32_t)((longest_step * (int32_t)ROBOT_TICK_MS) / (int32_t)max_step_deg);

  /* Give the servo a 1.3x margin over its theoretical maximum so it does not
     sit in saturation (which is what makes multi-axis moves look uncoordinated). */
  ms = (ms * 13) / 10;

  if (ms < (int32_t)ROBOT_MIN_TIME_MS) { ms = (int32_t)ROBOT_MIN_TIME_MS; }
  if (ms > (int32_t)ROBOT_MAX_TIME_MS) { ms = (int32_t)ROBOT_MAX_TIME_MS; }

  return (uint16_t)ms;
}

int ROBOT_MoveTo(const int16_t target_deg[SERVO_NUM], uint16_t duration_ms)
{
  return ROBOT_MoveToAuto(target_deg, duration_ms);
}

int ROBOT_MoveToAuto(const int16_t target_deg[SERVO_NUM], uint16_t duration_ms)
{
  uint32_t ms;
  uint32_t floor_ms;
  uint32_t primask;

  if (target_deg == NULL)      { return -1; }
  if (s_state == ROBOT_ESTOP)  { return -1; }

  /* The physical floor comes from the speed law, regardless of what the caller
     asked for. Without this clamp SET_TIME(50) followed by SET_ANGLE(180 deg)
     would demand the joint cover full travel in 50 ms -- it cannot, so it
     saturates and the move looks violent instead of merely fast. */
  floor_ms = auto_duration_ms(target_deg);

  ms = (duration_ms == 0u) ? floor_ms : duration_ms;

  if (ms < floor_ms)          { ms = floor_ms; }
  if (ms < ROBOT_MIN_TIME_MS) { ms = ROBOT_MIN_TIME_MS; }
  if (ms > ROBOT_MAX_TIME_MS) { ms = ROBOT_MAX_TIME_MS; }

  /* ROBOT_Tick() runs from the SysTick ISR and reads s_seg. If a new segment
     arrives while the previous one is still MOVING, the ISR can observe a
     half-updated start[]/target[] pair and emit one glitched pulse. Publish
     the whole segment atomically. */
  primask = __get_PRIMASK();
  __disable_irq();

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    s_seg.start[i]  = SERVO_GetActual(i);
    s_seg.target[i] = SERVO_Clamp(i, target_deg[i]);
  }

  s_seg.duration_ticks = ms / ROBOT_TICK_MS;
  if (s_seg.duration_ticks == 0u) { s_seg.duration_ticks = 1u; }
  s_seg.elapsed_ticks = 0u;

  s_progress = 0u;
  s_state    = ROBOT_MOVING;

  __set_PRIMASK(primask);

  return 0;
}

void ROBOT_Stop(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (s_state == ROBOT_MOVING)
  {
    /* Freeze where we are: keep the current actual position as the target so
       the next move starts from here. */
    for (uint8_t i = 0u; i < SERVO_NUM; i++)
    {
      s_seg.target[i] = SERVO_GetActual(i);
      s_seg.start[i]  = SERVO_GetActual(i);
    }
    s_state = ROBOT_IDLE;
  }

  __set_PRIMASK(primask);
  EXEC_ForceOff();
  TRACK_ForceOff();
  PICK_ForceOff();
  ZM_StopAll();
}

void ROBOT_EmergencyStop(void)
{
  /* Outputs are left untouched on purpose: zeroing PWM makes MG996R drop
     under gravity. Freezing the last frame is the safe behaviour. */
  s_state = ROBOT_ESTOP;
  EXEC_ForceOff();
  TRACK_ForceOff();
  PICK_ForceOff();
  ZM_StopAll();
}

void ROBOT_ClearEmergency(void)
{
  if (s_state == ROBOT_ESTOP)
  {
    s_progress = 0u;
    s_state    = ROBOT_IDLE;
  }
}

void ROBOT_GoHome(void)
{
  /* Stagger wake-up when channels start relaxed (0 us) to avoid 6-servo surge:
     lower -> elbow -> upper -> base -> wrist roll -> gripper. */
  static const uint8_t wake_order[SERVO_NUM] = { 1u, 2u, 3u, 0u, 4u, 5u };
  int16_t home[SERVO_NUM];
  uint8_t any_relaxed = 0u;

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    home[i] = SERVO_GetConfig(i)->home_deg;
    if (BSP_PWM_GetPulse(SERVO_ChannelOf(i)) == 0u)
    {
      any_relaxed = 1u;
    }
  }

  if (any_relaxed != 0u)
  {
    for (uint8_t k = 0u; k < SERVO_NUM; k++)
    {
      uint8_t j = wake_order[k];
      if (BSP_PWM_GetPulse(SERVO_ChannelOf(j)) == 0u)
      {
        SERVO_WriteAngle(j, home[j]);
        HAL_Delay(60u);
      }
    }
  }

  (void)ROBOT_MoveToAuto(home, 2000u);
}

/* --- Bring-up self test -------------------------------------------------- */

#define SELFTEST_STEP_MS   20u   /* one output update per 20ms PWM frame */

static void ramp_joint(uint8_t idx, int16_t to, uint16_t ms)
{
  int16_t  from  = SERVO_GetActual(idx);
  int32_t  delta = (int32_t)to - (int32_t)from;
  uint16_t steps = (uint16_t)(ms / SELFTEST_STEP_MS);

  if (steps == 0u) { steps = 1u; }

  for (uint16_t s = 1u; s <= steps; s++)
  {
    SERVO_WriteAngle(idx, (int16_t)((int32_t)from +
                                    (delta * (int32_t)s) / (int32_t)steps));
    HAL_Delay(SELFTEST_STEP_MS);
  }
}

static void selftest_begin(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  /* Take ownership of the outputs first: the planner runs from the SysTick
     ISR and would otherwise overwrite the sweep on the very next tick. */
  s_state              = ROBOT_IDLE;
  s_seg.duration_ticks = 0u;
  s_progress           = 0u;
  __set_PRIMASK(primask);
}

static void selftest_sweep_joint(uint8_t i, int16_t lo_deg, int16_t hi_deg,
                                 uint16_t leg_ms, uint16_t dwell_ms)
{
  /* SERVO_WriteAngle clamps against the joint's own limits, so a caller
     cannot drive a joint past its configured mechanical range. */
  ramp_joint(i, lo_deg, leg_ms);
  HAL_Delay(dwell_ms);
  ramp_joint(i, hi_deg, leg_ms);
  HAL_Delay(dwell_ms);
  ramp_joint(i, SERVO_GetConfig(i)->home_deg, leg_ms);
  HAL_Delay(dwell_ms);
}

void ROBOT_SelfTest(int16_t lo_deg, int16_t hi_deg,
                    uint16_t leg_ms, uint16_t dwell_ms)
{
  selftest_begin();

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    selftest_sweep_joint(i, lo_deg, hi_deg, leg_ms, dwell_ms);
  }
}

void ROBOT_SelfTestDelta(int16_t delta_deg,
                         uint16_t leg_ms, uint16_t dwell_ms)
{
  selftest_begin();

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    int16_t home = SERVO_GetConfig(i)->home_deg;
    selftest_sweep_joint(i, (int16_t)(home - delta_deg),
                         (int16_t)(home + delta_deg), leg_ms, dwell_ms);
  }
}

void ROBOT_Tick(void)
{
  uint32_t total;
  uint32_t speed;

  if (s_state != ROBOT_MOVING)
  {
    return;
  }

  /* Speed override stretches the segment instead of truncating it, so the
     path shape is preserved at any speed. */
  speed = s_speed_pct;
  total = (s_seg.duration_ticks * 100u) / speed;

  s_seg.elapsed_ticks++;

  if (s_seg.elapsed_ticks >= total)
  {
    for (uint8_t i = 0u; i < SERVO_NUM; i++)
    {
      SERVO_WriteAngle(i, s_seg.target[i]);
    }
    s_progress = 1000u;
    s_state    = ROBOT_IDLE;
    return;
  }

  s_progress = (uint16_t)((s_seg.elapsed_ticks * 1000u) / total);

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    SERVO_WriteAngle(i, interp(s_seg.start[i], s_seg.target[i],
                               s_seg.elapsed_ticks, total));
  }
}

robot_state_t ROBOT_GetState(void)
{
  return s_state;
}

uint8_t ROBOT_IsBusy(void)
{
  return (s_state == ROBOT_MOVING) ? 1u : 0u;
}

void ROBOT_SetSpeed(uint8_t percent)
{
  if (percent == 0u)   { percent = 1u; }
  if (percent > 100u)  { percent = 100u; }
  s_speed_pct = percent;
}

uint8_t ROBOT_GetSpeed(void)
{
  return s_speed_pct;
}

void ROBOT_GetPose(int16_t out[SERVO_NUM])
{
  uint32_t primask;

  if (out == NULL) { return; }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    out[i] = SERVO_GetActual(i);
  }
  __set_PRIMASK(primask);
}

uint16_t ROBOT_GetProgress(void)
{
  return s_progress;
}
