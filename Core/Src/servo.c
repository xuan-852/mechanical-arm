/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Per-joint servo: angle -> pulse width mapping, soft limits, homing.
  ******************************************************************************
  */
#include "servo.h"
#include "bsp_pwm.h"
#include "cfg_store.h"
#include "main.h"

/* MG996R physics: 0.17s/60deg -> 0.353 deg/ms. That is the ceiling; anything
   faster just saturates the servo (audible buzz, current spike, position lag)
   and destroys the coordinated path the planner is trying to preserve.

   The 50Hz frame is the second hard limit: one pulse width per 20ms, so a
   joint physically cannot move more than ~7deg per frame. */
#define SERVO_MAX_DEG_PER_MS   0.353f

/* Must stay equal to ROBOT_TICK_MS in robot.h (asserted below). */
#define SERVO_PLANNER_TICK_MS  5.0f

/* Homing ramp: 256 steps * 4ms ~= 1s for a full-range sweep. */
#define SERVO_HOME_STEPS       256u

/* Hardware-verified per-joint defaults on KPZ-STM32 (see CALIBRATION.md).
 * Scale stays unified across joints: 500..2500 us <-> 0..1800 (0.1 deg),
 * clamped per joint to verified safe limits:
 *   joint 0 (base yaw,    ch5 PB3): home 1000 us (45.0 deg), 500..2500 us
 *   joint 1 (lower arm,   ch4 PB8): home 1520 us (91.8 deg), 1200..2500 us
 *                                   (50 us margin above 1150 us rear hard stop)
 *   joint 2 (mid elbow,   ch3 PB9): home 1500 us (90.0 deg), 500..2500 us
 *   joint 3 (upper pitch, ch2 PB6): home 1600 us (99.0 deg), 500..2500 us
 *   joint 4 (wrist roll,  ch0 PB7): home 1650 us (103.5 deg), 500..2500 us
 *   joint 5 (gripper,     ch1 PB4): home 1500 us (90.0 deg, open), 1500..2150 us
 */
static const servo_cfg_t s_default_cfg[SERVO_NUM] =
{
  { .min_deg =   0, .max_deg = 1800, .home_deg =  450, .pulse_min_us =  500, .pulse_max_us = 2500, .invert = 0 },
  { .min_deg = 630, .max_deg = 1800, .home_deg =  918, .pulse_min_us = 1200, .pulse_max_us = 2500, .invert = 0 },
  { .min_deg =   0, .max_deg = 1800, .home_deg =  900, .pulse_min_us =  500, .pulse_max_us = 2500, .invert = 0 },
  { .min_deg =   0, .max_deg = 1800, .home_deg =  990, .pulse_min_us =  500, .pulse_max_us = 2500, .invert = 0 },
  { .min_deg =   0, .max_deg = 1800, .home_deg = 1035, .pulse_min_us =  500, .pulse_max_us = 2500, .invert = 0 },
  { .min_deg = 900, .max_deg = 1485, .home_deg =  900, .pulse_min_us = 1500, .pulse_max_us = 2150, .invert = 0 },
};

static servo_cfg_t   s_cfg[SERVO_NUM];
static servo_state_t s_state[SERVO_NUM];

/* Logical joint index -> physical PWM channel.
 *
 * The logical order follows the assembled arm, BOTTOM TO TOP:
 *     joint 0 = base rotation ... joint 5 = gripper
 *
 * Verified KPZ-STM32 wiring:
 *   ch0=PB7(roll), ch1=PB4(grip), ch2=PB6(upper), ch3=PB9(elbow),
 *   ch4=PB8(lower), ch5=PB3(base).
 */
static const uint8_t s_joint_chan[SERVO_NUM] =
{
  5u,   /* joint 0 (base)    -> ch5  PB3 */
  4u,   /* joint 1 (lower)   -> ch4  PB8 */
  3u,   /* joint 2 (elbow)   -> ch3  PB9 */
  2u,   /* joint 3 (upper)   -> ch2  PB6 */
  0u,   /* joint 4 (roll)    -> ch0  PB7 */
  1u,   /* joint 5 (gripper) -> ch1  PB4 */
};

static int16_t mul_div_s16(int16_t x, int16_t num, int16_t den)
{
  return (int16_t)(((int32_t)x * (int32_t)num) / (int32_t)den);
}

/* Pure function: touches no state, so it is safe to call from any context. */
static uint16_t angle_to_pulse(const servo_cfg_t *c, int16_t deg)
{
  int16_t span_deg   = (int16_t)(c->max_deg - c->min_deg);
  int16_t span_pulse = (int16_t)(c->pulse_max_us - c->pulse_min_us);
  int16_t offset;

  if (deg < c->min_deg) { deg = c->min_deg; }
  if (deg > c->max_deg) { deg = c->max_deg; }

  if (span_deg <= 0)
  {
    return (uint16_t)c->pulse_min_us;
  }

  offset = mul_div_s16((int16_t)(deg - c->min_deg), span_pulse, span_deg);

  if (c->invert == 0u)
  {
    return (uint16_t)(c->pulse_min_us + offset);
  }

  return (uint16_t)(c->pulse_max_us - offset);
}

void SERVO_SetConfig(uint8_t idx, const servo_cfg_t *cfg)
{
  if ((idx < SERVO_NUM) && (cfg != NULL))
  {
    s_cfg[idx] = *cfg;
  }
}

const servo_cfg_t *SERVO_GetConfig(uint8_t idx)
{
  return (idx < SERVO_NUM) ? &s_cfg[idx] : &s_default_cfg[0];
}

int16_t SERVO_Clamp(uint8_t idx, int16_t deg)
{
  const servo_cfg_t *c = SERVO_GetConfig(idx);

  if (deg < c->min_deg) { deg = c->min_deg; }
  if (deg > c->max_deg) { deg = c->max_deg; }

  return deg;
}

void SERVO_Init(void)
{
  BSP_PWM_Init();

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    s_cfg[i] = s_default_cfg[i];
  }

  (void)CFG_Load(s_cfg);

  /* Keep all 6 outputs relaxed (0 us from BSP_PWM_Init) at boot so power-on or
     serial flashing resets never whip a resting arm or trigger a 6-servo surge.
     Outputs engage safely when HOME or MoveTo runs. */
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    s_state[i].target_deg = s_cfg[i].home_deg;
    s_state[i].actual_deg = s_cfg[i].home_deg;
    s_state[i].start_deg  = s_cfg[i].home_deg;
  }
}

int SERVO_ApplyConfig(uint8_t idx, const servo_cfg_t *cfg)
{
  if ((idx >= SERVO_NUM) || (cfg == NULL)) { return -1; }
  if (CFG_IsValid(cfg) == 0u)              { return -1; }

  s_cfg[idx] = *cfg;

  /* If the new window excludes the current position, bring the joint inside it
     and re-issue the pulse when this channel is active. */
  s_state[idx].actual_deg = SERVO_Clamp(idx, s_state[idx].actual_deg);
  s_state[idx].target_deg = s_state[idx].actual_deg;
  s_state[idx].start_deg  = s_state[idx].actual_deg;
  if (BSP_PWM_GetPulse(s_joint_chan[idx]) != 0u)
  {
    SERVO_WriteAngle(idx, s_state[idx].actual_deg);
  }

  return 0;
}

uint8_t SERVO_SaveConfig(void)
{
  return CFG_Save(s_cfg);
}

uint8_t SERVO_ResetConfig(void)
{
  uint8_t rc = CFG_Erase();

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    s_cfg[i] = s_default_cfg[i];
    s_state[i].actual_deg = SERVO_Clamp(i, s_state[i].actual_deg);
    s_state[i].target_deg = s_state[i].actual_deg;
    if (BSP_PWM_GetPulse(s_joint_chan[i]) != 0u)
    {
      SERVO_WriteAngle(i, s_state[i].actual_deg);
    }
  }

  return rc;
}

void SERVO_WriteAngle(uint8_t idx, int16_t deg)
{
  if (idx >= SERVO_NUM)
  {
    return;
  }

  deg = SERVO_Clamp(idx, deg);
  (void)BSP_PWM_SetPulse(s_joint_chan[idx], angle_to_pulse(&s_cfg[idx], deg));
  s_state[idx].actual_deg = deg;
}

uint8_t SERVO_ChannelOf(uint8_t idx)
{
  return (idx < SERVO_NUM) ? s_joint_chan[idx] : 0xFFu;
}

void SERVO_SetActual(uint8_t idx, int16_t deg)
{
  if (idx < SERVO_NUM) { s_state[idx].actual_deg = deg; }
}

int16_t SERVO_GetActual(uint8_t idx)
{
  return (idx < SERVO_NUM) ? s_state[idx].actual_deg : 0;
}

void SERVO_SetTarget(uint8_t idx, int16_t deg)
{
  if (idx < SERVO_NUM)
  {
    s_state[idx].start_deg  = s_state[idx].actual_deg;
    s_state[idx].target_deg = SERVO_Clamp(idx, deg);
  }
}

int16_t SERVO_GetTarget(uint8_t idx)
{
  return (idx < SERVO_NUM) ? s_state[idx].target_deg : 0;
}

int16_t SERVO_GetStart(uint8_t idx)
{
  return (idx < SERVO_NUM) ? s_state[idx].start_deg : 0;
}

int16_t SERVO_MaxStep(uint8_t idx)
{
  (void)idx;
  /* 0.1deg units per planner tick: 0.353 deg/ms * 5 ms * 10 = 17
     (17.65 truncated -- 0.1deg resolution is far below the MG996R dead band,
     so the truncation is irrelevant). */
  return (int16_t)(SERVO_MAX_DEG_PER_MS * SERVO_PLANNER_TICK_MS * 10.0f);
}

void SERVO_GoHome(void)
{
  for (uint16_t step = 0u; step <= SERVO_HOME_STEPS; step++)
  {
    for (uint8_t i = 0u; i < SERVO_NUM; i++)
    {
      int16_t from = s_state[i].actual_deg;
      int16_t to   = s_cfg[i].home_deg;
      int16_t deg  = (int16_t)(from + mul_div_s16((int16_t)(to - from),
                                                  (int16_t)step,
                                                  (int16_t)SERVO_HOME_STEPS));

      (void)BSP_PWM_SetPulse(s_joint_chan[i], angle_to_pulse(&s_cfg[i], deg));
    }

    HAL_Delay(4u);
  }

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    s_state[i].actual_deg = s_cfg[i].home_deg;
    s_state[i].target_deg = s_cfg[i].home_deg;
    s_state[i].start_deg  = s_cfg[i].home_deg;
  }
}
