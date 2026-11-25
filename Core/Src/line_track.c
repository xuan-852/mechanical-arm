/**
  ******************************************************************************
  * @file    line_track.c
  * @brief   OpenMV @L frame parser and PD mecanum line-following controller.
  ******************************************************************************
  */
#include "line_track.h"
#include "exec_srv.h"
#include "zmotord.h"
#include "robot.h"
#include "main.h"

#include <stddef.h>

#define TRACK_SPEED_CEIL    900

static const track_cfg_t s_default_cfg =
{
  .vx_cruise = 600,
  .vx_min    = 420,
  .kp        = 38u,
  .kd        = 160u,
  .k_ang     = 20u,
  .wz_max    = 320u,
  .search_wz = 320u,
};

static track_cfg_t    s_cfg;
static uint8_t        s_mode;
static uint8_t        s_dry_run;
static uint8_t        s_state;
static uint8_t        s_valid;
static uint8_t        s_arm;
static int16_t        s_err;
static int16_t        s_ang;
static int16_t        s_vx_cmd;
static int16_t        s_wz_cmd;
static uint16_t       s_rx_frames;
static uint32_t       s_last_rx_ms;
static uint32_t       s_last_ctrl_ms;
static uint8_t        s_ctrl_primed;

static int16_t        s_prev_err;
static uint8_t        s_has_prev_err;
static int8_t         s_last_dir;
static uint8_t        s_lost_active;
static uint32_t       s_lost_start_ms;
static uint8_t        s_was_driving;

static void reset_ctrl_history(void)
{
  s_has_prev_err = 0u;
  s_prev_err     = 0;
  s_lost_active  = 0u;
  s_vx_cmd       = 0;
  s_wz_cmd       = 0;
}

static void stop_motors_if_driving(void)
{
  if (s_was_driving != 0u)
  {
    ZM_StopAll();
    s_was_driving = 0u;
  }
}

void TRACK_OnFrame(const char *fields, uint32_t now_ms)
{
  int32_t v[4];

  if (EXEC_ParseCsvI32(fields, v, 4u, 4u, NULL) != 0)
  {
    return;
  }

  s_err        = EXEC_ClampS16(v[0], -1000, 1000);
  s_ang        = EXEC_ClampS16(v[1], -90, 90);
  s_valid      = (uint8_t)EXEC_ClampS16(v[2], 0, 2);
  s_arm        = (v[3] != 0) ? 1u : 0u;
  s_last_rx_ms = now_ms;
  s_rx_frames++;
}

static void compute_step(uint32_t now_ms, int16_t *vx_out, int16_t *wz_out)
{
  if (s_valid == 0u)
  {
    uint32_t lost_ms;

    s_has_prev_err = 0u;
    if (s_lost_active == 0u)
    {
      s_lost_start_ms = now_ms;
      s_lost_active   = 1u;
    }

    lost_ms = now_ms - s_lost_start_ms;
    if ((s_last_dir != 0) && (lost_ms <= TRACK_SEARCH_MS))
    {
      s_state = (uint8_t)TRACK_ST_SEARCH;
      *vx_out = 0;
      *wz_out = (s_last_dir < 0)
              ? (int16_t)s_cfg.search_wz
              : (int16_t)-(int32_t)s_cfg.search_wz;
    }
    else
    {
      s_state = (uint8_t)TRACK_ST_LOST;
      *vx_out = 0;
      *wz_out = 0;
    }
    return;
  }

  s_lost_active = 0u;

  if ((s_valid == 1u) && (EXEC_Abs32((int32_t)s_err) >= 60))
  {
    s_last_dir = (s_err < 0) ? -1 : 1;
  }

  {
    int32_t e_comb;
    int32_t d_err = 0;
    int32_t wz_raw;
    int32_t abs_e;
    int32_t vx_span;

    if (s_valid == 2u)
    {
      /* Perpendicular bar / T-junction: hold gentle heading, avoid sudden swing. */
      s_state = (uint8_t)TRACK_ST_JUNCTION;
      e_comb  = (int32_t)EXEC_ClampS16(((int32_t)s_err / 2) +
                                        (((int32_t)s_ang * (int32_t)s_cfg.k_ang) / 20),
                                        -400, 400);
    }
    else
    {
      s_state = (uint8_t)TRACK_ST_FOLLOW;
      e_comb  = (int32_t)EXEC_ClampS16((int32_t)s_err +
                                        (((int32_t)s_ang * (int32_t)s_cfg.k_ang) / 10),
                                        -1200, 1200);
    }

    if (s_has_prev_err != 0u)
    {
      d_err = e_comb - (int32_t)s_prev_err;
      if (d_err < -120) { d_err = -120; }
      if (d_err >  120) { d_err =  120; }
    }
    s_prev_err     = (int16_t)e_comb;
    s_has_prev_err = 1u;

    /* Negative err (line on left) -> positive wz (CCW turn left). */
    wz_raw = -(((int32_t)s_cfg.kp * e_comb + (int32_t)s_cfg.kd * d_err) / 100);
    *wz_out = EXEC_ClampS16(wz_raw, -(int32_t)s_cfg.wz_max, (int32_t)s_cfg.wz_max);

    abs_e   = EXEC_Abs32(e_comb);
    if (abs_e > 1000) { abs_e = 1000; }
    vx_span = (int32_t)s_cfg.vx_cruise - (int32_t)s_cfg.vx_min;
    if (vx_span < 0) { vx_span = 0; }

    *vx_out = (int16_t)((int32_t)s_cfg.vx_cruise - (abs_e * vx_span) / 1000);
  }
}

void TRACK_Init(void)
{
  s_cfg          = s_default_cfg;
  s_mode         = (uint8_t)TRACK_MODE_OFF;
  s_dry_run      = 0u;
  s_state        = (uint8_t)TRACK_ST_IDLE;
  s_valid        = 0u;
  s_arm          = 0u;
  s_err          = 0;
  s_ang          = 0;
  s_rx_frames    = 0u;
  s_last_rx_ms   = 0u;
  s_last_ctrl_ms = 0u;
  s_ctrl_primed  = 0u;
  s_last_dir     = 0;
  s_was_driving  = 0u;

  reset_ctrl_history();
}

int TRACK_SetMode(uint8_t mode, uint8_t dry_run)
{
  if (mode > (uint8_t)TRACK_MODE_AUTO) { return -1; }
  if ((ROBOT_GetState() == ROBOT_ESTOP) &&
      (mode != (uint8_t)TRACK_MODE_OFF) && (dry_run == 0u))
  {
    return -1;
  }

  if (mode != (uint8_t)TRACK_MODE_OFF)
  {
    EXEC_ForceOff();
  }

  s_mode    = mode;
  s_dry_run = (dry_run != 0u) ? 1u : 0u;

  if ((s_mode == (uint8_t)TRACK_MODE_OFF) || (s_dry_run != 0u))
  {
    stop_motors_if_driving();
    if (s_mode == (uint8_t)TRACK_MODE_OFF)
    {
      s_state       = (uint8_t)TRACK_ST_IDLE;
      s_ctrl_primed = 0u;
      reset_ctrl_history();
    }
  }
  else
  {
    s_ctrl_primed = 0u;
  }

  return 0;
}

void TRACK_ForceOff(void)
{
  s_mode        = (uint8_t)TRACK_MODE_OFF;
  s_state       = (uint8_t)TRACK_ST_IDLE;
  s_was_driving = 0u;
  s_ctrl_primed = 0u;
  reset_ctrl_history();
}

int TRACK_SetConfig(const track_cfg_t *cfg)
{
  track_cfg_t c;

  if (cfg == NULL) { return -1; }

  c.vx_cruise = EXEC_ClampS16(cfg->vx_cruise, 0, TRACK_SPEED_CEIL);
  c.vx_min    = EXEC_ClampS16(cfg->vx_min, 0, c.vx_cruise);
  c.kp        = (uint16_t)EXEC_ClampS16((int32_t)cfg->kp, 0, 500);
  c.kd        = (uint16_t)EXEC_ClampS16((int32_t)cfg->kd, 0, 500);
  c.k_ang     = (uint16_t)EXEC_ClampS16((int32_t)cfg->k_ang, 0, 300);
  c.wz_max    = (uint16_t)EXEC_ClampS16((int32_t)cfg->wz_max, 0, TRACK_SPEED_CEIL);
  c.search_wz = (uint16_t)EXEC_ClampS16((int32_t)cfg->search_wz, 0, 600);

  s_cfg = c;
  return 0;
}

const track_cfg_t *TRACK_GetConfig(void)
{
  return &s_cfg;
}

void TRACK_GetStatus(track_status_t *out)
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
  out->err       = s_err;
  out->ang       = s_ang;
  out->vx_cmd    = s_vx_cmd;
  out->wz_cmd    = s_wz_cmd;
  out->age_ms    = (uint16_t)age;
  out->rx_frames = s_rx_frames;
}

void TRACK_Poll(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t  fresh;
  uint8_t  active = 0u;
  int16_t  vx = 0;
  int16_t  wz = 0;

  EXEC_PumpUartRx(now_ms);

  if ((ROBOT_GetState() == ROBOT_ESTOP) || (EXEC_IsArmed() != 0u))
  {
    if (s_mode == (uint8_t)TRACK_MODE_ACTIVE)
    {
      s_mode = (uint8_t)TRACK_MODE_OFF;
    }
    s_state       = (uint8_t)TRACK_ST_IDLE;
    s_was_driving = 0u;
    s_ctrl_primed = 0u;
    reset_ctrl_history();
    return;
  }

  fresh = ((s_rx_frames > 0u) &&
           ((now_ms - s_last_rx_ms) <= TRACK_RX_TIMEOUT_MS)) ? 1u : 0u;

  if ((s_mode == (uint8_t)TRACK_MODE_ACTIVE) && (fresh != 0u))
  {
    active = 1u;
  }
  else if ((s_mode == (uint8_t)TRACK_MODE_AUTO) && (fresh != 0u) && (s_arm != 0u))
  {
    active = 1u;
  }

  if (active == 0u)
  {
    stop_motors_if_driving();
    s_state       = (uint8_t)TRACK_ST_IDLE;
    s_ctrl_primed = 0u;
    reset_ctrl_history();
    return;
  }

  if ((s_ctrl_primed != 0u) &&
      ((now_ms - s_last_ctrl_ms) < TRACK_CTRL_PERIOD_MS))
  {
    return;
  }

  s_last_ctrl_ms = now_ms;
  s_ctrl_primed  = 1u;
  compute_step(now_ms, &vx, &wz);
  s_vx_cmd = vx;
  s_wz_cmd = wz;

  if (s_dry_run != 0u)
  {
    stop_motors_if_driving();
    return;
  }

  if ((vx == 0) && (wz == 0))
  {
    stop_motors_if_driving();
  }
  else
  {
    ZM_DriveMecanum(vx, 0, wz, TRACK_MOTOR_HOLD_MS);
    s_was_driving = 1u;
  }
}
