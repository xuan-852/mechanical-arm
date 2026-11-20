/**
  ******************************************************************************
  * @file    line_track.c
  * @brief   OpenMV @L frame parser and PD mecanum line-following controller.
  ******************************************************************************
  */
#include "line_track.h"
#include "cube_pick.h"
#include "zmotord.h"
#include "robot.h"
#include "main.h"

#include <stddef.h>

#define TRACK_LINE_BUF_LEN   48u
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

static char           s_line[TRACK_LINE_BUF_LEN];
static uint8_t        s_line_len;
static uint8_t        s_in_frame;

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

static int hex_nibble(char c)
{
  if ((c >= '0') && (c <= '9')) { return (int)(c - '0'); }
  if ((c >= 'A') && (c <= 'F')) { return (int)(c - 'A' + 10); }
  if ((c >= 'a') && (c <= 'f')) { return (int)(c - 'a' + 10); }
  return -1;
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

static void parse_line(uint32_t now_ms)
{
  const char *p;
  int32_t err_v, ang_v, val_v, arm_v;
  uint8_t star = 0u;
  uint8_t xor_calc = 0u;
  int hi, lo;

  if ((s_line_len < 11u) || (s_line[1] != ',') ||
      ((s_line[0] != 'L') && (s_line[0] != 'O')))
  {
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
    return;
  }

  s_line[star] = '\0';
  p = &s_line[2];

  if (s_line[0] == 'O')
  {
    PICK_OnFrame(p, now_ms);
    return;
  }

  if (parse_i32_field(&p, &err_v) != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &ang_v) != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &val_v) != 0 || *p != ',') { return; }
  p++;
  if (parse_i32_field(&p, &arm_v) != 0 || *p != '\0') { return; }

  s_err        = clamp_s16(err_v, -1000, 1000);
  s_ang        = clamp_s16(ang_v, -90, 90);
  s_valid      = (uint8_t)clamp_s16(val_v, 0, 2);
  s_arm        = (arm_v != 0) ? 1u : 0u;
  s_last_rx_ms = now_ms;
  s_rx_frames++;
}

static void pump_uart3_rx(uint32_t now_ms)
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
      else if (s_line_len < (uint8_t)(TRACK_LINE_BUF_LEN - 1u))
      {
        s_line[s_line_len++] = (char)b;
      }
      else
      {
        s_in_frame = 0u;
        s_line_len = 0u;
      }
    }
  }
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

  if ((s_valid == 1u) && (iabs32((int32_t)s_err) >= 60))
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
      e_comb  = (int32_t)clamp_s16(((int32_t)s_err / 2) +
                                   (((int32_t)s_ang * (int32_t)s_cfg.k_ang) / 20),
                                   -400, 400);
    }
    else
    {
      s_state = (uint8_t)TRACK_ST_FOLLOW;
      e_comb  = (int32_t)clamp_s16((int32_t)s_err +
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
    *wz_out = clamp_s16(wz_raw, -(int32_t)s_cfg.wz_max, (int32_t)s_cfg.wz_max);

    abs_e   = iabs32(e_comb);
    if (abs_e > 1000) { abs_e = 1000; }
    vx_span = (int32_t)s_cfg.vx_cruise - (int32_t)s_cfg.vx_min;
    if (vx_span < 0) { vx_span = 0; }

    *vx_out = (int16_t)((int32_t)s_cfg.vx_cruise - (abs_e * vx_span) / 1000);
  }
}

void TRACK_Init(void)
{
  s_cfg          = s_default_cfg;
  s_mode         = (uint8_t)TRACK_MODE_AUTO;
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
  s_in_frame     = 0u;
  s_line_len     = 0u;

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

  c.vx_cruise = clamp_s16(cfg->vx_cruise, 0, TRACK_SPEED_CEIL);
  c.vx_min    = clamp_s16(cfg->vx_min, 0, c.vx_cruise);
  c.kp        = (uint16_t)clamp_s16((int32_t)cfg->kp, 0, 500);
  c.kd        = (uint16_t)clamp_s16((int32_t)cfg->kd, 0, 500);
  c.k_ang     = (uint16_t)clamp_s16((int32_t)cfg->k_ang, 0, 300);
  c.wz_max    = (uint16_t)clamp_s16((int32_t)cfg->wz_max, 0, TRACK_SPEED_CEIL);
  c.search_wz = (uint16_t)clamp_s16((int32_t)cfg->search_wz, 0, 600);

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

  pump_uart3_rx(now_ms);

  if (ROBOT_GetState() == ROBOT_ESTOP)
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
