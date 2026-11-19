/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Single-joint servo abstraction: degrees -> pulse width, per-joint
  *          limits and slew-rate limit.
  *
  *          All angles are expressed in 0.1 degree units (int16_t) so that no
  *          floating point is needed during interpolation. 0.1 deg resolution
  *          equals ~2.8us of pulse width: far below the MG996R dead band
  *          (~10us), so a higher resolution would only waste CPU.
  ******************************************************************************
  */
#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SERVO_NUM        6u

/* Logical joint order is BOTTOM TO TOP of the assembled arm:
 *     joint 0 = base rotation, joint 5 = gripper.
 * This does not match the wiring order. servo.c owns the joint -> channel
 * map, so no layer above it needs to know a pin name. */

/* Angle unit: 0.1 deg. Full range is 0..1800 (=0..180.0 deg). */
#define SERVO_DEG(x)     ((int16_t)((x) * 10.0f))
#define SERVO_TOF(x)     ((float)(x) / 10.0f)

typedef struct
{
  int16_t min_deg;      /* mechanical lower limit, 0.1 deg */
  int16_t max_deg;      /* mechanical upper limit, 0.1 deg */
  int16_t home_deg;     /* power-up / HOME position */
  int16_t pulse_min_us; /* pulse width at min_deg */
  int16_t pulse_max_us; /* pulse width at max_deg */
  uint8_t invert;       /* 1 = mirrored joint (pulse decreases with angle) */
} servo_cfg_t;

typedef struct
{
  int16_t target_deg;   /* commanded position, 0.1 deg */
  int16_t actual_deg;   /* position currently being output, 0.1 deg */
  int16_t start_deg;    /* position when the current move started */
} servo_state_t;

void     SERVO_Init(void);
void     SERVO_SetConfig(uint8_t idx, const servo_cfg_t *cfg);

/* Clamp + map a logical angle to a pulse width and push it out immediately. */
void     SERVO_WriteAngle(uint8_t idx, int16_t deg);

/* Bookkeeping only, no hardware access (used by the motion planner). */
void     SERVO_SetActual(uint8_t idx, int16_t deg);
int16_t  SERVO_GetActual(uint8_t idx);
int16_t  SERVO_GetTarget(uint8_t idx);
int16_t  SERVO_GetStart(uint8_t idx);
void     SERVO_SetTarget(uint8_t idx, int16_t deg);

/* Clamp a logical angle to the joint's mechanical limits. */
int16_t  SERVO_Clamp(uint8_t idx, int16_t deg);

/* Validate and apply one joint's calibration. Returns 0 on success, -1 if the
   configuration is mechanically implausible (min >= max, home outside the
   window, pulse range outside what the PWM layer can emit, ...). */
int      SERVO_ApplyConfig(uint8_t idx, const servo_cfg_t *cfg);

/* Persist / erase the whole calibration set in the last flash page.
   Return 0 on success. */
uint8_t  SERVO_SaveConfig(void);
uint8_t  SERVO_ResetConfig(void);

/* Move every joint to its configured home position (blocking, ~2s). */
void     SERVO_GoHome(void);

/* Max travel (0.1 deg units) this joint may cover in one ROBOT_TICK_MS
   planner tick at 100% speed. 0.1deg/ms == 1deg per 10ms. */
int16_t  SERVO_MaxStep(uint8_t idx);

const servo_cfg_t *SERVO_GetConfig(uint8_t idx);

/* Physical PWM channel currently driven by a logical joint. For diagnostics
   and for the host tool's labels -- control code should not need this. */
uint8_t  SERVO_ChannelOf(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
