/**
  ******************************************************************************
  * @file    robot.h
  * @brief   Multi-axis synchronous motion planner.
  *
  *          Usage model
  *          -----------
  *          ROBOT_MoveTo(targets, duration_ms) starts one segment. All six
  *          joints start together and finish together; the joint that has the
  *          longest way to travel defines the segment time, and the others are
  *          slowed down proportionally (time-normalised interpolation):
  *
  *              actual[i](t) = start[i] + (target[i]-start[i]) * t / T
  *
  *          Because every joint is driven by the SAME normalised progress, a
  *          single global speed factor (0..100%) is enough to slow the whole
  *          arm down, and the arm never loses its coordinated shape.
  ******************************************************************************
  */
#ifndef __ROBOT_H
#define __ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "servo.h"

#define ROBOT_TICK_MS      5u      /* planner period -> 200Hz */
#define ROBOT_MAX_TIME_MS  30000u  /* sanity limit for one segment */
#define ROBOT_MIN_TIME_MS  100u

/* Boot self test: sweeps every joint in turn so the hardware can be verified
   with no host attached. OFF for normal operation -- a full-range sweep at
   power-on is a hazard once the arm is assembled. Run it on demand instead,
   with CMD_SELFTEST (0x08). */
#define ROBOT_BOOT_SELFTEST 0

/* Default global speed override, percent (1..100). 100 = the pace the planner
   computes. Lower is gentler: every segment is stretched by 100/percent, and
   because the stretch is applied to the normalised progress the coordinated
   shape of a multi-joint move is preserved. */
#define ROBOT_DEFAULT_SPEED_PCT  40u

typedef enum
{
  ROBOT_IDLE = 0,      /* nothing to do */
  ROBOT_MOVING,        /* executing a segment */
  ROBOT_ESTOP          /* frozen, ignores new motion until cleared */
} robot_state_t;

void          ROBOT_Init(void);

/* Queue one coordinated move. Returns 0 on success, -1 if stopped/rejected. */
int           ROBOT_MoveTo(const int16_t target_deg[SERVO_NUM],
                           uint16_t duration_ms);

/* Same, but lets the caller override the duration that the planner computed.
   duration_ms = 0 -> auto (longest joint at 100% speed). */
int           ROBOT_MoveToAuto(const int16_t target_deg[SERVO_NUM],
                               uint16_t duration_ms);

void          ROBOT_Stop(void);       /* hold current position (graceful) */
void          ROBOT_EmergencyStop(void); /* freeze immediately, latch state */
void          ROBOT_ClearEmergency(void);
void          ROBOT_GoHome(void);

/* Bring-up self test: one joint at a time, lo_deg -> hi_deg -> home, so every
   servo and every channel mapping can be confirmed by eye. BLOCKING; the
   planner is forced idle for the duration. lo_deg/hi_deg are clamped against
   each joint's configured limits, so an uncalibrated arm is not thrown across
   its full mechanical range. */
void          ROBOT_SelfTest(int16_t lo_deg, int16_t hi_deg,
                             uint16_t leg_ms, uint16_t dwell_ms);
void          ROBOT_SelfTestDelta(int16_t delta_deg,
                                  uint16_t leg_ms, uint16_t dwell_ms);

/* Called from the 5ms time base. Never called from the PWM timer ISR. */
void          ROBOT_Tick(void);

robot_state_t ROBOT_GetState(void);
uint8_t       ROBOT_IsBusy(void);

/* Global speed override 1..100 (%). 100 = planner-computed speed. */
void          ROBOT_SetSpeed(uint8_t percent);
uint8_t       ROBOT_GetSpeed(void);

/* Snapshot of the current pose into out[SERVO_NUM] (0.1 deg units). */
void          ROBOT_GetPose(int16_t out[SERVO_NUM]);

/* Current segment progress 0..1000 (for telemetry / debugging). */
uint16_t      ROBOT_GetProgress(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_H */
