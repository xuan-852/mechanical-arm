/* Host-side unit test for the firmware core.
 *
 * Compiles the real Core/Src/servo.c, robot.c and protocol.c against stubbed
 * hardware (no STM32 HAL, no CMSIS) and exercises the behaviour that cannot be
 * verified by reading the code:
 *
 *   1. CRC8 matches the hand-computed vector
 *   2. a READ_POSE frame is decoded and answered
 *   3. a SET_ANGLE frame drives all six joints to the requested angles
 *   4. the automatic segment duration respects the servo speed law
 *   5. E-stop rejects motion and is reported over the link
 *   6. the bring-up self test sweeps every channel across its full range
 *
 * Build (MSYS/MinGW gcc):
 *   gcc -std=c99 -Wall -Wextra -Werror -O1 -DZM_HOST_TEST \
 *       -Itools/fake -ICore/Inc -o tools/test_core.exe tools/test_core.c \
 *       Core/Src/servo.c Core/Src/robot.c Core/Src/protocol.c \
 *       Core/Src/cfg_store.c Core/Src/zmotord.c -lm
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "servo.h"
#include "robot.h"
#include "protocol.h"
#include "cfg_store.h"
#include "zmotord.h"
#include "line_track.h"
#include "bsp_pwm.h"
#include "usart.h"
#include "main.h"

/* --- test doubles -------------------------------------------------------- */

static uint16_t g_pulse[BSP_PWM_CHANNELS];
static uint16_t g_pulse_min[BSP_PWM_CHANNELS];
static uint16_t g_pulse_max[BSP_PWM_CHANNELS];

static uint8_t  g_tx[1024];
static uint16_t g_tx_len;

static uint8_t  g_rx[256];
static uint16_t g_rx_head, g_rx_tail;

static uint8_t  g_zm_tx[512];
static uint16_t g_zm_tx_len;

static uint8_t  g_zm_rx[256];
static uint16_t g_zm_rx_head, g_zm_rx_tail;
static uint32_t g_fake_tick_ms;

static uint8_t  g_fake_flash[CFG_PAGE_SIZE];
static uint8_t  g_flash_locked = 1u;

const uint8_t *cfg_fake_flash_ptr(uint32_t offset)
{
  return (offset < CFG_PAGE_SIZE) ? &g_fake_flash[offset] : &g_fake_flash[0];
}

uint32_t HAL_FLASH_Unlock(void) { g_flash_locked = 0u; return HAL_OK; }
uint32_t HAL_FLASH_Lock(void)   { g_flash_locked = 1u; return HAL_OK; }

uint32_t HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
  (void)pEraseInit;
  if (g_flash_locked != 0u) { return 1u; }
  if (PageError != NULL) { *PageError = 0xFFFFFFFFu; }
  memset(g_fake_flash, 0xFF, sizeof(g_fake_flash));
  return HAL_OK;
}

uint32_t HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  uint32_t off;
  (void)TypeProgram;
  if (g_flash_locked != 0u) { return 1u; }
  if ((Address < CFG_PAGE_ADDR) ||
      ((Address - CFG_PAGE_ADDR + 1u) >= CFG_PAGE_SIZE))
  {
    return 1u;
  }
  off = Address - CFG_PAGE_ADDR;
  g_fake_flash[off]      = (uint8_t)(Data & 0xFFu);
  g_fake_flash[off + 1u] = (uint8_t)((Data >> 8) & 0xFFu);
  return HAL_OK;
}

void ZM_SendRaw(const uint8_t *buf, uint16_t len)
{
  if ((buf != NULL) && ((g_zm_tx_len + len) <= (uint16_t)sizeof(g_zm_tx)))
  {
    memcpy(&g_zm_tx[g_zm_tx_len], buf, len);
    g_zm_tx_len = (uint16_t)(g_zm_tx_len + len);
  }
}

int ZM_ReadByte(uint8_t *out)
{
  if ((out == NULL) || (g_zm_rx_head == g_zm_rx_tail)) { return -1; }
  *out = g_zm_rx[g_zm_rx_tail++];
  if (g_zm_rx_tail >= (uint16_t)sizeof(g_zm_rx)) { g_zm_rx_tail = 0u; }
  return 0;
}

uint32_t HAL_GetTick(void)
{
  return g_fake_tick_ms;
}

uint16_t BSP_PWM_SetPulse(uint8_t ch, uint16_t pulse_us)
{
  if (ch < BSP_PWM_CHANNELS) { g_pulse[ch] = pulse_us; }
  return pulse_us;
}

uint16_t BSP_PWM_GetPulse(uint8_t ch)
{
  return (ch < BSP_PWM_CHANNELS) ? g_pulse[ch] : 0u;
}

void BSP_PWM_Init(void) { }

void BSP_PWM_AllOff(void)
{
  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++) { g_pulse[i] = 0u; }
}

const char *BSP_PWM_PinName(uint8_t ch)
{
  static const char *names[BSP_PWM_CHANNELS] = {
    "PB7", "PB4", "PB6", "PB9", "PB8", "PB3"
  };
  return (ch < BSP_PWM_CHANNELS) ? names[ch] : "?";
}

/* Sampling the output inside the delay lets the sweep be observed without a
   real time base. */
void HAL_Delay(uint32_t ms)
{
  (void)ms;
  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++)
  {
    if (g_pulse[i] < g_pulse_min[i]) { g_pulse_min[i] = g_pulse[i]; }
    if (g_pulse[i] > g_pulse_max[i]) { g_pulse_max[i] = g_pulse[i]; }
  }
}

void UART_WriteByte(uint8_t byte) { UART_WriteBuf(&byte, 1u); }

void UART_WriteBuf(const uint8_t *buf, uint16_t len)
{
  if ((g_tx_len + len) <= sizeof(g_tx))
  {
    memcpy(&g_tx[g_tx_len], buf, len);
    g_tx_len = (uint16_t)(g_tx_len + len);
  }
}

int UART_RxAvailable(void) { return (g_rx_head != g_rx_tail) ? 1 : 0; }

int UART_ReadByte(uint8_t *out)
{
  if ((out == NULL) || (g_rx_head == g_rx_tail)) { return -1; }
  *out = g_rx[g_rx_tail++];
  return 0;
}

/* --- helpers ------------------------------------------------------------- */

static int g_fail;

static void check(const char *label, int ok, const char *detail)
{
  printf("  [%s] %s%s%s\n", ok ? "ok" : "FAIL", label,
         detail ? " -- " : "", detail ? detail : "");
  if (!ok) { g_fail++; }
}

static uint8_t crc8(const uint8_t *d, uint16_t n)
{
  uint8_t crc = 0xFFu;
  for (uint16_t i = 0u; i < n; i++)
  {
    crc ^= d[i];
    for (uint8_t b = 0u; b < 8u; b++)
    {
      crc = ((crc & 0x80u) != 0u) ? (uint8_t)((crc << 1) ^ 0x31u)
                                  : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t frame(uint8_t *out, uint8_t cmd, const uint8_t *pl, uint8_t len)
{
  uint16_t n = 0u;
  out[n++] = PROTO_HEAD1;
  out[n++] = PROTO_HEAD2;
  out[n++] = len;
  out[n++] = cmd;
  for (uint8_t i = 0u; i < len; i++) { out[n++] = pl[i]; }
  out[n++] = crc8(&out[3], (uint16_t)(1u + len));
  return n;
}

static void feed(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0u; i < len; i++) { g_rx[g_rx_head++] = data[i]; }
  PROTO_Poll();
}

static void put_s16(uint8_t *p, uint8_t i, int16_t v)
{
  p[i * 2u]      = (uint8_t)((uint16_t)v & 0xFFu);
  p[i * 2u + 1u] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static int16_t get_s16(const uint8_t *p, uint8_t i)
{
  return (int16_t)((uint16_t)p[i * 2u] | ((uint16_t)p[i * 2u + 1u] << 8));
}

/* Find the reply frame with the given command in the TX log. Returns payload
   length, or -1. */
static int find_reply(uint8_t cmd, uint8_t *payload_out)
{
  uint16_t i = 0u;
  while ((i + 5u) <= g_tx_len)
  {
    if ((g_tx[i] == PROTO_HEAD1) && (g_tx[i + 1u] == PROTO_HEAD2))
    {
      uint8_t len = g_tx[i + 2u];
      if ((i + 5u + len) <= g_tx_len)
      {
        if (g_tx[i + 3u] == cmd)
        {
          if (payload_out != NULL)
          {
            memcpy(payload_out, &g_tx[i + 4u], len);
          }
          return (int)len;
        }
        i = (uint16_t)(i + 5u + len);
        continue;
      }
    }
    i++;
  }
  return -1;
}

static void reset_log(void) { g_tx_len = 0u; }
static void reset_zm(void)  { g_zm_tx_len = 0u; memset(g_zm_tx, 0, sizeof(g_zm_tx)); }

static void reset_sweep(void)
{
  for (uint8_t i = 0u; i < BSP_PWM_CHANNELS; i++)
  {
    g_pulse_min[i] = 0xFFFFu;
    g_pulse_max[i] = 0u;
  }
}

/* Run the planner until it goes idle, return the number of ticks consumed. */
static uint32_t run_to_idle(uint32_t max_ticks)
{
  uint32_t n = 0u;
  while ((ROBOT_GetState() == ROBOT_MOVING) && (n < max_ticks))
  {
    ROBOT_Tick();
    n++;
  }
  return n;
}

/* --- tests --------------------------------------------------------------- */

static void test_crc(void)
{
  printf("CRC8\n");
  uint8_t one = 0x03u;
  check("crc8(0x03) == 0xFF", PROTO_Crc8(&one, 1u) == 0xFFu, NULL);
}

static void test_read_pose(void)
{
  printf("READ_POSE frame & relaxed boot\n");

  for (uint8_t ch = 0u; ch < BSP_PWM_CHANNELS; ch++)
  {
    check("PWM channel starts relaxed (0us) on boot", g_pulse[ch] == 0u, NULL);
  }

  reset_log();
  uint8_t f[16];
  uint16_t n = frame(f, CMD_READ_POSE, NULL, 0u);
  feed(f, n);

  uint8_t payload[32];
  int len = find_reply(CMD_POSE, payload);
  check("POSE reply received", len == 12, NULL);
  if (len == 12)
  {
    char buf[96];
    sprintf(buf, "j0=%d j1=%d j2=%d j3=%d j4=%d j5=%d",
            get_s16(payload, 0u), get_s16(payload, 1u), get_s16(payload, 2u),
            get_s16(payload, 3u), get_s16(payload, 4u), get_s16(payload, 5u));
    check("joints report calibrated homes",
          get_s16(payload, 0u) == 450  && get_s16(payload, 1u) == 918 &&
          get_s16(payload, 2u) == 900  && get_s16(payload, 3u) == 990 &&
          get_s16(payload, 4u) == 1035 && get_s16(payload, 5u) == 900, buf);
  }
}

static void test_joint_map(void)
{
  printf("joint -> channel map & calibrated hard limits\n");

  static const uint8_t want[SERVO_NUM] = {5u, 4u, 3u, 2u, 0u, 1u};
  int ok = 1;
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    if (SERVO_ChannelOf(i) != want[i]) { ok = 0; }
  }
  check("table matches bottom-to-top KPZ-STM32 wiring", ok, NULL);

  SERVO_GoHome();
  check("home j0 (ch5 PB3) = 1000us", g_pulse[5u] == 1000u, NULL);
  check("home j1 (ch4 PB8) = 1520us", g_pulse[4u] == 1520u, NULL);
  check("home j2 (ch3 PB9) = 1500us", g_pulse[3u] == 1500u, NULL);
  check("home j3 (ch2 PB6) = 1600us", g_pulse[2u] == 1600u, NULL);
  check("home j4 (ch0 PB7) = 1650us", g_pulse[0u] == 1650u, NULL);
  check("home j5 (ch1 PB4) = 1500us", g_pulse[1u] == 1500u, NULL);

  SERVO_WriteAngle(0u, 1800);
  check("joint 0 drives ch5 (PB3) to 2500us", g_pulse[SERVO_ChannelOf(0u)] == 2500u, NULL);
  check("gripper ch1 untouched at 1500us", g_pulse[SERVO_ChannelOf(5u)] == 1500u, NULL);

  /* Joint 1 (ch4 PB8) has rear hard stop at 1150us -> software clamped >=1200us */
  SERVO_WriteAngle(1u, 0);
  check("joint 1 clamps to >=1200us safety floor", g_pulse[SERVO_ChannelOf(1u)] == 1200u, NULL);

  /* Joint 5 (ch1 PB4 gripper) strictly stays inside 1500..2150us */
  SERVO_WriteAngle(5u, 0);
  check("joint 5 clamps min to 1500us open", g_pulse[SERVO_ChannelOf(5u)] == 1500u, NULL);
  SERVO_WriteAngle(5u, 1800);
  check("joint 5 clamps max to 2150us closed", g_pulse[SERVO_ChannelOf(5u)] == 2150u, NULL);

  SERVO_GoHome();
}

static void test_set_angle(void)
{
  printf("SET_ANGLE frame\n");
  const int16_t want[SERVO_NUM] = {180, 700, 600, 1200, 1500, 1400};

  reset_log();
  uint8_t pl[SERVO_NUM * 2u];
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, want[i]); }

  uint8_t f[32];
  uint16_t n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);

  uint8_t payload[8];
  int len = find_reply(CMD_ACK, payload);
  check("ACK received", len == 2, NULL);
  if (len == 2)
  {
    check("ACK echoes cmd", payload[0] == CMD_SET_ANGLE, NULL);
    check("ACK status OK", payload[1] == ST_OK, NULL);
  }

  uint32_t ticks = run_to_idle(20000u);
  char buf[64];
  sprintf(buf, "%u ticks = %u ms", ticks, ticks * ROBOT_TICK_MS);
  check("planner completed the segment", ticks > 0u && ticks < 20000u, buf);

  int16_t pose[SERVO_NUM];
  ROBOT_GetPose(pose);
  int all_ok = 1;
  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    if (pose[i] != want[i]) { all_ok = 0; }
  }
  check("pose equals requested angles", all_ok, NULL);
}

static void test_speed_law(void)
{
  printf("speed override and servo speed law\n");

  char buf[96];

  /* The build must ship gentle: 40% by default, not 100. */
  ROBOT_SetSpeed(100u);   /* reset from whatever an earlier test left */
  ROBOT_SetSpeed(ROBOT_DEFAULT_SPEED_PCT);
  sprintf(buf, "default %u%%", ROBOT_GetSpeed());
  check("default speed is the gentle override",
        ROBOT_GetSpeed() == ROBOT_DEFAULT_SPEED_PCT, buf);

  /* Measure the law itself at 100%, so the override does not mask it. */
  ROBOT_SetSpeed(100u);

  /* Park everything at 0 first, with an explicit (generous) duration so the
     auto path is not involved. */
  uint8_t pl[SERVO_NUM * 2u];
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, 0); }

  uint8_t f[32];
  uint16_t n = frame(f, CMD_SET_TIME, (const uint8_t *)"\x10\x27", 2u); /* 10000 */
  feed(f, n);
  n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);
  run_to_idle(60000u);

  /* Now ask for a full 180 deg travel with duration 0 -> firmware auto. */
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, 1800); }
  n = frame(f, CMD_SET_TIME, (const uint8_t *)"\x00\x00", 2u);
  feed(f, n);
  n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);

  uint32_t ticks = run_to_idle(60000u);
  double ms = (double)ticks * (double)ROBOT_TICK_MS;
  double deg_per_ms = (ms > 0.0) ? 180.0 / ms : 1e9;

  sprintf(buf, "180 deg in %.0f ms -> %.3f deg/ms (cap 0.353)", ms, deg_per_ms);
  check("slower than the MG996R ceiling", deg_per_ms <= 0.353 * 1.02, buf);
  check("not absurdly slow", deg_per_ms > 0.10, buf);
}

static void test_duration_clamp(void)
{
  printf("explicit duration clamped to the physical floor\n");

  /* Park at 0 again. */
  uint8_t pl[SERVO_NUM * 2u];
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, 0); }

  uint8_t f[32];
  uint16_t n = frame(f, CMD_SET_TIME, (const uint8_t *)"\x10\x27", 2u); /* 10000 */
  feed(f, n);
  n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);
  run_to_idle(60000u);

  /* Demand a full 180 deg in 50 ms -- physically impossible, must be raised. */
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, 1800); }
  n = frame(f, CMD_SET_TIME, (const uint8_t *)"\x32\x00", 2u); /* 50 ms */
  feed(f, n);
  n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);

  uint32_t ticks = run_to_idle(60000u);
  double ms = (double)ticks * (double)ROBOT_TICK_MS;

  char buf[96];
  sprintf(buf, "requested 50 ms, ran %.0f ms", ms);
  check("impossible request was stretched, not obeyed", ms > 400.0, buf);
}

static void test_estop(void)
{
  printf("emergency stop\n");

  ROBOT_EmergencyStop();

  reset_log();
  uint8_t pl[SERVO_NUM * 2u];
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl, i, 1000); }

  uint8_t f[32];
  uint16_t n = frame(f, CMD_SET_ANGLE, pl, (uint8_t)sizeof(pl));
  feed(f, n);

  uint8_t payload[8];
  int len = find_reply(CMD_NAK, payload);
  check("NAK received while latched", len == 2, NULL);
  if (len == 2) { check("status is ESTOP", payload[1] == ST_ESTOP, NULL); }

  reset_log();
  n = frame(f, CMD_CLEAR_ESTOP, NULL, 0u);
  feed(f, n);
  check("CLEAR_ESTOP acknowledged",
        find_reply(CMD_ACK, payload) == 2 && payload[1] == ST_OK, NULL);
  check("state back to idle", ROBOT_GetState() == ROBOT_IDLE, NULL);
}

static void test_selftest(void)
{
  printf("bring-up self test\n");

  reset_sweep();
  ROBOT_SelfTest(0, 1800, 700u, 120u);

  int base_swept = (g_pulse_min[5u] <= 505u) && (g_pulse_max[5u] >= 2495u);
  int j1_clamped = (g_pulse_min[4u] >= 1200u) && (g_pulse_max[4u] >= 2495u);
  int j5_clamped = (g_pulse_min[1u] >= 1500u) && (g_pulse_max[1u] <= 2150u) &&
                   (g_pulse_max[1u] >= 2140u);
  char buf[128];
  sprintf(buf, "base %u..%u us, j1 %u..%u us, grip %u..%u us",
          g_pulse_min[5u], g_pulse_max[5u],
          g_pulse_min[4u], g_pulse_max[4u],
          g_pulse_min[1u], g_pulse_max[1u]);
  check("full sweep respects calibrated hard limits",
        base_swept && j1_clamped && j5_clamped, buf);

  /* The gentle profile used by CMD_SELFTEST stays +/-20 deg around each home
     and remains strictly inside joint 1 (>=1200) and joint 5 (1500..2150). */
  SERVO_GoHome();
  reset_sweep();
  ROBOT_SelfTestDelta(200, 1500u, 250u);

  int safe_delta = (g_pulse_min[4u] >= 1200u) &&
                   (g_pulse_min[1u] >= 1500u) && (g_pulse_max[1u] <= 1730u);
  sprintf(buf, "j1 %u..%u us, grip %u..%u us",
          g_pulse_min[4u], g_pulse_max[4u], g_pulse_min[1u], g_pulse_max[1u]);
  check("gentle delta profile stays inside calibrated bounds", safe_delta, buf);

  check("all channels parked at calibrated homes",
        g_pulse[5u] == 1000u && g_pulse[4u] == 1520u && g_pulse[3u] == 1500u &&
        g_pulse[2u] == 1600u && g_pulse[0u] == 1650u && g_pulse[1u] == 1500u, NULL);
}

static void test_servo_go_home_channels(void)
{
  printf("SERVO_GoHome & ROBOT_GoHome channel mapping\n");

  BSP_PWM_AllOff();
  reset_log();
  uint8_t f[16];
  uint16_t n = frame(f, CMD_HOME, NULL, 0u);
  feed(f, n);
  uint8_t payload[8];
  check("CMD_HOME wakes relaxed outputs & sends OK",
        find_reply(CMD_ACK, payload) == 2 && payload[1] == ST_OK &&
        g_pulse[5u] == 1000u && g_pulse[1u] == 1500u, NULL);
  run_to_idle(60000u);

  servo_cfg_t c = { 0, 1800, 1350, 500, 2500, 0 };
  (void)SERVO_ApplyConfig(0u, &c);
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { SERVO_WriteAngle(i, 900); }

  SERVO_GoHome();
  check("GoHome drove mapped ch for joint 0 to 2000us",
        g_pulse[SERVO_ChannelOf(0u)] == 2000u, NULL);
  check("GoHome left joint 5 ch at 1500us",
        g_pulse[SERVO_ChannelOf(5u)] == 1500u, NULL);

  (void)SERVO_ResetConfig();
  SERVO_GoHome();
}

static void test_cfg_persistence_and_busy(void)
{
  printf("flash calibration persistence & ST_BUSY guard\n");

  /* Trigger a move so planner is BUSY */
  uint8_t pl_ang[SERVO_NUM * 2u];
  for (uint8_t i = 0u; i < SERVO_NUM; i++) { put_s16(pl_ang, i, 1200); }
  uint8_t f[32];
  uint16_t n = frame(f, CMD_SET_ANGLE, pl_ang, (uint8_t)sizeof(pl_ang));
  feed(f, n);

  reset_log();
  n = frame(f, CMD_SAVE_CFG, NULL, 0u);
  feed(f, n);
  uint8_t payload[16];
  int len = find_reply(CMD_NAK, payload);
  check("SAVE_CFG rejected while MOVING", len == 2 && payload[1] == ST_BUSY, NULL);

  run_to_idle(60000u);

  /* Configure joint 0, query it back, save to fake flash, load, reset */
  uint8_t cfg_pl[12];
  cfg_pl[0] = 0u;
  put_s16(&cfg_pl[1], 0u, 100);
  put_s16(&cfg_pl[1], 1u, 1700);
  put_s16(&cfg_pl[1], 2u, 850);
  put_s16(&cfg_pl[1], 3u, 600);
  put_s16(&cfg_pl[1], 4u, 2400);
  cfg_pl[11] = 1u;

  reset_log();
  n = frame(f, CMD_SET_JOINT_CFG, cfg_pl, 12u);
  feed(f, n);
  len = find_reply(CMD_ACK, payload);
  check("SET_JOINT_CFG accepted when idle", len == 2 && payload[1] == ST_OK, NULL);

  reset_log();
  uint8_t j0 = 0u;
  n = frame(f, CMD_GET_JOINT_CFG, &j0, 1u);
  feed(f, n);
  len = find_reply(CMD_JOINT_CFG, payload);
  check("GET_JOINT_CFG roundtrip matches",
        len == 12 && memcmp(payload, cfg_pl, 12u) == 0, NULL);

  reset_log();
  n = frame(f, CMD_SAVE_CFG, NULL, 0u);
  feed(f, n);
  len = find_reply(CMD_ACK, payload);
  check("SAVE_CFG returns OK", len == 2 && payload[1] == ST_OK, NULL);

  servo_cfg_t loaded[SERVO_NUM];
  check("CFG_Load decodes saved blob", CFG_Load(loaded) == 0u &&
        loaded[0].home_deg == 850 && loaded[0].invert == 1u, NULL);

  reset_log();
  n = frame(f, CMD_RESET_CFG, NULL, 0u);
  feed(f, n);
  len = find_reply(CMD_ACK, payload);
  check("RESET_CFG erases flash", len == 2 && payload[1] == ST_OK &&
        CFG_Load(loaded) != 0u, NULL);

  SERVO_GoHome();
}

static void test_zmotord_mecanum(void)
{
  printf("ZMotorD verified chassis mapping & protocol\n");

  /* Verified hardware result: CAR 800 800 2200 2200 1000 = all wheels forward */
  reset_zm();
  ZM_DriveMecanum(700, 0, 0, 1000u);
  const char *want_fwd = "#006P0800T1000!#007P0800T1000!#008P2200T1000!#009P2200T1000!";
  check("forward vx=700 matches hardware-verified packet",
        g_zm_tx_len == 60u && memcmp(g_zm_tx, want_fwd, 60u) == 0,
        (const char *)g_zm_tx);

  /* Strafe left vy=400: fl=-400, fr=+400, rl=+400, rr=-400 */
  reset_zm();
  ZM_DriveMecanum(0, 400, 0, 500u);
  const char *want_strafe = "#006P1100T0500!#007P1900T0500!#008P1100T0500!#009P1900T0500!";
  check("strafe left vy=400 applies verified polarities",
        g_zm_tx_len == 60u && memcmp(g_zm_tx, want_strafe, 60u) == 0,
        (const char *)g_zm_tx);

  /* Proportional scaling when vx=800, vy=800 -> fl=0, fr=1600->1000, rl=1000, rr=0 */
  reset_zm();
  ZM_DriveMecanum(800, 800, 0, 200u);
  const char *want_scaled = "#006P0500T0200!#007P1500T0200!#008P1500T0200!#009P2500T0200!";
  check("kinematics scales proportionally on saturation",
        g_zm_tx_len == 60u && memcmp(g_zm_tx, want_scaled, 60u) == 0,
        (const char *)g_zm_tx);

  /* Protocol CMD_DRIVE_BASE */
  reset_log();
  reset_zm();
  uint8_t pl[8];
  put_s16(pl, 0u, 700);
  put_s16(pl, 1u, 0);
  put_s16(pl, 2u, 0);
  put_s16(pl, 3u, 1000);
  uint8_t f[32];
  uint16_t n = frame(f, CMD_DRIVE_BASE, pl, 8u);
  feed(f, n);
  uint8_t ack[8];
  check("CMD_DRIVE_BASE ACK + packet",
        find_reply(CMD_ACK, ack) == 2 && ack[1] == ST_OK &&
        g_zm_tx_len == 60u && memcmp(g_zm_tx, want_fwd, 60u) == 0, NULL);

  /* ESTOP stops wheels and blocks CMD_DRIVE_BASE */
  reset_log();
  reset_zm();
  n = frame(f, CMD_ESTOP, NULL, 0u);
  feed(f, n);
  check("ESTOP emits ZMotorD stop frames", g_zm_tx_len >= 60u &&
        memcmp(g_zm_tx, "#006P1500T0000!", 15u) == 0, NULL);

  reset_log();
  reset_zm();
  n = frame(f, CMD_DRIVE_BASE, pl, 8u);
  feed(f, n);
  check("CMD_DRIVE_BASE blocked during ESTOP",
        find_reply(CMD_NAK, ack) == 2 && ack[1] == ST_ESTOP && g_zm_tx_len == 0u, NULL);

  n = frame(f, CMD_CLEAR_ESTOP, NULL, 0u);
  feed(f, n);
}

static void test_selftest_delta(void)
{
  printf("per-joint delta selftest\n");

  servo_cfg_t c0 = { 0, 1800, 600, 500, 2500, 0 };
  servo_cfg_t c1 = { 0, 1800, 1200, 500, 2500, 0 };
  (void)SERVO_ApplyConfig(0u, &c0);
  (void)SERVO_ApplyConfig(1u, &c1);
  SERVO_GoHome();
  reset_sweep();

  ROBOT_SelfTestDelta(200, 300u, 40u);
  check("joint 0 swept around its own home (60 deg)",
        g_pulse_min[SERVO_ChannelOf(0u)] <= 950u &&
        g_pulse_max[SERVO_ChannelOf(0u)] >= 1380u &&
        g_pulse_max[SERVO_ChannelOf(0u)] < 1420u, NULL);
  check("joint 1 swept around its own home (120 deg)",
        g_pulse_min[SERVO_ChannelOf(1u)] >= 1600u &&
        g_pulse_min[SERVO_ChannelOf(1u)] <= 1620u &&
        g_pulse_max[SERVO_ChannelOf(1u)] >= 2050u, NULL);

  (void)SERVO_ResetConfig();
  SERVO_GoHome();
}

static void push_zm_rx(const char *s)
{
  while (*s != '\0')
  {
    g_zm_rx[g_zm_rx_head++] = (uint8_t)(*s++);
    if (g_zm_rx_head >= (uint16_t)sizeof(g_zm_rx)) { g_zm_rx_head = 0u; }
  }
}

static void feed_omv_pkt(int err, int ang, int valid, int arm)
{
  char body[40];
  char full[52];
  uint8_t csum = 0u;
  int n = sprintf(body, "L,%d,%d,%d,%d", err, ang, valid, arm);
  for (int i = 0; i < n; i++) { csum ^= (uint8_t)body[i]; }
  sprintf(full, "@%s*%02X\n", body, csum);
  push_zm_rx(full);
}

static void test_line_tracking(void)
{
  printf("OpenMV @L line tracking & protocol\n");

  g_fake_tick_ms = 100u;
  TRACK_Init();
  reset_zm();

  /* Bench preview arm=0 updates telemetry without moving wheels */
  feed_omv_pkt(-400, -10, 1, 0);
  TRACK_Poll();
  track_status_t st;
  TRACK_GetStatus(&st);
  check("arm=0 parses @L without driving",
        st.rx_frames == 1u && st.err == -400 && st.ang == -10 &&
        st.state == TRACK_ST_IDLE && g_zm_tx_len == 0u, NULL);

  /* Corrupt checksum & motor echo strings must be ignored */
  push_zm_rx("#006P1500T0000!@L,999,0,1,1*00\n");
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("bad checksum & motor echo rejected",
        st.rx_frames == 1u && st.err == -400, NULL);

  /* Autonomous arm=1 left curve (err<0 -> turn left wz>0 -> vx_R > vx_L) */
  g_fake_tick_ms = 150u;
  feed_omv_pkt(-400, -10, 1, 1);
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("err<0 produces positive wz (turn left) and reduced vx",
        st.state == TRACK_ST_FOLLOW && st.wz_cmd > 0 &&
        st.vx_cmd >= 400 && st.vx_cmd < 600 && g_zm_tx_len == 60u, NULL);

  /* Switch to right offset (err>0 -> turn right wz<0) */
  g_fake_tick_ms = 200u;
  reset_zm();
  feed_omv_pkt(500, 15, 1, 1);
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("err>0 produces negative wz (turn right)",
        st.state == TRACK_ST_FOLLOW && st.wz_cmd < 0 && g_zm_tx_len == 60u, NULL);

  /* Lost line spins toward last seen side (+direction = right -> wz<0), then stops */
  g_fake_tick_ms = 260u;
  reset_zm();
  feed_omv_pkt(500, 0, 0, 1);
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("lost line enters SEARCH spinning toward last side",
        st.state == TRACK_ST_SEARCH && st.vx_cmd == 0 && st.wz_cmd < 0, NULL);

  g_fake_tick_ms = 1600u;
  feed_omv_pkt(500, 0, 0, 1);
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("lost line timeout transitions to LOST and stops wheels",
        st.state == TRACK_ST_LOST && st.vx_cmd == 0 && st.wz_cmd == 0, NULL);

  /* Host CMD_TRACK_CFG & CMD_TRACK_MODE dry-run + CMD_READ_TRACK */
  uint8_t pl[14];
  put_s16(pl, 0u, 320);  /* vx_cruise */
  put_s16(pl, 1u, 180);  /* vx_min */
  put_s16(pl, 2u, 70);   /* kp */
  put_s16(pl, 3u, 30);   /* kd */
  put_s16(pl, 4u, 45);   /* k_ang */
  put_s16(pl, 5u, 450);  /* wz_max */
  put_s16(pl, 6u, 250);  /* search_wz */
  uint8_t f[32];
  reset_log();
  feed(f, frame(f, CMD_TRACK_CFG, pl, 14u));
  uint8_t ack[20];
  check("CMD_TRACK_CFG ACK", find_reply(CMD_ACK, ack) == 2 && ack[1] == ST_OK, NULL);

  uint8_t mode_pl[2] = { 1u, 1u }; /* ACTIVE, dry_run=1 */
  reset_log();
  feed(f, frame(f, CMD_TRACK_MODE, mode_pl, 2u));
  check("CMD_TRACK_MODE ACK", find_reply(CMD_ACK, ack) == 2 && ack[1] == ST_OK, NULL);

  g_fake_tick_ms = 1700u;
  reset_zm();
  feed_omv_pkt(-300, 0, 1, 0);
  TRACK_Poll();
  check("dry-run never emits motor frames", g_zm_tx_len == 0u, NULL);

  reset_log();
  feed(f, frame(f, CMD_READ_TRACK, NULL, 0u));
  check("CMD_READ_TRACK replies CMD_TRACK_STAT (16B)",
        find_reply(CMD_TRACK_STAT, ack) == 16 && ack[0] == 1u && ack[1] == 1u &&
        get_s16(&ack[4], 0u) == -300 && get_s16(&ack[4], 3u) > 0, NULL);

  /* Heartbeat timeout stops active drive */
  uint8_t live_pl[2] = { 1u, 0u };
  feed(f, frame(f, CMD_TRACK_MODE, live_pl, 2u));
  g_fake_tick_ms = 1750u;
  feed_omv_pkt(-200, 0, 1, 0);
  TRACK_Poll();
  reset_zm();
  g_fake_tick_ms = 2200u; /* > 350ms without OpenMV packet */
  TRACK_Poll();
  TRACK_GetStatus(&st);
  check("OpenMV RX timeout stops wheels & resets state",
        st.state == TRACK_ST_IDLE && g_zm_tx_len >= 60u &&
        memcmp(g_zm_tx, "#006P1500T0000!", 15u) == 0, NULL);

  TRACK_Init();
}

int main(void)
{
  printf("firmware core test\n\n");
  memset(g_fake_flash, 0xFF, sizeof(g_fake_flash));

  ROBOT_Init();
  PROTO_Init();
  reset_sweep();

  test_crc();
  test_read_pose();
  test_joint_map();
  test_set_angle();
  test_speed_law();
  test_duration_clamp();
  test_estop();
  test_selftest();
  test_servo_go_home_channels();
  test_cfg_persistence_and_busy();
  test_zmotord_mecanum();
  test_selftest_delta();
  test_line_tracking();

  printf("\n%s\n", g_fail == 0 ? "PASS" : "FAILURES PRESENT");
  return (g_fail == 0) ? 0 : 1;
}
