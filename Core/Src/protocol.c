/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   Byte-level frame assembler + command dispatcher.
  *
  *          Design notes
  *          ------------
  *          - A tiny state machine (hunt header -> len -> body -> crc) is more
  *            robust than "read N bytes then parse": a single lost byte only
  *            costs one frame, because the next 0xAA 0x55 re-syncs itself.
  *          - CRC covers CMD..PAYLOAD (not the header/len) so that the length
  *            byte cannot be silently corrupted into a huge allocation.
  *          - Replies are built on the stack and sent in one blocking call;
  *            frames are <= 27 bytes so this costs < 3ms at 115200.
  ******************************************************************************
  */
#include "protocol.h"
#include "usart.h"
#include "robot.h"
#include "zmotord.h"
#include "line_track.h"

typedef enum
{
  S_HUNT1 = 0,
  S_HUNT2,
  S_LEN,
  S_CMD,
  S_PAYLOAD,
  S_CRC
} rx_state_t;

static rx_state_t s_rx_state;
static uint8_t    s_len;
static uint8_t    s_cmd;
static uint8_t    s_payload[PROTO_MAX_PAYLOAD];
static uint8_t    s_idx;

static uint16_t   s_move_ms;   /* duration used by the next SET_ANGLE */

void PROTO_Init(void)
{
  s_rx_state = S_HUNT1;
  s_idx      = 0u;
  s_move_ms  = 0u;             /* 0 = planner chooses */
}

/* CRC8 (poly 0x31, init 0xFF) as an accumulating primitive. The receiver must
   run it over CMD then over PAYLOAD separately: the two live in different
   objects and C does not guarantee they are adjacent in memory. */
static uint8_t crc8_upd(uint8_t crc, const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0u; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0u; b < 8u; b++)
    {
      crc = ((crc & 0x80u) != 0u) ? (uint8_t)((crc << 1) ^ 0x31u)
                                  : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

uint8_t PROTO_Crc8(const uint8_t *data, uint16_t len)
{
  return crc8_upd(0xFFu, data, len);
}

/* --- TX helpers ---------------------------------------------------------- */

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
  uint8_t buf[6u + PROTO_MAX_PAYLOAD];
  uint8_t n = 0u;

  buf[n++] = PROTO_HEAD1;
  buf[n++] = PROTO_HEAD2;
  buf[n++] = len;
  buf[n++] = cmd;

  for (uint8_t i = 0u; i < len; i++)
  {
    buf[n++] = payload[i];
  }

  buf[n++] = PROTO_Crc8(&buf[3], (uint16_t)(1u + len));

  UART_WriteBuf(buf, n);
}

static void send_ack(uint8_t cmd, uint8_t status)
{
  uint8_t p[2] = { cmd, status };

  if (status == ST_OK)
  {
    send_frame(CMD_ACK, p, 2u);
  }
  else
  {
    send_frame(CMD_NAK, p, 2u);
  }
}

static void send_pose(void)
{
  int16_t  pose[SERVO_NUM];
  uint8_t  p[SERVO_NUM * 2u];

  ROBOT_GetPose(pose);

  for (uint8_t i = 0u; i < SERVO_NUM; i++)
  {
    p[i * 2u]      = (uint8_t)((uint16_t)pose[i] & 0xFFu);
    p[i * 2u + 1u] = (uint8_t)(((uint16_t)pose[i] >> 8) & 0xFFu);
  }

  send_frame(CMD_POSE, p, (uint8_t)sizeof(p));
}

/* --- Command execution --------------------------------------------------- */

static int16_t rd_s16(const uint8_t *p, uint8_t i)
{
  return (int16_t)((uint16_t)p[i * 2u] | ((uint16_t)p[i * 2u + 1u] << 8));
}

static uint16_t rd16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void wr16(uint8_t *p, int16_t v)
{
  uint16_t u = (uint16_t)v;

  p[0] = (uint8_t)(u & 0xFFu);
  p[1] = (uint8_t)((u >> 8) & 0xFFu);
}

static void handle_frame(uint8_t cmd, const uint8_t *pl, uint8_t len)
{
  int16_t targets[SERVO_NUM];

  switch (cmd)
  {
    case CMD_SET_ANGLE:
    {
      if (len != (SERVO_NUM * 2u)) { send_ack(cmd, ST_BAD_LEN); return; }

      for (uint8_t i = 0u; i < SERVO_NUM; i++)
      {
        targets[i] = rd_s16(pl, i);
      }

      if (ROBOT_MoveToAuto(targets, s_move_ms) != 0)
      {
        send_ack(cmd, ST_ESTOP);
        return;
      }

      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_SET_TIME:
    {
      if (len != 2u) { send_ack(cmd, ST_BAD_LEN); return; }
      s_move_ms = (uint16_t)((uint16_t)pl[0] | ((uint16_t)pl[1] << 8));
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_READ_POSE:
    {
      send_pose();
      break;
    }

    case CMD_HOME:
    {
      if (len != 0u) { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_GetState() == ROBOT_ESTOP) { send_ack(cmd, ST_ESTOP); return; }
      ROBOT_GoHome();
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_ESTOP:
    {
      ROBOT_EmergencyStop();
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_CLEAR_ESTOP:
    {
      ROBOT_ClearEmergency();
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_SET_SPEED:
    {
      if (len != 1u)       { send_ack(cmd, ST_BAD_LEN); return; }
      if (pl[0] == 0u)     { send_ack(cmd, ST_LIMIT);   return; }
      ROBOT_SetSpeed(pl[0]);
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_SELFTEST:
    {
      if (len != 0u) { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_GetState() == ROBOT_ESTOP) { send_ack(cmd, ST_ESTOP); return; }

      /* Gentle by design: +/-20 deg around each joint's own home rather than
         a full-range sweep. ACK first, because the sweep blocks the main
         loop; bytes that arrive meanwhile queue in the 256-byte RX ring. */
      send_ack(cmd, ST_OK);
      ROBOT_SelfTestDelta(200, 800u, 100u);
      break;
    }

    case CMD_STOP:
    {
      if (len != 0u) { send_ack(cmd, ST_BAD_LEN); return; }
      ROBOT_Stop();
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_SET_JOINT_CFG:
    {
      servo_cfg_t c;
      uint8_t     j;

      if (len != 12u)                { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_IsBusy() != 0u)      { send_ack(cmd, ST_BUSY);    return; }

      j = pl[0];
      if (j >= SERVO_NUM)            { send_ack(cmd, ST_LIMIT);   return; }

      c.min_deg      = (int16_t)rd16(&pl[1]);
      c.max_deg      = (int16_t)rd16(&pl[3]);
      c.home_deg     = (int16_t)rd16(&pl[5]);
      c.pulse_min_us = (int16_t)rd16(&pl[7]);
      c.pulse_max_us = (int16_t)rd16(&pl[9]);
      c.invert       = pl[11];

      if (SERVO_ApplyConfig(j, &c) != 0) { send_ack(cmd, ST_LIMIT); return; }
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_GET_JOINT_CFG:
    {
      const servo_cfg_t *c;
      uint8_t p[12];
      uint8_t j;

      if (len != 1u)      { send_ack(cmd, ST_BAD_LEN); return; }

      j = pl[0];
      if (j >= SERVO_NUM) { send_ack(cmd, ST_LIMIT);   return; }

      c     = SERVO_GetConfig(j);
      p[0]  = j;
      wr16(&p[1],  c->min_deg);
      wr16(&p[3],  c->max_deg);
      wr16(&p[5],  c->home_deg);
      wr16(&p[7],  c->pulse_min_us);
      wr16(&p[9],  c->pulse_max_us);
      p[11] = c->invert;

      send_frame(CMD_JOINT_CFG, p, 12u);
      break;
    }

    case CMD_SAVE_CFG:
    {
      if (len != 0u)            { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_IsBusy() != 0u) { send_ack(cmd, ST_BUSY);    return; }
      send_ack(cmd, (SERVO_SaveConfig() == 0u) ? ST_OK : ST_FLASH);
      break;
    }

    case CMD_RESET_CFG:
    {
      if (len != 0u)            { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_IsBusy() != 0u) { send_ack(cmd, ST_BUSY);    return; }
      send_ack(cmd, (SERVO_ResetConfig() == 0u) ? ST_OK : ST_FLASH);
      break;
    }

    case CMD_DRIVE_BASE:
    {
      if (len != 8u)                       { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_GetState() == ROBOT_ESTOP) { send_ack(cmd, ST_ESTOP);   return; }
      TRACK_ForceOff();
      ZM_DriveMecanum(rd_s16(pl, 0u), rd_s16(pl, 1u), rd_s16(pl, 2u), rd16(&pl[6]));
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_DRIVE_WHEELS:
    {
      if (len != 10u)                      { send_ack(cmd, ST_BAD_LEN); return; }
      if (ROBOT_GetState() == ROBOT_ESTOP) { send_ack(cmd, ST_ESTOP);   return; }
      TRACK_ForceOff();
      ZM_SetWheels(rd_s16(pl, 0u), rd_s16(pl, 1u), rd_s16(pl, 2u), rd_s16(pl, 3u),
                   rd16(&pl[8]));
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_TRACK_MODE:
    {
      if (len != 2u) { send_ack(cmd, ST_BAD_LEN); return; }
      if (TRACK_SetMode(pl[0], pl[1]) != 0)
      {
        send_ack(cmd, (ROBOT_GetState() == ROBOT_ESTOP) ? ST_ESTOP : ST_LIMIT);
        return;
      }
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_TRACK_CFG:
    {
      track_cfg_t c;

      if (len != 14u) { send_ack(cmd, ST_BAD_LEN); return; }

      c.vx_cruise = (int16_t)rd16(&pl[0]);
      c.vx_min    = (int16_t)rd16(&pl[2]);
      c.kp        = rd16(&pl[4]);
      c.kd        = rd16(&pl[6]);
      c.k_ang     = rd16(&pl[8]);
      c.wz_max    = rd16(&pl[10]);
      c.search_wz = rd16(&pl[12]);

      if (TRACK_SetConfig(&c) != 0) { send_ack(cmd, ST_LIMIT); return; }
      send_ack(cmd, ST_OK);
      break;
    }

    case CMD_READ_TRACK:
    {
      track_status_t st;
      uint8_t p[16];

      if (len != 0u) { send_ack(cmd, ST_BAD_LEN); return; }

      TRACK_GetStatus(&st);
      p[0] = st.mode;
      p[1] = st.dry_run;
      p[2] = st.state;
      p[3] = st.valid;
      wr16(&p[4],  st.err);
      wr16(&p[6],  st.ang);
      wr16(&p[8],  st.vx_cmd);
      wr16(&p[10], st.wz_cmd);
      wr16(&p[12], (int16_t)st.age_ms);
      wr16(&p[14], (int16_t)st.rx_frames);

      send_frame(CMD_TRACK_STAT, p, 16u);
      break;
    }

    default:
      send_ack(cmd, ST_BAD_CMD);
      break;
  }
}

/* --- Byte-level state machine -------------------------------------------- */

static void feed_byte(uint8_t b)
{
  switch (s_rx_state)
  {
    case S_HUNT1:
      if (b == PROTO_HEAD1) { s_rx_state = S_HUNT2; }
      break;

    case S_HUNT2:
      s_rx_state = (b == PROTO_HEAD2) ? S_LEN : S_HUNT1;
      break;

    case S_LEN:
      s_len = b;
      if (s_len > PROTO_MAX_PAYLOAD)
      {
        s_rx_state = S_HUNT1;   /* bogus length: resync */
      }
      else
      {
        s_rx_state = S_CMD;
      }
      break;

    case S_CMD:
      s_cmd = b;
      s_idx = 0u;
      s_rx_state = (s_len == 0u) ? S_CRC : S_PAYLOAD;
      break;

    case S_PAYLOAD:
      s_payload[s_idx++] = b;
      if (s_idx >= s_len) { s_rx_state = S_CRC; }
      break;

    case S_CRC:
    {
      uint8_t crc = crc8_upd(crc8_upd(0xFFu, &s_cmd, 1u), s_payload, s_len);

      if (crc == b)
      {
        handle_frame(s_cmd, s_payload, s_len);
      }
      else
      {
        send_ack(s_cmd, ST_BAD_CRC);
      }

      s_rx_state = S_HUNT1;
      break;
    }

    default:
      s_rx_state = S_HUNT1;
      break;
  }
}

void PROTO_Poll(void)
{
  uint8_t b;

  while (UART_ReadByte(&b) == 0)
  {
    feed_byte(b);
  }
}
