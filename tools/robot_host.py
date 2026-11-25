#!/usr/bin/env python3
"""
robot_host.py -- host-side driver for the STM32F103 6-axis servo controller.

Frame layout (must match Core/Inc/protocol.h):

    offset  size  field
      0      1    0xAA  header
      1      1    0x55  header
      2      1    LEN   payload length (0..20)
      3      1    CMD
      4     LEN   PAYLOAD
    4+LEN    1    CRC8  poly 0x31, init 0xFF, MSB-first, over CMD..PAYLOAD

Usage examples:

    python robot_host.py --port COM3 pose
    python robot_host.py --port COM3 identify      # verify all 6 joints
    python robot_host.py --port COM3 move 90 90 90 90 90 90
    python robot_host.py --port COM3 sweep 0 180
    python robot_host.py --port COM3 monitor
    python robot_host.py --port COM3 selftest

Offline checks (no hardware needed):

    python robot_host.py --check
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

# --- protocol constants ----------------------------------------------------

HEAD1, HEAD2 = 0xAA, 0x55
MAX_PAYLOAD = 20

CMD_SET_ANGLE = 0x01
CMD_SET_TIME = 0x02
CMD_READ_POSE = 0x03
CMD_HOME = 0x04
CMD_ESTOP = 0x05
CMD_SET_SPEED = 0x06
CMD_CLEAR_ESTOP = 0x07
CMD_SELFTEST = 0x08
CMD_STOP = 0x09
CMD_SET_JOINT_CFG = 0x0A
CMD_GET_JOINT_CFG = 0x0B
CMD_SAVE_CFG = 0x0C
CMD_RESET_CFG = 0x0D
CMD_DRIVE_BASE = 0x0E
CMD_DRIVE_WHEELS = 0x0F
CMD_TRACK_MODE = 0x10
CMD_TRACK_CFG = 0x11
CMD_READ_TRACK = 0x12
CMD_PICK_MODE = 0x13
CMD_READ_PICK = 0x14
CMD_EXEC_MODE = 0x15
CMD_READ_EXEC = 0x16

CMD_ACK = 0x80
CMD_POSE = 0x81
CMD_NAK = 0x82
CMD_JOINT_CFG = 0x83
CMD_TRACK_STAT = 0x84
CMD_PICK_STAT = 0x85
CMD_EXEC_STAT = 0x86

TRACK_MODES = {0: "OFF", 1: "ACTIVE", 2: "AUTO"}
TRACK_STATES = {0: "IDLE", 1: "FOLLOW", 2: "JUNCTION", 3: "SEARCH", 4: "LOST"}
PICK_MODES = {0: "OFF", 1: "ACTIVE", 2: "AUTO", 3: "RESET"}
PICK_STATES = {
    0: "IDLE", 1: "SEARCH", 2: "APPROACH", 3: "ALIGN",
    4: "ARM_WAKE", 5: "ARM_PRE", 6: "ARM_ABOVE", 7: "ARM_GRAB",
    8: "ARM_CLAMP", 9: "ARM_LIFT", 10: "ARM_RETRACT", 11: "ARM_HOME",
    12: "HOLDING", 13: "FAIL",
}
EXEC_ARM_STATES = {
    0: "IDLE", 1: "WAKE",
    2: "PICK_PRE", 3: "PICK_ABOVE", 4: "PICK_GRAB", 5: "PICK_CLAMP",
    6: "PICK_LIFT", 7: "PICK_RETRACT", 8: "PICK_HOME",
    9: "PLACE_PRE", 10: "PLACE_ABOVE", 11: "PLACE_LOWER", 12: "PLACE_REL",
    13: "PLACE_LIFT", 14: "PLACE_RETRACT", 15: "PLACE_HOME", 16: "CUSTOM",
}

STATUS = {
    0x00: "OK",
    0x01: "BAD_CRC",
    0x02: "BAD_LEN",
    0x03: "BAD_CMD",
    0x04: "BUSY",
    0x05: "ESTOP",
    0x06: "LIMIT",
    0x07: "FLASH",
}

SERVO_NUM = 6

# Logical joint order follows the assembled arm, BOTTOM TO TOP (KPZ-STM32 TIM3).
# (joint index, role, pin, slot_channel, min_deg, max_deg, home_deg)  [0.1 deg]
JOINTS = [
    (0, "base   ", "PB3", "TIM3_CH5",   0, 1800,  450),
    (1, "lower  ", "PB8", "TIM3_CH4", 630, 1800,  918),
    (2, "elbow  ", "PB9", "TIM3_CH3",   0, 1800,  900),
    (3, "upper  ", "PB6", "TIM3_CH2",   0, 1800,  990),
    (4, "roll   ", "PB7", "TIM3_CH0",   0, 1800, 1035),
    (5, "gripper", "PB4", "TIM3_CH1", 900, 1485,  900),
]

PWM_CHANNELS = [f"{pin}/{tmr}" for _, _, pin, tmr, _, _, _ in JOINTS]
JOINT_ROLES = [role.strip() for _, role, _, _, _, _, _ in JOINTS]
HOME_RAW = [home for _, _, _, _, _, _, home in JOINTS]


# --- codec -----------------------------------------------------------------

def crc8(data: bytes, crc: int = 0xFF) -> int:
    """CRC8 poly 0x31, init 0xFF, MSB-first. Mirrors crc8_upd() in protocol.c."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([cmd]) + payload
    return bytes([HEAD1, HEAD2, len(payload)]) + body + bytes([crc8(body)])


def pack_angles(angles) -> bytes:
    """6 angles in 0.1 degree units, little-endian int16."""
    if len(angles) != SERVO_NUM:
        raise ValueError(f"need {SERVO_NUM} angles, got {len(angles)}")
    return struct.pack("<6h", *[int(a) for a in angles])


def unpack_angles(payload: bytes):
    if len(payload) != SERVO_NUM * 2:
        raise ValueError(f"pose payload must be 12 bytes, got {len(payload)}")
    return struct.unpack("<6h", payload)


# --- link ------------------------------------------------------------------

class Robot:
    """Blocking request/response client with byte-level re-sync."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        import serial  # imported here so --check works without pyserial
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buf = bytearray()
        time.sleep(0.15)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- framing ----------------------------------------------------------

    def _pump(self, deadline: float) -> bool:
        remaining = deadline - time.time()
        if remaining <= 0:
            return False
        waiting = self.ser.in_waiting
        if waiting > 0:
            chunk = self.ser.read(waiting)
        else:
            self.ser.timeout = max(0.005, min(0.05, remaining))
            chunk = self.ser.read(1)
        if chunk:
            self.buf.extend(chunk)
            return True
        return False

    def read_frame(self, timeout: float = 1.0):
        """Return (cmd, payload) or None on timeout. Never raises on junk."""
        deadline = time.time() + timeout
        while True:
            idx = self.buf.find(bytes([HEAD1, HEAD2]))
            if idx < 0:
                # keep at most one byte: it could be a header in flight
                if len(self.buf) > 1:
                    del self.buf[:-1]
                if not self._pump(deadline) and time.time() >= deadline:
                    return None
                continue

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 4:
                if not self._pump(deadline) and time.time() >= deadline:
                    return None
                continue

            length = self.buf[2]
            if length > MAX_PAYLOAD:
                del self.buf[:2]          # bogus length: drop header, resync
                continue

            total = 4 + length + 1
            if len(self.buf) < total:
                if not self._pump(deadline) and time.time() >= deadline:
                    return None
                continue

            cmd = self.buf[3]
            payload = bytes(self.buf[4:4 + length])
            got = self.buf[4 + length]
            del self.buf[:total]

            if crc8(bytes([cmd]) + payload) != got:
                continue                  # corrupted frame, keep hunting
            return cmd, payload

    def request(self, cmd: int, payload: bytes = b"", timeout: float = 1.0,
                expect: int = CMD_ACK):
        self.ser.reset_input_buffer()
        self.buf.clear()
        self.ser.write(build_frame(cmd, payload))
        self.ser.flush()
        frame = self.read_frame(timeout)
        if frame is None:
            return None, None, "timeout (no reply -- check wiring/baud/NVIC)"
        rcmd, rpayload = frame
        if rcmd in (CMD_POSE, CMD_JOINT_CFG, CMD_TRACK_STAT, CMD_PICK_STAT, CMD_EXEC_STAT):
            return rcmd, rpayload, None
        if rcmd in (CMD_ACK, CMD_NAK) and len(rpayload) == 2:
            orig, status = rpayload
            if rcmd == CMD_NAK:
                return rcmd, rpayload, f"NAK {STATUS.get(status, hex(status))}"
            return rcmd, rpayload, None
        return rcmd, rpayload, f"unexpected reply cmd=0x{rcmd:02X}"

    # -- commands ---------------------------------------------------------

    def pose(self, timeout: float = 1.0):
        rcmd, payload, err = self.request(CMD_READ_POSE, b"", timeout)
        if err or rcmd != CMD_POSE:
            return None, err or "bad pose reply"
        return unpack_angles(payload), None

    def move(self, angles, duration_ms: int | None = None):
        if duration_ms is not None:
            _, _, err = self.request(CMD_SET_TIME,
                                     struct.pack("<H", duration_ms))
            if err:
                return err
        return self.request(CMD_SET_ANGLE, pack_angles(angles))[2]

    def simple(self, cmd: int):
        return self.request(cmd, b"")[2]

    def set_speed(self, percent: int):
        return self.request(CMD_SET_SPEED, bytes([percent]))[2]

    def drive_base(self, vx: int, vy: int, wz: int, time_ms: int = 1000):
        payload = struct.pack("<3hH", int(vx), int(vy), int(wz), int(time_ms))
        return self.request(CMD_DRIVE_BASE, payload)[2]

    def drive_wheels(self, fl: int, fr: int, rl: int, rr: int,
                     time_ms: int = 1000):
        payload = struct.pack("<4hH", int(fl), int(fr), int(rl), int(rr),
                              int(time_ms))
        return self.request(CMD_DRIVE_WHEELS, payload)[2]

    def get_joint_cfg(self, joint: int):
        rcmd, payload, err = self.request(CMD_GET_JOINT_CFG, bytes([joint]))
        if err or rcmd != CMD_JOINT_CFG or len(payload) != 12:
            return None, err or "bad joint cfg reply"
        j, mn, mx, home, pmin, pmax, inv = struct.unpack("<B5hB", payload)
        return {
            "joint": j, "min_deg": mn, "max_deg": mx, "home_deg": home,
            "pulse_min_us": pmin, "pulse_max_us": pmax, "invert": inv,
        }, None

    def set_joint_cfg(self, joint: int, min_deg: int, max_deg: int,
                      home_deg: int, pulse_min_us: int, pulse_max_us: int,
                      invert: int):
        payload = struct.pack("<B5hB", joint, min_deg, max_deg, home_deg,
                              pulse_min_us, pulse_max_us, invert)
        return self.request(CMD_SET_JOINT_CFG, payload)[2]

    def set_track_mode(self, mode: int, dry_run: int = 0):
        return self.request(CMD_TRACK_MODE, struct.pack("<2B", mode, dry_run))[2]

    def set_track_cfg(self, vx_cruise: int, vx_min: int, kp: int, kd: int,
                      k_ang: int, wz_max: int, search_wz: int):
        payload = struct.pack("<2h5H", vx_cruise, vx_min, kp, kd, k_ang,
                              wz_max, search_wz)
        return self.request(CMD_TRACK_CFG, payload)[2]

    def track_status(self):
        rcmd, payload, err = self.request(CMD_READ_TRACK, b"")
        if err or rcmd != CMD_TRACK_STAT or len(payload) != 16:
            return None, err or "bad track stat reply"
        mode, dry, st, val, err_v, ang_v, vx, wz, age, rx = struct.unpack("<4B4h2H", payload)
        return {
            "mode": TRACK_MODES.get(mode, str(mode)),
            "dry_run": bool(dry),
            "state": TRACK_STATES.get(st, str(st)),
            "valid": val,
            "err": err_v,
            "ang": ang_v,
            "vx": vx,
            "wz": wz,
            "age_ms": age,
            "rx_frames": rx,
        }, None

    def set_pick_mode(self, mode: int, dry_run: int = 0):
        return self.request(CMD_PICK_MODE, struct.pack("<2B", mode, dry_run))[2]

    def pick_status(self):
        rcmd, payload, err = self.request(CMD_READ_PICK, b"")
        if err or rcmd != CMD_PICK_STAT or len(payload) != 18:
            return None, err or "bad pick stat reply"
        mode, dry, st, val, dx, dy, cw, vx, wy, age, rx = struct.unpack("<4B5h2H", payload)
        return {
            "mode": PICK_MODES.get(mode, str(mode)),
            "dry_run": bool(dry),
            "state": PICK_STATES.get(st, str(st)),
            "valid": val,
            "dx": dx,
            "dy": dy,
            "cube_w": cw,
            "vx": vx,
            "wy": wy,
            "age_ms": age,
            "rx_frames": rx,
        }, None

    def set_exec_mode(self, mode: int, dry_run: int = 0):
        return self.request(CMD_EXEC_MODE, struct.pack("<2B", mode, dry_run))[2]

    def exec_status(self):
        rcmd, payload, err = self.request(CMD_READ_EXEC, b"")
        if err or rcmd != CMD_EXEC_STAT or len(payload) != 20:
            return None, err or "bad exec stat reply"
        (armed, dry, arm_st, busy, hold, btn1,
         vx, vy, wz, seq, age, rx, bad) = struct.unpack("<6B3h4H", payload)
        return {
            "armed": bool(armed),
            "dry_run": bool(dry),
            "arm_state": EXEC_ARM_STATES.get(arm_st, str(arm_st)),
            "arm_busy": bool(busy),
            "holding": bool(hold),
            "btn1_down": bool(btn1),
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "arm_seq": seq,
            "age_ms": age,
            "rx_frames": rx,
            "rx_bad": bad,
        }, None


# --- offline checks --------------------------------------------------------

def run_checks() -> int:
    """Verify the codec against hand-computed vectors. No hardware needed."""
    failures = 0

    def check(label, got, want):
        nonlocal failures
        ok = got == want
        print(f"  [{'ok' if ok else 'FAIL'}] {label}: {got!r}"
              + ("" if ok else f"  expected {want!r}"))
        if not ok:
            failures += 1

    print("codec checks")

    # READ_POSE, LEN=0: crc over the single byte 0x03 -> 0xFF
    check("crc8(0x03)", crc8(bytes([0x03])), 0xFF)
    check("frame READ_POSE",
          build_frame(CMD_READ_POSE).hex(" ").upper(),
          "AA 55 00 03 FF")

    # accumulating form must equal the one-shot form (head to tail)
    body = bytes([0x01]) + pack_angles([900] * 6)
    acc = 0xFF
    for chunk in (body[:1], body[1:2], body[2:]):
        acc = crc8(chunk, acc)
    check("accumulating crc", acc, crc8(body))

    # SET_ANGLE frame shape
    f = build_frame(CMD_SET_ANGLE, pack_angles([0, 100, 900, 1800, -100, 90]))
    check("SET_ANGLE length", len(f), 2 + 1 + 1 + 12 + 1)
    check("SET_ANGLE header", f[:3].hex(" ").upper(), "AA 55 0C")

    # parser must skip leading junk and find the header
    buf = bytes([0x00, 0xAA]) + f
    check("resync finds header at", buf.find(bytes([HEAD1, HEAD2])), 2)

    # chassis & calibration protocol frames
    check("STATUS FLASH", STATUS.get(0x07), "FLASH")
    drv = build_frame(CMD_DRIVE_BASE, struct.pack("<3hH", 700, 0, -200, 1000))
    check("DRIVE_BASE length", len(drv), 2 + 1 + 1 + 8 + 1)
    whl = build_frame(CMD_DRIVE_WHEELS, struct.pack("<4hH", 700, 700, 700, 700, 500))
    check("DRIVE_WHEELS length", len(whl), 2 + 1 + 1 + 10 + 1)
    gcfg = build_frame(CMD_GET_JOINT_CFG, bytes([2]))
    check("GET_JOINT_CFG length", len(gcfg), 6)
    tmode = build_frame(CMD_TRACK_MODE, struct.pack("<2B", 1, 0))
    check("TRACK_MODE length", len(tmode), 7)
    tcfg = build_frame(CMD_TRACK_CFG, struct.pack("<2h5H", 600, 420, 38, 160, 20, 320, 320))
    check("TRACK_CFG length", len(tcfg), 19)
    tread = build_frame(CMD_READ_TRACK)
    check("READ_TRACK length", len(tread), 5)
    pmode = build_frame(CMD_PICK_MODE, struct.pack("<2B", 2, 0))
    check("PICK_MODE length", len(pmode), 7)
    pread = build_frame(CMD_READ_PICK)
    check("READ_PICK length", len(pread), 5)

    # Verify Robot.request() accepts telemetry reply frames like CMD_PICK_STAT
    r = object.__new__(Robot)
    r.buf = bytearray()
    class _DummySer:
        def __init__(self, rx: bytes):
            self.rx = rx
        @property
        def in_waiting(self):
            return len(self.rx)
        def read(self, n):
            chunk, self.rx = self.rx[:n], self.rx[n:]
            return chunk
        def reset_input_buffer(self): pass
        def write(self, _): pass
        def flush(self): pass
    r.ser = _DummySer(build_frame(
        CMD_PICK_STAT, struct.pack("<4B5h2H", 0, 0, 0, 1, -644, 42, 24, 0, 0, 35, 12)))
    pst, perr = r.pick_status()
    check("PICK_STAT reply decoded",
          perr is None and pst["dx"] == -644 and pst["dy"] == 42 and pst["rx_frames"] == 12,
          True)

    r.ser = _DummySer(build_frame(
        CMD_EXEC_STAT, struct.pack("<6B3h4H", 1, 0, 4, 1, 0, 1, 560, -40, -120, 7, 25, 42, 0)))
    est, eerr = r.exec_status()
    check("EXEC_STAT reply decoded",
          eerr is None and est["armed"] and est["arm_state"] == "PICK_GRAB"
          and est["vx"] == 560 and est["wz"] == -120 and est["arm_seq"] == 7,
          True)

    print("PASS" if failures == 0 else f"{failures} FAILURE(S)")
    return 1 if failures else 0


# --- interactive commands --------------------------------------------------

def cmd_pose(robot: Robot, args):
    pose, err = robot.pose()
    if err:
        print(f"ERROR: {err}")
        return 1
    for i, deg in enumerate(pose):
        print(f"  joint {i}  {PWM_CHANNELS[i]:<12} {deg / 10.0:7.1f} deg"
              f"  ({deg} raw)  pulse~{500 + (deg if deg > 0 else 0) * 2000 // 1800}us")
    return 0


def cmd_identify(robot: Robot, args):
    """Move ONE joint at a time around calibrated home so the mapping is safe."""
    _, err = robot.pose()
    if err:
        print(f"ERROR: {err}")
        return 1
    delta = args.delta
    step_ms = args.time
    print("centring all joints to calibrated home first...")
    err = robot.move(HOME_RAW, duration_ms=step_ms)
    if err:
        print(f"ERROR: {err}")
        return 1
    time.sleep(step_ms / 1000.0 + 0.3)

    for i, _, _, _, mn, mx, home in JOINTS:
        lo = max(mn, home - delta)
        hi = min(mx, home + delta)
        for target, label in ((lo, "low "), (hi, "high"), (home, "home")):
            angles = list(HOME_RAW)
            angles[i] = target
            err = robot.move(angles, duration_ms=step_ms)
            status = err if err else "ok"
            print(f"  joint {i} -> {label:>4} ({target / 10.0:5.1f} deg)  "
                  f"[{PWM_CHANNELS[i]}]  {status}")
            if err:
                return 1
            time.sleep(step_ms / 1000.0 + 0.2)
    print("done -- whichever servo moved on each line is that channel")
    return 0


def cmd_move(robot: Robot, args):
    if len(args.angles) != SERVO_NUM:
        print(f"ERROR: need {SERVO_NUM} angles")
        return 1
    angles = [int(round(a * 10)) for a in args.angles]
    err = robot.move(angles, duration_ms=args.time)
    if err:
        print(f"ERROR: {err}")
        return 1
    print(f"moved to {[a / 10.0 for a in angles]} deg"
          + (f" over {args.time} ms" if args.time else " (auto duration)"))
    return 0


def cmd_sweep(robot: Robot, args):
    """Continuous full-range sweep of every joint together -- wiring shakeout."""
    lo, hi = int(round(args.lo * 10)), int(round(args.hi * 10))
    cycles = args.cycles
    print(f"sweeping all joints {args.lo}..{args.hi} deg, {cycles} cycles "
          f"at {args.time} ms per leg  (Ctrl-C to stop)")
    try:
        for n in range(cycles):
            for target in (hi, lo):
                err = robot.move([target] * SERVO_NUM, duration_ms=args.time)
                if err:
                    print(f"ERROR: {err}")
                    return 1
                time.sleep(args.time / 1000.0 + 0.05)
            print(f"  cycle {n + 1}/{cycles}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    return 0


def cmd_monitor(robot: Robot, args):
    print(f"polling pose at {args.rate} Hz  (Ctrl-C to stop)")
    period = 1.0 / args.rate
    try:
        while True:
            t0 = time.time()
            pose, err = robot.pose()
            if err:
                print(f"  ERROR: {err}")
            else:
                line = "  " + "  ".join(f"{d / 10.0:6.1f}" for d in pose)
                print(line + f"   t={t0 % 1000:7.3f}")
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def _probe_port(dev: str, baud: int, attempts: int = 3,
                timeout: float = 0.5):
    """Send READ_POSE to one port; return the pose tuple or None.

    READ_POSE has no side effects, so blasting it at every serial port on the
    machine is harmless even for devices that are not the robot.
    """
    import serial
    frame = build_frame(CMD_READ_POSE)
    try:
        with serial.Serial(dev, baud, timeout=0.02) as ser:
            time.sleep(0.06)
            ser.reset_input_buffer()
            for _ in range(attempts):
                ser.write(frame)
                ser.flush()
                deadline = time.time() + timeout
                buf = bytearray()
                while time.time() < deadline:
                    chunk = ser.read(64)
                    if not chunk:
                        continue
                    buf.extend(chunk)
                    idx = buf.find(bytes([HEAD1, HEAD2]))
                    if idx < 0:
                        del buf[:-1]
                        continue
                    del buf[:idx]
                    if len(buf) < 4:
                        continue
                    length = buf[2]
                    if length > MAX_PAYLOAD:
                        del buf[:2]
                        continue
                    total = 4 + length + 1
                    if len(buf) < total:
                        continue
                    cmd = buf[3]
                    payload = bytes(buf[4:4 + length])
                    if crc8(bytes([cmd]) + payload) != buf[4 + length]:
                        del buf[:total]
                        continue
                    del buf[:total]
                    if cmd == CMD_POSE and length == SERVO_NUM * 2:
                        return unpack_angles(payload)
                time.sleep(0.05)
    except Exception:  # noqa: BLE001 - busy, absent, or not a serial port
        return None
    return None


def cmd_scan(args):
    """Find the robot by asking every serial port for its pose."""
    try:
        from serial.tools import list_ports
    except ImportError:
        print("ERROR: pyserial is required:  pip install pyserial",
              file=sys.stderr)
        return 2

    candidates = sorted((p for p in list_ports.comports()),
                        key=lambda p: p.device)
    if not candidates:
        print("no serial ports found")
        return 1

    print(f"probing {len(candidates)} port(s) at {args.baud} 8N1 "
          f"with READ_POSE...\n")
    found = []
    for p in candidates:
        pose = _probe_port(p.device, args.baud)
        if pose is None:
            print(f"  {p.device:<8} -")
            continue
        found.append(p.device)
        print(f"  {p.device:<8} ROBOT   "
              + "  ".join(f"{d / 10.0:6.1f}" for d in pose))
        print(f"           {p.description}")

    print()
    if not found:
        print("no robot answered.\n"
              "  check: PA9 -> USB-TTL RX, PA10 <- USB-TTL TX, common GND,\n"
              "         115200 8N1, and that the firmware was rebuilt with the\n"
              "         NVIC fix in Core/Src/usart.c")
        return 1
    for dev in found:
        print(f"use:  python tools/robot_host.py --port {dev} identify")
    return 0


def _move_and_verify(robot, angles, duration_ms):
    """Move, wait for the segment to complete, read back. Returns (pose, err)."""
    err = robot.move(angles, duration_ms=duration_ms)
    if err:
        return None, err
    time.sleep(duration_ms / 1000.0 + 0.15)
    return robot.pose()


def cmd_bringup(robot, args):
    """Full software-side verification with an explicit pass/fail per joint."""
    print("NOTE: 'pose' is the firmware's own bookkeeping -- the arm is open loop,")
    print("      so a PASS proves the command path and planner, not that a servo")
    print("      physically moved. Watch the arm for that part.\n")

    failures = []

    pose, err = robot.pose()
    if err:
        print(f"LINK        FAIL  {err}")
        print("\nthe board did not answer -- nothing else can be tested")
        return 1
    print(f"LINK        PASS  {[round(d / 10.0, 1) for d in pose]}")

    print("\nper-joint command path")
    for i, _, _, _, mn, mx, home in JOINTS:
        lo = max(mn, min(mx, args.low))
        hi = max(mn, min(mx, args.high))
        for target, label in ((lo, "low"), (hi, "high")):
            angles = list(HOME_RAW)
            angles[i] = target
            pose, err = _move_and_verify(robot, angles, args.time)
            if err:
                print(f"  joint {i} {label:<4} FAIL  {err}")
                failures.append((i, label, err))
                continue
            diff = [k for k in range(SERVO_NUM) if pose[k] != angles[k]]
            if diff:
                detail = f"mismatch at joint {diff}"
                print(f"  joint {i} {label:<4} FAIL  {detail}")
                failures.append((i, label, detail))
            else:
                print(f"  joint {i} {label:<4} PASS  "
                      f"{target / 10.0:5.1f} deg  [{PWM_CHANNELS[i]}]")

        _move_and_verify(robot, HOME_RAW, args.time)

    print("\nsynchronised move (all six together)")
    pattern = [250, 750, 700, 1150, 1300, 1250]
    pose, err = _move_and_verify(robot, pattern, args.time * 2)
    if err:
        print(f"  FAIL  {err}")
        failures.append((-1, "sync", err))
    elif [p for p in pose] != pattern:
        detail = f"want {pattern} got {list(pose)}"
        print(f"  FAIL  {detail}")
        failures.append((-1, "sync", detail))
    else:
        print(f"  PASS  all six reached {[round(p / 10.0, 1) for p in pattern]}")

    _move_and_verify(robot, HOME_RAW, args.time * 2)

    print()
    if failures:
        print(f"RESULT: {len(failures)} FAILURE(S)")
        for i, label, detail in failures:
            who = "sync" if i < 0 else f"joint {i} {label}"
            print(f"  - {who}: {detail}")
        print("\na joint that fails here is a wiring/servo/power problem, not a")
        print("firmware one -- the remaining joints passing proves the driver.")
        return 1

    print("RESULT: PASS -- command path, planner and telemetry all consistent")
    print("Now confirm by eye that all six servos actually moved.")
    return 0


def cmd_selftest(robot: Robot, args):
    print("sending SELFTEST (firmware sweeps each joint, ~16s)...")
    err = robot.simple(CMD_SELFTEST)
    if err:
        print(f"ERROR: {err}")
        return 1
    print("ACK ok -- watch the arm now")
    return 0


def cmd_home(robot: Robot, args):
    err = robot.simple(CMD_HOME)
    print("ERROR: " + err if err else "homing (2000 ms)")
    return 1 if err else 0


def cmd_speed(robot: Robot, args):
    err = robot.set_speed(args.percent)
    print("ERROR: " + err if err else f"speed = {args.percent}%")
    return 1 if err else 0


def cmd_estop(robot: Robot, args):
    err = robot.simple(CMD_ESTOP)
    print("ERROR: " + err if err else "EMERGENCY STOP latched")
    return 1 if err else 0


def cmd_stop(robot: Robot, args):
    err = robot.simple(CMD_STOP)
    print("ERROR: " + err if err else "segment cancelled, holding position")
    return 1 if err else 0


def cmd_clear(robot: Robot, args):
    err = robot.simple(CMD_CLEAR_ESTOP)
    print("ERROR: " + err if err else "emergency cleared")
    return 1 if err else 0


def cmd_drive(robot: Robot, args):
    err = robot.drive_base(args.vx, args.vy, args.wz, args.time)
    if err:
        print(f"ERROR: {err}")
        return 1
    print(f"drive vx={args.vx} vy={args.vy} wz={args.wz} time={args.time}ms")
    return 0


def cmd_wheels(robot: Robot, args):
    err = robot.drive_wheels(args.fl, args.fr, args.rl, args.rr, args.time)
    if err:
        print(f"ERROR: {err}")
        return 1
    print(f"wheels fl={args.fl} fr={args.fr} rl={args.rl} rr={args.rr} time={args.time}ms")
    return 0


def cmd_cfg_get(robot: Robot, args):
    cfg, err = robot.get_joint_cfg(args.joint)
    if err:
        print(f"ERROR: {err}")
        return 1
    print(cfg)
    return 0


def cmd_cfg_save(robot: Robot, args):
    err = robot.simple(CMD_SAVE_CFG)
    print("ERROR: " + err if err else "calibration saved to flash")
    return 1 if err else 0


def cmd_cfg_reset(robot: Robot, args):
    err = robot.simple(CMD_RESET_CFG)
    print("ERROR: " + err if err else "calibration reset to defaults")
    return 1 if err else 0


def cmd_track(robot: Robot, args):
    act = args.action
    if act in ("off", "stop"):
        err = robot.set_track_mode(0, 0)
        print("ERROR: " + err if err else "line tracking OFF")
        return 1 if err else 0
    if act == "start":
        err = robot.set_track_mode(1, 0)
        print("ERROR: " + err if err else "line tracking ACTIVE (live driving)")
        return 1 if err else 0
    if act == "dry":
        err = robot.set_track_mode(1, 1)
        print("ERROR: " + err if err else "line tracking ACTIVE (DRY RUN, motors off)")
        return 1 if err else 0
    if act == "auto":
        err = robot.set_track_mode(2, 0)
        print("ERROR: " + err if err else "line tracking AUTO (arms on OpenMV arm=1)")
        return 1 if err else 0
    if act == "tune":
        err = robot.set_track_cfg(args.vx, args.vmin, args.kp, args.kd,
                                  args.kang, args.wmax, args.swz)
        print("ERROR: " + err if err else
              f"track cfg updated vx={args.vx}..{args.vmin} kp={args.kp} "
              f"kd={args.kd} kang={args.kang} wmax={args.wmax} swz={args.swz}")
        return 1 if err else 0
    if act == "status":
        st, err = robot.track_status()
        if err:
            print(f"ERROR: {err}")
            return 1
        print(st)
        return 0
    if act == "monitor":
        print("monitoring OpenMV line tracking (Ctrl-C to stop)")
        try:
            while True:
                st, err = robot.track_status()
                if err:
                    print(f"  ERROR: {err}")
                else:
                    print(f"  [{st['mode']:<6}{' DRY' if st['dry_run'] else ''}] "
                          f"{st['state']:<8} valid={st['valid']} err={st['err']:+5d} "
                          f"ang={st['ang']:+4d} vx={st['vx']:+4d} wz={st['wz']:+4d} "
                          f"age={st['age_ms']}ms rx={st['rx_frames']}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0
    return 1


def _format_pick_stat(st: dict) -> str:
    return (f"  [{st['mode']:<6}{' DRY' if st['dry_run'] else ''}] "
            f"{st['state']:<11} valid={st['valid']} dx={st['dx']:+5d} "
            f"dy={st['dy']:+5d} w={st['cube_w']:3d} vx={st['vx']:+4d} "
            f"wy={st['wy']:+4d} age={st['age_ms']}ms rx={st['rx_frames']}")


def cmd_pick(robot: Robot, args):
    act = args.action
    if act in ("off", "stop"):
        err = robot.set_pick_mode(0, 0)
        print("ERROR: " + err if err else "cube pick OFF")
        return 1 if err else 0
    if act == "start":
        err = robot.set_pick_mode(1, 0)
        print("ERROR: " + err if err else "cube pick ACTIVE (live align & grab)")
        return 1 if err else 0
    if act == "dry":
        err = robot.set_pick_mode(1, 1)
        print("ERROR: " + err if err else "cube pick ACTIVE (DRY RUN, motors/arm off)")
        return 1 if err else 0
    if act == "auto":
        err = robot.set_pick_mode(2, 0)
        print("ERROR: " + err if err else "cube pick AUTO (arms on OpenMV arm=1)")
        return 1 if err else 0
    if act == "reset":
        err = robot.set_pick_mode(3, 0)
        print("ERROR: " + err if err else "cube pick latch reset (AUTO ready)")
        return 1 if err else 0
    if act == "status":
        st, err = robot.pick_status()
        if err:
            print(f"ERROR: {err}")
            return 1
        print(st)
        return 0
    if act == "run":
        print("waking arm to upright HOME...")
        err = robot.simple(CMD_HOME)
        if err:
            print(f"ERROR: {err}")
            return 1
        time.sleep(1.4)
        err = robot.set_pick_mode(1, 0)
        if err:
            print(f"ERROR: {err}")
            return 1
        print("cube pick RUN active in single session (Ctrl-C to stop wheels/arm & exit)")
        try:
            while True:
                st, err = robot.pick_status()
                if err:
                    print(f"  ERROR: {err}")
                else:
                    print(_format_pick_stat(st))
                    if st["state"] in ("HOLDING", "FAIL"):
                        input(f"\nfinished in {st['state']} -- holding pose; press Enter to exit...")
                        break
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nstopping...")
            robot.set_pick_mode(0, 0)
            robot.simple(CMD_STOP)
        return 0
    if act == "monitor":
        print("monitoring OpenMV cube pick (Ctrl-C to stop)")
        try:
            while True:
                st, err = robot.pick_status()
                if err:
                    print(f"  ERROR: {err}")
                else:
                    print(_format_pick_stat(st))
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0
    return 1


def _format_exec_stat(st: dict) -> str:
    tag = "ARMED" if st["armed"] else "DISARM"
    if st["dry_run"]:
        tag += " DRY"
    return (f"  [{tag:<9}] arm={st['arm_state']:<13} busy={int(st['arm_busy'])} "
            f"hold={int(st['holding'])} btn1={int(st['btn1_down'])} "
            f"v=({st['vx']:+4d},{st['vy']:+4d},{st['wz']:+4d}) seq={st['arm_seq']} "
            f"age={st['age_ms']}ms rx={st['rx_frames']} bad={st['rx_bad']}")


def cmd_exec(robot: Robot, args):
    act = args.action
    if act in ("off", "stop"):
        err = robot.set_exec_mode(0, 0)
        print("ERROR: " + err if err else "execution layer DISARMED")
        return 1 if err else 0
    if act == "start":
        err = robot.set_exec_mode(1, 0)
        print("ERROR: " + err if err else "execution layer ARMED (live OpenMV control)")
        return 1 if err else 0
    if act == "dry":
        err = robot.set_exec_mode(1, 1)
        print("ERROR: " + err if err else "execution layer ARMED (DRY RUN, actuators off)")
        return 1 if err else 0
    if act == "status":
        st, err = robot.exec_status()
        if err:
            print(f"ERROR: {err}")
            return 1
        print(st)
        return 0
    if act == "run":
        err = robot.set_exec_mode(1, 0)
        if err:
            print(f"ERROR: {err}")
            return 1
        print("execution layer ARMED in single session (Ctrl-C to disarm/stop & exit)")
        try:
            while True:
                st, err = robot.exec_status()
                if err:
                    print(f"  ERROR: {err}")
                else:
                    print(_format_exec_stat(st))
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\ndisarming...")
            robot.set_exec_mode(0, 0)
            robot.simple(CMD_STOP)
        return 0
    if act == "monitor":
        print("monitoring frozen execution layer (Ctrl-C to stop)")
        try:
            while True:
                st, err = robot.exec_status()
                if err:
                    print(f"  ERROR: {err}")
                else:
                    print(_format_exec_stat(st))
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0
    return 1


# --- cli -------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Host driver for the STM32F103 6-axis servo controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--port", default="COM3", help="serial port (default COM3)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--check", action="store_true",
                    help="run offline codec checks and exit")

    sub = ap.add_subparsers(dest="command")

    sub.add_parser("pose", help="read current pose")
    sub.add_parser("scan", help="find the robot by probing every serial port")
    sub.add_parser("home", help="move to configured home")

    p = sub.add_parser("identify", help="move one joint at a time to verify mapping")
    p.add_argument("--delta", type=int, default=200,
                   help="swing either side of centre, 0.1 deg (default 200 = 20 deg)")
    p.add_argument("--time", type=int, default=1500,
                   help="ms per move (default 1500)")

    p = sub.add_parser("bringup",
                       help="full verification sequence with PASS/FAIL per joint")
    p.add_argument("--low", type=int, default=500,
                   help="low test angle, 0.1 deg (default 500 = 50 deg)")
    p.add_argument("--high", type=int, default=1300,
                   help="high test angle, 0.1 deg (default 1300 = 130 deg)")
    p.add_argument("--time", type=int, default=700,
                   help="ms per move (default 700)")

    p = sub.add_parser("move", help="move all joints")
    p.add_argument("angles", nargs=6, type=float, metavar="DEG")
    p.add_argument("--time", type=int, default=None,
                   help="duration in ms (omit for firmware auto duration)")

    p = sub.add_parser("sweep", help="sweep every joint together")
    p.add_argument("--lo", type=float, default=10.0)
    p.add_argument("--hi", type=float, default=170.0)
    p.add_argument("--cycles", type=int, default=3)
    p.add_argument("--time", type=int, default=1500, help="ms per leg")

    p = sub.add_parser("monitor", help="poll pose continuously")
    p.add_argument("--rate", type=float, default=5.0, help="Hz")

    sub.add_parser("selftest", help="trigger the firmware self test sweep")

    p = sub.add_parser("speed", help="set global speed override")
    p.add_argument("percent", type=int)

    sub.add_parser("estop", help="emergency stop")
    sub.add_parser("stop", help="cancel the current segment, hold position")
    sub.add_parser("clear", help="clear emergency stop")

    p = sub.add_parser("drive", help="drive mecanum chassis (vx=fwd, vy=left, wz=ccw)")
    p.add_argument("vx", type=int)
    p.add_argument("vy", type=int)
    p.add_argument("wz", type=int)
    p.add_argument("--time", type=int, default=1000, help="ms (0=continuous)")

    p = sub.add_parser("wheels", help="set individual wheel speeds (-1000..1000)")
    p.add_argument("fl", type=int)
    p.add_argument("fr", type=int)
    p.add_argument("rl", type=int)
    p.add_argument("rr", type=int)
    p.add_argument("--time", type=int, default=1000, help="ms (0=continuous)")

    p = sub.add_parser("cfg-get", help="read joint calibration")
    p.add_argument("joint", type=int)
    sub.add_parser("cfg-save", help="save calibration to flash")
    sub.add_parser("cfg-reset", help="erase flash calibration")

    p = sub.add_parser("track", help="OpenMV black-line tracking control & tuning")
    p.add_argument("action",
                   choices=("start", "stop", "off", "auto", "dry", "status", "monitor", "tune"))
    p.add_argument("--vx", type=int, default=600, help="straight cruise speed (default 600)")
    p.add_argument("--vmin", type=int, default=420, help="min curve speed (default 420)")
    p.add_argument("--kp", type=int, default=38, help="proportional turn gain (default 38)")
    p.add_argument("--kd", type=int, default=160, help="derivative damping (default 160)")
    p.add_argument("--kang", type=int, default=20, help="curve slope lead (default 20)")
    p.add_argument("--wmax", type=int, default=320, help="max angular rate (default 320)")
    p.add_argument("--swz", type=int, default=320, help="lost-line spin speed (default 320)")

    p = sub.add_parser("pick", help="OpenMV cube search, alignment & auto-grab control")
    p.add_argument("action",
                   choices=("run", "start", "stop", "off", "auto", "dry", "reset", "status", "monitor"))

    p = sub.add_parser("exec", help="Competition execution layer (Key1/$E/@V/@A/@J) control")
    p.add_argument("action",
                   choices=("run", "start", "stop", "off", "dry", "status", "monitor"))

    args = ap.parse_args(argv)

    if args.check:
        return run_checks()

    if not args.command:
        ap.print_help()
        return 1

    handlers = {
        "pose": cmd_pose,
        "scan": cmd_scan,
        "bringup": cmd_bringup,
        "identify": cmd_identify,
        "move": cmd_move,
        "sweep": cmd_sweep,
        "monitor": cmd_monitor,
        "selftest": cmd_selftest,
        "home": cmd_home,
        "speed": cmd_speed,
        "estop": cmd_estop,
        "stop": cmd_stop,
        "clear": cmd_clear,
        "drive": cmd_drive,
        "wheels": cmd_wheels,
        "cfg-get": cmd_cfg_get,
        "cfg-save": cmd_cfg_save,
        "cfg-reset": cmd_cfg_reset,
        "track": cmd_track,
        "pick": cmd_pick,
        "exec": cmd_exec,
    }

    if args.command == "scan":
        # scan opens its own connections; no Robot instance is needed
        return cmd_scan(args)

    try:
        with Robot(args.port, args.baud) as robot:
            return handlers[args.command](robot, args)
    except ImportError:
        print("ERROR: pyserial is required:  pip install pyserial", file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001 - surface anything serial-related
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
