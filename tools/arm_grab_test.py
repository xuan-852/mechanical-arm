"""Interactive single-session arm grab test & pose calibration tool.

Keeps COM10 open inside a single connection so CH340 DTR/RTS never resets
STM32 mid-test (which would relax PWM to 0 us and drop the arm). Disables
chassis line tracking while calibrating the arm, wakes servos safely via
CMD_HOME, lets you jog joints smoothly, record key poses, and replay a full
pick-up test cycle (`pre -> above -> grab -> clamp -> lift -> home`).

Usage:
    python tools/arm_grab_test.py --port COM10
    python tools/arm_grab_test.py --check
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from robot_host import (  # noqa: E402
    CMD_CLEAR_ESTOP,
    CMD_ESTOP,
    CMD_HOME,
    CMD_STOP,
    SERVO_NUM,
    Robot,
)

JOINT_NAMES = (
    "0:base(PB3)",
    "1:lower(PB8)",
    "2:elbow(PB9)",
    "3:upper(PB6)",
    "4:roll(PB7)",
    "5:grip(PB4)",
)

# Verified hard/software bounds matching Core/Src/servo.c (deg, us_min, us_max).
JOINT_BOUNDS = (
    (0.0,   180.0, 500,  2500),  # j0 base yaw   (>45 left CCW, <45 right CW)
    (63.0,  180.0, 1200, 2500),  # j1 lower arm  (>91.8 forward, floor >=1200us)
    (0.0,   180.0, 500,  2500),  # j2 elbow      (<90.0 forward, >90.0 back)
    (0.0,   180.0, 500,  2500),  # j3 upper pitch(>99.0 forward/down)
    (0.0,   180.0, 500,  2500),  # j4 wrist roll
    (90.0,  148.5, 1500, 2150),  # j5 gripper    (90.0=open 1500, 148.5=2150 max)
)

DEFAULT_POSES = {
    "home":  [45.0,  91.8, 90.0,  99.0, 103.5,  90.0],
    # Safe high clearance above OpenMV with wrist already rotated horizontal (j4=180)
    "pre":   [45.0, 112.0, 82.0, 112.0, 180.0,  90.0],
    # ~5cm hover directly above verified floor grab pose
    "above": [45.0, 152.0, 75.9, 113.0, 180.0,  90.0],
    # Hardware-verified floor grab pose (open horizontal jaws)
    "grab":  [45.0, 171.6, 75.9, 113.0, 180.0,  90.0],
    # Gentle placement pose (~1.6 deg above grab to avoid pressing onto target pad)
    "place": [45.0, 170.0, 75.9, 113.0, 180.0, 131.9],
    # Hardware-verified cube clamp pose (1966 us grip)
    "clamp": [45.0, 171.6, 75.9, 113.0, 180.0, 131.9],
}

DEFAULT_POSES_PATH = os.path.join(SCRIPT_DIR, "grab_poses.json")
GRIP_OPEN_DEG = 90.0
MAX_STEP_DEG = 15.0


def clamp_joint(idx: int, deg: float) -> float:
    """Clamp joint angle to verified Core/Src/servo.c limits."""
    lo, hi, _, _ = JOINT_BOUNDS[idx]
    return round(max(lo, min(hi, float(deg))), 1)


def clamp_pose(pose: list[float]) -> list[float]:
    if len(pose) != SERVO_NUM:
        raise ValueError(f"expected {SERVO_NUM} angles, got {len(pose)}")
    return [clamp_joint(i, pose[i]) for i in range(SERVO_NUM)]


def deg_to_us(idx: int, deg: float) -> int:
    lo, hi, p_lo, p_hi = JOINT_BOUNDS[idx]
    d = clamp_joint(idx, deg)
    span = hi - lo
    if span <= 0.0:
        return p_lo
    return int(round(p_lo + ((d - lo) * (p_hi - p_lo)) / span))


def format_pose(pose: list[float]) -> str:
    parts = [f"j{i}={pose[i]:5.1f}({deg_to_us(i, pose[i]):4d}us)"
             for i in range(SERVO_NUM)]
    return " ".join(parts)


def load_poses(path: str) -> dict[str, list[float]]:
    poses = {k: clamp_pose(v) for k, v in DEFAULT_POSES.items()}
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
        if isinstance(raw, dict):
            for k, v in raw.items():
                if isinstance(v, list) and len(v) == SERVO_NUM:
                    poses[k] = clamp_pose(v)
    return poses


def save_poses(path: str, poses: dict[str, list[float]]) -> None:
    clean = {k: clamp_pose(v) for k, v in poses.items()}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(clean, fh, indent=2)


def move_and_wait(robot: Robot, current: list[float], target: list[float],
                  dur_ms: int = 800, speed_pct: int = 50) -> list[float]:
    """Send clamped segment and wait until firmware planner converges."""
    tgt = clamp_pose(target)
    raw_angles = [int(round(a * 10.0)) for a in tgt]
    err = robot.move(raw_angles, duration_ms=dur_ms)
    if err:
        raise RuntimeError(err)

    max_delta = max(abs(tgt[i] - current[i]) for i in range(SERVO_NUM))
    est_s = max(dur_ms / 1000.0, max_delta / 140.0) * (100.0 / max(10, speed_pct))
    deadline = time.time() + max(0.8, est_s + 1.2)

    latest = list(tgt)
    while time.time() < deadline:
        time.sleep(0.12)
        pose_raw, perr = robot.pose()
        if perr or pose_raw is None:
            continue
        latest = [a / 10.0 for a in pose_raw]
        if max(abs(latest[i] - tgt[i]) for i in range(SERVO_NUM)) <= 0.4:
            return latest
    return latest


def _exec_steps(robot: Robot, current: list[float], steps: list[tuple[str, list[float], bool]],
                dur_ms: int, speed_pct: int, auto: bool) -> tuple[list[float], bool]:
    cur = list(current)
    for label, tgt, ask in steps:
        print(f"\n[STEP] {label}")
        print(f"       target: {format_pose(tgt)}")
        if (not auto) and ask:
            ans = input("       Press Enter to run (or 's' to abort cycle): ").strip().lower()
            if ans == "s":
                print("  aborted cycle cleanly")
                return cur, False
        cur = move_and_wait(robot, cur, tgt, dur_ms=dur_ms, speed_pct=speed_pct)
        print(f"       actual: {format_pose(cur)}")
        time.sleep(0.25)
    return cur, True


def run_grab_cycle(robot: Robot, current: list[float],
                   poses: dict[str, list[float]], dur_ms: int,
                   speed_pct: int, auto: bool = False) -> list[float]:
    """Execute staged pick-up cycle: pre -> above -> grab -> clamp -> lift -> home."""
    clamp_deg = poses["clamp"][5]
    steps = [
        ("1. PRE (high clear)",      poses["pre"][:5]   + [GRIP_OPEN_DEG], True),
        ("2. ABOVE (open hover)",    poses["above"][:5] + [GRIP_OPEN_DEG], True),
        ("3. GRAB (lower open jaw)", poses["grab"][:5]  + [GRIP_OPEN_DEG], True),
        ("4. CLAMP (close gripper)", poses["grab"][:5]  + [clamp_deg],     True),
        ("5. LIFT (vertical clear)", poses["above"][:5] + [clamp_deg],     False),
        ("6. RETRACT (clear OpenMV)",poses["pre"][:5]   + [clamp_deg],     False),
        ("7. CARRY HOME (keep hold)",poses["home"][:5]  + [clamp_deg],     False),
    ]
    cur, ok = _exec_steps(robot, current, steps, dur_ms, speed_pct, auto)
    if ok:
        print("\n[DONE] Cube lifted to HOME! Use 'put' to place it on a pad, or 'open' to drop.")
    return cur


def run_place_cycle(robot: Robot, current: list[float],
                    poses: dict[str, list[float]], dur_ms: int,
                    speed_pct: int, auto: bool = False) -> list[float]:
    """Execute staged put-down cycle: pre -> above -> place -> release -> lift -> home."""
    hold_deg = current[5] if current[5] > (GRIP_OPEN_DEG + 10.0) else poses["clamp"][5]
    steps = [
        ("1. PRE (carry clear)",       poses["pre"][:5]   + [hold_deg],      True),
        ("2. ABOVE (over target pad)", poses["above"][:5] + [hold_deg],      True),
        ("3. PLACE (set onto pad)",    poses["place"][:5] + [hold_deg],      True),
        ("4. RELEASE (open gripper)",  poses["place"][:5] + [GRIP_OPEN_DEG], True),
        ("5. LIFT (clear placed cube)",poses["above"][:5] + [GRIP_OPEN_DEG], False),
        ("6. RETRACT (clear OpenMV)",  poses["pre"][:5]   + [GRIP_OPEN_DEG], False),
        ("7. HOME (empty open)",       poses["home"][:5]  + [GRIP_OPEN_DEG], False),
    ]
    cur, ok = _exec_steps(robot, current, steps, dur_ms, speed_pct, auto)
    if ok:
        print("\n[DONE] Cube placed and arm returned cleanly to HOME!")
    return cur


def print_help() -> None:
    print("""
Commands:
  p                 read & show current pose
  h / home          lift to upright HOME (safe wake pose)
  pre / above / grab / clamp / place   move to saved pose
  0..5 <val|+d|-d>  set or jog one joint (e.g. '1 +3', '2 -5', '5 130')
                    j0:base j1:lower j2:elbow j3:upper j4:roll j5:grip
  dip [deg]         nudge tip down/forward (default +5 deg coordinated)
  up  [deg]         nudge tip up/back     (default +5 deg coordinated)
  open              open gripper wide (90.0 deg / 1500 us)
  close [deg]       close gripper (default saved clamp or 131.9 deg)
  save <name>       save current pose as pre/above/grab/clamp/place
  run [auto]        run 7-step pick-up cycle (cube -> HOME)
  put [auto]        run 7-step put-down cycle (HOME -> pad -> open HOME)
  estop / clear / stop   safety controls
  q                 safely return HOME and quit
""")


def interactive_loop(args: argparse.Namespace) -> int:
    poses = load_poses(args.poses)
    print(f"Loaded poses from {args.poses if os.path.isfile(args.poses) else 'built-in defaults'}.")
    print("SAFETY: Keep 12V battery ON, place arm roughly upright before starting.")

    with Robot(args.port, args.baud) as robot:
        # Prevent chassis line tracking from driving while testing arm
        err = robot.set_track_mode(0, 0)
        if err:
            print(f"WARNING: failed to disable track mode: {err}")

        err = robot.set_speed(args.speed)
        if err:
            print(f"WARNING: set_speed failed: {err}")

        print("Waking servos gently to HOME...")
        err = robot.simple(CMD_HOME)
        if err:
            print(f"ERROR homing: {err}")
            return 1
        time.sleep(2.2)

        raw, err = robot.pose()
        if err or raw is None:
            print(f"ERROR reading pose: {err}")
            return 1
        cur = [a / 10.0 for a in raw]
        print(f"Ready: {format_pose(cur)}")
        print_help()

        while True:
            try:
                line = input("arm> ").strip()
            except (EOFError, KeyboardInterrupt):
                line = "q"

            if not line:
                continue
            toks = line.split()
            cmd = toks[0].lower()

            try:
                if cmd in ("?", "help"):
                    print_help()
                elif cmd in ("p", "pose"):
                    raw, err = robot.pose()
                    if err or raw is None:
                        print(f"  ERROR: {err}")
                    else:
                        cur = [a / 10.0 for a in raw]
                        print(f"  {format_pose(cur)}")
                elif cmd in ("h", "home"):
                    # Keep current gripper state when returning home so held cubes stay clamped
                    cur = move_and_wait(robot, cur, poses["pre"][:5] + [cur[5]],
                                        dur_ms=args.dur, speed_pct=args.speed)
                    cur = move_and_wait(robot, cur, poses["home"][:5] + [cur[5]],
                                        dur_ms=args.dur, speed_pct=args.speed)
                    print(f"  {format_pose(cur)}")
                elif cmd in ("pre", "above", "grab", "clamp", "place"):
                    tgt = list(poses[cmd])
                    if cmd == "place":
                        tgt[5] = cur[5]  # preserve grip when testing pad height
                    cur = move_and_wait(robot, cur, tgt,
                                        dur_ms=args.dur, speed_pct=args.speed)
                    print(f"  {format_pose(cur)}")
                elif cmd == "open":
                    tgt = list(cur)
                    tgt[5] = GRIP_OPEN_DEG
                    cur = move_and_wait(robot, cur, tgt, dur_ms=400, speed_pct=args.speed)
                    print(f"  {format_pose(cur)}")
                elif cmd == "close":
                    cdeg = float(toks[1]) if len(toks) > 1 else poses["clamp"][5]
                    tgt = list(cur)
                    tgt[5] = clamp_joint(5, cdeg)
                    cur = move_and_wait(robot, cur, tgt, dur_ms=450, speed_pct=args.speed)
                    print(f"  {format_pose(cur)}")
                elif cmd in ("dip", "up"):
                    step = float(toks[1]) if len(toks) > 1 else 5.0
                    step = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, step))
                    sgn = 1.0 if cmd == "dip" else -1.0
                    tgt = list(cur)
                    # +j1 lowers boom forward; +j2 (inverted elbow) & -j3 hold wrist level
                    tgt[1] += sgn * step * 1.0
                    tgt[2] += sgn * step * 0.6
                    tgt[3] -= sgn * step * 0.4
                    cur = move_and_wait(robot, cur, tgt, dur_ms=500, speed_pct=args.speed)
                    print(f"  {format_pose(cur)}")
                elif cmd in ("0", "1", "2", "3", "4", "5") or (
                        cmd.startswith("j") and len(cmd) == 2 and cmd[1] in "012345"):
                    idx = int(cmd[-1])
                    if len(toks) < 2:
                        print(f"  usage: {cmd} <angle|+delta|-delta>  (now {cur[idx]:.1f})")
                        continue
                    val_s = toks[1]
                    if val_s.startswith(("+", "-")):
                        delta = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, float(val_s)))
                        new_a = cur[idx] + delta
                    else:
                        new_a = float(val_s)
                    tgt = list(cur)
                    tgt[idx] = clamp_joint(idx, new_a)
                    cur = move_and_wait(robot, cur, tgt, dur_ms=500, speed_pct=args.speed)
                    print(f"  {JOINT_NAMES[idx]} -> {format_pose(cur)}")
                elif cmd == "save":
                    if len(toks) < 2 or toks[1] not in ("pre", "above", "grab", "clamp", "place"):
                        print("  usage: save <pre|above|grab|clamp|place>")
                        continue
                    key = toks[1]
                    poses[key] = clamp_pose(cur)
                    if key == "grab":
                        # Keep clamp trajectory synchronized with updated grab pose
                        poses["clamp"] = list(poses["grab"][:5]) + [poses["clamp"][5]]
                    elif key == "clamp":
                        # Also sync open-jaw grab landing pose when saved at clamp depth
                        poses["grab"] = list(poses["clamp"][:5]) + [GRIP_OPEN_DEG]
                    elif key == "place":
                        poses["place"][5] = poses["clamp"][5]
                    save_poses(args.poses, poses)
                    print(f"  saved '{key}' to {args.poses}: {format_pose(poses[key])}")
                elif cmd == "run":
                    poses = load_poses(args.poses)
                    auto = len(toks) > 1 and toks[1].lower() == "auto"
                    cur = run_grab_cycle(robot, cur, poses, dur_ms=args.dur,
                                         speed_pct=args.speed, auto=auto)
                elif cmd == "put":
                    poses = load_poses(args.poses)
                    auto = len(toks) > 1 and toks[1].lower() == "auto"
                    cur = run_place_cycle(robot, cur, poses, dur_ms=args.dur,
                                          speed_pct=args.speed, auto=auto)
                elif cmd == "estop":
                    print("  ESTOP:", robot.simple(CMD_ESTOP) or "latched")
                elif cmd == "clear":
                    print("  CLEAR:", robot.simple(CMD_CLEAR_ESTOP) or "ok")
                elif cmd == "stop":
                    print("  STOP:", robot.simple(CMD_STOP) or "ok")
                elif cmd in ("q", "quit", "exit"):
                    dist_home = max(abs(cur[i] - poses["home"][i]) for i in range(5))
                    if dist_home > 3.0:
                        ans = input("Arm is not upright at HOME. Park to HOME before power-drop? [Y/n]: ").strip().lower()
                        if ans != "n":
                            cur = move_and_wait(robot, cur, poses["pre"][:5] + [cur[5]],
                                                dur_ms=args.dur, speed_pct=args.speed)
                            cur = move_and_wait(robot, cur, poses["home"][:5] + [cur[5]],
                                                dur_ms=args.dur, speed_pct=args.speed)
                    print("Exiting cleanly.")
                    return 0
                else:
                    print(f"  unknown command '{cmd}' (type '?' for help)")
            except Exception as exc:  # noqa: BLE001
                print(f"  ERROR: {exc}")
    return 0


def run_checks() -> int:
    fails = 0

    def chk(name: str, cond: bool, detail: str = "") -> None:
        nonlocal fails
        if not cond:
            fails += 1
            print(f"  [FAIL] {name}: {detail}")
        else:
            print(f"  [ok] {name}")

    print("arm_grab_test checks")
    chk("lower floor clamp >=1200us", clamp_joint(1, 10.0) == 63.0 and deg_to_us(1, 63.0) == 1200)
    chk("lower home = 1520us", deg_to_us(1, 91.8) == 1520, f"got {deg_to_us(1, 91.8)}")
    chk("gripper open = 1500us", deg_to_us(5, 90.0) == 1500, f"got {deg_to_us(5, 90.0)}")
    chk("gripper max = 2150us", deg_to_us(5, 148.5) == 2150, f"got {deg_to_us(5, 148.5)}")
    chk("gripper clamps min/max", clamp_joint(5, 20.0) == 90.0 and clamp_joint(5, 180.0) == 148.5)
    for name, p in DEFAULT_POSES.items():
        chk(f"preset '{name}' in bounds", clamp_pose(p) == p, str(p))

    import tempfile
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, "poses.json")
        mod = dict(DEFAULT_POSES)
        mod["grab"] = [45.0, 145.0, 52.0, 140.0, 103.5, 90.0]
        save_poses(jp, mod)
        loaded = load_poses(jp)
        chk("poses JSON roundtrip", loaded["grab"] == mod["grab"])

    print("PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Interactive single-session arm grab & pose teach console")
    ap.add_argument("--port", default="COM10", help="STM32 serial port (default COM10)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dur", type=int, default=800, help="base segment duration ms (default 800)")
    ap.add_argument("--speed", type=int, default=50, help="servo speed pct 1..100 (default 50)")
    ap.add_argument("--poses", default=DEFAULT_POSES_PATH, help="JSON pose preset file")
    ap.add_argument("--check", action="store_true", help="run offline validation checks")
    args = ap.parse_args()

    if args.check:
        return run_checks()

    try:
        return interactive_loop(args)
    except ImportError:
        print("ERROR: pyserial is required: pip install pyserial", file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
