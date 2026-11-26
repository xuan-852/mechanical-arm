# Unified 5-Stage Closed-Loop Competition Script (OpenMV Cam H7 Plus + frozen STM32)
# Revision: 09-1127A (verified offline frame-by-frame against openmv/test13..test29.mp4)
#
# Mission Priority:
#   1. Guaranteed single-block placement on (b) 50pt ring (Stages 1 -> 2 -> 3)
#      - Stage 1 (HOME exit): Closed-loop E3 (sandwiched roadway '+') -> E4 (leftmost vertical
#        outer rail + roadway '+' on right) event chain; 100% immune to test27 inner-island '+'
#        hijack and test25 full-battery overshoot.
#      - Stage 2 (Outer pick): Outer-rail-priority PD (tapes[0] + OUTER_RECOVER) + tightened
#        ALIGN_PICK (TOL_X=8, 70ms fine yaw pulse to avoid test27 claw-on-rim miss) + 7-step @J PICK.
#      - Stage 3 (Cross to b): Dual-landmark closed-loop TURN_TO_INNER (shaded-mat adaptive (b) '+'
#        in x=[155..295] OR direct (b) concentric ring lock) -> field-proven ALIGN_B_RING -> 7-step PUT.
#   2. Score-doubling (x2) HOME return without bumping placed block (Stages 4 -> 5, gated by RETURN_TO_HOME):
#      - Stage 4 (Leave b via North straight): Reverse 13cm -> short NW left turn (bypasses left arc)
#        -> detect_outer_rail_ahead (skips inner rail, stops at North outer rail) -> JOIN_OUTER_CW
#        at full torque (wz=-540, ang<=65 deg) -> CW outer normalization lap.
#      - Stage 5 (East->West HOME dock for 21x27cm car in 32x32cm box): 2-wall left gate (W=1->W=2+10cm)
#        -> anti-glare HOME back-border level lock (passes test28 ceiling glare, rejects inner oval)
#        -> visual back-border depth anchor (y>=224 + short 5.5cm pull-in, +-1.2cm within +-2.5cm margin).
#
# Quick Competition Switches:
#   RETURN_TO_HOME = True   # Set False for ultra-safe placement-only mode (backs 13cm after PUT & stops)
#   TEST_STAGE     = "FULL_1_BLOCK"

import math
import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# COMPETITION SWITCHES & CALIBRATED CONSTANTS
# ===========================================================================

REQUIRE_KEY1             = True
RETURN_TO_HOME           = True      # False = stop safely after (b) placement + 13cm reverse (guarantees base score)
TEST_STAGE               = "FULL_1_BLOCK"
# Available TEST_STAGE values:
#   "FULL_1_BLOCK"      : Full competition run (Stages 1 -> 2 -> 3 -> [4 -> 5 if RETURN_TO_HOME])
#   "EXIT_TEST"         : Test Stage 1 (HOME exit -> outer rail lock) then stop
#   "B_PICK_PLACE"      : Stages 1 -> 2 -> 3 (HOME -> Pick -> Put b -> reverse 13cm -> stop)
#   "B_APPROACH_TEST"   : Start at pickup stop -> Stage 3 align to (b) without releasing
#   "B_RING_TEST"       : Start facing (b) -> test final ring IBVS only
#   "FROM_B_OUTER_LOOP" : Start at (b) post-placement -> Stage 4 + Stage 5 return to HOME
#   "HOME_LEFT_TEST"    : Start on bottom outer rail East of HOME facing West -> Stage 5 dock only

RUN_REV                  = "09-1127A"
RUN_SUMMARY_FILE         = "map_last_run.txt"

# --- Measured chassis kinematics (floor + test24/test28/test29 verified) ---
# Avoid wz=440..480 continuous spins (static friction stall cliff in test28/test29).
DRIVE_CAL_VX             = 560
DRIVE_DEADBAND_MS        = 112
SPEED_CM_PER_S_560       = 16.0
TURN_CAL_WZ              = 640
TURN_90_MS_640           = 950       # Full-battery ~95-105 deg/s; timers are safety caps, NOT primary stops
TURN_SEARCH_WZ           = 540       # Reliable torque band that overcomes static friction on low & full battery
TURN_SLOW_WZ             = 460       # Slower search torque once target landmark enters view


def cm_to_ms(cm):
    if cm <= 0.0:
        return 0
    return int(round(DRIVE_DEADBAND_MS + (float(cm) * 1000.0) / SPEED_CM_PER_S_560))


def deg_to_ms(deg):
    if deg <= 0.0:
        return 0
    return int(round((float(deg) / 90.0) * TURN_90_MS_640))


# --- Image & Outer-Rail PD Follower (QVGA sampled on 160x120-equiv grid) ---
IMG_W                    = 320
IMG_H                    = 240
LINE_W                   = 160
LINE_CX                  = 80

VX_CRUISE                = 620
VX_MIN                   = 440
KP                       = 0.42
KD                       = 1.60
K_ANG                    = 2.0
WZ_MAX                   = 440
SEARCH_WZ                = 540
SEARCH_MS                = 1000
LOCK_HOLD_MS             = 600
LATCH_LOST_SEC           = 3.5

MAX_BLACK_GRAY           = 155
MIN_VALLEY_DROP          = 20
MIN_TAPE_W               = 3
MAX_TAPE_W               = 24
WIDE_JUNC_W              = 58
MAX_ANCHOR_JUMP          = 48
PARALLEL_RAIL_GAP_Q      = 24
LEFT_OUTER_RECOVER_FRAMES = 6
OUTER_TRACK_BIAS_Q       = 0.0
SCAN_BANDS               = ((96, 0.45), (86, 0.35), (76, 0.20))

# --- Stage 1: HOME -> OUTER_WEST (real-video verified E3/E4 event chain) ---
HOME_PRE_TURN_MS         = 520       # Safe pre-turn cap (~40 deg full bat / ~28 deg low bat); cuts short on E3/E4
HOME_PRE_TURN_WZ         = TURN_SEARCH_WZ
HOME_EXIT_MS             = 850       # Max forward exit; cuts short IMMEDIATELY once E3 or E4 appears (fixes test25)
HOME_EXIT_VX             = DRIVE_CAL_VX
HOME_CROSS_TURN_WZ       = 520       # Reliable CCW sweep until E4 (left outer rail + roadway '+' on right)
HOME_CROSS_SLOW_WZ       = TURN_SLOW_WZ
HOME_CROSS_TURN_MAX_MS   = 2600
HOME_E3_MIN_BOT_Y        = 156       # Sandwiched roadway '+' (left outer rail + right inner rail)
HOME_E4_MIN_BOT_Y        = 156       # Final lock: leftmost vertical rail in [35..205] + '+' at cx > rail_x + 30
HOME_E4_MAX_ANG          = 65.0      # Real-video verified (test26 Run 3 outer rail perspective angle = 56 deg)
HOME_E4_CONFIRM          = 2
OUTER_LOCK_INIT_CX       = 40.0

# --- Stage 2: Outer-Rail Cylinder Pickup (R/B/G/K + optional W) ---
MIN_TRACK_BEFORE_PICK_MS = 450
TRIG_ROI                 = (0, 110, 320, 130)
TRIG_BOT_Y               = 168
TRIG_MIN_W               = 20
TRIG_MIN_H               = 24
TRIG_FRAMES              = 2

ENABLE_RED               = True
ENABLE_BLUE              = True
ENABLE_GREEN             = True
ENABLE_BLACK             = True
ENABLE_WHITE             = False

THR_RED                  = (10, 95, 18, 127, 0, 127)
THR_BLUE                 = (10, 90, -35, 35, -128, -18)
THR_GREEN                = (10, 85, -128, -14, 5, 127)
THR_BLACK                = (0, 15, -14, 14, -14, 14)
THR_CROSS                = (0, 44, -18, 18, -18, 18)

FLOOR_ROI                = (0, 110, 320, 130)
BLACK_ROI                = (0, 128, 320, 112)
TARGET_CX                = 130       # Calibrated center under straight-ahead jaws
TARGET_BOT_Y             = 239
TOL_X                    = 8         # Tightened from 10 to prevent claw-pad landing on cylinder rim (test27)
TOL_Y_NEAR               = 236
TOL_Y_FAR                = 240
MAX_CLIP_TOP_Y           = 192

ALIGN_CRUISE_VX          = 560
ALIGN_KP_WZ              = 3.5
ALIGN_MAX_WZ             = 460
FINE_ENTER_Y             = 212
PULSE_VX_FWD             = 580
PULSE_VX_REV             = -560
PULSE_WZ_MAG             = 540
FINE_YAW_PULSE_MS        = 70        # Shortened from 95ms so full-battery pulse won't overshoot c164->c120
COARSE_YAW_PULSE_MS      = 115
PULSE_COOLDOWN_MS        = 165
ALIGN_SETTLE_FRAMES      = 3
CHASSIS_SETTLE_MS        = 180
LOST_CYL_TIMEOUT_MS      = 1800

# --- Stage 3: Cross to Inner (b) Target Ring & Place (4/4 verified test15/16/18/19) ---
TO_INNER_WZ_SIGN         = -1        # -1 CW turn inward from West outer curve toward North (b)
TO_INNER_MIN_MS          = 350       # Brief ignore of outer curve before checking (b) '+' & (b) ring
TO_INNER_MAX_MS          = 2600
TO_INNER_CROSS_MIN_BOT_Y = 148
TO_INNER_CROSS_MIN_X     = 155       # Verified on test16(269..277), test18(250..271), test19(255..267)
TO_INNER_CROSS_MAX_X     = 295
TO_INNER_RING_MIN_X      = 95        # Direct (b) ring lock covers test15 (where '+' clips x=319) & test27
TO_INNER_RING_MAX_X      = 225
TO_INNER_CONFIRM         = 2

CROSS_STEP_MS            = 500
CROSS_STEP_LIMIT         = 7
CROSS_ROAD_MAX_MS        = 12000

B_RING_ALIGN_MS          = 28000
B_RING_LOCK_FRAMES       = 2
B_RING_READY_FRAMES      = 3
B_RING_TOL_Y             = 4
B_RING_MIN_Y             = 128
B_RING_VALLEY_DROP       = 10
B_RING_FAR_Y             = 186
B_RING_FINE_Y            = 212
B_RING_APPROACH_TOL_X    = 20
B_RING_TURN_MS           = 80
B_RING_FAR_STEP_MS       = 520
B_RING_MID_STEP_MS       = 340
B_RING_SETTLE_MS         = 120
B_RING_BACK_Y            = 220
B_RING_LOST_FRAMES       = 2
B_RING_BACK_MS           = 200
B_RING_BACK_SETTLE_MS    = 120
B_RING_BACK_LIMIT        = 3
B_PUT_EXTRA_CM           = 2.5       # Calibrated center nudge into (b) 50pt ring

# --- Stage 4: Leave (b) -> Bypass Left Arc -> North Straight Outer Rail -> CW Lap ---
PAD_REVERSE_CM           = 13.0      # Back clear of placed cylinder at (b) before any turn
NW_TURN_WZ_SIGN          = 1         # +1 CCW left ~35-40 deg NW toward North outer straight (test29 00:07-08)
NW_TURN_MS               = 450
OUTER_CROSS_RAIL_Y       = 200       # Stop once FURTHEST white-mat rail (North outer rail) reaches y>=200
OUTER_CROSS_MAX_MS       = cm_to_ms(55.0)
JOIN_CW_WZ_SIGN          = -1        # Facing NW/North on North outer rail -> CW right (-540) to head East (CW)
JOIN_CW_MIN_MS           = 180
JOIN_CW_MAX_MS           = 3200
JOIN_CW_MAX_ERR          = 580
JOIN_CW_MAX_ANG          = 65        # Real-video verified (test29 10.50s North straight ang=59.4 deg)
JOIN_CW_LOCK_FRAMES      = 2
OUTER_LOOP_ARM_MS        = 8500      # Ride CW North -> East -> South straight before arming HOME detector

# --- Stage 5: East->West Bottom Outer Rail -> 2-Wall Left Gate -> Anti-Glare HOME Dock ---
HOME_APPROACH_SIDE       = "LEFT"
HOME_BRANCH_MIN_MS       = 300
HOME_BRANCH_CONFIRM      = 2
HOME_WALL_CLEAR_FRAMES   = 3
HOME_WALL_MIN_GAP_MS     = 700
HOME_WALL_MAX_GAP_MS     = 4000
HOME_WALL2_ADVANCE_CM    = 10.0      # Verified 2/2 in test22 & test28 (centers pivot at HOME doorway)
HOME_WALL1_FALLBACK_CM   = 42.0
HOME_SEARCH_MAX_MS       = 45000

HOME_TURN_WZ_SIGN        = 1         # +1 CCW left into HOME
HOME_TURN_MIN_MS         = 350       # Skip early <30 deg lines (verified test28 22.25s)
HOME_TURN_MAX_MS         = 2200      # Generous cap at wz=540 so low battery completes 90 deg
HOME_TURN_TILT_TOL       = 15.0
HOME_ALIGN_TOL_TILT      = 12.0      # Tight horizontal squaring for 21x27cm chassis inside 32x32cm HOME
HOME_ALIGN_WAIT_MS       = 2600
HOME_PULSE_WZ_MAG        = 520
HOME_PULSE_SETTLE_MS     = 140
HOME_WALL_CONFIRM_FRAMES = 2

# 21x27cm chassis inside 32x32cm HOME has +-2.5cm length margin -> use visual back-border anchor!
HOME_PARK_STOP_Y         = None      # Optional exact stop row if back border stays in FOV when centered by hand
HOME_ANCHOR_Y            = 224       # Visual distance zero-point as back border nears bottom of FOV
HOME_POST_ANCHOR_CM      = 5.5       # Short battery-tolerant pull-in after visual anchor (+-1.2cm error)
HOME_PARK_MAX_CM         = 18.0      # Hard safety cap so car never drives out the back wall
HOME_LOST_MS             = 300

MAX_RUN_SEC              = 120.0
COVER_STOP_L             = 10
COVER_STOP_FRAMES        = 8

# --- Calibrated 7-step Arm Sequences (0.1 deg, frozen STM32 @J protocol) ---
ARM_HOME    = (450, 918, 900, 990, 1035)
ARM_PRE     = (450, 1120, 820, 1060, 1650)
ARM_ABOVE   = (450, 1600, 680, 1130, 1650)
ARM_CONTACT = (450, 1799, 792, 1060, 1650)
JAW_OPEN    = 900
JAW_CLAMPED = 1150

PICK_STEPS = (
    ("PRE",)     + ARM_PRE     + (JAW_OPEN,    750,  80),
    ("ABOVE",)   + ARM_ABOVE   + (JAW_OPEN,    700,  80),
    ("GRAB",)    + ARM_CONTACT + (JAW_OPEN,    650, 100),
    ("CLAMP",)   + ARM_CONTACT + (JAW_CLAMPED, 550, 260),
    ("LIFT",)    + ARM_ABOVE   + (JAW_CLAMPED, 650,  80),
    ("RETRACT",) + ARM_PRE     + (JAW_CLAMPED, 750,  80),
    ("HOME",)    + ARM_HOME    + (JAW_CLAMPED, 800,  60),
)

PLACE_STEPS = (
    ("PRE",)     + ARM_PRE     + (JAW_CLAMPED, 750,  80),
    ("ABOVE",)   + ARM_ABOVE   + (JAW_CLAMPED, 700,  80),
    ("PLACE",)   + ARM_CONTACT + (JAW_CLAMPED, 650, 180),
    ("RELEASE",) + ARM_CONTACT + (JAW_OPEN,    500, 220),
    ("LIFT",)    + ARM_ABOVE   + (JAW_OPEN,    650,  80),
    ("RETRACT",) + ARM_PRE     + (JAW_OPEN,    750,  80),
    ("HOME",)    + ARM_HOME    + (JAW_OPEN,    800,  60),
)

# ===========================================================================
# HARDWARE & FROZEN STM32 UART3 BRIDGE
# ===========================================================================

led_r = LED(1)
led_g = LED(2)
led_b = LED(3)


def set_led(r, g, b):
    led_r.on() if r else led_r.off()
    led_g.on() if g else led_g.off()
    led_b.on() if b else led_b.off()


try:
    uart = UART(3, 115200, timeout_char=10, read_buf_len=512)
except TypeError:
    uart = UART(3, 115200, timeout_char=10)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
try:
    sensor.set_auto_whitebal(False)
except Exception:
    pass
sensor.skip_frames(time=800)


def _send_packet(body):
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    uart.write("@%s*%02X\n" % (body, csum))


def send_vel(vx, vy=0, wz=0, ms=0):
    _send_packet("V,%d,%d,%d,%d" % (int(vx), int(vy), int(wz), int(ms)))


def send_joints(j0, j1, j2, j3, j4, j5, dur_ms=0):
    _send_packet("J,%d,%d,%d,%d,%d,%d,0,%d" % (
        int(j0), int(j1), int(j2), int(j3), int(j4), int(j5), int(dur_ms)
    ))


def send_sys(sub, val=0):
    _send_packet("S,%d,%d" % (int(sub), int(val)))


class Stm32Link:
    def __init__(self):
        self.rx_buf = b""
        self.armed = False
        self.arm_state = 0
        self.arm_busy = False
        self.holding = False
        self.robot_state = 0
        self.arm_seq = 0
        self.last_rx_ms = 0
        self.has_rx = False

    def poll(self, now_ms):
        n = uart.any()
        if n > 0:
            chunk = uart.read(n)
            if chunk:
                self.rx_buf += bytes(chunk)
                if len(self.rx_buf) > 512:
                    self.rx_buf = self.rx_buf[-256:]
        while True:
            idx = self.rx_buf.find(b"\n")
            if idx < 0:
                break
            raw = self.rx_buf[:idx].strip()
            self.rx_buf = self.rx_buf[idx + 1:]
            self._parse_line(raw, now_ms)

    def _parse_line(self, raw, now_ms):
        pos = -1
        cur = 0
        while True:
            hit = raw.find(b"$E,", cur)
            if hit < 0:
                break
            pos = hit
            cur = hit + 3
        if pos < 0:
            return
        pkt = raw[pos:]
        if len(pkt) < 10 or pkt[-3] != 42:
            return
        star = len(pkt) - 3
        try:
            body = pkt[1:star].decode()
            got_csum = int(pkt[star + 1:].decode(), 16)
        except Exception:
            return
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        if calc != got_csum:
            return
        parts = body.split(",")
        if len(parts) != 7:
            return
        try:
            self.armed = int(parts[1]) != 0
            self.arm_state = int(parts[2])
            self.arm_busy = int(parts[3]) != 0
            self.holding = int(parts[4]) != 0
            self.robot_state = int(parts[5])
            self.arm_seq = int(parts[6])
            self.last_rx_ms = now_ms
            self.has_rx = True
        except Exception:
            return

    def link_ok(self, now_ms):
        return self.has_rx and time.ticks_diff(now_ms, self.last_rx_ms) <= 700


# ===========================================================================
# VISION DETECTORS (ALL REAL-VIDEO VERIFIED ON openmv/test13..test29.mp4)
# ===========================================================================

_RING16 = tuple(
    (math.cos(i * math.pi / 8.0), math.sin(i * math.pi / 8.0)) for i in range(16)
)


def _px_gray(img, x, y):
    r, g, b = img.get_pixel(x, y)
    return (r * 77 + g * 150 + b * 29) >> 8


def find_scanline_tapes(row):
    """Find dark black tapes bounded by brighter white mat on BOTH shoulders.
    Off-mat corridor floor fails min(lb, rb) - v >= MIN_VALLEY_DROP, so tapes[0] is the
    LEFTMOST painted track rail (= OUTER rail on CW loop)."""
    s_sorted = sorted(row)
    bg_ref = s_sorted[min(LINE_W - 1, (LINE_W * 95) // 100)]

    junc_run = 0
    max_junc_run = 0
    for x in range(15, 145):
        v = row[x]
        if v <= MAX_BLACK_GRAY and ((bg_ref - v) >= 24 or v <= 95):
            junc_run += 1
            if junc_run > max_junc_run:
                max_junc_run = junc_run
        else:
            junc_run = 0
    wide_hit = max_junc_run >= WIDE_JUNC_W

    tapes = []
    m = 3
    while m <= LINE_W - 4:
        v = row[m]
        if (v <= MAX_BLACK_GRAY and
                v <= row[m - 1] and v <= row[m + 1] and
                v <= row[m - 2] and v <= row[m + 2]):
            lb = max(row[max(0, m - 14) : m - 1])
            rb = max(row[m + 2 : min(LINE_W, m + 15)])
            ref = lb if lb < rb else rb
            drop = ref - v
            if drop >= MIN_VALLEY_DROP:
                half = 0.5 * (ref + v)
                x0 = m
                while x0 > 0 and row[x0 - 1] <= half:
                    x0 -= 1
                x1 = m
                while x1 < (LINE_W - 1) and row[x1 + 1] <= half:
                    x1 += 1
                bw = x1 - x0 + 1
                if MIN_TAPE_W <= bw <= MAX_TAPE_W and x0 >= 1 and x1 <= (LINE_W - 2):
                    ws = 0.0
                    wsum = 0.0
                    for k in range(x0, x1 + 1):
                        d = half - row[k]
                        if d > 0.0:
                            ws += d
                            wsum += k * d
                    cx = (wsum / ws) if ws > 0.0 else float(m)
                    if tapes and (cx - tapes[-1][0]) < 10.0:
                        if drop > tapes[-1][1]:
                            tapes[-1] = (cx, drop, x0, x1)
                    else:
                        tapes.append((cx, drop, x0, x1))
                    m = x1 + 3
                    continue
        m += 1
    return tapes, wide_hit


def detect_outer_line(img, anchor_cx, had_lock, prefer_leftmost=True):
    """Outer-rail-priority line tracker for CW outer loop.
    Set prefer_leftmost=False on HOME straight so HOME's outer border on left never steals lock."""
    pts = []
    any_wide = False
    parallel_left_cx = None
    anchor = anchor_cx

    for cy_q, weight in SCAN_BANDS:
        y = cy_q * 2
        row = [_px_gray(img, xq * 2, y) for xq in range(LINE_W)]
        tapes, wide = find_scanline_tapes(row)
        if wide:
            any_wide = True
        if not tapes:
            continue

        if not had_lock:
            chosen = tapes[0] if prefer_leftmost else min(tapes, key=lambda t: abs(t[0] - anchor))
        else:
            in_range = [t for t in tapes if abs(t[0] - anchor) <= MAX_ANCHOR_JUMP]
            if not in_range:
                continue
            chosen = in_range[0] if prefer_leftmost else min(in_range, key=lambda t: abs(t[0] - anchor))
            leftmost = tapes[0][0]
            if prefer_leftmost and (chosen[0] - leftmost) >= PARALLEL_RAIL_GAP_Q and parallel_left_cx is None:
                parallel_left_cx = leftmost

        cx = chosen[0]
        anchor = cx
        pts.append((cx, float(cy_q), weight))

    return pts, any_wide, parallel_left_cx


def _is_cross_ring16(img, x, y, w, h):
    """Rotation- & shade-invariant '+' topology check via 16-point ellipse ring.
    Offline verified on shaded North mat (hi=114..122 in test16/18/19 -> hi>=85, hi-base>=24)
    and tilted 0..45 deg crosses; strictly rejects lines/diagonals(2), L(2), T(3)."""
    cx, cy = x + w // 2, y + h // 2
    arm_c = _px_gray(img, cx, cy)
    if arm_c > 118:
        return False
    rx = max(4.0, 0.38 * w)
    ry = max(3.5, 0.38 * h)
    vals = []
    for c_a, s_a in _RING16:
        px = max(0, min(IMG_W - 1, int(round(cx + rx * c_a))))
        py = max(0, min(IMG_H - 1, int(round(cy + ry * s_a))))
        vals.append(_px_gray(img, px, py))
    hi = max(vals)
    lo = min(vals)
    base = arm_c if arm_c > lo else lo
    if hi < 85 or (hi - base) < 24:
        return False
    thr = 0.5 * (hi + base)
    dark = [(v <= thr) for v in vals]
    if sum(1 for d in dark if not d) < 4:
        return False
    runs = 0
    for i in range(16):
        if dark[i] and (not dark[(i - 1) & 15]):
            runs += 1
    return runs == 4


def detect_crosses_all(img, allow_bottom_clip=True, allow_right_clip=False):
    """Return ALL valid '+' bounding boxes (x, y, w, h) in lower floor ROI y=128..239."""
    roi = (0, 128, IMG_W, IMG_H - 128)
    blobs = img.find_blobs([THR_CROSS], roi=roi, x_stride=2, y_stride=1,
                           pixels_threshold=45, area_threshold=60)
    out = []
    for b in blobs:
        x, y, w, h = b.rect()
        if not (16 <= w <= 215 and 8 <= h <= 115 and w <= 7.5 * h and h <= 3.0 * w):
            continue
        if y <= roi[1]:
            continue
        if (not allow_right_clip) and (x + w >= IMG_W - 1):
            continue
        if (not allow_bottom_clip) and (y + h >= IMG_H - 1):
            continue
        sol = float(b.pixels()) / float(max(1, w * h))
        if sol < 0.18 or sol > 0.62:
            continue
        if _is_cross_ring16(img, x, y, w, h):
            out.append((x, y, w, h))
    return out


def detect_stage1_events(img, left_pts, left_ang):
    """Stage 1 (HOME exit) topological event detector — offline verified 8/8 TP & 0/3 test27 hijack.
    Returns (crosses, e3_box, e4_box):
      e3_box (Fig 3): roadway '+' sandwiched between LEFT outer rail AND RIGHT inner rail.
      e4_box (Fig 4): leftmost vertical outer rail (pts>=2, x in [35..205], |ang|<=65)
                      with roadway '+' cleanly to its RIGHT (cx > rail_x + 30)."""
    crosses = detect_crosses_all(img, allow_bottom_clip=True, allow_right_clip=False)
    cx_l = None
    if len(left_pts) >= 2:
        wsum = sum(p[2] for p in left_pts)
        cx_l = (sum(p[0] * p[2] for p in left_pts) / wsum) * 2.0

    e3_box = None
    e4_box = None
    for box in crosses:
        x, y, w, h = box
        cx, cy, bot = x + w // 2, y + h // 2, y + h
        if bot < HOME_E3_MIN_BOT_Y:
            continue
        # Check surrounding rails on scanlines around the cross
        has_l = False
        has_r = False
        for ry in (max(134, cy - 12), cy, min(234, cy + 12)):
            row = [_px_gray(img, xq * 2, ry) for xq in range(LINE_W)]
            tapes, _ = find_scanline_tapes(row)
            for t in tapes:
                tx = t[0] * 2.0
                if tx < x - 6:
                    has_l = True
                elif tx > x + w + 6:
                    has_r = True
        if has_l and has_r and bot >= HOME_E3_MIN_BOT_Y:
            if e3_box is None or bot > (e3_box[1] + e3_box[3]):
                e3_box = box
        if (cx_l is not None and 35.0 <= cx_l <= 205.0 and
                left_ang is not None and abs(left_ang) <= HOME_E4_MAX_ANG and
                cx > cx_l + 30.0 and bot >= HOME_E4_MIN_BOT_Y):
            if e4_box is None or bot > (e4_box[1] + e4_box[3]):
                e4_box = box
    return crosses, e3_box, e4_box


def detect_b_cross(img):
    """Select the nearest (b) '+' cross inside lower floor ROI (allows slight right-edge touch)."""
    crosses = detect_crosses_all(img, allow_bottom_clip=True, allow_right_clip=True)
    best = None
    best_bot = -1
    for box in crosses:
        bot = box[1] + box[3]
        if bot >= TO_INNER_CROSS_MIN_BOT_Y and bot > best_bot:
            best = box
            best_bot = bot
    return best


# --- Cylinder detector (Stage 2, verified 7/7 on test14..test21) ---
def build_cyl_thresholds():
    thrs, labels = [], []
    if ENABLE_RED:
        thrs.append(THR_RED)
        labels.append("R")
    if ENABLE_BLUE:
        thrs.append(THR_BLUE)
        labels.append("B")
    if ENABLE_GREEN:
        thrs.append(THR_GREEN)
        labels.append("G")
    return thrs, labels


CYL_THRS, CYL_LABELS = build_cyl_thresholds()


def detect_white_cyl(img):
    for y_mid in (165, 185, 205):
        col_g = []
        col_min = []
        for x in range(0, IMG_W, 4):
            s = 0
            mn = 255
            for dy in (-8, -4, 0, 4, 8):
                v = _px_gray(img, x, min(IMG_H - 1, y_mid + dy))
                s += v
                if v < mn:
                    mn = v
            col_g.append(s // 5)
            col_min.append(mn)
        ref = sorted(col_g)[(len(col_g) * 3) // 4]
        n = len(col_g)
        for i in range(2, n - 6):
            if col_min[i] >= 95 and 18 <= (ref - col_g[i]) <= 68:
                pk = max(col_g[max(0, i - 9) : min(n, i + 10)])
                if (pk - col_g[i]) >= 42 and pk >= max(175, ref + 10):
                    cx = i * 4
                    if min(_px_gray(img, cx, y_mid - 16), _px_gray(img, cx, y_mid - 10)) >= 95:
                        bot_y = min(IMG_H, y_mid + 30)
                        return (cx, max(110, y_mid - 35), bot_y, 38, 65, 1600, "W",
                                (max(0, cx - 19), max(110, y_mid - 35), 38, 65))
    return None


def black_blob_follows_tape(img, rect):
    bx, by, bw, _ = rect
    if bx <= 1 or bx + bw >= IMG_W - 1:
        return True
    previous_x = None
    for y in (by + 4, by - 3, by - 9):
        row = [_px_gray(img, x * 2, y) for x in range(LINE_W)]
        tapes, _ = find_scanline_tapes(row)
        centers = [tape[0] * 2 for tape in tapes]
        if previous_x is None:
            centers = [x for x in centers if bx - 2 <= x <= bx + bw + 2]
            anchor = bx + bw / 2
        else:
            centers = [x for x in centers if abs(x - previous_x) <= 20]
            anchor = previous_x
        if not centers:
            return False
        previous_x = min(centers, key=lambda x: abs(x - anchor))
    return True


def detect_cylinder(img, roi=FLOOR_ROI):
    groups = []
    if CYL_THRS:
        groups.append((CYL_LABELS, img.find_blobs(
            CYL_THRS, roi=roi, pixels_threshold=140, area_threshold=180, merge=True, margin=6
        )))
    if ENABLE_BLACK:
        blk_roi = (roi[0], max(roi[1], BLACK_ROI[1]), roi[2],
                   max(1, min(roi[1] + roi[3], BLACK_ROI[1] + BLACK_ROI[3]) - max(roi[1], BLACK_ROI[1])))
        groups.append((("K",), img.find_blobs(
            [THR_BLACK], roi=blk_roi, pixels_threshold=110, area_threshold=150, merge=False
        )))
    best = None
    best_score = -1
    for lbls, blist in groups:
        for b in blist:
            bx, by, bw, bh = b.rect()
            bot_y = by + bh
            if bw < 14 or bw > 135 or bh < 18 or bh > 165 or bot_y < 115:
                continue
            sol = float(b.pixels()) / float(max(1, bw * bh))
            if sol < 0.45:
                continue
            if len(lbls) == 1:
                tag = lbls[0]
            else:
                code = b.code()
                tag = "C"
                for idx, lbl in enumerate(lbls):
                    if code == (1 << idx):
                        tag = lbl
                        break
            ratio = float(bh) / float(max(1, bw))
            min_ratio = 0.70 if tag == "K" else (0.25 if bot_y >= 236 else 0.50)
            if (tag == "K" and bw > 95) or ratio < min_ratio or ratio > 3.20:
                continue
            if tag == "K" and black_blob_follows_tape(img, (bx, by, bw, bh)):
                continue
            score = bot_y * 1000 + min(999, b.pixels() // 20)
            if score > best_score:
                best_score = score
                best = (b.cx(), by, bot_y, bw, bh, b.pixels(), tag, (bx, by, bw, bh))
    if best is None and ENABLE_WHITE:
        best = detect_white_cyl(img)
    return best


def compute_align_cmd(cx, top_y, bot_y):
    ex = cx - TARGET_CX
    ey = TARGET_BOT_Y - bot_y
    in_x = abs(ex) <= TOL_X
    too_close = (bot_y >= 238) and (top_y > MAX_CLIP_TOP_Y)
    in_y = (bot_y >= TOL_Y_NEAR) and (bot_y <= TOL_Y_FAR) and (not too_close)
    if in_x and in_y:
        return True, "STOP", 0, 0, 0, ex, ey

    if bot_y < FINE_ENTER_Y:
        if abs(ex) >= 75:
            wz = -PULSE_WZ_MAG if ex > 0 else PULSE_WZ_MAG
            return False, "CRUISE", 0, wz, 0, ex, ey
        wz = int(max(-ALIGN_MAX_WZ, min(ALIGN_MAX_WZ, round(-ALIGN_KP_WZ * ex))))
        return False, "CRUISE", ALIGN_CRUISE_VX, wz, 0, ex, ey

    if not in_x:
        wz = -PULSE_WZ_MAG if ex > 0 else PULSE_WZ_MAG
        ms = COARSE_YAW_PULSE_MS if abs(ex) >= 28 else FINE_YAW_PULSE_MS
        return False, "PULSE", 0, wz, ms, ex, ey

    if too_close:
        return False, "PULSE", PULSE_VX_REV, 0, 100, ex, ey

    ms = 145 if ey >= 16 else 105
    return False, "PULSE", PULSE_VX_FWD, 0, ms, ex, ey


# --- Concentric (b) Ring Detector (Stage 3, verified 4/4 on test15/16/18/19) ---
def fit_ring_chords(midpoints, widths_sq, first_y, step):
    z0, z1, z2, z3, z4 = widths_sq
    a = (2 * (z0 + z4 - z2) - z1 - z3) / 14
    b = (2 * (z4 - z0) + z3 - z1) / 10
    c = (sum(widths_sq) - 10 * a) / 5
    if a >= -1:
        return None
    t = -b / (2 * a)
    center_y = first_y + (2 + t) * step
    width_sq = c + b * t + a * t * t
    max_residual = 0.08 * max(widths_sq) + 30
    for i, width in enumerate(widths_sq):
        j = i - 2
        if abs(c + b * j + a * j * j - width) > max_residual:
            return None
    center_x = sum(midpoints) / 5 + t * (
        2 * (midpoints[4] - midpoints[0]) + midpoints[3] - midpoints[1]) / 10
    return center_x, center_y, width_sq


def ring_row_chord(img, y, hint_x, half_search):
    x0 = max(0, hint_x - half_search - 115)
    x1 = min(IMG_W - 1, hint_x + half_search + 115)
    row = [_px_gray(img, x, y) for x in range(x0, x1 + 1)]
    valleys = []
    for i in range(5, len(row) - 5):
        value = row[i]
        if value > min(row[i - 2], row[i - 1], row[i + 1], row[i + 2]):
            continue
        drop = min(max(row[i - 5:i - 1]), max(row[i + 2:i + 6])) - value
        if drop < B_RING_VALLEY_DROP:
            continue
        sample = (x0 + i, drop)
        if valleys and sample[0] - valleys[-1][0] < 4:
            if drop > valleys[-1][1]:
                valleys[-1] = sample
        else:
            valleys.append(sample)
    if len(valleys) < 6:
        return None

    best = None
    best_score = 0
    for axis in range(max(16, hint_x - half_search), min(IMG_W - 16, hint_x + half_search) + 1, 3):
        pairs = []
        left, right = 0, len(valleys) - 1
        while left < right:
            lx, ld = valleys[left]
            rx, rd = valleys[right]
            offset = lx + rx - 2 * axis
            if offset < -9:
                left += 1
            elif offset > 9:
                right -= 1
            else:
                span = rx - lx
                if 16 <= span <= 260:
                    pairs.append(((lx + rx) / 2, span, min(ld, rd)))
                left += 1
                right -= 1
        for i in range(len(pairs) - 2):
            cx, span, drop = pairs[i]
            gap_a = span - pairs[i + 1][1]
            gap_b = pairs[i + 1][1] - pairs[i + 2][1]
            if min(gap_a, gap_b) < 10 or max(gap_a, gap_b) > 1.65 * min(gap_a, gap_b):
                continue
            count = 3
            for j in range(i + 3, len(pairs)):
                gap = pairs[j - 1][1] - pairs[j][1]
                if not 0.65 * gap_a <= gap <= 1.5 * gap_a:
                    break
                count += 1
            score = 12 * count + 0.28 * min(drop, 35) + 1.55 * span - 0.30 * abs(axis - hint_x)
            if score > best_score:
                best, best_score = (cx, span, score), score
    return best


def scan_b_ring(img, hint_x, max_y, previous_ring, wide_search=False):
    lo = B_RING_MIN_Y - 10
    hi = min(IMG_H, max_y + 8)
    wide_acquire = wide_search and previous_ring is None
    if wide_acquire:
        hint_x = IMG_W // 2
        hi = min(hi, 212)
    half_search = 138 if wide_acquire else 72
    row_step = 1 if (wide_search or (previous_ring is not None and previous_ring[1] < 185)) else 2
    if previous_ring is not None:
        hint_x, prev_y, _ = previous_ring
        lo = max(lo, prev_y - 28)
        hi = min(hi, prev_y + 30)
        half_search = 112 if wide_search else 35
    if hi == IMG_H:
        lo += (IMG_H - 1 - lo) % row_step
    rows = [ring_row_chord(img, y, hint_x, half_search) for y in range(lo, hi, row_step)]
    best_ring = None
    best_score = 0
    for stride in (1, 2):
        step = row_step * stride
        for start in range(len(rows) - 4 * stride):
            samples = rows[start:start + 4 * stride + 1:stride]
            if None in samples:
                continue
            midpoints = [s[0] for s in samples]
            if max(midpoints) - min(midpoints) > 24:
                continue
            widths_sq = [s[1] * s[1] for s in samples]
            y0 = lo + start * row_step
            fit = fit_ring_chords(midpoints, widths_sq, y0, step)
            if fit is None:
                continue
            cx, cy, width_sq = fit
            widest_sq = max(widths_sq)
            if not (max(B_RING_MIN_Y, y0 - 3) <= cy <= min(max_y, y0 + 4 * step + 10) and
                    abs(cx - hint_x) <= half_search and 0.86 * widest_sq <= width_sq <= 2.1 * widest_sq):
                continue
            span = int(round(math.sqrt(width_sq)))
            if wide_search and previous_ring is not None:
                prev_span = previous_ring[2]
                if not 0.65 * prev_span <= span <= 1.45 * prev_span:
                    continue
            score = sum(s[2] for s in samples) - abs(cy - (y0 + 2 * step))
            if score > best_score:
                best_ring, best_score = (int(round(cx)), int(round(cy)), span), score
    return best_ring


def detect_b_ring(img, cross_box, previous_ring, wide_search=False):
    hint_x = TARGET_CX
    max_y = IMG_H + 12
    if cross_box is not None:
        hint_x = cross_box[0] + cross_box[2] // 2
        max_y = cross_box[1] + cross_box[3] // 2 - 10
    return scan_b_ring(img, hint_x, max_y, previous_ring, wide_search)


# --- Stage 4 & 5 Detectors (Outer Rail Ahead, 2-Wall Left Branch, Anti-Glare Back Border) ---
def column_mat_rails(img, x, first_y=140):
    """All white-mat-bounded dark stripe rows in column x, sorted far->near (ascending y)."""
    column = [_px_gray(img, x, y) for y in range(first_y, IMG_H)]
    stripes = []
    end = len(column) - 11
    while end >= 10:
        if column[end] > 115:
            end -= 1
            continue
        start = end
        while start > 0 and column[start - 1] <= 115:
            start -= 1
        if start >= 10 and 4 <= (end - start + 1) <= 26:
            ahead = sum(column[start - 10:start - 4]) / 6.0
            behind = sum(column[end + 5:end + 11]) / 6.0
            if min(ahead, behind) >= 140.0 and abs(ahead - behind) <= 40.0:
                stripes.append(first_y + (start + end) // 2)
        end = start - 1
    stripes.reverse()
    return stripes


def detect_outer_rail_ahead(img):
    """Pick FURTHEST white-mat-bounded rail (skips inner rail, stops at North outer rail)."""
    cols = (100, 130, 160, 190, 220, 250, 280)
    outer_rows = []
    dual_cols = 0
    for x in cols:
        st = column_mat_rails(img, x)
        if st:
            outer_rows.append(st[0])
            if len(st) >= 2 and (st[-1] - st[0]) >= 24:
                dual_cols += 1
        else:
            outer_rows.append(None)

    best_y = None
    for i in range(len(outer_rows) - 2):
        trip = outer_rows[i:i + 3]
        if None in trip:
            continue
        if abs(trip[0] - trip[1]) <= 18 and abs(trip[1] - trip[2]) <= 18:
            mid_y = trip[1]
            if best_y is None or mid_y > best_y:
                best_y = mid_y
    return best_y, (dual_cols >= 2)


def detect_home_branch(img, pts, side="LEFT"):
    """Detect HOME perpendicular side wall touching fitted outer rail on `side`."""
    if len(pts) < 2:
        return None
    near_x, near_y, _ = pts[0]
    far_x, far_y, _ = pts[-1]
    if abs(near_y - far_y) < 1e-3:
        return None
    slope = (near_x - far_x) / (near_y - far_y)
    direction = -1 if side == "LEFT" else 1
    for y in range(220, 145, -6):
        rail_x = int(round(near_x * 2 + (y - near_y * 2) * slope))
        if not (0 <= rail_x < IMG_W):
            continue
        end_x = rail_x
        gap = 0
        for offset in range(0, 108, 2):
            x = rail_x + direction * offset
            if not (0 <= x < IMG_W):
                break
            if min(_px_gray(img, x, y), _px_gray(img, x, min(IMG_H - 1, y + 4))) <= 120:
                end_x = x
                gap = 0
            else:
                gap += 1
                if gap > 2:
                    break
        span = abs(end_x - rail_x)
        if span >= 54:
            middle = (rail_x + end_x) // 2
            above_g = _px_gray(img, middle, max(0, y - 10))
            below_g = _px_gray(img, middle, min(IMG_H - 1, y + 12))
            if min(above_g, below_g) >= 145:
                return (side, min(rail_x, end_x), y - 2, span + 1, 8, span)
    return None


def _find_col_home_back_border(img, x):
    """Anti-glare column search for HOME's furthest black back border.
    Offline verified 11/11 frames on test28 00:23-26 bright floor glare & 0/6 on test22 00:15 oval."""
    col = [_px_gray(img, x, y) for y in range(65, IMG_H)]
    off = 65
    wall_g = sum(col[0:50]) / 50.0  # y = 65..114 corridor wall / doors darkness
    best_y = None
    y = 232
    while y >= 132:
        if col[y - off] > 118:
            y -= 1
            continue
        y_bot = y
        y_top = y
        while y_top > 120 and (y_bot - y_top) < 34 and col[(y_top - 1) - off] <= 118:
            y_top -= 1
        band_h = y_bot - y_top + 1
        if band_h >= 3:
            b0 = min(IMG_H - 1, y_bot + 4) - off
            b1 = min(IMG_H, y_bot + 16) - off
            if (b1 - b0) >= 3:
                below_g = sum(col[b0:b1]) / float(b1 - b0)
                if below_g >= 135.0:
                    band_min = min(col[(y_top - off):(y_bot - off + 1)])
                    if band_min <= 105 and (below_g - band_min) >= 35.0:
                        if band_h >= 28:
                            is_border = True
                        else:
                            a0 = max(100, y_top - 16) - off
                            a1 = max(a0 + 1, (y_top - 4) - off)
                            above_g = sum(col[a0:a1]) / float(a1 - a0)
                            # Accept if immediate above is darker floor OR shiny floor + dark corridor wall
                            is_border = ((below_g - above_g) >= 20.0) or (wall_g <= 125.0)
                        if is_border:
                            best_y = y_bot  # keep scanning up to select FURTHEST (back) border
        y = y_top - 2
    return best_y


def detect_home_back_wall(img):
    """Anti-glare 7-column straight back-border detector for HOME squaring & depth anchoring.
    Returns (center_y, tilt_px, n_inliers, inlier_pts) or None."""
    xs = (50, 86, 122, 160, 198, 234, 270)
    pts = []
    for x in xs:
        yb = _find_col_home_back_border(img, x)
        if yb is not None:
            pts.append((float(x), float(yb)))
    if len(pts) < 4:
        return None

    best_inliers = []
    best_err = 1e9
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        for j in range(i + 1, n):
            x2, y2 = pts[j]
            if (x2 - x1) < 60.0:
                continue
            a = (y2 - y1) / (x2 - x1)
            if abs(a) > 0.45:
                continue
            b = y1 - a * x1
            inl = []
            err_sum = 0.0
            for px, py in pts:
                e = abs(py - (a * px + b))
                if e <= 8.0:
                    inl.append((px, py))
                    err_sum += e
            if len(inl) > len(best_inliers) or (len(inl) == len(best_inliers) and err_sum < best_err):
                best_inliers = inl
                best_err = err_sum

    if len(best_inliers) < 4 or (best_inliers[-1][0] - best_inliers[0][0]) < 100.0:
        return None

    m = float(len(best_inliers))
    sx = sum(p[0] for p in best_inliers)
    sy = sum(p[1] for p in best_inliers)
    sxx = sum(p[0] * p[0] for p in best_inliers)
    sxy = sum(p[0] * p[1] for p in best_inliers)
    denom = m * sxx - sx * sx
    if abs(denom) < 1e-6:
        return None
    a = (m * sxy - sx * sy) / denom
    b = (sy - a * sx) / m
    if abs(a) > 0.45 or max(abs(py - (a * px + b)) for px, py in best_inliers) > 8.0:
        return None
    center_y = a * 160.0 + b
    if not (132.0 <= center_y <= 234.0):
        return None
    return (center_y, a * 200.0, len(best_inliers), best_inliers)


def follow_line_pd(line_valid, err_out, ang_out, has_prev_err, prev_e_comb):
    e_comb = max(-400.0, min(400.0, 0.5 * err_out + 0.5 * ang_out * K_ANG)) if line_valid == 2 else (
        max(-1200.0, min(1200.0, err_out + ang_out * K_ANG)))
    d_err = max(-120.0, min(120.0, e_comb - prev_e_comb)) if has_prev_err else 0.0
    wz_cmd = int(max(-WZ_MAX, min(WZ_MAX, round(-(KP * e_comb + KD * d_err)))))
    vx_cmd = int(round(VX_CRUISE - (min(1000.0, abs(e_comb)) * max(0, VX_CRUISE - VX_MIN)) / 1000.0))
    return vx_cmd, wz_cmd, e_comb


def wall_yaw_pulse(tilt_px):
    wz = -HOME_PULSE_WZ_MAG if tilt_px > 0.0 else HOME_PULSE_WZ_MAG
    ms = 110 if abs(tilt_px) >= 26.0 else 75
    return wz, ms


# ===========================================================================
# UNIFIED 5-STAGE CLOSED-LOOP STATE MACHINE
# ===========================================================================

link = Stm32Link()
clock = time.clock()
fsm_state = "WAIT_KEY1"
stop_reason = None
seen_disarmed = False
summary_written = False

t_run_start = 0
t_stage_start = 0
t_line_lost_start = None
t_cyl_lost_start = None
t_pulse_done = 0
t_pause_until = 0
t_step_start = 0
t_step_wait_until = 0
t_b_aligned = None
t_park_anchor = None
t_last_ping = 0
t_last_log = 0

# Line tracker state
last_cx = float(LINE_CX)
had_lock = False
err_filt = 0.0
has_prev_err = False
prev_e_comb = 0.0
last_dir = 0
left_rail_streak = 0
outer_shifts = 0

# Stage 1 / 2 / 3 state
saw_exit_e3 = False
turn_lock_streak = 0
ring_lock_streak = 0
cyl_trig_cnt = 0
settle_cnt = 0
cross_steps = 0
b_ring = None
b_ring_streak = 0
b_ring_misses = 0
b_ready_streak = 0
b_back_steps = 0
b_back_active = False
b_preview_ring = None
pick_idx = 0
pick_sent_seq = 0
arm_phase = "PICK"
active_steps = PICK_STEPS

# Stage 4 / 5 state
last_outer_rail_y = None
home_branch_streak = 0
last_branch_y = None
home_wall_count = 0
home_wall_clear_streak = 0
home_wall_saw_gap = False
t_home_wall1 = 0
gate_advance_cm = HOME_WALL1_FALLBACK_CM
gate_src = "none"
home_wall_streak = 0
last_back_y = None
last_back_tilt = None
align_pulses = 0
seen_park_anchor = False

cover_cnt = 0
last_log_key = None


def reset_line_state(init_cx=None):
    global last_cx, had_lock, err_filt, has_prev_err, prev_e_comb, last_dir
    global t_line_lost_start, left_rail_streak
    last_cx = float(LINE_CX) if init_cx is None else float(init_cx)
    had_lock = False
    err_filt = 0.0
    has_prev_err = False
    prev_e_comb = 0.0
    last_dir = 0
    t_line_lost_start = None
    left_rail_streak = 0


def reset_home_wall_state():
    global home_branch_streak, last_branch_y, home_wall_count, home_wall_clear_streak
    global home_wall_saw_gap, t_home_wall1, gate_advance_cm, gate_src
    home_branch_streak = 0
    last_branch_y = None
    home_wall_count = 0
    home_wall_clear_streak = 0
    home_wall_saw_gap = False
    t_home_wall1 = 0
    gate_advance_cm = HOME_WALL1_FALLBACK_CM
    gate_src = "none"


def enter_stage(state_name, now_ms):
    global fsm_state, t_stage_start, turn_lock_streak, ring_lock_streak, last_outer_rail_y
    global home_wall_streak, t_pulse_done, align_pulses, cross_steps
    global b_ring, b_ring_streak, b_ring_misses, b_ready_streak, b_back_steps, b_back_active, t_b_aligned
    global seen_park_anchor, t_park_anchor
    fsm_state = state_name
    t_stage_start = now_ms
    turn_lock_streak = 0
    ring_lock_streak = 0
    last_outer_rail_y = None
    home_wall_streak = 0
    t_pulse_done = now_ms
    if state_name == "CROSS_TO_INNER":
        cross_steps = 0
        b_ring = None
        b_ring_streak = 0
    elif state_name == "ALIGN_B_RING":
        b_ring_streak = 0
        b_ring_misses = 0
        b_ready_streak = 0
        b_back_steps = 0
        b_back_active = False
        t_b_aligned = None
    elif state_name == "HOME_VERIFY_ALIGN":
        align_pulses = 0
    elif state_name == "HOME_PARK_IN_BOX":
        seen_park_anchor = False
        t_park_anchor = None


def write_summary(now_ms, reason):
    global summary_written
    if summary_written:
        return
    summary_written = True
    by_s = ("%d" % int(round(last_back_y))) if last_back_y is not None else "None"
    tilt_s = ("%.1f" % last_back_tilt) if last_back_tilt is not None else "None"
    ring_s = ("%d,%d" % b_ring[:2]) if b_ring is not None else "None"
    line = (
        "rev=%s stage=%s mode=%s ret=%d stop=%s elapsed_ms=%d stage_ms=%d "
        "ring=%s walls=%d gate=%s shifts=%d pulses=%d back_y=%s tilt=%s anchor=%d\n" % (
            RUN_REV, fsm_state, TEST_STAGE, int(RETURN_TO_HOME), reason,
            time.ticks_diff(now_ms, t_run_start), time.ticks_diff(now_ms, t_stage_start),
            ring_s, home_wall_count, gate_src, outer_shifts, align_pulses,
            by_s, tilt_s, int(seen_park_anchor)
        )
    )
    print("SUMMARY " + line.strip())
    try:
        with open(RUN_SUMMARY_FILE, "w") as f:
            f.write(line)
    except Exception:
        pass


def latch_stop(reason, now_ms, soft_stop=False):
    global stop_reason, fsm_state
    send_vel(0, 0, 0, 0)
    if soft_stop:
        send_sys(0, 0)
    write_summary(now_ms, reason)
    stop_reason = reason
    fsm_state = "LATCH_STOP"


print("COMP09 %s mode=%s ret_home=%d tol_x=%d fine_ms=%d" % (
    RUN_REV, TEST_STAGE, int(RETURN_TO_HOME), TOL_X, FINE_YAW_PULSE_MS
))

while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    link.poll(now_ms)
    link_ok = link.link_ok(now_ms)

    if not link_ok:
        seen_disarmed = False
    elif not link.armed:
        seen_disarmed = True

    st_roi = img.get_statistics(roi=(40, 80, 240, 120))
    cover_cnt = (cover_cnt + 1) if (st_roi.l_mean() <= COVER_STOP_L) else 0

    # --- 1. VISION PASS (strictly BEFORE any debug overlays on img) ---
    prefer_left = fsm_state in (
        "WAIT_KEY1", "HOME_PRE_TURN", "LEAVE_HOME", "HOME_FIND_CROSS",
        "FOLLOW_OUTER_PICK", "JOIN_OUTER_CW", "FOLLOW_OUTER_LOOP"
    )
    pts, any_wide, parallel_left_cx = detect_outer_line(img, last_cx, had_lock, prefer_leftmost=prefer_left)
    if parallel_left_cx is not None and fsm_state in ("FOLLOW_OUTER_PICK", "FOLLOW_OUTER_LOOP"):
        left_rail_streak += 1
        if left_rail_streak >= LEFT_OUTER_RECOVER_FRAMES:
            last_cx = parallel_left_cx
            had_lock = False
            left_rail_streak = 0
            outer_shifts += 1
            print("OUTER_RECOVER shifted left to cx=%.1f" % parallel_left_cx)
    else:
        left_rail_streak = 0

    if pts:
        t_line_lost_start = None
        w_sum = sum(w for _, _, w in pts)
        cx_avg = sum(cx * w for cx, _, w in pts) / w_sum
        last_cx = pts[0][0] if len(pts) == 1 else cx_avg
        had_lock = True
        target_line_cx = float(LINE_CX) + OUTER_TRACK_BIAS_Q
        err_raw = ((cx_avg - target_line_cx) * 1000.0) / float(LINE_CX)
        err_filt = 0.75 * err_raw + 0.25 * err_filt
        err_out = int(max(-1000, min(1000, round(err_filt))))
        if abs(err_out) >= 60:
            last_dir = -1 if err_out < 0 else 1
        if len(pts) >= 2:
            lo_cx, lo_cy, _ = pts[0]
            hi_cx, hi_cy, _ = pts[-1]
            ang_out = int(max(-90, min(90, round(
                math.degrees(math.atan2(hi_cx - lo_cx, max(1.0, lo_cy - hi_cy)))
            ))))
        else:
            ang_out = 0
        line_valid = 2 if (any_wide and len(pts) <= 1) else 1
    else:
        if t_line_lost_start is None:
            t_line_lost_start = now_ms
        elif time.ticks_diff(now_ms, t_line_lost_start) >= LOCK_HOLD_MS:
            had_lock = False
        err_out = int(max(-1000, min(1000, round(err_filt))))
        ang_out = 0
        line_valid = 0

    # Stage 1 detectors
    s1_crosses = []
    e3_box = None
    e4_box = None
    if fsm_state in ("WAIT_KEY1", "HOME_PRE_TURN", "LEAVE_HOME", "HOME_FIND_CROSS"):
        s1_crosses, e3_box, e4_box = detect_stage1_events(img, pts, float(ang_out) if len(pts) >= 2 else None)

    # Stage 2 detector
    cyl_hit = None
    if fsm_state in ("FOLLOW_OUTER_PICK", "ALIGN_PICK"):
        cyl_hit = detect_cylinder(img, FLOOR_ROI if fsm_state == "ALIGN_PICK" else TRIG_ROI)

    # Stage 3 detectors ((b) cross + concentric (b) ring)
    b_cross = detect_b_cross(img) if fsm_state == "TURN_TO_INNER" else None
    b_cross_cx = (b_cross[0] + b_cross[2] // 2) if b_cross is not None else None
    b_cross_bot = (b_cross[1] + b_cross[3]) if b_cross is not None else None

    ring_hit = None
    if fsm_state == "TURN_TO_INNER" and time.ticks_diff(now_ms, t_stage_start) >= TO_INNER_MIN_MS:
        ring_hit = detect_b_ring(img, None, b_ring, wide_search=True)
    elif fsm_state == "CROSS_TO_INNER" and cross_steps > 0 and time.ticks_diff(now_ms, t_pulse_done) >= 0:
        ring_hit = detect_b_ring(img, None, b_ring, wide_search=(b_ring is None))
    elif (fsm_state == "ALIGN_B_RING" and time.ticks_diff(now_ms, t_pulse_done) >= 0) or (stop_reason == "B AIM READY"):
        wide_s = (fsm_state == "ALIGN_B_RING" and b_ring_misses >= B_RING_LOST_FRAMES and
                  b_ring is not None and b_ring[1] < B_RING_BACK_Y)
        ring_hit = detect_b_ring(img, None, b_ring, wide_s)

    if TEST_STAGE == "B_RING_TEST" and fsm_state in ("WAIT_KEY1", "WAIT_ARM_WAKE"):
        b_preview_ring = detect_b_ring(img, None, b_preview_ring)
    else:
        b_preview_ring = None
    display_ring = ring_hit if ring_hit is not None else b_preview_ring

    if ring_hit is not None:
        b_ring = ring_hit
        b_ring_streak += 1
    else:
        b_ring_streak = 0
        if fsm_state in ("TURN_TO_INNER", "CROSS_TO_INNER"):
            b_ring = None
    if fsm_state == "ALIGN_B_RING":
        if time.ticks_diff(now_ms, t_pulse_done) < 0:
            b_ring_streak = 0
            b_ring_misses = 0
        else:
            b_ring_misses = (b_ring_misses + 1) if ring_hit is None else 0

    # Stage 4 & 5 detectors
    outer_rail_y, saw_dual_rails = (
        detect_outer_rail_ahead(img) if fsm_state == "CROSS_TO_OUTER" else (None, False)
    )
    home_branch = (
        detect_home_branch(img, pts, HOME_APPROACH_SIDE)
        if fsm_state in ("FOLLOW_OUTER_TO_HOME", "HOME_CENTER_GATE") else None
    )
    if home_branch is not None:
        by = home_branch[2]
        if last_branch_y is not None and abs(by - last_branch_y) <= 18:
            home_branch_streak += 1
        else:
            home_branch_streak = 1
        last_branch_y = by
        home_wall_clear_streak = 0
    else:
        home_branch_streak = 0
        last_branch_y = None
        if home_wall_count >= 1:
            home_wall_clear_streak += 1
            if home_wall_clear_streak >= HOME_WALL_CLEAR_FRAMES:
                home_wall_saw_gap = True

    need_wall = (
        fsm_state in ("HOME_TURN_INTO_BOX", "HOME_VERIFY_ALIGN", "HOME_PARK_IN_BOX") or
        (fsm_state == "LATCH_STOP" and stop_reason in ("DONE HOME", "HOME CHECK", "HOME ALIGN", "NO HOME WALL"))
    )
    pulse_settling = (fsm_state == "HOME_VERIFY_ALIGN" and time.ticks_diff(now_ms, t_pulse_done) < 0)
    home_wall = detect_home_back_wall(img) if (need_wall and not pulse_settling) else None
    if home_wall is not None:
        last_back_y = home_wall[0]
        last_back_tilt = home_wall[1]

    # --- 2. DEBUG OVERLAYS (strictly AFTER all vision detectors) ---
    if fsm_state not in ("ALIGN_B_RING",) and TEST_STAGE != "B_RING_TEST":
        for cy_q, _ in SCAN_BANDS:
            img.draw_line(0, cy_q * 2, IMG_W - 1, cy_q * 2, color=(70, 70, 70))
        for cx_pt, cy_q, _ in pts:
            img.draw_cross(int(cx_pt * 2), int(cy_q * 2), color=(0, 255, 0), size=5)
    for box in s1_crosses:
        img.draw_rectangle(box, color=(120, 120, 255))
    if e3_box is not None:
        img.draw_rectangle(e3_box, color=(255, 255, 0))
    if e4_box is not None:
        img.draw_rectangle(e4_box, color=(255, 0, 255))
    if b_cross is not None:
        img.draw_rectangle(b_cross, color=(255, 0, 255))
    if display_ring is not None:
        img.draw_cross(display_ring[0], max(0, min(IMG_H - 1, display_ring[1])), color=(255, 220, 0), size=6)
    if cyl_hit is not None:
        cx, _, bot_y, _, _, _, ctag, rect = cyl_hit
        img.draw_rectangle(rect, color=(0, 255, 255))
        img.draw_string(rect[0], max(15, rect[1] - 12), "%s c%d b%d" % (ctag, cx, bot_y), color=(0, 255, 255))
    if outer_rail_y is not None:
        img.draw_line(60, outer_rail_y, 260, outer_rail_y, color=(0, 255, 255))
    if home_branch is not None:
        _, hx, hy, hw, hh, _ = home_branch
        img.draw_rectangle((hx, hy, hw, hh), color=(255, 255, 0))
    if home_wall is not None:
        wy, wtilt, _, wpts = home_wall
        sw = wtilt / 200.0
        x0_w, x1_w = int(round(wpts[0][0])), int(round(wpts[-1][0]))
        y0_w = max(0, min(IMG_H - 1, int(round(wy + sw * (x0_w - 160.0)))))
        y1_w = max(0, min(IMG_H - 1, int(round(wy + sw * (x1_w - 160.0)))))
        img.draw_line(x0_w, y0_w, x1_w, y1_w, color=(0, 255, 255))
        for wx_pt, wy_pt in wpts:
            img.draw_cross(int(round(wx_pt)), int(round(wy_pt)), color=(255, 255, 0), size=4)
    img.draw_cross(TARGET_CX, TARGET_BOT_Y, color=(0, 255, 0), size=7)

    # --- 3. STATE MACHINE ---
    is_armed = (link_ok and seen_disarmed and link.armed) if REQUIRE_KEY1 else True
    vx_cmd = 0
    wz_cmd = 0

    if not is_armed:
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
            if stop_reason is None:
                write_summary(now_ms, "DISARM" if link_ok else "LINK LOST")
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        summary_written = False
        outer_shifts = 0
        saw_exit_e3 = False
        cyl_trig_cnt = 0
        settle_cnt = 0
        pick_idx = 0
        t_cyl_lost_start = None
        last_back_y = None
        last_back_tilt = None
        b_ring = None
        reset_line_state()
        reset_home_wall_state()
        if time.ticks_diff(now_ms, t_last_ping) >= 250:
            send_sys(1, 0)
            t_last_ping = now_ms
    else:
        if stop_reason is not None:
            fsm_state = "LATCH_STOP"
        elif fsm_state == "WAIT_KEY1":
            t_run_start = now_ms
            cover_cnt = 0
            outer_shifts = 0
            saw_exit_e3 = False
            cyl_trig_cnt = 0
            settle_cnt = 0
            pick_idx = 0
            t_cyl_lost_start = None
            last_back_y = None
            last_back_tilt = None
            b_ring = None
            summary_written = False
            reset_line_state(float(LINE_CX) if TEST_STAGE == "HOME_LEFT_TEST" else OUTER_LOCK_INIT_CX)
            reset_home_wall_state()
            enter_stage("WAIT_ARM_WAKE", now_ms)
        else:
            if cover_cnt >= COVER_STOP_FRAMES:
                latch_stop("COVER", now_ms, soft_stop=True)
            elif (MAX_RUN_SEC > 0) and (not fsm_state.startswith("ARM_")) and (fsm_state != "LATCH_STOP") and (
                    time.ticks_diff(now_ms, t_run_start) / 1000.0 >= MAX_RUN_SEC):
                latch_stop("TIME", now_ms, soft_stop=True)

        if fsm_state == "WAIT_ARM_WAKE":
            send_vel(0, 0, 0, 0)
            if not link.arm_busy:
                t_run_start = now_ms
                if TEST_STAGE == "B_APPROACH_TEST":
                    enter_stage("TURN_TO_INNER", now_ms)
                elif TEST_STAGE == "B_RING_TEST":
                    enter_stage("ALIGN_B_RING", now_ms)
                elif TEST_STAGE == "FROM_B_OUTER_LOOP":
                    enter_stage("PAD_REVERSE", now_ms)
                elif TEST_STAGE == "HOME_LEFT_TEST":
                    reset_line_state(float(LINE_CX))
                    enter_stage("FOLLOW_OUTER_TO_HOME", now_ms)
                else:
                    enter_stage("HOME_PRE_TURN", now_ms)

        # ---- STAGE 1: HOME EXIT -> OUTER_WEST (E3 -> E4 closed-loop) ----
        elif fsm_state == "HOME_PRE_TURN":
            if e3_box is not None:
                saw_exit_e3 = True
            if e4_box is not None or e3_box is not None:
                send_vel(0, 0, 0, 0)
                print("S1 pre_turn early e3=%s e4=%s" % (e3_box is not None, e4_box is not None))
                enter_stage("HOME_FIND_CROSS", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= HOME_PRE_TURN_MS:
                enter_stage("LEAVE_HOME", now_ms)
            else:
                wz_cmd = HOME_PRE_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "LEAVE_HOME":
            if e3_box is not None:
                saw_exit_e3 = True
            if e4_box is not None or e3_box is not None:
                send_vel(0, 0, 0, 0)
                print("S1 leave early e3=%s e4=%s" % (e3_box is not None, e4_box is not None))
                enter_stage("HOME_FIND_CROSS", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= HOME_EXIT_MS:
                send_vel(0, 0, 0, 0)
                enter_stage("HOME_FIND_CROSS", now_ms)
            else:
                vx_cmd = HOME_EXIT_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "HOME_FIND_CROSS":
            el = time.ticks_diff(now_ms, t_stage_start)
            if e3_box is not None:
                saw_exit_e3 = True
            # Lock on Fig 4 (E4: leftmost outer rail + roadway '+' on right),
            # OR fallback lock if Fig 3 (E3) already confirmed and outer rail is now cleanly centered
            rail_centered_after_e3 = (
                saw_exit_e3 and len(pts) >= 2 and line_valid == 1 and
                abs(err_out) <= 280 and abs(ang_out) <= 50
            )
            if e4_box is not None or rail_centered_after_e3:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= HOME_E4_CONFIRM:
                    print("S1 locked e4=%s e3_fb=%d e=%d ang=%d el=%d" % (
                        e4_box, int(rail_centered_after_e3), err_out, ang_out, el))
                    if TEST_STAGE == "EXIT_TEST":
                        latch_stop("EXIT OK", now_ms)
                    else:
                        reset_line_state(OUTER_LOCK_INIT_CX)
                        enter_stage("FOLLOW_OUTER_PICK", now_ms)
            elif el >= HOME_CROSS_TURN_MAX_MS:
                latch_stop("NO HOME X", now_ms)
            else:
                turn_lock_streak = 0
                # Overshoot self-rescue: if E3 already passed and outer rail slid far right, steer CW right back!
                if saw_exit_e3 and ((line_valid >= 1 and err_out > 320) or (line_valid == 0 and last_dir > 0)):
                    wz_cmd = -HOME_CROSS_SLOW_WZ
                elif saw_exit_e3 or e3_box is not None:
                    wz_cmd = HOME_CROSS_SLOW_WZ
                else:
                    wz_cmd = HOME_CROSS_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

        # ---- STAGE 2: OUTER RAIL TRACK -> ALIGN_PICK -> 7-STEP PICK ----
        elif fsm_state == "FOLLOW_OUTER_PICK":
            track_ms = time.ticks_diff(now_ms, t_stage_start)
            if (track_ms >= MIN_TRACK_BEFORE_PICK_MS and cyl_hit is not None and
                    cyl_hit[2] >= TRIG_BOT_Y and cyl_hit[3] >= TRIG_MIN_W and cyl_hit[4] >= TRIG_MIN_H):
                cyl_trig_cnt += 1
            else:
                cyl_trig_cnt = 0

            if cyl_trig_cnt >= TRIG_FRAMES:
                send_vel(0, 0, 0, 0)
                t_pause_until = time.ticks_add(now_ms, CHASSIS_SETTLE_MS)
                fsm_state = "BRAKE_PICK"
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    latch_stop("LOST OUT", now_ms)
                elif last_dir != 0 and lost_ms <= SEARCH_MS:
                    wz_cmd = SEARCH_WZ if last_dir < 0 else -SEARCH_WZ
                    send_vel(0, 0, wz_cmd, 0)
                else:
                    send_vel(0, 0, 0, 0)
            else:
                vx_cmd, wz_cmd, prev_e_comb = follow_line_pd(
                    line_valid, err_out, ang_out, has_prev_err, prev_e_comb
                )
                has_prev_err = True
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "BRAKE_PICK":
            send_vel(0, 0, 0, 0)
            if time.ticks_diff(now_ms, t_pause_until) >= 0:
                t_cyl_lost_start = None
                settle_cnt = 0
                t_pulse_done = 0
                fsm_state = "ALIGN_PICK"

        elif fsm_state == "ALIGN_PICK":
            if cyl_hit is None:
                send_vel(0, 0, 0, 0)
                settle_cnt = 0
                if t_cyl_lost_start is None:
                    t_cyl_lost_start = now_ms
                elif time.ticks_diff(now_ms, t_cyl_lost_start) >= LOST_CYL_TIMEOUT_MS:
                    latch_stop("NO CYL", now_ms)
            else:
                t_cyl_lost_start = None
                cx, top_y, bot_y, _, _, _, _, _ = cyl_hit
                ready, cmd_t, vx, wz, p_ms, _, _ = compute_align_cmd(cx, top_y, bot_y)
                if ready:
                    send_vel(0, 0, 0, 0)
                    if time.ticks_diff(now_ms, t_pulse_done) >= CHASSIS_SETTLE_MS:
                        settle_cnt += 1
                    if settle_cnt >= ALIGN_SETTLE_FRAMES:
                        arm_phase = "PICK"
                        active_steps = PICK_STEPS
                        pick_idx = 0
                        fsm_state = "ARM_SEND"
                else:
                    settle_cnt = 0
                    if cmd_t == "CRUISE":
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, 0)
                    elif time.ticks_diff(now_ms, t_pulse_done) >= PULSE_COOLDOWN_MS:
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, p_ms)
                        t_pulse_done = time.ticks_add(now_ms, p_ms)

        # ---- STAGE 3: DUAL-LANDMARK TURN_TO_INNER -> ALIGN_B_RING -> PUT ----
        elif fsm_state == "TURN_TO_INNER":
            el = time.ticks_diff(now_ms, t_stage_start)
            direct_ring_ok = (
                el >= TO_INNER_MIN_MS and ring_hit is not None and
                TO_INNER_RING_MIN_X <= ring_hit[0] <= TO_INNER_RING_MAX_X
            )
            cross_ok = (
                el >= TO_INNER_MIN_MS and b_cross is not None and
                TO_INNER_CROSS_MIN_X <= b_cross_cx <= TO_INNER_CROSS_MAX_X
            )
            if direct_ring_ok:
                send_vel(0, 0, 0, 0)
                ring_lock_streak += 1
                turn_lock_streak = 0
                if ring_lock_streak >= TO_INNER_CONFIRM:
                    print("S3 direct_ring=(%d,%d) el=%d" % (ring_hit[0], ring_hit[1], el))
                    saved_ring = ring_hit
                    enter_stage("ALIGN_B_RING", now_ms)
                    b_ring = saved_ring
            elif cross_ok:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                ring_lock_streak = 0
                if turn_lock_streak >= TO_INNER_CONFIRM:
                    print("S3 b_cross=(%d,%d) el=%d" % (b_cross_cx, b_cross_bot, el))
                    reset_line_state()
                    enter_stage("CROSS_TO_INNER", now_ms)
            elif el >= TO_INNER_MAX_MS:
                latch_stop("NO B TARGET", now_ms)
            else:
                turn_lock_streak = 0
                ring_lock_streak = 0
                if el >= TO_INNER_MIN_MS and b_cross is not None:
                    wz_cmd = TURN_SLOW_WZ if b_cross_cx < TO_INNER_CROSS_MIN_X else -TURN_SLOW_WZ
                else:
                    wz_cmd = TO_INNER_WZ_SIGN * TURN_SEARCH_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "CROSS_TO_INNER":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= CROSS_ROAD_MAX_MS:
                latch_stop("NO B RING", now_ms)
            elif ring_hit is not None:
                send_vel(0, 0, 0, 0)
                if b_ring_streak >= B_RING_LOCK_FRAMES:
                    print("S3 cross_ring=(%d,%d) el=%d" % (ring_hit[0], ring_hit[1], el))
                    saved_ring = ring_hit
                    enter_stage("ALIGN_B_RING", now_ms)
                    b_ring = saved_ring
            elif time.ticks_diff(now_ms, t_pulse_done) >= 0:
                if cross_steps >= CROSS_STEP_LIMIT:
                    latch_stop("NO B RING", now_ms)
                else:
                    cross_steps += 1
                    vx_cmd = DRIVE_CAL_VX
                    send_vel(vx_cmd, 0, 0, CROSS_STEP_MS)
                    t_pulse_done = time.ticks_add(time.ticks_ms(), CROSS_STEP_MS + B_RING_SETTLE_MS)

        elif fsm_state == "ALIGN_B_RING":
            if time.ticks_diff(now_ms, t_stage_start) >= B_RING_ALIGN_MS:
                latch_stop("B ALIGN TO" if b_ring is not None else "NO B RING", now_ms)
            elif time.ticks_diff(now_ms, t_pulse_done) >= 0:
                b_back_active = False
                lost_close = (b_ring_misses >= B_RING_LOST_FRAMES and b_ring is not None and
                              b_ring[1] >= B_RING_BACK_Y)
                overshot = (b_ring_streak >= B_RING_LOCK_FRAMES and ring_hit is not None and
                            ring_hit[1] > TARGET_BOT_Y + B_RING_TOL_Y)
                if lost_close or overshot:
                    b_ready_streak = 0
                    t_b_aligned = None
                    if b_back_steps >= B_RING_BACK_LIMIT:
                        latch_stop("B BACK LIMIT", now_ms)
                    else:
                        b_back_steps += 1
                        b_back_active = True
                        vx_cmd = PULSE_VX_REV
                        send_vel(vx_cmd, 0, 0, B_RING_BACK_MS)
                        t_pulse_done = time.ticks_add(time.ticks_ms(), B_RING_BACK_MS + B_RING_BACK_SETTLE_MS)
                        b_ring_streak = 0
                        b_ring_misses = 0
                elif b_ring_streak < B_RING_LOCK_FRAMES or ring_hit is None:
                    b_ready_streak = 0
                    t_b_aligned = None
                    send_vel(0, 0, 0, 0)
                else:
                    ring_x, ring_y, _ = ring_hit
                    error_x = ring_x - TARGET_CX
                    error_y = TARGET_BOT_Y - ring_y
                    if abs(error_x) <= TOL_X and abs(error_y) <= B_RING_TOL_Y:
                        send_vel(0, 0, 0, 0)
                        b_ready_streak += 1
                        if t_b_aligned is None:
                            t_b_aligned = now_ms
                        if (b_ready_streak >= B_RING_READY_FRAMES and
                                time.ticks_diff(now_ms, t_b_aligned) >= CHASSIS_SETTLE_MS and
                                not link.arm_busy):
                            if TEST_STAGE in ("B_APPROACH_TEST", "B_RING_TEST"):
                                latch_stop("B AIM READY", now_ms)
                            elif B_PUT_EXTRA_CM > 0.0:
                                enter_stage("B_PUT_NUDGE", now_ms)
                            else:
                                arm_phase = "PUT"
                                active_steps = PLACE_STEPS
                                pick_idx = 0
                                fsm_state = "ARM_SEND"
                    else:
                        b_ready_streak = 0
                        t_b_aligned = None
                        approach_tol_x = B_RING_APPROACH_TOL_X if ring_y < B_RING_FINE_Y else TOL_X
                        if abs(error_x) > approach_tol_x:
                            wz_cmd = -PULSE_WZ_MAG if error_x > 0 else PULSE_WZ_MAG
                            pulse_ms = B_RING_TURN_MS
                        else:
                            vx_cmd = PULSE_VX_FWD
                            if ring_y < B_RING_FAR_Y:
                                pulse_ms = B_RING_FAR_STEP_MS
                            elif ring_y < B_RING_FINE_Y:
                                pulse_ms = B_RING_MID_STEP_MS
                            else:
                                pulse_ms = 145 if error_y >= 16 else 105
                        send_vel(vx_cmd, 0, wz_cmd, pulse_ms)
                        t_pulse_done = time.ticks_add(time.ticks_ms(), pulse_ms + B_RING_SETTLE_MS)
                        b_ring_streak = 0
                        b_ring_misses = 0

        elif fsm_state == "B_PUT_NUDGE":
            el = time.ticks_diff(now_ms, t_stage_start)
            nudge_ms = cm_to_ms(B_PUT_EXTRA_CM)
            if el >= nudge_ms + CHASSIS_SETTLE_MS:
                send_vel(0, 0, 0, 0)
                arm_phase = "PUT"
                active_steps = PLACE_STEPS
                pick_idx = 0
                fsm_state = "ARM_SEND"
            elif el >= nudge_ms:
                send_vel(0, 0, 0, 0)
            else:
                vx_cmd = DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        # ---- STAGE 4: LEAVE (b) -> NORTH STRAIGHT OUTER RAIL -> CW LAP ----
        elif fsm_state == "PAD_REVERSE":
            if time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(PAD_REVERSE_CM):
                send_vel(0, 0, 0, 0)
                if (not RETURN_TO_HOME) or TEST_STAGE == "B_PICK_PLACE":
                    latch_stop("PLACED", now_ms)
                else:
                    enter_stage("TURN_TO_TOP_RAIL", now_ms)
            else:
                vx_cmd = -DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "TURN_TO_TOP_RAIL":
            if time.ticks_diff(now_ms, t_stage_start) >= NW_TURN_MS:
                send_vel(0, 0, 0, 0)
                reset_line_state()
                enter_stage("CROSS_TO_OUTER", now_ms)
            else:
                wz_cmd = NW_TURN_WZ_SIGN * TURN_SEARCH_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "CROSS_TO_OUTER":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= OUTER_CROSS_MAX_MS:
                latch_stop("NO OUT RAIL", now_ms)
            elif outer_rail_y is not None and outer_rail_y >= OUTER_CROSS_RAIL_Y and (not saw_dual_rails):
                send_vel(0, 0, 0, 0)
                if last_outer_rail_y is not None and abs(outer_rail_y - last_outer_rail_y) <= 14:
                    turn_lock_streak += 1
                else:
                    turn_lock_streak = 1
                last_outer_rail_y = outer_rail_y
                if turn_lock_streak >= 2:
                    print("S4 north_outer_y=%d el=%d" % (outer_rail_y, el))
                    reset_line_state(OUTER_LOCK_INIT_CX)
                    enter_stage("JOIN_OUTER_CW", now_ms)
            else:
                turn_lock_streak = 0
                last_outer_rail_y = None
                vx_cmd = DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "JOIN_OUTER_CW":
            el = time.ticks_diff(now_ms, t_stage_start)
            locked_now = (
                el >= JOIN_CW_MIN_MS and len(pts) >= 2 and line_valid == 1 and
                abs(err_out) <= JOIN_CW_MAX_ERR and abs(ang_out) <= JOIN_CW_MAX_ANG
            )
            if locked_now:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= JOIN_CW_LOCK_FRAMES:
                    print("S4 joined_cw e=%d ang=%d el=%d" % (err_out, ang_out, el))
                    has_prev_err = False
                    reset_home_wall_state()
                    enter_stage("FOLLOW_OUTER_LOOP", now_ms)
            elif el >= JOIN_CW_MAX_MS:
                latch_stop("JOIN CW TO", now_ms)
            else:
                turn_lock_streak = 0
                wz_cmd = JOIN_CW_WZ_SIGN * TURN_SEARCH_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "FOLLOW_OUTER_LOOP":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= OUTER_LOOP_ARM_MS:
                reset_home_wall_state()
                enter_stage("FOLLOW_OUTER_TO_HOME", now_ms)
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    latch_stop("LOST LOOP", now_ms)
                elif last_dir != 0 and lost_ms <= SEARCH_MS:
                    wz_cmd = SEARCH_WZ if last_dir < 0 else -SEARCH_WZ
                    send_vel(0, 0, wz_cmd, 0)
                else:
                    send_vel(0, 0, 0, 0)
            else:
                vx_cmd, wz_cmd, prev_e_comb = follow_line_pd(
                    line_valid, err_out, ang_out, has_prev_err, prev_e_comb
                )
                has_prev_err = True
                send_vel(vx_cmd, 0, wz_cmd, 0)

        # ---- STAGE 5: EAST->WEST 2-WALL GATE -> ANTI-GLARE BACK-WALL HOME DOCK ----
        elif fsm_state == "FOLLOW_OUTER_TO_HOME":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= HOME_SEARCH_MAX_MS:
                latch_stop("NO HOME", now_ms)
            elif el >= HOME_BRANCH_MIN_MS and home_branch_streak >= HOME_BRANCH_CONFIRM and home_branch is not None:
                home_wall_count = 1
                t_home_wall1 = now_ms
                home_wall_clear_streak = 0
                home_wall_saw_gap = False
                gate_advance_cm = HOME_WALL1_FALLBACK_CM
                gate_src = "wall1_fallback"
                print("S5 wall1 y=%d span=%d" % (home_branch[2] + 2, home_branch[5]))
                enter_stage("HOME_CENTER_GATE", now_ms)
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    latch_stop("LOST HOME", now_ms)
                elif last_dir != 0 and lost_ms <= SEARCH_MS:
                    wz_cmd = SEARCH_WZ if last_dir < 0 else -SEARCH_WZ
                    send_vel(0, 0, wz_cmd, 0)
                else:
                    send_vel(0, 0, 0, 0)
            else:
                vx_cmd, wz_cmd, prev_e_comb = follow_line_pd(
                    line_valid, err_out, ang_out, has_prev_err, prev_e_comb
                )
                has_prev_err = True
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "HOME_CENTER_GATE":
            since_w1 = time.ticks_diff(now_ms, t_home_wall1)
            if (home_wall_count == 1 and home_wall_saw_gap and
                    HOME_WALL_MIN_GAP_MS <= since_w1 <= HOME_WALL_MAX_GAP_MS and
                    home_branch_streak >= HOME_BRANCH_CONFIRM and home_branch is not None):
                home_wall_count = 2
                home_wall_saw_gap = False
                gate_advance_cm = HOME_WALL2_ADVANCE_CM
                gate_src = "wall2"
                t_stage_start = now_ms
                print("S5 wall2 y=%d span=%d gap_ms=%d" % (home_branch[2] + 2, home_branch[5], since_w1))

            if line_valid == 0:
                send_vel(0, 0, 0, 0)
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms >= HOME_LOST_MS:
                    latch_stop("HOME CENTER", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(gate_advance_cm):
                send_vel(0, 0, 0, 0)
                print("S5 gate_ready src=%s walls=%d" % (gate_src, home_wall_count))
                enter_stage("HOME_TURN_INTO_BOX", now_ms)
            else:
                vx_cmd = DRIVE_CAL_VX
                _, wz_pd, prev_e_comb = follow_line_pd(
                    line_valid, err_out, ang_out, has_prev_err, prev_e_comb
                )
                has_prev_err = True
                wz_cmd = int(max(-220, min(220, wz_pd)))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "HOME_TURN_INTO_BOX":
            el = time.ticks_diff(now_ms, t_stage_start)
            signed_tilt = (home_wall[1] * HOME_TURN_WZ_SIGN) if home_wall is not None else None
            wall_level = (
                el >= HOME_TURN_MIN_MS and signed_tilt is not None and
                (-HOME_TURN_TILT_TOL <= signed_tilt <= 40.0)
            )
            if wall_level:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= 2:
                    print("S5 turn_locked y=%.0f tilt=%.1f n=%d el=%d" % (
                        home_wall[0], home_wall[1], home_wall[2], el))
                    enter_stage("HOME_VERIFY_ALIGN", now_ms)
            elif el >= HOME_TURN_MAX_MS:
                send_vel(0, 0, 0, 0)
                enter_stage("HOME_VERIFY_ALIGN", now_ms)
            else:
                turn_lock_streak = 0
                wz_mag = TURN_SLOW_WZ if (el >= HOME_TURN_MIN_MS and home_wall is not None) else TURN_SEARCH_WZ
                wz_cmd = HOME_TURN_WZ_SIGN * wz_mag
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "HOME_VERIFY_ALIGN":
            if time.ticks_diff(now_ms, t_stage_start) >= HOME_ALIGN_WAIT_MS:
                latch_stop("HOME ALIGN" if home_wall is not None else "NO HOME WALL", now_ms)
            elif not pulse_settling:
                if home_wall is None:
                    home_wall_streak = 0
                    send_vel(0, 0, 0, 0)
                else:
                    wy, wtilt, wn, _ = home_wall
                    if abs(wtilt) <= HOME_ALIGN_TOL_TILT:
                        send_vel(0, 0, 0, 0)
                        home_wall_streak += 1
                        if home_wall_streak >= HOME_WALL_CONFIRM_FRAMES:
                            print("S5 align_ok y=%.0f tilt=%.1f n=%d" % (wy, wtilt, wn))
                            enter_stage("HOME_PARK_IN_BOX", now_ms)
                    else:
                        home_wall_streak = 0
                        wz_cmd, p_ms = wall_yaw_pulse(wtilt)
                        align_pulses += 1
                        send_vel(0, 0, wz_cmd, p_ms)
                        t_pulse_done = time.ticks_add(time.ticks_ms(), p_ms + HOME_PULSE_SETTLE_MS)

        elif fsm_state == "HOME_PARK_IN_BOX":
            el = time.ticks_diff(now_ms, t_stage_start)
            if HOME_PARK_STOP_Y is not None and home_wall is not None and home_wall[0] >= HOME_PARK_STOP_Y:
                send_vel(0, 0, 0, 0)
                home_wall_streak += 1
                if home_wall_streak >= HOME_WALL_CONFIRM_FRAMES:
                    latch_stop("DONE HOME", now_ms)
            else:
                home_wall_streak = 0
                if (not seen_park_anchor) and (home_wall is not None and home_wall[0] >= HOME_ANCHOR_Y):
                    seen_park_anchor = True
                    t_park_anchor = now_ms
                    print("S5 park_anchor y=%.1f el=%d" % (home_wall[0], el))
                anchor_done = (
                    seen_park_anchor and t_park_anchor is not None and
                    time.ticks_diff(now_ms, t_park_anchor) >= cm_to_ms(HOME_POST_ANCHOR_CM)
                )
                if anchor_done or el >= cm_to_ms(HOME_PARK_MAX_CM):
                    latch_stop("DONE HOME", now_ms)
                else:
                    vx_cmd = DRIVE_CAL_VX
                    wz_cmd = int(max(-160, min(160, round(-4.5 * home_wall[1])))) if home_wall is not None else 0
                    send_vel(vx_cmd, 0, wz_cmd, 0)

        # ---- FROZEN STM32 7-STEP ARM HANDSHAKE ----
        elif fsm_state == "ARM_SEND":
            send_vel(0, 0, 0, 0)
            if pick_idx >= len(active_steps):
                if arm_phase == "PICK":
                    enter_stage("TURN_TO_INNER", now_ms)
                else:
                    enter_stage("PAD_REVERSE", now_ms)
            else:
                _, j0, j1, j2, j3, j4, j5, dur_ms, _ = active_steps[pick_idx]
                pick_sent_seq = link.arm_seq
                t_step_start = now_ms
                send_joints(j0, j1, j2, j3, j4, j5, dur_ms)
                fsm_state = "ARM_WAIT"

        elif fsm_state == "ARM_WAIT":
            _, _, _, _, _, _, _, dur_ms, dwell_ms = active_steps[pick_idx]
            elapsed_ms = time.ticks_diff(now_ms, t_step_start)
            if link.arm_seq != pick_sent_seq and not link.arm_busy and elapsed_ms >= int(dur_ms * 0.7):
                t_step_wait_until = time.ticks_add(now_ms, dwell_ms)
                fsm_state = "ARM_DWELL"
            elif elapsed_ms >= max(dur_ms + 1200, 2200):
                latch_stop("ARM TO", now_ms, soft_stop=True)

        elif fsm_state == "ARM_DWELL":
            if time.ticks_diff(now_ms, t_step_wait_until) >= 0:
                pick_idx += 1
                fsm_state = "ARM_SEND"

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    # --- 4. HUD & TELEMETRY ---
    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s W=%d %s" % (stop_reason, home_wall_count, RUN_REV), color=(255, 255, 255))
        if stop_reason in ("DONE HOME", "PLACED", "EXIT OK", "B AIM READY"):
            set_led(0, 1, 0)
        else:
            set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        msg = "NO STM32" if not link_ok else ("DISARM KEY1" if link.armed else "READY KEY1 e=%d" % err_out)
        if e4_box is not None:
            msg += " E4!"
        elif e3_box is not None:
            msg += " E3"
        img.draw_string(2, 2, "%s %s" % (msg, RUN_REV), color=(255, 255, 255))
        set_led(1, (now_ms // 300) % 2, 0) if not link_ok else set_led(0, 0, (now_ms // 250) % 2)
    elif fsm_state.startswith("ARM_"):
        img.draw_string(2, 2, "%s %d/7 %s" % (arm_phase, pick_idx + 1, active_steps[min(pick_idx, 6)][0]),
                        color=(255, 255, 0))
        set_led(0, 1, 1)
    elif fsm_state == "ALIGN_B_RING":
        msg = ("B BACK %d/%d" % (b_back_steps, B_RING_BACK_LIMIT)) if b_back_active else (
            "B SEARCH" if ring_hit is None else "B RING x=%d y=%d" % ring_hit[:2])
        img.draw_string(2, 2, msg, color=(255, 255, 255))
        set_led(0, 1, 0)
    else:
        tag = ("T%d" % int(round(home_wall[1]))) if home_wall is not None else ("e%d" % err_out)
        img.draw_string(2, 2, "%s W=%d %s v%d" % (fsm_state[:10], home_wall_count, tag, vx_cmd),
                        color=(255, 255, 255))
        set_led(0, 1, 0)

    log_key = (fsm_state, stop_reason, home_wall_count, e4_box is not None, display_ring is not None, home_wall is not None)
    if log_key != last_log_key or time.ticks_diff(now_ms, t_last_log) >= 500:
        wall_s = (int(round(home_wall[0])), round(home_wall[1], 1), home_wall[2]) if home_wall is not None else None
        print("C09 %s stop=%s arm=%d e=%d ang=%d v=%d wz=%d e3=%d e4=%d ring=%s W=%d wall=%s" % (
            fsm_state, stop_reason, is_armed, err_out, ang_out, vx_cmd, wz_cmd,
            int(e3_box is not None), int(e4_box is not None),
            display_ring[:2] if display_ring is not None else None, home_wall_count, wall_s
        ))
        last_log_key = log_key
        t_last_log = now_ms
