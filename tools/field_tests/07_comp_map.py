# Topological-Map Competition Mission for OpenMV Cam H7 Plus + frozen STM32.
# One-cylinder route using painted landmarks:
#   HOME -> spin left 1000ms -> diagonal exit 1000ms -> outer oval line follow
#   -> detect ANY cylinder (R/B/G/K + optional W) -> align -> @J 7-step PICK
#   -> cross white road to inner oval -> follow inner bottom rail to the (b) cross
#   -> use the cross to find the (b) rings -> align the ring center to the pickup reticle
#   -> reverse the pickup trajectory to set the cylinder down -> @J release
#   -> reverse & rejoin outer rail -> visually detect HOME side-wall T-junction
#      (per full-loop OpenMV view QQ20261125-022327.mp4) -> advance to gate center -> park in HOME.
#
# B_APPROACH_TEST starts at the previously recorded outer-rail pickup stop, with the same heading.
# B_RING_TEST starts inside the inner oval, already facing (b). Both tests stop before release.
# B_PICK_PLACE starts inside HOME and stops after the one-block placement at (b).

import math
import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# TOPOLOGICAL MAP & ROUTE PARAMETERS (all venue-tunable constants in one block)
# ===========================================================================

REQUIRE_KEY1            = True
# Stage selector:
#   "FULL_1_BLOCK"     : complete HOME -> Pick -> Put (b) -> visual HOME return
#   "OUTER_PICK_ONLY"  : stop safely right after @J PICK on outer oval
#   "HOME_RETURN_TEST" : place robot on outer rail directly -> test visual HOME T-junction + park
#   "B_APPROACH_TEST"  : start at pickup stop -> inner rail -> align to (b), no arm sequence
#   "B_RING_TEST"     : face (b) inside the inner oval -> test final visual alignment only
#   "B_PICK_PLACE"     : HOME -> Pick -> Put (b), then stop
TEST_STAGE              = "FULL_1_BLOCK"
RUN_REV                 = "07-1126E"
RUN_SUMMARY_FILE        = "map_last_run.txt"

# Route planner: tuple of inner-bottom '+' cross indices to visit (1=(b), 2=(d), 3=(e))
# Reverse order (3, 2, 1) places farthest target first so parked cylinders don't block path.
TARGET_CROSS_ORDER      = (1,)
RETURN_TO_HOME          = True     # True = avoid 0.5x missing-HOME penalty after last put

# --- Edge 0: HOME -> OUTER_WEST (closed-loop '+' lock while moving, battery-voltage immune) ---
HOME_PRE_TURN_MS        = 550      # Short safe CCW pre-turn (~35-42 deg full bat / ~25 deg low bat; never overshoots '+')
HOME_PRE_TURN_WZ        = 420
HOME_EXIT_MS            = 850      # Max diagonal exit out of HOME; cuts short IMMEDIATELY once roadway '+' appears
HOME_EXIT_VX            = 540
HOME_EXIT_WZ            = 0
HOME_CROSS_TURN_WZ      = 380      # Gentle blur-free CCW sweep when '+' not yet visible
HOME_CROSS_SLOW_WZ      = 280      # Bidirectional fine turn toward visible '+' while rotating
HOME_CROSS_TURN_MAX_MS  = 2400     # Generous timeout for low battery too
HOME_CROSS_MIN_BOT_Y    = 145      # Near roadway '+' only (rejects far horizon crosses)
HOME_CROSS_MIN_X        = 75       # Confirm exit once roadway '+' moves into front-center window
HOME_CROSS_MAX_X        = 255
HOME_CROSS_CONFIRM      = 2
OUTER_LOCK_INIT_CX      = 40.0     # Bias first FOLLOW_OUTER_PICK lock to left/outer rail (x_px=80)

# --- Line PD Follower (QVGA sampled at 160x120-equiv grid) ---
VX_CRUISE               = 620
VX_MIN                  = 440
KP                      = 0.42
KD                      = 1.60
K_ANG                   = 2.0
WZ_MAX                  = 440
SEARCH_WZ               = 520
SEARCH_MS               = 1000
LOCK_HOLD_MS            = 600
LATCH_LOST_SEC          = 3.5

MAX_BLACK_GRAY          = 155
MIN_VALLEY_DROP         = 20
MIN_TAPE_W              = 3
MAX_TAPE_W              = 24
WIDE_JUNC_W             = 58
MAX_ANCHOR_JUMP         = 48
SCAN_BANDS              = ((96, 0.45), (86, 0.35), (76, 0.20))

# --- Edge 1: OUTER_WEST cylinder pickup (verified R/B/G/K + optional W) ---
MIN_TRACK_BEFORE_PICK_MS = 500
TRIG_ROI                = (0, 110, 320, 130)
TRIG_BOT_Y              = 168
TRIG_MIN_W              = 20
TRIG_MIN_H              = 24
TRIG_FRAMES             = 2

ENABLE_RED              = True
ENABLE_BLUE             = True
ENABLE_GREEN            = True
ENABLE_BLACK            = True
ENABLE_WHITE            = False    # Enable after confirming zero false triggers on mat

THR_RED                 = (10, 95, 18, 127, 0, 127)
THR_BLUE                = (10, 90, -35, 35, -128, -18)
THR_GREEN               = (10, 85, -128, -14, 5, 127)
THR_BLACK               = (0, 15, -14, 14, -14, 14)
THR_CROSS               = (0, 44, -18, 18, -18, 18)

FLOOR_ROI               = (0, 110, 320, 130)
BLACK_ROI               = (0, 128, 320, 112)
TARGET_CX               = 130      # Tuned from test14 & user grab frame (test13 right-missed at c145)
TARGET_BOT_Y            = 239
TOL_X                   = 10
TOL_Y_NEAR              = 236
TOL_Y_FAR               = 240
MAX_CLIP_TOP_Y          = 192

ALIGN_CRUISE_VX         = 560
ALIGN_KP_WZ             = 3.5
ALIGN_MAX_WZ            = 460
FINE_ENTER_Y            = 212
PULSE_VX_FWD            = 580
PULSE_VX_REV            = -560
PULSE_WZ_MAG            = 560
PULSE_COOLDOWN_MS       = 170
ALIGN_SETTLE_FRAMES     = 3
CHASSIS_SETTLE_MS       = 180
LOST_CYL_TIMEOUT_MS     = 1800

# --- Hardware-measured chassis calibration (from real floor drive tests) ---
#   drive 560 0 0 --time 500  ->  6.2 cm
#   drive 560 0 0 --time 1000 -> 14.2 cm  => steady 16.0 cm/s + 112.5 ms startup lag
#   drive 0 0 640 --time 1300 -> 90.0 deg => ~14.44 ms/deg at wz=640
DRIVE_CAL_VX            = 560
DRIVE_DEADBAND_MS       = 112
SPEED_CM_PER_S_560      = 16.0
TURN_CAL_WZ             = 640
TURN_90_MS_640          = 900      # Recalibrated from test24 full-battery (~105 deg/s); closed-loop MAX_DEG covers low bat


def cm_to_ms(cm):
    if cm <= 0.0:
        return 0
    return int(round(DRIVE_DEADBAND_MS + (float(cm) * 1000.0) / SPEED_CM_PER_S_560))


def deg_to_ms(deg):
    if deg <= 0.0:
        return 0
    return int(round((float(deg) / 90.0) * TURN_90_MS_640))


# --- Edge 2: OUTER_WEST -> INNER_BOT rail (visual line capture + measured angle) ---
# WZ sign convention (verified zmotord.c): +1 = CCW left turn, -1 = CW right turn
TO_INNER_WZ_SIGN        = -1       # -1 CW turn inward from outer oval toward (b) '+' cross
TO_INNER_TURN_WZ        = 460      # Gentle CW spin for blur-free '+' detection while rotating
TO_INNER_SLOW_WZ        = 300      # Bidirectional fine turn once near (b) '+' enters view
TO_INNER_MIN_DEG        = 35.0     # Ignore outer oval briefly, then start locking '+' while turning
TO_INNER_MAX_DEG        = 220.0    # Hard fail-stop if (b) '+' cross not found (generous for low battery)
TO_INNER_CROSS_MIN_BOT_Y = 148     # Reject far horizon crosses
TO_INNER_CROSS_MIN_X    = 135      # When (b) '+' sits in [135..255], (b) rings sit dead-center at x~130..165
TO_INNER_CROSS_MAX_X    = 255
TO_INNER_CROSS_CONFIRM  = 2
CROSS_STEP_MS           = 520      # ~6.5 cm/step at DRIVE_CAL_VX so rings enter y>=165 lock zone fast
CROSS_STEP_LIMIT        = 7
CROSS_ROAD_MAX_MS       = 12000
JOIN_INNER_WZ_SIGN      = -1       # Northbound crossing -> eastbound on the inner bottom straight
JOIN_INNER_MAX_DEG      = 115.0    # Max turn budget to capture inner line (hard fail-stop if missed)
TURN_LOCK_MIN_DEG       = 60.0
EXIT_LOCK_MIN_DEG       = 25.0     # Exiting (b) may already face NE; allow early lock onto inner rail
TURN_LOCK_FRAMES        = 3
TURN_LOCK_MAX_ANGLE     = 26

# --- Edge 3: inner-island '+' -> target (b). Roadway crosses are HOME landmarks. ---
CROSS_CONFIRM_FRAMES    = 2
CROSS_CLEAR_FRAMES      = 3        # Require >=3 cross-free frames before counting the next '+' cross
CROSS_COOLDOWN_MS       = 700
B_SEARCH_MAX_MS         = 5500     # Short search on the inner bottom straight after the known entry
B_MARK_MIN_LEFT_GAP     = 16       # QVGA pixels between cross center and rail at the same row
B_MARK_MAX_JUMP         = 58       # Bound the change in marker center between detections
PAD_TURN_MAX_MS         = 3000     # Time budget for visually centering on the selected cross
B_RING_ACQUIRE_MS       = 3600     # Limit cross-guided travel while looking for the rings
B_CROSS_STOP_Y          = 218      # Stop if the cross is close and the rings are still missing
B_RING_ALIGN_MS         = 28000    # Allow the stop-and-check pulses observed in test10
B_RING_LOCK_FRAMES      = 2
B_RING_READY_FRAMES     = 3
B_RING_TOL_Y            = 4        # Allow fit error when the ring is cut by the bottom of the frame
B_RING_MIN_Y            = 128      # Center must be on the visible floor below the far wall
B_RING_VALLEY_DROP      = 10       # Contrast against the pixels on either side of a thin stroke
B_RING_FAR_Y            = 186      # Largest steps while the ring is still distant
B_RING_FINE_Y           = 212      # Slow down and tighten lateral alignment near the target
B_RING_APPROACH_TOL_X   = 22
B_RING_TURN_MS          = 95
B_RING_FAR_STEP_MS      = 560
B_RING_MID_STEP_MS      = 365
B_RING_SETTLE_MS        = 110
B_RING_BACK_Y           = 220      # A lost lock is recoverable only after seeing a close ring
B_RING_LOST_FRAMES      = 2
B_RING_BACK_MS          = 220
B_RING_BACK_SETTLE_MS   = 110
B_RING_BACK_LIMIT       = 3
B_PUT_EXTRA_CM          = 2.5      # Nudge cylinder center into (b) 50pt ring (test15 landed 2.5cm short)

# --- Edge 4: Target pad (b) -> ~160 deg South -> Outer rail -> Visual HOME T-junction -> Park ---
PAD_REVERSE_CM          = 16.0     # Back clear of placed cylinder before turning South
EXIT_INNER_WZ_SIGN      = -1       # -1 CW spin from North to South
EXIT_OUTER_MAX_DEG      = 125.0
RETURN_OUT_TURN_DEG     = 160.0    # CW turn from North (at b) -> dead South (test15 180 deg overshot ~25 deg SW)
RETURN_CROSS_MAX_MS     = cm_to_ms(30.0)  # Stop before driving through the apron to the mat edge
RETURN_OUT_RAIL_Y       = 210      # Near projected row of the outer oval, including diagonal approaches
RETURN_JOIN_WZ_SIGN     = 1        # Southbound crossing -> eastbound on the outer bottom straight
RETURN_JOIN_LOCK_MIN_DEG = 55.0
RETURN_JOIN_MAX_ERR     = 680      # test16 01:12 outer rail sits right-offset at e=+397 (test6 started at e=+564)
RETURN_JOIN_MAX_ANG     = 55
RETURN_JOIN_LOCK_FRAMES = 2
RETURN_OUTER_INIT_CX    = 120.0    # Bias RETURN_JOIN_OUTER anchor to right-hand outer rail (x_px=240)
RETURN_JOIN_MAX_DEG     = 145.0
HOME_APPROACH_SIDE      = "RIGHT"  # Eastbound on the bottom straight, HOME is on the right
HOME_BRANCH_MIN_MS      = 150
HOME_BRANCH_CONFIRM     = 2        # Consecutive frames seeing T-branch wall touching outer rail
HOME_SEARCH_MAX_MS      = 12000    # Fail-stop timeout if HOME T-junction never spotted on outer rail
HOME_BRANCH_ADVANCE_CM  = 42.0     # test4 at 26 cm turned early; test5 at 53 cm turned late
HOME_TURN_DEG           = 90.0     # Turn 90 deg toward the detected HOME side (+CCW if LEFT, -CW if RIGHT)
HOME_DOCK_CX            = TARGET_CX  # Calibrated near-ground center under the straight-ahead jaws
HOME_PARK_MAX_CM        = 22.0     # Stop after at most this much commanded travel inside the gate
HOME_PARK_STOP_Y        = None     # Set from a QVGA frame of the whole car correctly parked in HOME
HOME_BACK_GUARD_Y       = 202      # Initial end-wall braking row for the uncalibrated test
HOME_BOX_CONFIRM_FRAMES = 3
HOME_BOX_WAIT_MS        = 750
HOME_LOST_MS            = 250

MAX_RUN_SEC             = 120.0
COVER_STOP_L            = 10
COVER_STOP_FRAMES       = 8

# Calibrated poses (0.1 deg). Use the same contact pose in both directions.
ARM_HOME = (450, 918, 900, 990, 1035)
ARM_PRE = (450, 1120, 820, 1060, 1650)
ARM_ABOVE = (450, 1600, 680, 1130, 1650)
ARM_CONTACT = (450, 1799, 792, 1060, 1650)
JAW_OPEN = 900
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

IMG_W                   = 320
IMG_H                   = 240
LINE_W                  = 160
LINE_CX                 = 80

# ===========================================================================
# HARDWARE & FROZEN STM32 BRIDGE
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
# VISION: TRACK LINE, TILT-TOLERANT '+' CROSS & 5-COLOR CYLINDER DETECTORS
# ===========================================================================

def _px_gray(img, x, y):
    r, g, b = img.get_pixel(x, y)
    return (r * 77 + g * 150 + b * 29) >> 8


def find_scanline_tapes(row):
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


def detect_line(img, anchor_cx, had_lock):
    pts = []
    any_wide = False
    anchor = anchor_cx
    for cy_q, weight in SCAN_BANDS:
        y = cy_q * 2
        row = [_px_gray(img, xq * 2, y) for xq in range(LINE_W)]
        tapes, wide = find_scanline_tapes(row)
        if wide:
            any_wide = True
        if not tapes:
            continue
        best = min(tapes, key=lambda t: abs(t[0] - anchor))
        if had_lock and abs(best[0] - anchor) > MAX_ANCHOR_JUMP:
            continue
        cx = best[0]
        anchor = cx
        pts.append((cx, float(cy_q), weight))
    return pts, any_wide


def detect_home_branch(img, pts, side):
    """Look for a dark branch connected to the fitted rail on the route's HOME side."""
    if len(pts) < 2:
        return None
    near_x, near_y, _ = pts[0]
    far_x, far_y, _ = pts[-1]
    slope = (near_x - far_x) / (near_y - far_y)
    direction = -1 if side == "LEFT" else 1
    for y in range(220, 145, -6):
        rail_x = int(round(near_x * 2 + (y - near_y * 2) * slope))
        if not (0 <= rail_x < IMG_W):
            continue
        end_x = rail_x
        gap = 0
        for offset in range(0, 96, 2):
            x = rail_x + direction * offset
            if not (0 <= x < IMG_W):
                break
            if min(_px_gray(img, x, y), _px_gray(img, x, y + 4)) <= 120:
                end_x = x
                gap = 0
            else:
                gap += 1
                if gap > 2:
                    break
        span = abs(end_x - rail_x)
        if span >= 56:
            middle = (rail_x + end_x) // 2
            if min(_px_gray(img, middle, y - 10), _px_gray(img, middle, y + 12)) >= 145:
                return (side, min(rail_x, end_x), y - 2, span + 1, 8, span)
    return None


def dark_span(img, x0, x1, y):
    """Check for a wide dark band with lighter floor beside it in depth."""
    if x1 - x0 < 32:
        return False
    xs = range(x0, x1 + 1, 8)
    dark = sum(min(_px_gray(img, x, y), _px_gray(img, x, y + 4)) <= 120 for x in xs)
    if dark * 5 < 4 * len(xs):
        return False
    center = (x0 + x1) // 2
    return max(_px_gray(img, center, y - 12), _px_gray(img, center, y + 14)) >= 145


def return_rail_column(img, x):
    """Find the nearest stripe with similarly lit mat on both sides."""
    first_y = 145
    column = [_px_gray(img, x, y) for y in range(first_y, IMG_H)]
    end = len(column) - 11
    while end >= 10:
        if column[end] > 112:
            end -= 1
            continue
        start = end
        while start > 0 and column[start - 1] <= 112:
            start -= 1
        if start >= 10 and 4 <= end - start + 1 <= 25:
            ahead = sum(column[start - 10:start - 4]) / 6
            behind = sum(column[end + 5:end + 11]) / 6
            if min(ahead, behind) >= 140 and abs(ahead - behind) <= 38:
                return first_y + (start + end) // 2
        end = start - 1
    return None


def detect_return_rail(img):
    """Confirm one near oval stripe across neighboring columns, including a diagonal."""
    rows = [return_rail_column(img, x) for x in (100, 130, 160, 190, 220, 250, 280)]
    for i in range(len(rows) - 2):
        left, center, right = rows[i:i + 3]
        if (left is not None and center is not None and right is not None and
                abs(left - right) <= 36 and abs(2 * center - left - right) <= 10 and
                max(left, center, right) >= RETURN_OUT_RAIL_Y):
            return max(left, center, right)
    return None


def detect_home_box(img):
    """Return the two side-wall midpoints and the end-wall row facing the HOME box."""
    pairs = []
    center_q = HOME_DOCK_CX / 2
    for y in (234, 222, 208, 194, 180, 166, 152, 138):
        row = [_px_gray(img, x * 2, y) for x in range(LINE_W)]
        tapes, _ = find_scanline_tapes(row)
        left = [t[0] for t in tapes if t[0] < center_q - 6]
        right = [t[0] for t in tapes if t[0] > center_q + 6]
        if left and right:
            lx, rx = max(left) * 2, min(right) * 2
            if rx - lx >= 65:
                pairs.append((y, lx, rx))
    if len(pairs) < 2:
        return None
    near_y, near_l, near_r = pairs[0]
    far_y, far_l, far_r = pairs[-1]
    if near_r - near_l < far_r - far_l - 12:
        return None
    for y in range(far_y - 2, max(131, far_y - 24), -2):
        depth = (y - far_y) / float(near_y - far_y)
        x0 = max(0, int(far_l + depth * (near_l - far_l)) + 8)
        x1 = min(IMG_W - 1, int(far_r + depth * (near_r - far_r)) - 8)
        if dark_span(img, x0, x1, y):
            return ((near_l + near_r) / 2, (far_l + far_r) / 2, y)
    return None


_RING16 = tuple(
    (math.cos(i * math.pi / 8.0), math.sin(i * math.pi / 8.0)) for i in range(16)
)


def _is_cross_ring16(img, x, y, w, h):
    """Rotation- and perspective-invariant '+' topology check using a 16-point ellipse ring.
    A '+' at ANY spin angle (0..90 deg) has a dark center and exactly 4 dark arm runs separated
    by 4 bright white pockets; lines/diagonals/L-corners have 2 runs, T-junctions have 3."""
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
    if hi < 135 or (hi - base) < 22:
        return False
    thr = min(130.0, 0.5 * (hi + base))
    dark = [(v <= thr) for v in vals]
    white_cnt = sum(1 for d in dark if not d)
    if white_cnt < 4:
        return False
    runs = 0
    for i in range(16):
        if dark[i] and (not dark[(i - 1) & 15]):
            runs += 1
    return runs == 4


def detect_cross(img, rail_pts=None, previous_box=None, allow_bottom_clip=False):
    """Find a '+' left of the fitted rail on search, then follow that marker."""
    if rail_pts is not None:
        if len(rail_pts) < 2:
            return None
        near_x, near_y, _ = rail_pts[0]
        far_x, far_y, _ = rail_pts[-1]
        slope = (near_x - far_x) / (near_y - far_y)
    if previous_box is not None:
        px, py, pw, ph = previous_box
        prev_cx, prev_cy = px + pw // 2, py + ph // 2

    roi = (0, 128, IMG_W, IMG_H - 128)
    blobs = img.find_blobs([THR_CROSS], roi=roi, x_stride=2, y_stride=1,
                           pixels_threshold=45, area_threshold=60)
    best_box = None
    best_score = -1000
    for b in blobs:
        x, y, w, h = b.rect()
        if not (16 <= w <= 215 and 8 <= h <= 115 and w <= 7.5 * h and h <= 3.0 * w):
            continue
        if y <= roi[1] or x + w >= IMG_W - 1 or ((not allow_bottom_clip) and y + h >= IMG_H - 1):
            continue
        sol = float(b.pixels()) / float(max(1, w * h))
        if sol < 0.18 or sol > 0.62:
            continue
        cx, cy = x + w // 2, y + h // 2
        if rail_pts is not None:
            rail_x = 2 * near_x + slope * (cy - 2 * near_y)
            if not (cx + B_MARK_MIN_LEFT_GAP <= rail_x < IMG_W):
                continue
        score = y + h
        if previous_box is not None:
            dx, dy = abs(cx - prev_cx), abs(cy - prev_cy)
            if max(dx, dy) > B_MARK_MAX_JUMP:
                continue
            score = -(dx + dy)
        if _is_cross_ring16(img, x, y, w, h) and score > best_score:
            best_box = (x, y, w, h)
            best_score = score
    return best_box


def fit_ring_chords(midpoints, widths_sq, first_y, step):
    """Fit the ellipse center from five evenly spaced horizontal chords."""
    z0, z1, z2, z3, z4 = widths_sq
    # Least squares on rows indexed by t = -2, -1, 0, 1, 2.
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
    """Find a chord with two narrower, regularly spaced chords in the same row."""
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
    """Fit concentric floor arcs from locally dark pairs across multiple rows."""
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
    """Find actual concentric ring arcs beyond the (b) cross."""
    hint_x = TARGET_CX
    max_y = IMG_H + 12
    if cross_box is not None:
        hint_x = cross_box[0] + cross_box[2] // 2
        max_y = cross_box[1] + cross_box[3] // 2 - 10
    return scan_b_ring(img, hint_x, max_y, previous_ring, wide_search)


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
    """Detect upright 3D white cylinder via shaded flank paired with highlight face."""
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
    """Follow a narrow dark strip above the blob using floor contrast, not its LAB threshold."""
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
        wz, ms = yaw_alignment_pulse(ex)
        return False, "PULSE", 0, wz, ms, ex, ey

    if too_close:
        return False, "PULSE", PULSE_VX_REV, 0, 110, ex, ey

    ms = 155 if ey >= 16 else 115
    return False, "PULSE", PULSE_VX_FWD, 0, ms, ex, ey


def yaw_alignment_pulse(error_px):
    wz = -PULSE_WZ_MAG if error_px > 0 else PULSE_WZ_MAG
    ms = 135 if abs(error_px) >= 28 else 95
    return wz, ms


def follow_line_pd(line_valid, err_out, ang_out, has_prev_err, prev_e_comb):
    e_comb = max(-400.0, min(400.0, 0.5 * err_out + 0.5 * ang_out * K_ANG)) if line_valid == 2 else (
        max(-1200.0, min(1200.0, err_out + ang_out * K_ANG)))
    d_err = max(-120.0, min(120.0, e_comb - prev_e_comb)) if has_prev_err else 0.0
    wz_cmd = int(max(-WZ_MAX, min(WZ_MAX, round(-(KP * e_comb + KD * d_err)))))
    vx_cmd = int(round(VX_CRUISE - (min(1000.0, abs(e_comb)) * max(0, VX_CRUISE - VX_MIN)) / 1000.0))
    return vx_cmd, wz_cmd, e_comb


# ===========================================================================
# TOPOLOGICAL MAP STATE MACHINE
# ===========================================================================

link = Stm32Link()
clock = time.clock()
fsm_state = "WAIT_KEY1"
stop_reason = None
seen_disarmed = False

job_idx = 0
t_run_start = 0
t_stage_start = 0
t_line_lost_start = None
t_cyl_lost_start = None
t_pulse_done = 0
t_pause_until = 0
t_step_start = 0
t_step_wait_until = 0
t_last_cross = 0
t_last_ping = 0
t_last_log = 0

pick_idx = 0
arm_phase = "PICK"
active_steps = PICK_STEPS
cyl_trig_cnt = 0
cross_streak = 0
cross_clear_streak = CROSS_CLEAR_FRAMES
cross_saw_gap = True
cross_count = 0
b_mark_box = None
b_ring = None
b_preview_ring = None
b_ring_streak = 0
b_ring_misses = 0
cross_steps = 0
cross_ring_hits = 0
b_forward_ms = 0
b_turn_steps = 0
b_back_steps = 0
b_back_active = False
b_ready_streak = 0
t_b_aligned = None
home_branch_streak = 0
last_home_branch_y = None
last_home_side = HOME_APPROACH_SIDE
home_box_streak = 0
t_home_box_lost = None
turn_lock_streak = 0
last_return_rail_y = None
settle_cnt = 0
cover_cnt = 0
pick_sent_seq = -1
last_log_key = None
run_summary_saved = False

last_cx = float(LINE_CX)
had_lock = False
err_filt = 0.0
has_prev_err = False
prev_e_comb = 0.0
last_dir = 0


def reset_line_state(init_cx=None):
    global last_cx, had_lock, err_filt, has_prev_err, prev_e_comb, last_dir, t_line_lost_start
    last_cx = float(LINE_CX) if init_cx is None else float(init_cx)
    had_lock = False
    err_filt = 0.0
    has_prev_err = False
    prev_e_comb = 0.0
    last_dir = 0
    t_line_lost_start = None


def enter_timed_stage(state_name, now_ms):
    global fsm_state, t_stage_start, turn_lock_streak
    global home_box_streak, t_home_box_lost, last_return_rail_y
    global b_ring, b_ring_streak, b_ring_misses, b_back_steps, b_back_active
    global cross_steps, cross_ring_hits, t_pulse_done
    global b_forward_ms, b_turn_steps
    t_stage_start = now_ms
    fsm_state = state_name
    turn_lock_streak = 0
    home_box_streak = 0
    t_home_box_lost = None
    if state_name == "RETURN_OUT_CROSS":
        last_return_rail_y = None
    if state_name == "CROSS_TO_INNER":
        b_ring = None
        b_ring_streak = 0
        cross_steps = 0
        cross_ring_hits = 0
        t_pulse_done = time.ticks_add(time.ticks_ms(), B_RING_SETTLE_MS)
    if state_name == "ALIGN_B_RING":
        b_ring_misses = 0
        b_back_steps = 0
        b_back_active = False
        b_forward_ms = 0
        b_turn_steps = 0


def fail_stop(reason, stop_arm=False):
    global stop_reason, fsm_state
    send_vel(0, 0, 0, 0)
    if stop_arm:
        send_sys(0, 0)
    stop_reason = reason
    fsm_state = "LATCH_STOP"


def save_run_summary(stage, reason, now_ms):
    line = ("rev=%s mode=%s stage=%s stop=%s elapsed_ms=%d stage_ms=%d "
            "cross_steps=%d cross_hits=%d ring=%s b_forward_ms=%d b_turn_steps=%d" % (
                RUN_REV, TEST_STAGE, stage, reason, time.ticks_diff(now_ms, t_run_start),
                time.ticks_diff(now_ms, t_stage_start), cross_steps, cross_ring_hits,
                b_ring[:2] if b_ring is not None else None, b_forward_ms, b_turn_steps))
    print("MAP LAST " + line)
    try:
        with open(RUN_SUMMARY_FILE, "w") as summary:
            summary.write(line + "\n")
    except OSError as error:
        print("MAP summary write failed", error)


print("MAP %s %s b_reticle=(%d,%d) HOME side=%s gate_cm=%.1f park_row=%s" % (
    RUN_REV, TEST_STAGE, TARGET_CX, TARGET_BOT_Y, HOME_APPROACH_SIDE, HOME_BRANCH_ADVANCE_CM, HOME_PARK_STOP_Y))

while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    frame_stage = fsm_state
    link.poll(now_ms)
    link_ok = link.link_ok(now_ms)

    if not link_ok:
        seen_disarmed = False
    elif not link.armed:
        seen_disarmed = True

    st_roi = img.get_statistics(roi=(40, 80, 240, 120))
    cover_cnt = (cover_cnt + 1) if (st_roi.l_mean() <= COVER_STOP_L) else 0

    pts, any_wide = detect_line(img, last_cx, had_lock)
    if pts:
        t_line_lost_start = None
        w_sum = sum(w for _, _, w in pts)
        cx_avg = sum(cx * w for cx, _, w in pts) / w_sum
        last_cx = pts[0][0] if len(pts) == 1 else cx_avg
        had_lock = True
        err_raw = ((cx_avg - LINE_CX) * 1000.0) / float(LINE_CX)
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

    # Run all vision detectors BEFORE drawing debug overlays on img
    cyl_hit = None
    if fsm_state in ("FOLLOW_OUTER_PICK", "ALIGN_PICK"):
        cyl_hit = detect_cylinder(img, FLOOR_ROI if fsm_state == "ALIGN_PICK" else TRIG_ROI)
    cross_box = None
    if fsm_state in ("WAIT_KEY1", "WAIT_ARM_WAKE", "HOME_PRE_TURN", "LEAVE_HOME", "HOME_FIND_CROSS", "TURN_TO_INNER"):
        cross_box = detect_cross(img, allow_bottom_clip=True)
    elif fsm_state == "FOLLOW_INNER_TARGET":
        cross_box = detect_cross(img, rail_pts=pts, previous_box=b_mark_box)
        b_mark_box = cross_box
    elif b_mark_box is not None and (
            fsm_state in ("TURN_PAD_INWARD", "ACQUIRE_B_RING")):
        cross_box = detect_cross(img, previous_box=b_mark_box)
        if cross_box is not None:
            b_mark_box = cross_box
    cross_cx = cross_box[0] + cross_box[2] // 2 if cross_box is not None else None
    cross_bot = cross_box[1] + cross_box[3] if cross_box is not None else None
    ring_hit = None
    if fsm_state == "ACQUIRE_B_RING" and cross_box is not None:
        ring_hit = detect_b_ring(img, cross_box, b_ring)
    elif (fsm_state == "CROSS_TO_INNER" and cross_steps > 0 and
          time.ticks_diff(now_ms, t_pulse_done) >= 0):
        ring_hit = detect_b_ring(img, None, b_ring, wide_search=b_ring is None)
        if ring_hit is not None:
            cross_ring_hits += 1
    elif ((fsm_state == "ALIGN_B_RING" and time.ticks_diff(now_ms, t_pulse_done) >= 0) or
          stop_reason == "B AIM READY"):
        wide_search = (fsm_state == "ALIGN_B_RING" and b_ring_misses >= B_RING_LOST_FRAMES and
                       b_ring is not None and b_ring[1] < B_RING_BACK_Y)
        ring_hit = detect_b_ring(img, None, b_ring, wide_search)
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
        if fsm_state in ("ACQUIRE_B_RING", "CROSS_TO_INNER"):
            b_ring = None
    if fsm_state == "ALIGN_B_RING":
        if time.ticks_diff(now_ms, t_pulse_done) < 0:
            b_ring_streak = 0
            b_ring_misses = 0
        else:
            b_ring_misses = b_ring_misses + 1 if ring_hit is None else 0
    home_branch = detect_home_branch(img, pts, HOME_APPROACH_SIDE) if (
        fsm_state == "RETURN_FOLLOW_OUTER") else None
    rail_y = detect_return_rail(img) if fsm_state == "RETURN_OUT_CROSS" else None
    show_home_box = (
        fsm_state in ("RETURN_VERIFY_HOME_GATE", "RETURN_PARK_IN_HOME") or
        (fsm_state == "LATCH_STOP" and stop_reason in (
            "HOME CHECK", "DONE HOME", "HOME ALIGN", "HOME DEPTH", "NO HOME BOX", "LOST BOX"))
    )
    home_box = detect_home_box(img) if show_home_box else None
    home_back_y = home_box[2] if home_box is not None else None

    if cross_box is not None:
        cross_clear_streak = 0
        cross_streak += 1
        if (fsm_state == "FOLLOW_INNER_TARGET" and cross_saw_gap and
                cross_streak >= CROSS_CONFIRM_FRAMES and
                time.ticks_diff(now_ms, t_last_cross) >= CROSS_COOLDOWN_MS):
            cross_count += 1
            t_last_cross = now_ms
            cross_saw_gap = False
            cross_clear_streak = 0
    else:
        cross_streak = 0
        cross_clear_streak += 1
        if cross_clear_streak >= CROSS_CLEAR_FRAMES:
            cross_saw_gap = True

    if home_branch is not None:
        branch_y = home_branch[2]
        if last_home_branch_y is not None and abs(branch_y - last_home_branch_y) <= 18:
            home_branch_streak += 1
        else:
            home_branch_streak = 1
        last_home_branch_y = branch_y
    else:
        home_branch_streak = 0
        last_home_branch_y = None

    if fsm_state == "EXIT_INNER_TURN":
        min_lock_deg = EXIT_LOCK_MIN_DEG
        max_lock_err = 240
        max_lock_ang = TURN_LOCK_MAX_ANGLE
    elif fsm_state == "RETURN_JOIN_OUTER":
        min_lock_deg = RETURN_JOIN_LOCK_MIN_DEG
        max_lock_err = RETURN_JOIN_MAX_ERR
        max_lock_ang = RETURN_JOIN_MAX_ANG
    else:
        min_lock_deg = TURN_LOCK_MIN_DEG
        max_lock_err = 240
        max_lock_ang = TURN_LOCK_MAX_ANGLE
    if (fsm_state in ("JOIN_INNER_LINE", "EXIT_INNER_TURN", "RETURN_JOIN_OUTER") and
            time.ticks_diff(now_ms, t_stage_start) >= deg_to_ms(min_lock_deg) and
            len(pts) >= 2 and line_valid == 1 and abs(err_out) <= max_lock_err and
            abs(ang_out) <= max_lock_ang):
        turn_lock_streak += 1
    elif fsm_state not in ("TURN_PAD_INWARD", "HOME_FIND_CROSS", "TURN_TO_INNER", "RETURN_OUT_CROSS"):
        turn_lock_streak = 0

    # Draw debug overlays strictly AFTER vision detection
    if TEST_STAGE != "B_RING_TEST" and fsm_state not in ("ACQUIRE_B_RING", "ALIGN_B_RING"):
        for cy_q, _ in SCAN_BANDS:
            img.draw_line(0, cy_q * 2, IMG_W - 1, cy_q * 2, color=(70, 70, 70))
        for cx_pt, cy_q, _ in pts:
            img.draw_cross(int(cx_pt * 2), int(cy_q * 2), color=(0, 255, 0), size=5)
    if cross_box is not None:
        img.draw_rectangle(cross_box, color=(255, 0, 255))
    if display_ring is not None:
        ring_x, ring_y, _ = display_ring
        img.draw_cross(ring_x, max(0, min(IMG_H - 1, ring_y)), color=(255, 220, 0), size=6)
    if home_branch is not None:
        _, hx, hy, hw, hh, _ = home_branch
        img.draw_rectangle((hx, hy, hw, hh), color=(255, 255, 0))
    if home_back_y is not None:
        img.draw_line(0, home_back_y, IMG_W - 1, home_back_y, color=(255, 255, 0))
    img.draw_rectangle(TARGET_CX - TOL_X, TOL_Y_NEAR, TOL_X * 2, IMG_H - TOL_Y_NEAR, color=(0, 180, 0))
    img.draw_cross(TARGET_CX, TARGET_BOT_Y, color=(0, 255, 0), size=8)
    if cyl_hit is not None:
        cx, _, bot_y, _, _, _, ctag, rect = cyl_hit
        img.draw_rectangle(rect, color=(0, 255, 255))
        img.draw_string(rect[0], max(15, rect[1] - 12), "%s c%d b%d" % (ctag, cx, bot_y), color=(0, 255, 255))

    is_armed = (link_ok and seen_disarmed and link.armed) if REQUIRE_KEY1 else True
    vx_cmd = 0
    wz_cmd = 0

    if not is_armed:
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
            if not run_summary_saved and fsm_state != "LATCH_STOP":
                save_run_summary(frame_stage, "DISARMED" if link_ok else "STM32 LINK LOST", now_ms)
                run_summary_saved = True
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        job_idx = 0
        cyl_trig_cnt = 0
        cross_count = 0
        cross_streak = 0
        cross_clear_streak = CROSS_CLEAR_FRAMES
        cross_saw_gap = True
        b_mark_box = None
        b_ring = None
        b_ring_streak = 0
        b_ready_streak = 0
        t_b_aligned = None
        home_branch_streak = 0
        t_cyl_lost_start = None
        settle_cnt = 0
        pick_idx = 0
        reset_line_state()
        if time.ticks_diff(now_ms, t_last_ping) >= 250:
            send_sys(1, 0)
            t_last_ping = now_ms
    else:
        if stop_reason is not None:
            fsm_state = "LATCH_STOP"
        elif fsm_state == "WAIT_KEY1":
            t_run_start = now_ms
            run_summary_saved = False
            cross_steps = 0
            cross_ring_hits = 0
            b_forward_ms = 0
            b_turn_steps = 0
            job_idx = 0
            cover_cnt = 0
            cyl_trig_cnt = 0
            cross_count = 0
            cross_streak = 0
            cross_clear_streak = CROSS_CLEAR_FRAMES
            cross_saw_gap = True
            b_mark_box = None
            b_ring = None
            b_ring_streak = 0
            b_ready_streak = 0
            t_b_aligned = None
            home_branch_streak = 0
            t_cyl_lost_start = None
            settle_cnt = 0
            pick_idx = 0
            reset_line_state()
            enter_timed_stage("WAIT_ARM_WAKE", now_ms)
        else:
            if cover_cnt >= COVER_STOP_FRAMES:
                stop_reason = "COVER"
            elif (MAX_RUN_SEC > 0) and (not fsm_state.startswith("ARM_")) and (fsm_state != "LATCH_STOP") and (
                    time.ticks_diff(now_ms, t_run_start) / 1000.0 >= MAX_RUN_SEC):
                stop_reason = "TIME"
            if stop_reason is not None:
                send_vel(0, 0, 0, 0)
                send_sys(0, 0)
                fsm_state = "LATCH_STOP"

        if fsm_state == "WAIT_ARM_WAKE":
            send_vel(0, 0, 0, 0)
            if not link.arm_busy:
                t_run_start = now_ms
                if TEST_STAGE == "B_APPROACH_TEST":
                    enter_timed_stage("TURN_TO_INNER", now_ms)
                elif TEST_STAGE == "B_RING_TEST":
                    t_pulse_done = now_ms
                    enter_timed_stage("ALIGN_B_RING", now_ms)
                elif TEST_STAGE == "HOME_RETURN_TEST":
                    enter_timed_stage("RETURN_FOLLOW_OUTER", now_ms)
                else:
                    enter_timed_stage("HOME_PRE_TURN", now_ms)

        elif fsm_state == "HOME_PRE_TURN":
            near_cross = (cross_box is not None and cross_bot >= HOME_CROSS_MIN_BOT_Y and
                          45 <= cross_cx <= 275)
            if near_cross:
                # Roadway '+' already in view while rotating -> hand straight to closed-loop lock!
                send_vel(0, 0, 0, 0)
                print("HOME pre_turn early_cross=(%d,%d)" % (cross_cx, cross_bot))
                enter_timed_stage("HOME_FIND_CROSS", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= HOME_PRE_TURN_MS:
                enter_timed_stage("LEAVE_HOME", now_ms)
            else:
                wz_cmd = HOME_PRE_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "LEAVE_HOME":
            near_cross = (cross_box is not None and cross_bot >= HOME_CROSS_MIN_BOT_Y and
                          45 <= cross_cx <= 275)
            if near_cross:
                # Detect '+' while moving out of HOME -> stop immediately, never drive past it!
                send_vel(0, 0, 0, 0)
                print("HOME leave early_cross=(%d,%d)" % (cross_cx, cross_bot))
                enter_timed_stage("HOME_FIND_CROSS", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= HOME_EXIT_MS:
                send_vel(0, 0, 0, 0)
                enter_timed_stage("HOME_FIND_CROSS", now_ms)
            else:
                vx_cmd, wz_cmd = HOME_EXIT_VX, HOME_EXIT_WZ
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "HOME_FIND_CROSS":
            el = time.ticks_diff(now_ms, t_stage_start)
            near_cross = (cross_box is not None and cross_bot >= HOME_CROSS_MIN_BOT_Y)
            if near_cross and HOME_CROSS_MIN_X <= cross_cx <= HOME_CROSS_MAX_X:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= HOME_CROSS_CONFIRM:
                    print("HOME exit_cross=(%d,%d) el=%d" % (cross_cx, cross_bot, el))
                    reset_line_state(OUTER_LOCK_INIT_CX)
                    enter_timed_stage("FOLLOW_OUTER_PICK", now_ms)
            elif el >= HOME_CROSS_TURN_MAX_MS:
                fail_stop("NO HOME X")
            else:
                turn_lock_streak = 0
                if near_cross:
                    # Bidirectional closed-loop: steer toward '+' (right if cx>max, left if cx<min)
                    wz_cmd = -HOME_CROSS_SLOW_WZ if cross_cx > HOME_CROSS_MAX_X else HOME_CROSS_SLOW_WZ
                else:
                    wz_cmd = HOME_CROSS_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

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
                    fail_stop("LOST OUT")
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
                    fail_stop("NO CYL")
            else:
                t_cyl_lost_start = None
                cx, top_y, bot_y, _, _, _, _, _ = cyl_hit
                aligned, cmd_t, vx, wz, p_ms, _, _ = compute_align_cmd(cx, top_y, bot_y)
                if aligned:
                    send_vel(0, 0, 0, 0)
                    if time.ticks_diff(now_ms, t_pulse_done) >= 0:
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

        elif fsm_state == "TURN_TO_INNER":
            el = time.ticks_diff(now_ms, t_stage_start)
            near_cross = (cross_box is not None and cross_bot >= TO_INNER_CROSS_MIN_BOT_Y)
            if (el >= deg_to_ms(TO_INNER_MIN_DEG) and near_cross and
                    TO_INNER_CROSS_MIN_X <= cross_cx <= TO_INNER_CROSS_MAX_X):
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= TO_INNER_CROSS_CONFIRM:
                    print("INNER b_cross=(%d,%d) el=%d" % (cross_cx, cross_bot, el))
                    reset_line_state()
                    enter_timed_stage("CROSS_TO_INNER", now_ms)
            elif el >= deg_to_ms(TO_INNER_MAX_DEG):
                fail_stop("NO B CROSS")
            else:
                turn_lock_streak = 0
                if el >= deg_to_ms(TO_INNER_MIN_DEG) and near_cross:
                    # Bidirectional: if (b) '+' drifted left, steer CCW left; if right, steer CW right
                    wz_cmd = TO_INNER_SLOW_WZ if cross_cx < TO_INNER_CROSS_MIN_X else -TO_INNER_SLOW_WZ
                else:
                    wz_cmd = TO_INNER_WZ_SIGN * TO_INNER_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "CROSS_TO_INNER":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= CROSS_ROAD_MAX_MS:
                fail_stop("NO B RING")
            elif ring_hit is not None:
                send_vel(0, 0, 0, 0)
                if b_ring_streak >= B_RING_LOCK_FRAMES:
                    print("CROSS direct_ring=(%d,%d) el=%d" % (ring_hit[0], ring_hit[1], el))
                    t_pulse_done = now_ms
                    enter_timed_stage("ALIGN_B_RING", now_ms)
            elif time.ticks_diff(now_ms, t_pulse_done) >= 0:
                if cross_steps >= CROSS_STEP_LIMIT:
                    fail_stop("NO B RING")
                else:
                    cross_steps += 1
                    vx_cmd = DRIVE_CAL_VX
                    send_vel(vx_cmd, 0, 0, CROSS_STEP_MS)
                    t_pulse_done = time.ticks_add(time.ticks_ms(), CROSS_STEP_MS + B_RING_SETTLE_MS)

        elif fsm_state == "JOIN_INNER_LINE":
            el = time.ticks_diff(now_ms, t_stage_start)
            if turn_lock_streak >= TURN_LOCK_FRAMES:
                send_vel(0, 0, 0, 0)
                cross_count = 0
                cross_streak = 0
                cross_clear_streak = CROSS_CLEAR_FRAMES
                cross_saw_gap = True
                b_mark_box = None
                has_prev_err = False
                enter_timed_stage("FOLLOW_INNER_TARGET", now_ms)
            elif el >= deg_to_ms(JOIN_INNER_MAX_DEG):
                fail_stop("JOIN TO")
            else:
                wz_cmd = JOIN_INNER_WZ_SIGN * TURN_CAL_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "FOLLOW_INNER_TARGET":
            target_x_num = TARGET_CROSS_ORDER[min(job_idx, len(TARGET_CROSS_ORDER) - 1)]
            if time.ticks_diff(now_ms, t_stage_start) >= B_SEARCH_MAX_MS:
                fail_stop("NO B MARK")
            elif cross_count >= target_x_num:
                send_vel(0, 0, 0, 0)
                t_pulse_done = now_ms
                enter_timed_stage("TURN_PAD_INWARD", now_ms)
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    fail_stop("LOST INN")
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
                vx_cmd = min(vx_cmd, DRIVE_CAL_VX)
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "TURN_PAD_INWARD":
            if time.ticks_diff(now_ms, t_stage_start) >= PAD_TURN_MAX_MS:
                fail_stop("PAD TURN")
            elif time.ticks_diff(now_ms, t_pulse_done) >= 0:
                if cross_cx is None:
                    turn_lock_streak = 0
                    send_vel(0, 0, 0, 0)
                elif abs(cross_cx - TARGET_CX) <= TOL_X:
                    send_vel(0, 0, 0, 0)
                    turn_lock_streak += 1
                    if turn_lock_streak >= CROSS_CONFIRM_FRAMES:
                        enter_timed_stage("ACQUIRE_B_RING", now_ms)
                else:
                    turn_lock_streak = 0
                    wz_cmd, pulse_ms = yaw_alignment_pulse(cross_cx - TARGET_CX)
                    send_vel(0, 0, wz_cmd, pulse_ms)
                    t_pulse_done = time.ticks_add(now_ms, PULSE_COOLDOWN_MS)

        elif fsm_state == "ACQUIRE_B_RING":
            if time.ticks_diff(now_ms, t_stage_start) >= B_RING_ACQUIRE_MS:
                fail_stop("NO B RING")
            elif ring_hit is not None:
                send_vel(0, 0, 0, 0)
                if b_ring_streak >= B_RING_LOCK_FRAMES:
                    enter_timed_stage("ALIGN_B_RING", now_ms)
            elif cross_box is None or cross_bot >= B_CROSS_STOP_Y:
                send_vel(0, 0, 0, 0)
            else:
                vx_cmd = DRIVE_CAL_VX
                wz_cmd = int(max(-260, min(260, round(-2.5 * (cross_cx - TARGET_CX)))))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "ALIGN_B_RING":
            if time.ticks_diff(now_ms, t_stage_start) >= B_RING_ALIGN_MS:
                fail_stop("B ALIGN TO" if b_ring is not None else "NO B RING")
            elif time.ticks_diff(now_ms, t_pulse_done) >= 0:
                b_back_active = False
                lost_close = (b_ring_misses >= B_RING_LOST_FRAMES and b_ring is not None and
                              b_ring[1] >= B_RING_BACK_Y)
                overshot = (b_ring_streak >= B_RING_LOCK_FRAMES and
                            ring_hit[1] > TARGET_BOT_Y + B_RING_TOL_Y)
                if lost_close or overshot:
                    b_ready_streak = 0
                    t_b_aligned = None
                    if b_back_steps >= B_RING_BACK_LIMIT:
                        fail_stop("B BACK LIMIT")
                    else:
                        b_back_steps += 1
                        b_back_active = True
                        vx_cmd = PULSE_VX_REV
                        send_vel(vx_cmd, 0, 0, B_RING_BACK_MS)
                        t_pulse_done = time.ticks_add(time.ticks_ms(), B_RING_BACK_MS + B_RING_BACK_SETTLE_MS)
                        b_ring_streak = 0
                        b_ring_misses = 0
                elif b_ring_streak < B_RING_LOCK_FRAMES:
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
                                stop_reason = "B AIM READY"
                                fsm_state = "LATCH_STOP"
                            elif B_PUT_EXTRA_CM > 0.0:
                                enter_timed_stage("B_PUT_NUDGE", now_ms)
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
                            b_turn_steps += 1
                        else:
                            vx_cmd = PULSE_VX_FWD
                            if ring_y < B_RING_FAR_Y:
                                pulse_ms = B_RING_FAR_STEP_MS
                            elif ring_y < B_RING_FINE_Y:
                                pulse_ms = B_RING_MID_STEP_MS
                            else:
                                pulse_ms = 155 if error_y >= 16 else 115
                            b_forward_ms += pulse_ms
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

        elif fsm_state == "PAD_REVERSE":
            if time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(PAD_REVERSE_CM):
                send_vel(0, 0, 0, 0)
                reset_line_state()
                enter_timed_stage("RETURN_OUT_TURN", now_ms)
            else:
                vx_cmd = -DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "EXIT_INNER_TURN":
            el = time.ticks_diff(now_ms, t_stage_start)
            if turn_lock_streak >= TURN_LOCK_FRAMES:
                send_vel(0, 0, 0, 0)
                enter_timed_stage("RETURN_OUT_TURN", now_ms)
            elif el >= deg_to_ms(EXIT_OUTER_MAX_DEG):
                fail_stop("EXIT INN")
            else:
                wz_cmd = EXIT_INNER_WZ_SIGN * TURN_CAL_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "RETURN_OUT_TURN":
            if time.ticks_diff(now_ms, t_stage_start) >= deg_to_ms(RETURN_OUT_TURN_DEG):
                send_vel(0, 0, 0, 0)
                reset_line_state()
                enter_timed_stage("RETURN_OUT_CROSS", now_ms)
            else:
                wz_cmd = EXIT_INNER_WZ_SIGN * TURN_CAL_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "RETURN_OUT_CROSS":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= RETURN_CROSS_MAX_MS:
                fail_stop("NO OUT LN")
            elif rail_y is not None:
                send_vel(0, 0, 0, 0)
                if last_return_rail_y is not None and abs(rail_y - last_return_rail_y) <= 12:
                    turn_lock_streak += 1
                else:
                    turn_lock_streak = 1
                last_return_rail_y = rail_y
                if turn_lock_streak >= 2:
                    print("RETURN outer_rail y=%d el=%d" % (rail_y, el))
                    reset_line_state(RETURN_OUTER_INIT_CX)
                    enter_timed_stage("RETURN_JOIN_OUTER", now_ms)
            else:
                turn_lock_streak = 0
                last_return_rail_y = None
                vx_cmd = DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "RETURN_JOIN_OUTER":
            el = time.ticks_diff(now_ms, t_stage_start)
            if turn_lock_streak >= RETURN_JOIN_LOCK_FRAMES:
                send_vel(0, 0, 0, 0)
                has_prev_err = False
                home_branch_streak = 0
                print("RETURN joined_outer e=%d ang=%d el=%d" % (err_out, ang_out, el))
                enter_timed_stage("RETURN_FOLLOW_OUTER", now_ms)
            elif turn_lock_streak >= 1:
                send_vel(0, 0, 0, 0)
            elif el >= deg_to_ms(RETURN_JOIN_MAX_DEG):
                fail_stop("JOIN OUT")
            else:
                wz_cmd = RETURN_JOIN_WZ_SIGN * TURN_CAL_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "RETURN_FOLLOW_OUTER":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= HOME_BRANCH_MIN_MS and home_branch_streak >= HOME_BRANCH_CONFIRM and home_branch is not None:
                last_home_side = home_branch[0]
                print("HOME branch=%s y=%d span=%d" % (
                    last_home_side, home_branch[2] + 2, home_branch[5]))
                enter_timed_stage("RETURN_CENTER_HOME_GATE", now_ms)
            elif el >= HOME_SEARCH_MAX_MS:
                fail_stop("NO HOME")
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    fail_stop("LOST HOME")
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

        elif fsm_state == "RETURN_CENTER_HOME_GATE":
            if line_valid == 0:
                send_vel(0, 0, 0, 0)
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms >= HOME_LOST_MS:
                    fail_stop("HOME CENTER")
            elif time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(HOME_BRANCH_ADVANCE_CM):
                send_vel(0, 0, 0, 0)
                enter_timed_stage("RETURN_TURN_INTO_HOME", now_ms)
            else:
                vx_cmd = DRIVE_CAL_VX
                _, wz_pd, prev_e_comb = follow_line_pd(
                    line_valid, err_out, ang_out, has_prev_err, prev_e_comb
                )
                has_prev_err = True
                wz_cmd = int(max(-220, min(220, wz_pd)))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "RETURN_TURN_INTO_HOME":
            if time.ticks_diff(now_ms, t_stage_start) >= deg_to_ms(HOME_TURN_DEG):
                send_vel(0, 0, 0, 0)
                enter_timed_stage("RETURN_VERIFY_HOME_GATE", now_ms)
            else:
                wz_cmd = TURN_CAL_WZ if last_home_side == "LEFT" else -TURN_CAL_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "RETURN_VERIFY_HOME_GATE":
            aligned = (home_box is not None and abs(home_box[0] - HOME_DOCK_CX) <= 32 and
                       abs(home_box[0] - home_box[1]) <= 32)
            home_box_streak = (home_box_streak + 1) if aligned else 0
            if home_box_streak >= HOME_BOX_CONFIRM_FRAMES:
                send_vel(0, 0, 0, 0)
                enter_timed_stage("RETURN_PARK_IN_HOME", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= max(HOME_BOX_WAIT_MS, 1400):
                fail_stop("HOME ALIGN" if home_box is not None else "NO HOME BOX")
            elif home_box is not None and not aligned and time.ticks_diff(now_ms, t_pulse_done) >= 0:
                wz_cmd, pulse_ms = yaw_alignment_pulse(home_box[0] - HOME_DOCK_CX)
                send_vel(0, 0, wz_cmd, pulse_ms)
                t_pulse_done = time.ticks_add(time.ticks_ms(), pulse_ms + B_RING_SETTLE_MS)
            else:
                send_vel(0, 0, 0, 0)

        elif fsm_state == "RETURN_PARK_IN_HOME":
            stop_y = HOME_PARK_STOP_Y if HOME_PARK_STOP_Y is not None else HOME_BACK_GUARD_Y
            if home_box is None:
                home_box_streak = 0
                send_vel(0, 0, 0, 0)
                if t_home_box_lost is None:
                    t_home_box_lost = now_ms
                elif time.ticks_diff(now_ms, t_home_box_lost) >= HOME_LOST_MS:
                    fail_stop("LOST BOX")
            elif abs(home_box[0] - HOME_DOCK_CX) > 32 or abs(home_box[0] - home_box[1]) > 32:
                fail_stop("HOME ALIGN")
            elif home_back_y >= stop_y:
                t_home_box_lost = None
                send_vel(0, 0, 0, 0)
                home_box_streak += 1
                if home_box_streak >= HOME_BOX_CONFIRM_FRAMES:
                    stop_reason = "DONE HOME" if HOME_PARK_STOP_Y is not None else "HOME CHECK"
                    fsm_state = "LATCH_STOP"
            elif time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(HOME_PARK_MAX_CM):
                fail_stop("HOME DEPTH" if HOME_PARK_STOP_Y is not None else "HOME CHECK")
            else:
                t_home_box_lost = None
                home_box_streak = 0
                vx_cmd = DRIVE_CAL_VX
                error_px = home_box[0] - HOME_DOCK_CX + 0.4 * (home_box[1] - home_box[0])
                wz_cmd = int(max(-180, min(180, round(-3.0 * error_px))))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "ARM_SEND":
            send_vel(0, 0, 0, 0)
            if pick_idx >= len(active_steps):
                if arm_phase == "PICK":
                    if TEST_STAGE == "OUTER_PICK_ONLY":
                        stop_reason = "PICKED"
                        fsm_state = "LATCH_STOP"
                    else:
                        enter_timed_stage("TURN_TO_INNER", now_ms)
                else:
                    job_idx += 1
                    if TEST_STAGE == "B_PICK_PLACE":
                        stop_reason = "B PUT DONE"
                        fsm_state = "LATCH_STOP"
                    elif RETURN_TO_HOME and job_idx >= len(TARGET_CROSS_ORDER):
                        enter_timed_stage("PAD_REVERSE", now_ms)
                    else:
                        stop_reason = "PLACED"
                        fsm_state = "LATCH_STOP"
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
                fail_stop("ARM TO", stop_arm=True)

        elif fsm_state == "ARM_DWELL":
            if time.ticks_diff(now_ms, t_step_wait_until) >= 0:
                pick_idx += 1
                fsm_state = "ARM_SEND"

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    if fsm_state == "LATCH_STOP" and stop_reason is not None and not run_summary_saved:
        save_run_summary(frame_stage, stop_reason, now_ms)
        run_summary_saved = True

    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s X=%d" % (stop_reason, cross_count), color=(255, 255, 255))
        if stop_reason in ("DONE HOME", "PLACED", "PICKED", "B AIM READY", "B PUT DONE"):
            set_led(0, 1, 0)
        else:
            set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        msg = "NO STM32" if not link_ok else ("DISARM KEY1" if link.armed else "READY KEY1 e=%d" % err_out)
        msg += " " + RUN_REV
        if TEST_STAGE == "B_RING_TEST" and display_ring is not None:
            msg += " B=%d,%d" % display_ring[:2]
        img.draw_string(2, 2, msg, color=(255, 255, 255))
        set_led(1, (now_ms // 300) % 2, 0) if not link_ok else set_led(0, 0, (now_ms // 250) % 2)
    elif fsm_state.startswith("ARM_"):
        img.draw_string(2, 2, "%s %d/7 %s" % (arm_phase, pick_idx + 1, active_steps[min(pick_idx, 6)][0]),
                        color=(255, 255, 0))
        set_led(0, 1, 1)
    elif fsm_state in ("ACQUIRE_B_RING", "ALIGN_B_RING"):
        if fsm_state == "ALIGN_B_RING" and b_back_active:
            msg = "B BACK %d/%d" % (b_back_steps, B_RING_BACK_LIMIT)
        else:
            msg = "B SEARCH RING" if ring_hit is None else "B RING x=%d y=%d" % ring_hit[:2]
        img.draw_string(2, 2, msg, color=(255, 255, 255))
        set_led(0, 1, 0)
    else:
        img.draw_string(2, 2, "%s X=%d e%d v%d" % (fsm_state[:8], cross_count, err_out, vx_cmd),
                        color=(255, 255, 255))
        set_led(0, 1, 0)

    log_key = (fsm_state, stop_reason, cross_count, cyl_hit is not None, b_back_steps, display_ring is not None)
    if log_key != last_log_key or time.ticks_diff(now_ms, t_last_log) >= 500:
        print("MAP %s stop=%s link=%d arm=%d busy=%d X=%d e=%d v=%d wz=%d rail_y=%s box_x=%s box_y=%s cross=%s ring=%s cyl=%s" % (
            fsm_state, stop_reason, link_ok, is_armed, link.arm_busy, cross_count, err_out, vx_cmd, wz_cmd,
            rail_y, int(round(home_box[0])) if home_box is not None else None, home_back_y,
            (cross_cx, cross_bot) if cross_cx is not None else None, display_ring[:2] if display_ring is not None else None,
            ("%s@(%d,%d)" % (cyl_hit[6], cyl_hit[0], cyl_hit[2])) if cyl_hit else "None"
        ))
        last_log_key = log_key
        t_last_log = now_ms
