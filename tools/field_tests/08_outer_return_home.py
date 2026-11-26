# Standalone Outer-Rail Lock & Left-Turn Return-to-HOME Script (OpenMV H7 Plus + frozen STM32)
# Purpose:
#   1. Guarantee locking onto the OUTER oval rail (never the INNER rail) after placing a block:
#      - Southbound crossing (`detect_outer_rail_ahead`) picks the FURTHEST white-mat-bounded rail
#        so an intermediate inner rail is skipped while dark off-mat corridor floor is rejected.
#      - Clockwise (CW) outer-loop tracking (`detect_outer_line`) enforces the geometric invariant:
#        on a CW loop (exact QQ20261125-022327.mp4 direction), the OUTER rail is ALWAYS the
#        LEFTMOST white-mat-bounded tape (`tapes[0]`), while the inner rail sits to its RIGHT.
#   2. Use long-runway outer-rail tracking to normalize robot pose before reaching HOME from the East
#      (East -> West, HOME on LEFT, roadway '+' on RIGHT — exact QQ20261125-022327.mp4 00:44..00:49).
#   3. Detect HOME's 2 left side walls (320 mm apart) -> advance short distance to gate center
#      -> left-turn locking onto HOME's straight back border (wide-FOV safe even at doorway)
#      -> level yaw trim -> straight pull-in to center of HOME box.
#
# Modes (`RUN_MODE` below):
#   "HOME_LEFT_TEST"    : Place robot on bottom outer rail EAST of HOME facing WEST (QQ20261125-022327
#                         00:40..00:49 angle) to verify 2-wall HOME detection + left-turn box docking.
#   "FROM_B_OUTER_LOOP" : Place robot at (b) post-placement pose (facing North) -> reverse 14 cm ->
#                         turn LEFT ~90 deg (West) -> cross to West OUTER rail (skipping inner rail) ->
#                         join CW North/East/South -> normalize around outer oval -> left-turn dock into HOME.

import math
import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# CONFIGURATION & CALIBRATED CONSTANTS
# ===========================================================================

REQUIRE_KEY1             = True
RUN_MODE                 = "HOME_LEFT_TEST"      # "HOME_LEFT_TEST" or "FROM_B_OUTER_LOOP"
RUN_REV                  = "08-1126C"
RUN_SUMMARY_FILE         = "map_last_run.txt"

# --- Measured chassis kinematics (floor verified) ---
#   vx=560 -> 6.2cm@500ms, 14.2cm@1000ms => 16.0 cm/s + 112 ms deadband
#   wz=640 -> ~105 deg/s on full battery (~900 ms / 90 deg, measured test24); ~69 deg/s on low battery
#   All visual-lock turns use gentle TURN_SEARCH_WZ=440 + short MIN_DEG + generous MAX_DEG so BOTH voltages lock!
DRIVE_CAL_VX             = 560
DRIVE_DEADBAND_MS        = 112
SPEED_CM_PER_S_560       = 16.0
TURN_CAL_WZ              = 640
TURN_90_MS_640           = 900
TURN_SEARCH_WZ           = 440


def cm_to_ms(cm):
    if cm <= 0.0:
        return 0
    return int(round(DRIVE_DEADBAND_MS + (float(cm) * 1000.0) / SPEED_CM_PER_S_560))


def deg_to_ms(deg):
    if deg <= 0.0:
        return 0
    return int(round((float(deg) / 90.0) * TURN_90_MS_640))


# --- Outer-Rail PD Follower (QVGA sampled at 160x120-equiv grid) ---
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
SEARCH_WZ                = 520
SEARCH_MS                = 1000
LOCK_HOLD_MS             = 600
LATCH_LOST_SEC           = 3.5

MAX_BLACK_GRAY           = 155
MIN_VALLEY_DROP          = 20
MIN_TAPE_W               = 3
MAX_TAPE_W               = 24
WIDE_JUNC_W              = 58
MAX_ANCHOR_JUMP          = 48
PARALLEL_RAIL_GAP_Q      = 24       # Min separation (in 160-grid = 48 px QVGA) between outer & inner rails
LEFT_OUTER_RECOVER_FRAMES = 6       # If a parallel tape persists left of current lock, snap back to outer
OUTER_TRACK_BIAS_Q       = 0.0      # Optional small inward (+right) offset (e.g. +4.0) if clearing outer cylinders
SCAN_BANDS               = ((96, 0.45), (86, 0.35), (76, 0.20))

# --- Stage A (FROM_B_OUTER_LOOP): Exit (b) -> Turn LEFT West -> Cross to West OUTER rail -> Join CW ---
PAD_REVERSE_CM           = 14.0     # Back clear of placed cylinder at (b)
WEST_TURN_WZ_SIGN        = 1        # +1 CCW LEFT turn from North (at b) to West (avoids right-side cylinders & halves error)
WEST_TURN_WZ             = 480      # Gentle left turn (~85-90 deg full bat / ~65 deg low bat)
WEST_TURN_MS             = 1050
OUTER_CROSS_RAIL_Y       = 200      # Stop once FURTHEST white-mat rail (outer rail) reaches y >= 200
OUTER_CROSS_MAX_MS       = cm_to_ms(55.0)
JOIN_CW_WZ_SIGN          = -1       # Facing West on West outer rail -> gentle CW right (-1) to head NORTH (CW outer loop)
JOIN_CW_MIN_DEG          = 15.0     # Lock WHILE rotating (never blind-spin 50 deg off the rail like test24!)
JOIN_CW_MAX_DEG          = 190.0
JOIN_CW_MAX_ERR          = 580
JOIN_CW_MAX_ANG          = 50
JOIN_CW_LOCK_FRAMES      = 2
OUTER_LOOP_ARM_MS        = 9000     # Ride CW loop (North -> East -> bottom) before arming HOME detector

# --- Stage B: Normalize along bottom outer rail East->West -> Visual 2-Wall HOME Detection ---
HOME_APPROACH_SIDE       = "LEFT"   # East -> West (QQ20261125-022327 00:44..00:49): HOME is on LEFT
HOME_BRANCH_MIN_MS       = 300      # Ignore startup edge frames right after arming
HOME_BRANCH_CONFIRM      = 2        # Consecutive frames seeing connected two-sided-white T-branch
HOME_WALL_CLEAR_FRAMES   = 3        # Branch-free frames inside HOME mouth before counting Wall 2
HOME_WALL_MIN_GAP_MS     = 700      # Minimum time between Wall 1 and Wall 2 (~32 cm apart -> ~2000 ms)
HOME_WALL_MAX_GAP_MS     = 4000     # Max window after Wall 1 to expect Wall 2
HOME_WALL2_ADVANCE_CM    = 10.0     # After Wall 2 hits camera (~26 cm lookahead), pivot needs ~10 cm to center
HOME_WALL1_FALLBACK_CM   = 42.0     # Fallback advance from Wall 1 if Wall 2 missed (calibrated in test6)
HOME_SEARCH_MAX_MS       = 45000    # Enough budget for full CW outer lap + HOME search

# --- Stage C: Left-Turn onto HOME Back Border -> Level Trim -> Pull Into HOME ---
# Note: Standing at HOME's doorway, the 320 mm wide box exceeds near-ground FOV so left/right side
# walls are off-screen (verified in test22 00:13). HOME's straight black back border (white mat
# below, dark corridor floor above) stays 100% visible across the frame for closed-loop squaring.
HOME_TURN_WZ_SIGN        = 1        # +1 CCW left turn into HOME (when HOME_APPROACH_SIDE == "LEFT")
HOME_TURN_SLOW_WZ        = 320      # Slow CCW spin once HOME back border enters view
HOME_TURN_MIN_DEG        = 35.0     # Short min guard so full battery starts checking well before 90 deg
HOME_TURN_MAX_DEG        = 165.0    # Generous cap for low battery while staying away from inner oval
HOME_TURN_TILT_TOL       = 16.0     # Stop turn when |tilt_px| <= 16 (or barely crosses horizontal)
HOME_ALIGN_TOL_TILT      = 14.0     # Strict stationary horizontal tolerance before pulling in
HOME_ALIGN_WAIT_MS       = 2600     # Budget for stationary yaw pulses inside HOME_VERIFY_ALIGN
HOME_PULSE_WZ_MAG        = 480
HOME_PULSE_SETTLE_MS     = 140
HOME_WALL_CONFIRM_FRAMES = 2
HOME_PARK_CM             = 17.0     # Calibrated pull-in from doorway line to center of 320mm HOME box
HOME_LOST_MS             = 300

# --- Roadway '+' pre-cue detector (logged only, mirrors QQ20261125-022327 00:44) ---
THR_CROSS                = (0, 44, -18, 18, -18, 18)

MAX_RUN_SEC              = 90.0
COVER_STOP_L             = 10
COVER_STOP_FRAMES        = 8

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
# VISION: OUTER-RAIL-LOCKED TRACKER, MULTI-RAIL CROSSING, 2-WALL HOME & BOX
# ===========================================================================

def _px_gray(img, x, y):
    r, g, b = img.get_pixel(x, y)
    return (r * 77 + g * 150 + b * 29) >> 8


def find_scanline_tapes(row):
    """Find dark black tapes bounded by brighter white mat on BOTH left and right shoulders.
    Note: dark off-mat corridor floor on the left fails min(lb, rb) - v >= MIN_VALLEY_DROP,
    so tapes[0] is guaranteed to be the LEFTMOST painted track tape (= OUTER rail on CW loop)."""
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
    """Outer-rail-priority line tracker for clockwise (CW) outer oval navigation.
    Invariant: On a CW loop, the OUTER rail is ALWAYS the LEFTMOST valid white-mat tape (tapes[0]),
    and the INNER rail lies to its RIGHT.
    Set prefer_leftmost=False on the HOME straight (FOLLOW_OUTER_TO_HOME / HOME_CENTER_GATE) once
    locked so HOME's own outside border on the left never steals the top scanband.
    Returns (pts, any_wide, parallel_left_cx)."""
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
            # Acquisition: leftmost rail on CW loop; nearest to anchor on HOME straight
            chosen = tapes[0] if prefer_leftmost else min(tapes, key=lambda t: abs(t[0] - anchor))
        else:
            in_range = [t for t in tapes if abs(t[0] - anchor) <= MAX_ANCHOR_JUMP]
            if not in_range:
                continue
            chosen = in_range[0] if prefer_leftmost else min(in_range, key=lambda t: abs(t[0] - anchor))
            # Check if a separate valid painted rail exists clearly to the LEFT of chosen
            leftmost = tapes[0][0]
            if prefer_leftmost and (chosen[0] - leftmost) >= PARALLEL_RAIL_GAP_Q and parallel_left_cx is None:
                parallel_left_cx = leftmost

        cx = chosen[0]
        anchor = cx
        pts.append((cx, float(cy_q), weight))

    return pts, any_wide, parallel_left_cx


def column_mat_rails(img, x, first_y=140):
    """Return ALL white-mat-bounded dark stripe center rows in column x, sorted far->near (ascending y).
    Dark off-mat corridor floor fails min(ahead, behind) >= 140 so mat outer border is excluded."""
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
    stripes.reverse()  # ascending y: stripes[0] is FURTHEST (= OUTER rail when heading South!)
    return stripes


def detect_outer_rail_ahead(img):
    """When driving South out of (b) toward the outer bottom rail, BOTH inner and outer rails may
    appear in view. Because the dark corridor floor beyond the mat fails two-sided white-mat checks,
    the FURTHEST (smallest y) valid stripe in each column is GUARANTEED to be the OUTER rail.
    Returns (outer_y, saw_dual_rails) or (None, False)."""
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

    # Confirm across 3 neighboring columns (handles diagonal approaches up to ~30 deg)
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
    """Detect HOME's perpendicular side wall touching the fitted outer rail on `side`.
    Enforces two-sided bright white mat (`min(above, below) >= 145`) so off-mat corridor floor
    never causes a false trigger."""
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


def _find_col_mat_edge(img, x, y_start=232, y_end=124):
    """Scan upward in column x for the near edge of HOME's black back border:
    bright white HOME mat BELOW (>= 135) -> dark border band (<= 118, >= 3 px) -> darker off-mat floor ABOVE.
    Interior tapes (Wall 2, oval rails) have bright white mat on BOTH sides and are skipped."""
    off = y_end - 18
    col = [_px_gray(img, x, y) for y in range(off, IMG_H)]
    y = y_start
    while y >= y_end:
        if col[y - off] > 118:
            y -= 1
            continue
        y_bot = y
        y_top = y
        while y_top > y_end and (y_bot - y_top) < 34 and col[(y_top - 1) - off] <= 118:
            y_top -= 1
        band_h = y_bot - y_top + 1
        if band_h >= 3:
            b0 = min(IMG_H - 1, y_bot + 4) - off
            b1 = min(IMG_H, y_bot + 15) - off
            if (b1 - b0) >= 3:
                below_g = sum(col[b0:b1]) / float(b1 - b0)
                if below_g >= 135.0:
                    if band_h >= 30:
                        # Thick dark region merges directly into dark off-mat corridor floor
                        above_g = sum(col[(y_top - off):(y_top - off + 6)]) / 6.0
                    else:
                        a0 = max(0, (y_top - 16) - off)
                        a1 = max(a0 + 1, (y_top - 4) - off)
                        above_g = sum(col[a0:a1]) / float(a1 - a0)
                    if (below_g - above_g) >= 22.0 and above_g <= 145.0:
                        return y_bot
        y = y_top - 2
    return None


def detect_home_back_wall(img):
    """Detect HOME's straight black back border across 7 columns (FOV-safe right at HOME's doorway).
    Returns (center_y, tilt_px, n_inliers, inlier_pts) or None.
      tilt_px = slope * 200.0 (y_right - y_left across 200 px span):
        tilt_px < 0 => under-turned CCW (test22 00:13 ~ -38) => need +wz (CCW left)
        tilt_px > 0 => over-turned CCW                        => need -wz (CW right)
        |tilt_px| <= 14 => robot is dead-square into HOME."""
    xs = (50, 86, 122, 160, 198, 234, 270)
    pts = []
    for x in xs:
        yb = _find_col_mat_edge(img, x)
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
                if e <= 9.0:
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
    if abs(a) > 0.45:
        return None
    if max(abs(py - (a * px + b)) for px, py in best_inliers) > 9.0:
        return None
    center_y = a * 160.0 + b
    if not (132.0 <= center_y <= 234.0):
        return None
    tilt_px = a * 200.0
    return (center_y, tilt_px, len(best_inliers), best_inliers)


_RING16 = tuple(
    (math.cos(i * math.pi / 8.0), math.sin(i * math.pi / 8.0)) for i in range(16)
)


def _is_cross_ring16(img, x, y, w, h):
    cx, cy = x + w // 2, y + h // 2
    arm_c = _px_gray(img, cx, cy)
    if arm_c > 118:
        return False
    rx = max(4.0, 0.38 * w)
    ry = max(3.5, 0.38 * h)
    vals = [
        _px_gray(img, max(0, min(IMG_W - 1, int(round(cx + rx * c_a)))),
                 max(0, min(IMG_H - 1, int(round(cy + ry * s_a)))))
        for c_a, s_a in _RING16
    ]
    hi, lo = max(vals), min(vals)
    base = arm_c if arm_c > lo else lo
    if hi < 135 or (hi - base) < 22:
        return False
    thr = min(130.0, 0.5 * (hi + base))
    dark = [(v <= thr) for v in vals]
    if sum(1 for d in dark if not d) < 4:
        return False
    return sum(1 for i in range(16) if dark[i] and (not dark[(i - 1) & 15])) == 4


def detect_roadway_cross_right(img, pts):
    """Non-blocking pre-cue matching QQ20261125-022327 00:44: roadway '+' on RIGHT of outer rail."""
    if len(pts) < 2:
        return None
    near_x, near_y, _ = pts[0]
    far_x, far_y, _ = pts[-1]
    if abs(near_y - far_y) < 1e-3:
        return None
    slope = (near_x - far_x) / (near_y - far_y)
    roi = (0, 128, IMG_W, IMG_H - 128)
    blobs = img.find_blobs([THR_CROSS], roi=roi, x_stride=2, y_stride=1,
                           pixels_threshold=45, area_threshold=60)
    for b in blobs:
        x, y, w, h = b.rect()
        if not (16 <= w <= 215 and 8 <= h <= 115 and w <= 7.5 * h and h <= 3.0 * w):
            continue
        if y <= roi[1] or x + w >= IMG_W - 1 or y + h >= IMG_H - 1:
            continue
        sol = float(b.pixels()) / float(max(1, w * h))
        if sol < 0.18 or sol > 0.62:
            continue
        cx, cy = x + w // 2, y + h // 2
        rail_x = 2 * near_x + slope * (cy - 2 * near_y)
        if cx < rail_x + 16:
            continue
        if _is_cross_ring16(img, x, y, w, h):
            return (x, y, w, h)
    return None


def follow_line_pd(line_valid, err_out, ang_out, has_prev_err, prev_e_comb):
    e_comb = max(-400.0, min(400.0, 0.5 * err_out + 0.5 * ang_out * K_ANG)) if line_valid == 2 else (
        max(-1200.0, min(1200.0, err_out + ang_out * K_ANG)))
    d_err = max(-120.0, min(120.0, e_comb - prev_e_comb)) if has_prev_err else 0.0
    wz_cmd = int(max(-WZ_MAX, min(WZ_MAX, round(-(KP * e_comb + KD * d_err)))))
    vx_cmd = int(round(VX_CRUISE - (min(1000.0, abs(e_comb)) * max(0, VX_CRUISE - VX_MIN)) / 1000.0))
    return vx_cmd, wz_cmd, e_comb


def wall_yaw_pulse(tilt_px):
    """Short timed yaw pulse to level HOME's back border.
    tilt_px < 0 => under-turned CCW => +wz (CCW left); tilt_px > 0 => over-turned => -wz (CW right)."""
    wz = -HOME_PULSE_WZ_MAG if tilt_px > 0.0 else HOME_PULSE_WZ_MAG
    ms = 120 if abs(tilt_px) >= 28.0 else 80
    return wz, ms


# ===========================================================================
# STATE MACHINE
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
t_pulse_done = 0
t_last_ping = 0
t_last_log = 0

last_cx = float(LINE_CX)
had_lock = False
err_filt = 0.0
has_prev_err = False
prev_e_comb = 0.0
last_dir = 0
left_rail_streak = 0
outer_shifts = 0

turn_lock_streak = 0
last_outer_rail_y = None
road_cross_seen = False

# 2-wall HOME tracking
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
    global home_wall_saw_gap, t_home_wall1, gate_advance_cm, gate_src, road_cross_seen
    home_branch_streak = 0
    last_branch_y = None
    home_wall_count = 0
    home_wall_clear_streak = 0
    home_wall_saw_gap = False
    t_home_wall1 = 0
    gate_advance_cm = HOME_WALL1_FALLBACK_CM
    gate_src = "none"
    road_cross_seen = False


def enter_stage(state_name, now_ms):
    global fsm_state, t_stage_start, turn_lock_streak, last_outer_rail_y
    global home_wall_streak, t_pulse_done, align_pulses
    fsm_state = state_name
    t_stage_start = now_ms
    turn_lock_streak = 0
    last_outer_rail_y = None
    home_wall_streak = 0
    t_pulse_done = now_ms
    if state_name == "HOME_VERIFY_ALIGN":
        align_pulses = 0


def write_summary(now_ms, reason, back_y=None):
    global summary_written
    if summary_written:
        return
    summary_written = True
    by_s = ("%d" % int(round(back_y))) if back_y is not None else "None"
    tilt_s = ("%.1f" % last_back_tilt) if last_back_tilt is not None else "None"
    line = (
        "rev=%s mode=%s stage=%s stop=%s elapsed_ms=%d stage_ms=%d walls=%d gate=%s "
        "road_x=%d shifts=%d pulses=%d back_y=%s tilt=%s\n" % (
            RUN_REV, RUN_MODE, fsm_state, reason,
            time.ticks_diff(now_ms, t_run_start), time.ticks_diff(now_ms, t_stage_start),
            home_wall_count, gate_src, int(road_cross_seen), outer_shifts, align_pulses,
            by_s, tilt_s
        )
    )
    print("SUMMARY " + line.strip())
    try:
        with open(RUN_SUMMARY_FILE, "w") as f:
            f.write(line)
    except Exception:
        pass


def latch_stop(reason, now_ms, back_y=None, soft_stop=False):
    global stop_reason, fsm_state
    send_vel(0, 0, 0, 0)
    if soft_stop:
        send_sys(0, 0)
    write_summary(now_ms, reason, back_y)
    stop_reason = reason
    fsm_state = "LATCH_STOP"


print("OUTER_HOME %s mode=%s side=%s wall2_cm=%.1f wall1_cm=%.1f park_cm=%.1f" % (
    RUN_REV, RUN_MODE, HOME_APPROACH_SIDE, HOME_WALL2_ADVANCE_CM,
    HOME_WALL1_FALLBACK_CM, HOME_PARK_CM
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

    # --- 1. VISION PASS (strictly BEFORE any debug drawing on img) ---
    prefer_left = fsm_state in ("JOIN_OUTER_CW", "FOLLOW_OUTER_LOOP")
    pts, any_wide, parallel_left_cx = detect_outer_line(img, last_cx, had_lock, prefer_leftmost=prefer_left)
    if parallel_left_cx is not None and fsm_state == "FOLLOW_OUTER_LOOP":
        left_rail_streak += 1
        if left_rail_streak >= LEFT_OUTER_RECOVER_FRAMES:
            # Persistent valid rail to our LEFT => we drifted onto inner rail; snap back to outer!
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

    outer_rail_y, saw_dual_rails = (
        detect_outer_rail_ahead(img) if fsm_state == "CROSS_TO_OUTER" else (None, False)
    )
    home_branch = (
        detect_home_branch(img, pts, HOME_APPROACH_SIDE)
        if fsm_state in ("FOLLOW_OUTER_TO_HOME", "HOME_CENTER_GATE") else None
    )
    road_cross = (
        detect_roadway_cross_right(img, pts)
        if (fsm_state == "FOLLOW_OUTER_TO_HOME" and not road_cross_seen) else None
    )
    if road_cross is not None:
        road_cross_seen = True
        print("HOME pre_cue roadway_cross_right=%s" % (road_cross,))

    need_wall = (
        fsm_state in ("HOME_TURN_INTO_BOX", "HOME_VERIFY_ALIGN", "HOME_PARK_IN_BOX") or
        (fsm_state == "LATCH_STOP" and stop_reason in (
            "HOME CHECK", "DONE HOME", "HOME ALIGN", "NO HOME WALL"))
    )
    # Skip wall detection during active timed yaw pulse so we only evaluate fresh stationary frames
    pulse_settling = (fsm_state == "HOME_VERIFY_ALIGN" and time.ticks_diff(now_ms, t_pulse_done) < 0)
    home_wall = detect_home_back_wall(img) if (need_wall and not pulse_settling) else None
    if home_wall is not None:
        last_back_y = home_wall[0]
        last_back_tilt = home_wall[1]

    # Track 2-wall HOME left branches cleanly (Wall 1 -> opening gap -> Wall 2)
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

    # --- 2. DEBUG OVERLAYS (strictly AFTER all vision detectors) ---
    for cy_q, _ in SCAN_BANDS:
        img.draw_line(0, cy_q * 2, IMG_W - 1, cy_q * 2, color=(70, 70, 70))
    for cx_pt, cy_q, _ in pts:
        img.draw_cross(int(cx_pt * 2), int(cy_q * 2), color=(0, 255, 0), size=5)
    if outer_rail_y is not None:
        img.draw_line(60, outer_rail_y, 260, outer_rail_y, color=(0, 255, 255))
    if road_cross is not None:
        img.draw_rectangle(road_cross, color=(255, 0, 255))
    if home_branch is not None:
        _, hx, hy, hw, hh, _ = home_branch
        img.draw_rectangle((hx, hy, hw, hh), color=(255, 255, 0))
    if home_wall is not None:
        wy, wtilt, _, wpts = home_wall
        slope_w = wtilt / 200.0
        x0_w = int(round(wpts[0][0]))
        x1_w = int(round(wpts[-1][0]))
        y0_w = max(0, min(IMG_H - 1, int(round(wy + slope_w * (x0_w - 160.0)))))
        y1_w = max(0, min(IMG_H - 1, int(round(wy + slope_w * (x1_w - 160.0)))))
        img.draw_line(x0_w, y0_w, x1_w, y1_w, color=(0, 255, 255))
        for wx_pt, wy_pt in wpts:
            img.draw_cross(int(round(wx_pt)), int(round(wy_pt)), color=(255, 255, 0), size=4)

    # --- 3. STATE MACHINE ---
    is_armed = (link_ok and seen_disarmed and link.armed) if REQUIRE_KEY1 else True
    vx_cmd = 0
    wz_cmd = 0

    if not is_armed:
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
            if stop_reason is None:
                write_summary(now_ms, "DISARM" if link_ok else "LINK LOST", last_back_y)
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        summary_written = False
        outer_shifts = 0
        last_back_y = None
        last_back_tilt = None
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
            last_back_y = None
            last_back_tilt = None
            summary_written = False
            reset_line_state(40.0 if RUN_MODE == "FROM_B_OUTER_LOOP" else float(LINE_CX))
            reset_home_wall_state()
            enter_stage("WAIT_ARM_WAKE", now_ms)
        else:
            if cover_cnt >= COVER_STOP_FRAMES:
                latch_stop("COVER", now_ms, last_back_y, soft_stop=True)
            elif (MAX_RUN_SEC > 0) and (fsm_state != "LATCH_STOP") and (
                    time.ticks_diff(now_ms, t_run_start) / 1000.0 >= MAX_RUN_SEC):
                latch_stop("TIME", now_ms, last_back_y, soft_stop=True)

        if fsm_state == "WAIT_ARM_WAKE":
            send_vel(0, 0, 0, 0)
            if not link.arm_busy:
                t_run_start = now_ms
                if RUN_MODE == "FROM_B_OUTER_LOOP":
                    enter_stage("PAD_REVERSE", now_ms)
                else:
                    reset_line_state(float(LINE_CX))
                    enter_stage("FOLLOW_OUTER_TO_HOME", now_ms)

        elif fsm_state == "PAD_REVERSE":
            if time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(PAD_REVERSE_CM):
                send_vel(0, 0, 0, 0)
                enter_stage("TURN_FACE_WEST", now_ms)
            else:
                vx_cmd = -DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "TURN_FACE_WEST":
            if time.ticks_diff(now_ms, t_stage_start) >= WEST_TURN_MS:
                send_vel(0, 0, 0, 0)
                reset_line_state()
                enter_stage("CROSS_TO_OUTER", now_ms)
            else:
                wz_cmd = WEST_TURN_WZ_SIGN * WEST_TURN_WZ
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
                    print("CROSS_TO_OUTER locked outer_y=%d el=%d" % (outer_rail_y, el))
                    reset_line_state(40.0)  # Bias JOIN_OUTER_CW to LEFTMOST (outer) rail
                    enter_stage("JOIN_OUTER_CW", now_ms)
            else:
                turn_lock_streak = 0
                last_outer_rail_y = None
                vx_cmd = DRIVE_CAL_VX
                send_vel(vx_cmd, 0, 0, 0)

        elif fsm_state == "JOIN_OUTER_CW":
            el = time.ticks_diff(now_ms, t_stage_start)
            locked_now = (
                el >= deg_to_ms(JOIN_CW_MIN_DEG) and len(pts) >= 2 and line_valid == 1 and
                abs(err_out) <= JOIN_CW_MAX_ERR and abs(ang_out) <= JOIN_CW_MAX_ANG
            )
            if locked_now:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= JOIN_CW_LOCK_FRAMES:
                    print("JOIN_OUTER_CW locked e=%d ang=%d el=%d" % (err_out, ang_out, el))
                    has_prev_err = False
                    reset_home_wall_state()
                    enter_stage("FOLLOW_OUTER_LOOP", now_ms)
            elif el >= deg_to_ms(JOIN_CW_MAX_DEG):
                latch_stop("JOIN CW TO", now_ms)
            else:
                turn_lock_streak = 0
                wz_cmd = JOIN_CW_WZ_SIGN * TURN_SEARCH_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "FOLLOW_OUTER_LOOP":
            # Normalize pose around the CW outer oval before arming HOME detection
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
                print("HOME wall1 side=%s y=%d span=%d" % (
                    home_branch[0], home_branch[2] + 2, home_branch[5]))
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
            # After Wall 1, keep tracking outer rail; if Wall 2 (320mm later) is confirmed,
            # reset remaining advance to short HOME_WALL2_ADVANCE_CM (~10 cm).
            since_w1 = time.ticks_diff(now_ms, t_home_wall1)
            if (home_wall_count == 1 and home_wall_saw_gap and
                    HOME_WALL_MIN_GAP_MS <= since_w1 <= HOME_WALL_MAX_GAP_MS and
                    home_branch_streak >= HOME_BRANCH_CONFIRM and home_branch is not None):
                home_wall_count = 2
                home_wall_saw_gap = False
                gate_advance_cm = HOME_WALL2_ADVANCE_CM
                gate_src = "wall2"
                t_stage_start = now_ms
                print("HOME wall2 y=%d span=%d gap_ms=%d adv=%.1f" % (
                    home_branch[2] + 2, home_branch[5], since_w1, gate_advance_cm))

            if line_valid == 0:
                send_vel(0, 0, 0, 0)
                lost_ms = time.ticks_diff(now_ms, t_line_lost_start) if t_line_lost_start is not None else 0
                if lost_ms >= HOME_LOST_MS:
                    latch_stop("HOME CENTER", now_ms)
            elif time.ticks_diff(now_ms, t_stage_start) >= cm_to_ms(gate_advance_cm):
                send_vel(0, 0, 0, 0)
                print("HOME gate_ready src=%s walls=%d" % (gate_src, home_wall_count))
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
                el >= deg_to_ms(HOME_TURN_MIN_DEG) and signed_tilt is not None and
                (-HOME_TURN_TILT_TOL <= signed_tilt <= 40.0)
            )
            if wall_level:
                send_vel(0, 0, 0, 0)
                turn_lock_streak += 1
                if turn_lock_streak >= 2:
                    print("HOME turn_locked back_y=%.0f tilt=%.1f n=%d el=%d" % (
                        home_wall[0], home_wall[1], home_wall[2], el))
                    enter_stage("HOME_VERIFY_ALIGN", now_ms)
            elif el >= deg_to_ms(HOME_TURN_MAX_DEG):
                send_vel(0, 0, 0, 0)
                enter_stage("HOME_VERIFY_ALIGN", now_ms)
            else:
                turn_lock_streak = 0
                wz_mag = HOME_TURN_SLOW_WZ if (el >= deg_to_ms(HOME_TURN_MIN_DEG) and home_wall is not None) else TURN_SEARCH_WZ
                wz_cmd = HOME_TURN_WZ_SIGN * wz_mag
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "HOME_VERIFY_ALIGN":
            if time.ticks_diff(now_ms, t_stage_start) >= HOME_ALIGN_WAIT_MS:
                latch_stop("HOME ALIGN" if home_wall is not None else "NO HOME WALL", now_ms, last_back_y)
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
                            print("HOME align_ok back_y=%.0f tilt=%.1f n=%d" % (wy, wtilt, wn))
                            enter_stage("HOME_PARK_IN_BOX", now_ms)
                    else:
                        home_wall_streak = 0
                        wz_cmd, p_ms = wall_yaw_pulse(wtilt)
                        align_pulses += 1
                        send_vel(0, 0, wz_cmd, p_ms)
                        t_pulse_done = time.ticks_add(time.ticks_ms(), p_ms + HOME_PULSE_SETTLE_MS)

        elif fsm_state == "HOME_PARK_IN_BOX":
            el = time.ticks_diff(now_ms, t_stage_start)
            if el >= cm_to_ms(HOME_PARK_CM):
                latch_stop("DONE HOME", now_ms, last_back_y)
            else:
                vx_cmd = DRIVE_CAL_VX
                if home_wall is not None:
                    # Steer gently to keep HOME back border horizontal until it exits bottom of FOV
                    wz_cmd = int(max(-160, min(160, round(-4.5 * home_wall[1]))))
                else:
                    wz_cmd = 0
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    # --- 4. HUD & TELEMETRY ---
    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s W=%d %s" % (stop_reason, home_wall_count, RUN_REV), color=(255, 255, 255))
        if stop_reason in ("DONE HOME", "HOME CHECK"):
            set_led(0, 1, 0)
        else:
            set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        msg = "NO STM32" if not link_ok else ("DISARM KEY1" if link.armed else "READY KEY1 e=%d" % err_out)
        img.draw_string(2, 2, "%s %s" % (msg, RUN_REV), color=(255, 255, 255))
        set_led(1, (now_ms // 300) % 2, 0) if not link_ok else set_led(0, 0, (now_ms // 250) % 2)
    else:
        t_hud = ("T%d" % int(round(home_wall[1]))) if home_wall is not None else ("e%d" % err_out)
        img.draw_string(2, 2, "%s W=%d %s v%d" % (fsm_state[:10], home_wall_count, t_hud, vx_cmd),
                        color=(255, 255, 255))
        set_led(0, 1, 0)

    log_key = (fsm_state, stop_reason, home_wall_count, home_wall is not None, outer_rail_y is not None)
    if log_key != last_log_key or time.ticks_diff(now_ms, t_last_log) >= 500:
        wall_s = (int(round(home_wall[0])), round(home_wall[1], 1), home_wall[2]) if home_wall is not None else None
        print("OUTER %s stop=%s link=%d arm=%d W=%d e=%d ang=%d v=%d wz=%d out_y=%s dual=%d wall=%s" % (
            fsm_state, stop_reason, link_ok, is_armed, home_wall_count, err_out, ang_out, vx_cmd, wz_cmd,
            outer_rail_y, int(saw_dual_rails), wall_s
        ))
        last_log_key = log_key
        t_last_log = now_ms
