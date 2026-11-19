# OpenMV black-tape line / dual-rail corridor tracker for Carbot(C5) KPZ-STM32.
# Tested layout: OpenMV Cam H7 Plus (OV5640).
#
# Wiring (USART3 shared bus @ 115200 8N1):
#   OpenMV P4 (UART3 TX) -> KPZ-STM32 yellow 4-pin header "RX" (STM32 PB11)
#   OpenMV GND           -> KPZ-STM32 yellow 4-pin header "GND"
#   OpenMV VIN (5V)      -> KPZ-STM32 yellow 4-pin header "5V" (when USB unplugged)
#   (Leave OpenMV P5 RX unconnected so motor bus traffic never disturbs OpenMV.)
#
# Output packet (never starts with '#' so ZMotorD ignores it completely):
#   @L,<err>,<ang>,<valid>,<arm>*<XX>\n
#     err   : -1000..+1000  (<0 lane/line is left of centre, >0 right)
#     ang   : -90..+90 deg  (lookahead curve slope; <0 curves left, >0 right)
#     valid : 0=lost, 1=normal, 2=wide junction/cross-bar
#     arm   : 0=bench preview / latched-stop, 1=request STM32 autonomous drive
#
# Safety stops when AUTO_ARM = True:
#   1. Blue LED 3s countdown on boot before arm=1 (hands-clear start).
#   2. Cover lens by hand (dark uniform image >=5 frames) -> instant latched stop.
#   3. Line lost > LATCH_LOST_SEC (0.8s) -> latches arm=0 (carrying won't restart).
#   4. Max single run > MAX_RUN_SEC (15s) -> latches arm=0 (set 0 to disable).

import math
import time
import sensor
import image
from pyb import UART, LED

# --- user setup ------------------------------------------------------------

LANE_MODE           = "dual"  # "dual" = between 2 rails, "single" = on 1 line
AUTO_ARM            = True    # Set True when saving as standalone main.py
WAIT_BEFORE_ARM_SEC = 3.0     # Blue LED countdown before arm=1
MAX_RUN_SEC         = 15.0    # Max run seconds in AUTO_ARM (0 = unlimited)
LATCH_LOST_SEC      = 0.8     # Latch stop if line gone this long in AUTO_ARM
COVER_STOP_BG       = 18      # Max bg brightness treated as covered/picked-up
COVER_STOP_FRAMES   = 5       # Consecutive dark covered frames to latch stop

FLIP_Y              = False   # Set True if OpenMV image is upside-down
MIRROR_X            = False   # Set True if image left/right is mirrored

MAX_BLACK_GRAY      = 155     # Ceiling gray value accepted as black tape
MIN_VALLEY_DROP     = 16      # Min drop below BOTH nearby floor shoulders
MIN_TAPE_W          = 2       # Minimum tape half-depth width (px)
MAX_TAPE_W          = 14      # Reject wide chair feet / shoes (px)
WIDE_JUNC_W         = 60      # Continuous dark cross-bar width threshold (px)
MIN_LANE_PX         = 36      # Minimum separation between left & right rails
SINGLE_RAIL_BIAS    = 0.50    # Half-corridor offset fraction when 1 rail disappears

# Floor-sweet-spot scan rows (y=95..103) cleanly below top-horizon shoes/chair
# legs (y<=88) and above near-bumper border clipping (y>=108): (y, weight, lane_px).
SCAN_BANDS = (
    (103, 0.45, 136.0),  # near floor
    ( 99, 0.35, 118.0),  # mid floor
    ( 95, 0.20, 100.0),  # far floor lookahead
)

IMG_W = 160
IMG_H = 120
IMG_CX = IMG_W // 2

# --- hardware init ---------------------------------------------------------

led_r = LED(1)
led_g = LED(2)
led_b = LED(3)

def set_led(r, g, b):
    led_r.on() if r else led_r.off()
    led_g.on() if g else led_g.off()
    led_b.on() if b else led_b.off()

uart = UART(3, 115200, timeout_char=20)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
try:
    sensor.set_brightness(0)
    sensor.set_contrast(2)
except Exception:
    pass
if FLIP_Y:
    sensor.set_vflip(True)
if MIRROR_X:
    sensor.set_hmirror(True)
sensor.skip_frames(time=600)

# --- vision helpers --------------------------------------------------------

def send_line_pkt(err, ang, valid, arm):
    body = "L,%d,%d,%d,%d" % (err, ang, valid, arm)
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    uart.write("@%s*%02X\n" % (body, csum))

def sample_band_row(img, cy):
    # Average 3 rows (cy-1, cy, cy+1) so oblique diagonal tapes keep full valley
    # contrast without 1px speckle noise.
    return [
        (img.get_pixel(x, cy - 1) + img.get_pixel(x, cy) + img.get_pixel(x, cy + 1)) // 3
        for x in range(IMG_W)
    ]

def find_scanline_tapes(row):
    # Local shoulder valley detector: immune to global corner shadows/bright lamps
    # and keeps thin diagonal tapes from blurring inside 2D bounding boxes.
    s_sorted = sorted(row)
    bg_med = s_sorted[IMG_W // 2]

    # Wide horizontal cross-bar check in central track window (x=15..145)
    junc_run = 0
    max_junc_run = 0
    for x in range(15, 145):
        v = row[x]
        if v <= MAX_BLACK_GRAY and (bg_med - v) >= 24:
            junc_run += 1
            if junc_run > max_junc_run:
                max_junc_run = junc_run
        else:
            junc_run = 0
    wide_hit = max_junc_run >= WIDE_JUNC_W

    tapes = []
    m = 3
    while m <= IMG_W - 4:
        v = row[m]
        if (v <= MAX_BLACK_GRAY and
                v <= row[m - 1] and v <= row[m + 1] and
                v <= row[m - 2] and v <= row[m + 2]):
            lb = max(row[max(0, m - 14) : m - 1])
            rb = max(row[m + 2 : min(IMG_W, m + 15)])
            ref = lb if lb < rb else rb
            drop = ref - v
            if drop >= MIN_VALLEY_DROP:
                half = 0.5 * (ref + v)
                x0 = m
                while x0 > 0 and row[x0 - 1] <= half:
                    x0 -= 1
                x1 = m
                while x1 < (IMG_W - 1) and row[x1 + 1] <= half:
                    x1 += 1
                bw = x1 - x0 + 1
                # Require bright floor shoulders inside image (reject border clutter)
                if MIN_TAPE_W <= bw <= MAX_TAPE_W and x0 >= 1 and x1 <= (IMG_W - 2):
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

    return tapes, wide_hit, bg_med

def solve_dual_band(tapes, anchor_cx, last_lx, last_rx, lane_w, force_side, last_cx_1):
    if not tapes:
        return None

    best_pair = None
    best_cost = 1e9
    n = len(tapes)
    for i in range(n):
        for j in range(i + 1, n):
            sep = tapes[j][0] - tapes[i][0]
            if sep < MIN_LANE_PX:
                continue
            mid = 0.5 * (tapes[i][0] + tapes[j][0])
            cost = abs(mid - anchor_cx) + 0.25 * abs(sep - lane_w)
            if cost < best_cost:
                best_cost = cost
                best_pair = (tapes[i], tapes[j])

    if best_pair is not None:
        lx = best_pair[0][0]
        rx = best_pair[1][0]
        return (0.5 * (lx + rx), lx, rx, rx - lx, True, 0, None)

    # Single-rail fallback: lock rail side across consecutive 1-rail frames so
    # approaching a rail near x=80 never mislabels left rail <-> right rail.
    if force_side != 0 and last_cx_1 is not None:
        near = [t for t in tapes if abs(t[0] - last_cx_1) <= 32.0]
        if near:
            cand = min(near, key=lambda t: abs(t[0] - last_cx_1))
            side = force_side
        else:
            cand = min(tapes, key=lambda t: min(abs(t[0] - last_lx), abs(t[0] - last_rx)))
            side = -1 if abs(cand[0] - last_lx) <= abs(cand[0] - last_rx) else 1
    else:
        cand = min(tapes, key=lambda t: min(abs(t[0] - last_lx), abs(t[0] - last_rx)))
        side = -1 if abs(cand[0] - last_lx) <= abs(cand[0] - last_rx) else 1

    cx = cand[0]
    half_off = SINGLE_RAIL_BIAS * lane_w
    if side < 0:
        mid = max(0.0, min(float(IMG_W), cx + half_off))
        return (mid, cx, cx + lane_w, lane_w, False, -1, cx)
    else:
        mid = max(0.0, min(float(IMG_W), cx - half_off))
        return (mid, cx - lane_w, cx, lane_w, False, 1, cx)

def solve_single_band(tapes, anchor_cx):
    if not tapes:
        return None
    best = min(tapes, key=lambda t: abs(t[0] - anchor_cx))
    cx = best[0]
    return (cx, cx, cx, 0.0, False)

# --- main loop -------------------------------------------------------------

t_boot = time.ticks_ms()
t_armed = None
t_lost_start = None
clock = time.clock()
last_cx = float(IMG_CX)
roi_lx = [float(IMG_CX) - 0.5 * b[2] for b in SCAN_BANDS]
roi_rx = [float(IMG_CX) + 0.5 * b[2] for b in SCAN_BANDS]
roi_lw = [b[2] for b in SCAN_BANDS]
single_side = [0, 0, 0]
single_cx = [None, None, None]
err_filt = 0.0
cover_cnt = 0
stop_reason = None

while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, t_boot) / 1000.0

    # Sample raw pixel scan rows BEFORE drawing any HUD overlays!
    band_scans = []
    max_bg = 0
    any_wide = False
    for cy, weight, _ in SCAN_BANDS:
        row = sample_band_row(img, cy)
        tapes, wide, bg = find_scanline_tapes(row)
        if bg > max_bg:
            max_bg = bg
        if wide:
            any_wide = True
        band_scans.append((cy, weight, tapes))

    dual_pts = []
    single_pts = []
    anchor = last_cx

    for idx, (cy, weight, tapes) in enumerate(band_scans):
        img.draw_line(0, cy - 2, IMG_W - 1, cy - 2, color=170)
        img.draw_line(0, cy + 2, IMG_W - 1, cy + 2, color=170)
        for cx, _, x0, x1 in tapes:
            img.draw_rectangle(x0, cy - 3, x1 - x0 + 1, 7, color=90)

        if LANE_MODE == "dual":
            res = solve_dual_band(tapes, anchor, roi_lx[idx], roi_rx[idx],
                                  roi_lw[idx], single_side[idx], single_cx[idx])
            if res is not None:
                mid_cx, l_x, r_x, sep, two_rails, side_1, cx_1 = res
                anchor = mid_cx
                roi_lx[idx] = l_x
                roi_rx[idx] = r_x
                if two_rails:
                    roi_lw[idx] = 0.75 * roi_lw[idx] + 0.25 * sep
                    single_side[idx] = 0
                    single_cx[idx] = None
                    dual_pts.append((mid_cx, float(cy), weight))
                    img.draw_cross(int(mid_cx), cy, color=255, size=5)
                else:
                    single_side[idx] = side_1
                    single_cx[idx] = cx_1
                    single_pts.append((mid_cx, float(cy), weight))
        else:
            res = solve_single_band(tapes, anchor)
            if res is not None:
                mid_cx = res[0]
                anchor = mid_cx
                dual_pts.append((mid_cx, float(cy), weight))
                img.draw_cross(int(mid_cx), cy, color=255, size=5)

    # Prefer true dual-rail fixes whenever >=1 band sees both rails cleanly
    pts = dual_pts if dual_pts else single_pts
    if not dual_pts:
        for cx, cy, _ in single_pts:
            img.draw_cross(int(cx), int(cy), color=210, size=3)

    if pts:
        t_lost_start = None
        w_sum = sum(w for _, _, w in pts)
        cx_avg = sum(cx * w for cx, _, w in pts) / w_sum
        last_cx = pts[0][0] if len(pts) == 1 else cx_avg

        err_raw = ((cx_avg - IMG_CX) * 1000.0) / float(IMG_CX)
        err_filt = 0.75 * err_raw + 0.25 * err_filt
        err_out = int(max(-1000, min(1000, round(err_filt))))

        if len(pts) >= 2:
            lo_cx, lo_cy, _ = pts[0]
            hi_cx, hi_cy, _ = pts[-1]
            dy = max(1.0, lo_cy - hi_cy)
            ang_out = int(max(-90, min(90, round(math.degrees(math.atan2(hi_cx - lo_cx, dy))))))
            img.draw_line(int(lo_cx), int(lo_cy), int(hi_cx), int(hi_cy), color=255)
        else:
            ang_out = 0

        valid = 2 if (any_wide and len(pts) <= 1) else 1
    else:
        if t_lost_start is None:
            t_lost_start = now_ms
        err_out = int(max(-1000, min(1000, round(err_filt))))
        ang_out = 0
        valid = 0

    # --- safety latch checks (for standalone AUTO_ARM mode) -----------------
    if not AUTO_ARM:
        t_lost_start = None
        cover_cnt = 0
        arm_flag = 0
    elif stop_reason is not None:
        arm_flag = 0
    elif elapsed_s < WAIT_BEFORE_ARM_SEC:
        # Ignore lost/cover history during initial 3s countdown
        t_lost_start = None
        cover_cnt = 0
        arm_flag = 0
    else:
        if max_bg <= COVER_STOP_BG:
            cover_cnt += 1
        else:
            cover_cnt = 0

        if t_armed is None:
            t_armed = now_ms
        run_s = time.ticks_diff(now_ms, t_armed) / 1000.0
        if cover_cnt >= COVER_STOP_FRAMES:
            stop_reason = "COVER"
            arm_flag = 0
        elif (LATCH_LOST_SEC > 0) and (t_lost_start is not None) and (time.ticks_diff(now_ms, t_lost_start) / 1000.0 >= LATCH_LOST_SEC):
            stop_reason = "END"
            arm_flag = 0
        elif (MAX_RUN_SEC > 0) and (run_s >= MAX_RUN_SEC):
            stop_reason = "TIME"
            arm_flag = 0
        else:
            arm_flag = 1

    # --- HUD + LED status --------------------------------------------------
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s" % stop_reason, color=255)
        blink = int(elapsed_s * 6) % 2
        set_led(1, 0, blink)
    elif AUTO_ARM and elapsed_s < WAIT_BEFORE_ARM_SEC:
        img.draw_string(2, 2, "WAIT %.1fs" % (WAIT_BEFORE_ARM_SEC - elapsed_s), color=255)
        set_led(0, 0, int(elapsed_s * 5) % 2)
    elif valid == 2:
        img.draw_string(2, 2, "JUNC e=%d" % err_out, color=255)
        set_led(0, 1, 1)
    elif valid == 1:
        tag = "DUAL" if LANE_MODE == "dual" else "LINE"
        img.draw_string(2, 2, "%s e=%d a=%d" % (tag, err_out, ang_out), color=255)
        set_led(0, 1, 0)
    else:
        img.draw_string(2, 2, "LOST", color=255)
        set_led(1, 0, 0)

    send_line_pkt(err_out, ang_out, valid, arm_flag)
