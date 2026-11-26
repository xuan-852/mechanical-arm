# Pre-staged competition integration draft:
#   HOME exit (optional) -> outer clockwise line follow -> auto detect side cylinder
#   -> brake -> QVGA visual align -> @J 7-step pick (j4=165.0, clamp=115.0) -> HOME.
#
# Runs completely in QVGA RGB565 (no camera mode-switch blackout):
#   - Line detector samples 160x120-equivalent grid (2x stride) so proven 01_loop PD
#     parameters work identically.
#   - Color cylinder detector watches foreground road margin while tracking.

import math
import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# TUNABLE TRACK + CYLINDER PICK PARAMETERS
# ===========================================================================

REQUIRE_KEY1            = True
START_FROM_HOME         = True     # False = placed directly on black line; True = spin-then-exit HOME
HOME_PRE_TURN_MS        = 1000      # Step 1: spin CCW left ~45-60 deg in place inside HOME
HOME_PRE_TURN_WZ        = 540      # (+wz = CCW left turn in zmotord.c; breaks 7.4V mat friction)
HOME_EXIT_MS            = 1000      # Step 2: blind diagonal forward out of HOME past left border
HOME_EXIT_VX            = 580
HOME_EXIT_WZ            = 60       # Slight gentle CCW curve while merging onto outer oval

# Outer line PD follower (160x120-equivalent scale matching proven 01_loop.py)
VX_CRUISE               = 620
VX_MIN                  = 440
KP                      = 0.42
KD                      = 1.60
K_ANG                   = 2.0
WZ_MAX                  = 440
SEARCH_WZ               = 520
SEARCH_MS               = 1000
LOCK_HOLD_MS            = 600
LATCH_LOST_SEC          = 3.0

MAX_BLACK_GRAY          = 155
MIN_VALLEY_DROP         = 20
MIN_TAPE_W              = 3
MAX_TAPE_W              = 24
WIDE_JUNC_W             = 58
MAX_ANCHOR_JUMP         = 48
SCAN_BANDS              = (
    (96, 0.45),
    (86, 0.35),
    (76, 0.20),
)

# '+' Cross counter (for Stage 2/3 target turn counting)
CROSS_COOLDOWN_MS       = 600
CROSS_CONFIRM_FRAMES    = 2

# Cylinder detector (calibrated from 04_pick_cyl.py + test2.mp4)
MIN_TRACK_BEFORE_PICK_MS = 500     # Ignore HOME corner clutter right after joining outer line
TRIG_ROI                = (0, 105, 320, 135)
TRIG_BOT_Y              = 168      # Trigger stop+align once foreground cylinder reaches here
TRIG_MIN_W              = 24
TRIG_MIN_H              = 30
TRIG_FRAMES             = 2

ENABLE_RED              = True
ENABLE_BLUE             = True
ENABLE_GREEN            = True
ENABLE_BLACK            = True     # Calibrated L<=15 in BLACK_ROI (y>=128) with solidity>=0.45
THR_RED                 = (10, 95, 18, 127, 0, 127)
THR_BLUE                = (10, 90, -35, 35, -128, -18)
THR_GREEN               = (10, 85, -128, -14, 5, 127)
THR_BLACK               = (0, 15, -14, 14, -14, 14)
THR_CROSS               = (0, 44, -18, 18, -18, 18)

FLOOR_ROI               = (0, 110, 320, 130)
BLACK_ROI               = (0, 128, 320, 112)
TARGET_CX               = 136
TARGET_BOT_Y            = 239
TOL_X                   = 12
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
STOP_BEFORE_PICK_MS     = 180
LOST_CYL_TIMEOUT_MS     = 1800
MAX_RUN_SEC             = 50.0
COVER_STOP_L            = 10
COVER_STOP_FRAMES       = 8

PICK_STEPS = (
    ("PRE",     450, 1120, 820, 1060, 1650,  900, 750,  80),
    ("ABOVE",   450, 1600, 680, 1130, 1650,  900, 700,  80),
    ("GRAB",    450, 1799, 792, 1060, 1650,  900, 650, 100),
    ("CLAMP",   450, 1799, 792, 1060, 1650, 1150, 550, 260),
    ("LIFT",    450, 1600, 680, 1130, 1650, 1150, 650,  80),
    ("RETRACT", 450, 1120, 820, 1060, 1650, 1150, 750,  80),
    ("HOME",    450,  918, 900,  990, 1035, 1150, 800,  60),
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
# UNIFIED LINE, CROSS & CYLINDER DETECTORS
# ===========================================================================

def _px_gray(img, x, y):
    r, g, b = img.get_pixel(x, y)
    return (r * 77 + g * 150 + b * 29) >> 8


def find_scanline_tapes(row):
    s_sorted = sorted(row)
    bg_ref = s_sorted[(LINE_W * 85) // 100]

    junc_run = 0
    max_junc_run = 0
    for x in range(15, 145):
        v = row[x]
        if v <= MAX_BLACK_GRAY and (bg_ref - v) >= 24:
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
        img.draw_line(0, y, IMG_W - 1, y, color=(70, 70, 70))
        if not tapes:
            continue
        best = min(tapes, key=lambda t: abs(t[0] - anchor))
        if had_lock and abs(best[0] - anchor) > MAX_ANCHOR_JUMP:
            continue
        cx = best[0]
        anchor = cx
        pts.append((cx, float(cy_q), weight))
        img.draw_cross(int(cx * 2), y, color=(0, 255, 0), size=5)
    return pts, any_wide


def detect_cross(img):
    roi = (0, 145, IMG_W, IMG_H - 145)
    blobs = img.find_blobs([THR_CROSS], roi=roi, x_stride=2, y_stride=2,
                           pixels_threshold=80, area_threshold=110)
    for b in blobs:
        x, y, w, h = b.rect()
        if not (16 <= w <= 215 and 12 <= h <= 115 and w <= 4.5 * h and h <= 3.0 * w):
            continue
        if x <= 2 or y <= roi[1] or x + w >= IMG_W - 2 or y + h >= IMG_H - 1:
            continue
        sol = float(b.pixels()) / float(max(1, w * h))
        if sol < 0.20 or sol > 0.62:
            continue
        cx, cy = x + w // 2, y + h // 2
        left, right = x + w // 5, x + 4 * w // 5
        top, bottom = y + h // 5, y + 4 * h // 5
        arm_c = _px_gray(img, cx, cy)
        arm_l = min(_px_gray(img, left, max(0, min(IMG_H - 1, cy + dy))) for dy in (-4, 0, 4))
        arm_r = min(_px_gray(img, right, max(0, min(IMG_H - 1, cy + dy))) for dy in (-4, 0, 4))
        arm_t = min(_px_gray(img, max(0, min(IMG_W - 1, cx + dx)), top) for dx in (-4, 0, 4))
        arm_b = min(_px_gray(img, max(0, min(IMG_W - 1, cx + dx)), bottom) for dx in (-4, 0, 4))
        arms_max = max(arm_c, arm_l, arm_r, arm_t, arm_b)
        corners = sorted([
            _px_gray(img, left, top), _px_gray(img, right, top),
            _px_gray(img, left, bottom), _px_gray(img, right, bottom),
        ])
        if arms_max <= 110 and (corners[1] - arms_max) >= 22:
            return (x, y, w, h)
    return None


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
            [THR_BLACK], roi=blk_roi, pixels_threshold=110, area_threshold=150, merge=True, margin=4
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
            score = bot_y * 1000 + min(999, b.pixels() // 20)
            if score > best_score:
                best_score = score
                best = (b.cx(), by, bot_y, bw, bh, b.pixels(), tag, (bx, by, bw, bh))
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
        ms = 135 if abs(ex) >= 28 else 95
        return False, "PULSE", 0, wz, ms, ex, ey

    if too_close:
        return False, "PULSE", PULSE_VX_REV, 0, 110, ex, ey

    ms = 155 if ey >= 16 else 115
    return False, "PULSE", PULSE_VX_FWD, 0, ms, ex, ey


# ===========================================================================
# MAIN MISSION FSM
# ===========================================================================

link = Stm32Link()
clock = time.clock()
fsm_state = "WAIT_KEY1"
stop_reason = None
seen_disarmed = False

t_run_start = 0
t_stage_start = 0
t_lost_start = None
t_pulse_done = 0
t_pick_pause_until = 0
t_step_start = 0
t_step_wait_until = 0
t_last_cross = 0
t_last_ping = 0
t_last_log = 0

pick_idx = 0
cyl_trig_cnt = 0
cross_streak = 0
cross_count = 0
settle_cnt = 0
cover_cnt = 0
pick_sent_seq = -1
last_log_key = None

last_cx = float(LINE_CX)
had_lock = False
err_filt = 0.0
has_prev_err = False
prev_e_comb = 0.0
last_dir = 0


def reset_line_state():
    global last_cx, had_lock, err_filt, has_prev_err, prev_e_comb, last_dir, t_lost_start
    last_cx = float(LINE_CX)
    had_lock = False
    err_filt = 0.0
    has_prev_err = False
    prev_e_comb = 0.0
    last_dir = 0
    t_lost_start = None


def first_active_state():
    if not START_FROM_HOME:
        return "FOLLOW_SEARCH"
    if HOME_PRE_TURN_MS > 0:
        return "HOME_PRE_TURN"
    if HOME_EXIT_MS > 0:
        return "LEAVE_HOME"
    return "FOLLOW_SEARCH"


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

    pts, any_wide = detect_line(img, last_cx, had_lock)
    if pts:
        t_lost_start = None
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
        if t_lost_start is None:
            t_lost_start = now_ms
        elif time.ticks_diff(now_ms, t_lost_start) >= LOCK_HOLD_MS:
            had_lock = False
        err_out = int(max(-1000, min(1000, round(err_filt))))
        ang_out = 0
        line_valid = 0

    cyl_hit = detect_cylinder(img, FLOOR_ROI if fsm_state == "ALIGN" else TRIG_ROI)
    cross_box = detect_cross(img) if fsm_state == "FOLLOW_SEARCH" else None
    if cross_box is not None:
        img.draw_rectangle(cross_box, color=(255, 0, 255))
        cross_streak += 1
        if (cross_streak >= CROSS_CONFIRM_FRAMES and
                time.ticks_diff(now_ms, t_last_cross) >= CROSS_COOLDOWN_MS):
            cross_count += 1
            t_last_cross = now_ms
    else:
        cross_streak = 0

    # Target overlay
    img.draw_rectangle(TARGET_CX - TOL_X, TOL_Y_NEAR, TOL_X * 2, IMG_H - TOL_Y_NEAR, color=(0, 180, 0))
    img.draw_cross(TARGET_CX, TARGET_BOT_Y, color=(0, 255, 0), size=8)
    if cyl_hit is not None:
        cx, top_y, bot_y, bw, bh, pix, ctag, rect = cyl_hit
        img.draw_rectangle(rect, color=(0, 255, 255))
        img.draw_string(rect[0], max(15, rect[1] - 12), "%s c%d b%d" % (ctag, cx, bot_y), color=(0, 255, 255))

    is_armed = (link_ok and seen_disarmed and link.armed) if REQUIRE_KEY1 else True
    vx_cmd = 0
    wz_cmd = 0

    if not is_armed:
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        cyl_trig_cnt = 0
        cross_count = 0
        cross_streak = 0
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
            t_stage_start = now_ms
            cover_cnt = 0
            cyl_trig_cnt = 0
            cross_count = 0
            cross_streak = 0
            settle_cnt = 0
            pick_idx = 0
            reset_line_state()
            if link.arm_busy:
                fsm_state = "WAIT_ARM_WAKE"
            else:
                fsm_state = first_active_state()
        else:
            if cover_cnt >= COVER_STOP_FRAMES:
                stop_reason = "COVER"
            elif (MAX_RUN_SEC > 0) and (fsm_state in ("HOME_PRE_TURN", "LEAVE_HOME", "FOLLOW_SEARCH", "ALIGN")) and (
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
                t_stage_start = now_ms
                fsm_state = first_active_state()

        elif fsm_state == "HOME_PRE_TURN":
            if time.ticks_diff(now_ms, t_stage_start) >= HOME_PRE_TURN_MS:
                t_stage_start = now_ms
                if HOME_EXIT_MS > 0:
                    fsm_state = "LEAVE_HOME"
                else:
                    reset_line_state()
                    fsm_state = "FOLLOW_SEARCH"
            else:
                wz_cmd = HOME_PRE_TURN_WZ
                send_vel(0, 0, wz_cmd, 0)

        elif fsm_state == "LEAVE_HOME":
            if time.ticks_diff(now_ms, t_stage_start) >= HOME_EXIT_MS:
                t_stage_start = now_ms
                reset_line_state()
                fsm_state = "FOLLOW_SEARCH"
            else:
                vx_cmd = HOME_EXIT_VX
                wz_cmd = HOME_EXIT_WZ
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "FOLLOW_SEARCH":
            track_ms = time.ticks_diff(now_ms, t_stage_start)
            if (track_ms >= MIN_TRACK_BEFORE_PICK_MS and cyl_hit is not None and
                    cyl_hit[2] >= TRIG_BOT_Y and cyl_hit[3] >= TRIG_MIN_W and cyl_hit[4] >= TRIG_MIN_H):
                cyl_trig_cnt += 1
            else:
                cyl_trig_cnt = 0

            if cyl_trig_cnt >= TRIG_FRAMES:
                send_vel(0, 0, 0, 0)
                t_pick_pause_until = time.ticks_add(now_ms, STOP_BEFORE_PICK_MS)
                fsm_state = "BRAKE_CHECK"
            elif line_valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_lost_start) if t_lost_start is not None else 0
                if lost_ms / 1000.0 >= LATCH_LOST_SEC:
                    stop_reason = "LOST"
                    fsm_state = "LATCH_STOP"
                    send_vel(0, 0, 0, 0)
                elif last_dir != 0 and lost_ms <= SEARCH_MS:
                    wz_cmd = SEARCH_WZ if last_dir < 0 else -SEARCH_WZ
                    send_vel(0, 0, wz_cmd, 0)
                else:
                    send_vel(0, 0, 0, 0)
            else:
                e_comb = max(-400.0, min(400.0, 0.5 * err_out + 0.5 * ang_out * K_ANG)) if line_valid == 2 else (
                    max(-1200.0, min(1200.0, err_out + ang_out * K_ANG)))
                d_err = max(-120.0, min(120.0, e_comb - prev_e_comb)) if has_prev_err else 0.0
                prev_e_comb = e_comb
                has_prev_err = True
                wz_cmd = int(max(-WZ_MAX, min(WZ_MAX, round(-(KP * e_comb + KD * d_err)))))
                vx_cmd = int(round(VX_CRUISE - (min(1000.0, abs(e_comb)) * max(0, VX_CRUISE - VX_MIN)) / 1000.0))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "BRAKE_CHECK":
            send_vel(0, 0, 0, 0)
            if time.ticks_diff(now_ms, t_pick_pause_until) >= 0:
                t_lost_start = None
                settle_cnt = 0
                t_pulse_done = 0
                fsm_state = "ALIGN"

        elif fsm_state == "ALIGN":
            if cyl_hit is None:
                send_vel(0, 0, 0, 0)
                settle_cnt = 0
                if t_lost_start is None:
                    t_lost_start = now_ms
                elif time.ticks_diff(now_ms, t_lost_start) >= LOST_CYL_TIMEOUT_MS:
                    stop_reason = "NO CYL"
                    fsm_state = "LATCH_STOP"
            else:
                t_lost_start = None
                cx, top_y, bot_y, bw, bh, _, _, _ = cyl_hit
                aligned, cmd_t, vx, wz, p_ms, ex, ey = compute_align_cmd(cx, top_y, bot_y)
                if aligned:
                    send_vel(0, 0, 0, 0)
                    if time.ticks_diff(now_ms, t_pulse_done) >= 0:
                        settle_cnt += 1
                    if settle_cnt >= ALIGN_SETTLE_FRAMES:
                        t_pick_pause_until = time.ticks_add(now_ms, STOP_BEFORE_PICK_MS)
                        pick_idx = 0
                        fsm_state = "SETTLE_PICK"
                else:
                    settle_cnt = 0
                    if cmd_t == "CRUISE":
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, 0)
                    elif time.ticks_diff(now_ms, t_pulse_done) >= PULSE_COOLDOWN_MS:
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, p_ms)
                        t_pulse_done = time.ticks_add(now_ms, p_ms)

        elif fsm_state == "SETTLE_PICK":
            send_vel(0, 0, 0, 0)
            if time.ticks_diff(now_ms, t_pick_pause_until) >= 0:
                pick_idx = 0
                fsm_state = "ARM_SEND"

        elif fsm_state == "ARM_SEND":
            send_vel(0, 0, 0, 0)
            if pick_idx >= len(PICK_STEPS):
                stop_reason = "PICKED"
                fsm_state = "LATCH_STOP"
            else:
                _, j0, j1, j2, j3, j4, j5, dur_ms, _ = PICK_STEPS[pick_idx]
                pick_sent_seq = link.arm_seq
                t_step_start = now_ms
                send_joints(j0, j1, j2, j3, j4, j5, dur_ms)
                fsm_state = "ARM_WAIT"

        elif fsm_state == "ARM_WAIT":
            _, _, _, _, _, _, _, dur_ms, dwell_ms = PICK_STEPS[pick_idx]
            elapsed_ms = time.ticks_diff(now_ms, t_step_start)
            if ((link.arm_seq != pick_sent_seq and not link.arm_busy and elapsed_ms >= int(dur_ms * 0.7)) or
                    elapsed_ms >= max(dur_ms + 900, 1800)):
                t_step_wait_until = time.ticks_add(now_ms, dwell_ms)
                fsm_state = "ARM_DWELL"

        elif fsm_state == "ARM_DWELL":
            if time.ticks_diff(now_ms, t_step_wait_until) >= 0:
                pick_idx += 1
                fsm_state = "ARM_SEND"

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    # HUD & logs
    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s X=%d" % (stop_reason, cross_count), color=(255, 255, 255))
        set_led(0, 1, 0) if stop_reason == "PICKED" else set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        msg = "NO STM32" if not link_ok else ("DISARM KEY1" if link.armed else "READY KEY1 e=%d" % err_out)
        img.draw_string(2, 2, msg, color=(255, 255, 255))
        set_led(1, (now_ms // 300) % 2, 0) if not link_ok else set_led(0, 0, (now_ms // 250) % 2)
    elif fsm_state.startswith("ARM_"):
        img.draw_string(2, 2, "PICK %d/7 %s" % (pick_idx + 1, PICK_STEPS[min(pick_idx, 6)][0]), color=(255, 255, 0))
        set_led(0, 1, 1)
    else:
        img.draw_string(2, 2, "%s X=%d e%d v%d" % (fsm_state[:6], cross_count, err_out, vx_cmd), color=(255, 255, 255))
        set_led(0, 1, 0)

    log_key = (fsm_state, stop_reason, cross_count, cyl_hit is not None)
    if log_key != last_log_key or time.ticks_diff(now_ms, t_last_log) >= 500:
        print("RUN %s link=%d arm=%d X=%d e=%d v=%d wz=%d cyl=%s" % (
            fsm_state, link_ok, is_armed, cross_count, err_out, vx_cmd, wz_cmd,
            ("%s@(%d,%d)" % (cyl_hit[6], cyl_hit[0], cyl_hit[2])) if cyl_hit else "None"
        ))
        last_log_key = log_key
        t_last_log = now_ms
