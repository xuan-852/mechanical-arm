# Leave Key1 disarmed and move the camera over crosses by hand.
# Generated from tools/openmv_comp.py by tools/build_openmv_field_tests.py.
# OpenMV Cam H7 Plus upper-layer brain for the KPZ-STM32 frozen execution layer.
# Saves onto OpenMV internal flash as F:/main.py (or runs from OpenMV IDE).
#
# 4-wire connection (KPZ32 top-left yellow header <-> OpenMV):
#   OpenMV P4  (UART3 TX) -> STM32 PB11 (RX) : sends @V / @A / @J / @S packets
#   OpenMV P5  (UART3 RX) <- STM32 PB10 (TX) : reads $E telemetry (ignores #... motor traffic)
#   OpenMV GND            -> GND
#   OpenMV VIN            -> 5V
#
# Quick flow on the track:
#   1. Power robot: LED blinks BLUE (WAIT_KEY1) while streaming live HUD preview.
#   2. Put robot on line (or in HOME box with HOME_EXIT_MS > 0).
#   3. Press onboard Key1 (PA8) (or run `python tools/robot_host.py --port COMxx exec run`):
#      STM32 wakes arm to upright HOME, sets armed=1, OpenMV takes off!
#   4. Press Key1 again anytime (or cover lens) to brake cleanly and reset.

import math
import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# TUNABLE TRACK PARAMETERS (all upper-layer logic lives here; STM32 stays frozen)
# ===========================================================================

MISSION_MODE        = "LOOP"   # Night 1: loop test around mat
FIELD_TEST          = "CROSS_VIEW"     # Replaced by build_openmv_field_tests.py for standalone tests
LANE_MODE           = "single" # Competition mat has one 20mm oval track loop; "dual" = dorm two-rail corridor
REQUIRE_KEY1        = True     # True = wait for STM32 Key1 / PC `exec start` ($E armed=1)

# Startup exit from 320x320mm HOME box:
#   0   = robot starts placed on the 20mm black oval line
#   450 = robot starts centered inside HOME facing forward onto track
HOME_EXIT_MS        = 0
HOME_EXIT_VX        = 580

# PD follower (QQVGA 160x120 GRAYSCALE, floor bands y=76..96)
# Gains mirror line_track.c (kp=38..42 / 100 = 0.42, kd=160 / 100 = 1.60),
# with speeds tuned for 7.4V battery to break vinyl-mat static friction.
VX_CRUISE           = 640      # straight speed (-950..950)
VX_MIN              = 440      # curve slow-down speed (lets corner turn tighten cleanly)
KP                  = 0.42     # lateral error P gain -> wz (matches line_track.c 0.38 scale)
KD                  = 1.60     # error derivative D damping (matches line_track.c 1.60 scale)
K_ANG               = 2.0      # lookahead curve slope weight (deg -> err equiv)
WZ_MAX              = 440      # max steering turn rate
SEARCH_WZ           = 520      # lost-line in-place spin rate (breaks 7.4V mat friction)
SEARCH_MS           = 1000     # max spin search window before braking
LOCK_HOLD_MS        = 600      # keep anchor lock across brief gaps so '+' never steals lock
LATCH_LOST_SEC      = 3.0      # latch STOP:LOST if line missing this long (0 disables latch)

# Black tape detector vs mat markings (2mm bullseye rings & lighting)
MAX_BLACK_GRAY      = 155      # ceiling grayscale darkness accepted as black tape
MIN_VALLEY_DROP     = 20       # tape must be darker than BOTH nearby floor shoulders
MIN_TAPE_W          = 3        # rejects <=2px noise and thin 2mm scoring rings
MAX_TAPE_W          = 24       # allows diagonal corner slices while rejecting crossbars
WIDE_JUNC_W         = 58       # wide horizontal border / HOME edge threshold (px)
MIN_LANE_PX         = 72       # dual-rail minimum separation (avoids pairing rail with center '+')
SINGLE_RAIL_BIAS    = 0.50     # dual fallback offset fraction when 1 rail temporarily disappears
MAX_ANCHOR_JUMP     = 48       # single-mode max lateral px jump allowed when locked

# Floor scan bands (y, weight, nominal_dual_lane_px), bottom-to-top (horizon ~y=56)
SCAN_BANDS = (
    (96, 0.45, 130.0),  # near floor
    (86, 0.35, 112.0),  # mid floor
    (76, 0.20,  94.0),  # far lookahead
)

# Safety limits
MAX_RUN_SEC         = 45.0     # max runtime per launch (0 = unlimited loops)
COVER_STOP_BG       = 14       # covered dark lens threshold
COVER_STOP_FRAMES   = 8        # consecutive covered frames to stop

FLIP_Y              = False
MIRROR_X            = False

# Competition cube grab/place presets (adjustable per venue cube size without reflashing STM32)
CUBE_PICK_J0        = 450      # 0.1 deg (45.0 deg straight center)
CUBE_PICK_J1        = 1716     # 0.1 deg (lower arm floor grab height)
CUBE_CLAMP_J5       = 1319     # 0.1 deg (1966 us grip for 3cm cube)
CUBE_PLACE_J1       = 1700     # 0.1 deg (gentle drop height)
CUBE_OPEN_J5        = 900      # 0.1 deg (1500 us full open)

IMG_W               = 160
IMG_H               = 120
IMG_CX              = IMG_W // 2

# ===========================================================================
# HARDWARE & STM32 FROZEN PROTOCOL BRIDGE
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


def _send_packet(body):
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    uart.write("@%s*%02X\n" % (body, csum))


def send_vel(vx, vy=0, wz=0, ms=0):
    _send_packet("V,%d,%d,%d,%d" % (int(vx), int(vy), int(wz), int(ms)))


def send_arm_op(op, j0=-1, j1=-1, j5=-1):
    _send_packet("A,%d,%d,%d,%d" % (int(op), int(j0), int(j1), int(j5)))


def send_joints(j0, j1, j2, j3, j4, j5, dur_ms=0):
    _send_packet("J,%d,%d,%d,%d,%d,%d,%d" % (
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
        # ZMotorD frames (#006P...!) contain no '\n', so a preceding motor
        # burst may sit at the start of raw before $E,...*XX.
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
            got_csum = int(pkt[star + 1 :].decode(), 16)
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
        return self.has_rx and time.ticks_diff(now_ms, self.last_rx_ms) <= 800


# ===========================================================================
# 20mm BLACK LINE SCANLINE DETECTOR
# ===========================================================================

def sample_band_row(img, cy):
    return [
        (img.get_pixel(x, cy - 1) + img.get_pixel(x, cy) + img.get_pixel(x, cy + 1)) // 3
        for x in range(IMG_W)
    ]


def find_scanline_tapes(row):
    s_sorted = sorted(row)
    bg_med = s_sorted[IMG_W // 2]

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


def detect_line(img, anchor_cx, had_lock):
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
    anchor = anchor_cx
    for idx, (cy, weight, tapes) in enumerate(band_scans):
        img.draw_line(0, cy - 2, IMG_W - 1, cy - 2, color=160)
        img.draw_line(0, cy + 2, IMG_W - 1, cy + 2, color=160)
        for _, _, x0, x1 in tapes:
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
            if not tapes:
                continue
            best = min(tapes, key=lambda t: abs(t[0] - anchor))
            if had_lock and abs(best[0] - anchor) > MAX_ANCHOR_JUMP:
                continue
            cx = best[0]
            anchor = cx
            dual_pts.append((cx, float(cy), weight))
            img.draw_cross(int(cx), cy, color=255, size=5)

    pts = dual_pts if dual_pts else single_pts
    if not dual_pts:
        for cx, cy, _ in single_pts:
            img.draw_cross(int(cx), int(cy), color=210, size=3)

    return pts, any_wide, max_bg


def cross_candidates(img):
    """Preview isolated '+' candidates without issuing any drive commands."""
    roi = (0, 64, IMG_W, IMG_H - 64)
    blobs = img.find_blobs([(0, MAX_BLACK_GRAY)], roi=roi, x_stride=1, y_stride=1,
                           pixels_threshold=16, area_threshold=30)
    candidates = []
    for blob in blobs:
        x, y, w, h = blob.rect()
        if not (6 <= w <= 34 and 5 <= h <= 28 and w <= 4 * h and h <= 3 * w):
            continue
        if x == 0 or y == roi[1] or x + w >= IMG_W or y + h >= IMG_H:
            continue

        left, cx, right = x + w // 5, x + w // 2, x + 4 * w // 5
        top, cy, bottom = y + h // 5, y + h // 2, y + 4 * h // 5
        arms = ((cx, cy), (left, cy), (right, cy), (cx, top), (cx, bottom))
        corners = ((left, top), (right, top), (left, bottom), (right, bottom))
        if all(img.get_pixel(px, py) <= MAX_BLACK_GRAY for px, py in arms) and all(
                img.get_pixel(px, py) > MAX_BLACK_GRAY for px, py in corners):
            candidates.append((x, y, w, h))
    return candidates


# ===========================================================================
# UPPER-LAYER FSM
# ===========================================================================

link = Stm32Link()
clock = time.clock()

# Clear any boot motor/RX garbage
if FIELD_TEST != "CROSS_VIEW":
    uart.read(uart.any()) if uart.any() else None
    send_sys(1, 0)

if FIELD_TEST:
    print("FIELD %s KP=%.2f KD=%.2f HOME_EXIT_MS=%d" % (
        FIELD_TEST, KP, KD, HOME_EXIT_MS))
field_seen_disarmed = False
field_home_line_frames = 0
field_last_event = None
field_last_log = None

fsm_state = "WAIT_KEY1"
t_run_start = 0
t_exit_start = 0
t_lost_start = None
t_last_ping = 0
cover_cnt = 0
stop_reason = None

last_cx = float(IMG_CX)
roi_lx = [float(IMG_CX) - 0.5 * b[2] for b in SCAN_BANDS]
roi_rx = [float(IMG_CX) + 0.5 * b[2] for b in SCAN_BANDS]
roi_lw = [b[2] for b in SCAN_BANDS]
single_side = [0, 0, 0]
single_cx = [None, None, None]
had_lock = False
err_filt = 0.0
has_prev_err = False
prev_e_comb = 0.0
last_dir = 0


def reset_tracker():
    global last_cx, roi_lx, roi_rx, roi_lw, single_side, single_cx
    global had_lock, err_filt, has_prev_err, prev_e_comb, last_dir, t_lost_start
    last_cx = float(IMG_CX)
    roi_lx = [float(IMG_CX) - 0.5 * b[2] for b in SCAN_BANDS]
    roi_rx = [float(IMG_CX) + 0.5 * b[2] for b in SCAN_BANDS]
    roi_lw = [b[2] for b in SCAN_BANDS]
    single_side = [0, 0, 0]
    single_cx = [None, None, None]
    had_lock = False
    err_filt = 0.0
    has_prev_err = False
    prev_e_comb = 0.0
    last_dir = 0
    t_lost_start = None


while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    if FIELD_TEST == "CROSS_VIEW":
        candidates = cross_candidates(img)
        for rect in candidates:
            img.draw_rectangle(*rect, color=255)
        img.draw_rectangle(0, 0, IMG_W, 13, color=0, fill=True)
        img.draw_string(2, 2, "CROSS VIEW n=%d" % len(candidates), color=255)
        if field_last_log is None or time.ticks_diff(now_ms, field_last_log) >= 500:
            print("CROSS t=%d boxes=%s" % (now_ms, candidates))
            field_last_log = now_ms
        continue

    link.poll(now_ms)
    link_ok = link.link_ok(now_ms)
    if FIELD_TEST:
        # A switched script must observe a fresh disarmed state before it can drive.
        # Allow the firmware's up-to-360-ms staggered servo wake before gating again.
        link_ok = link.has_rx and time.ticks_diff(now_ms, link.last_rx_ms) <= 600
        if not link_ok:
            field_seen_disarmed = False
        elif not link.armed:
            field_seen_disarmed = True

    pts, any_wide, max_bg = detect_line(img, last_cx, had_lock)

    if pts:
        t_lost_start = None
        w_sum = sum(w for _, _, w in pts)
        cx_avg = sum(cx * w for cx, _, w in pts) / w_sum
        last_cx = pts[0][0] if len(pts) == 1 else cx_avg
        had_lock = True

        err_raw = ((cx_avg - IMG_CX) * 1000.0) / float(IMG_CX)
        err_filt = 0.75 * err_raw + 0.25 * err_filt
        err_out = int(max(-1000, min(1000, round(err_filt))))

        if abs(err_out) >= 60:
            last_dir = -1 if err_out < 0 else 1

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
        elif time.ticks_diff(now_ms, t_lost_start) >= LOCK_HOLD_MS:
            had_lock = False
        err_out = int(max(-1000, min(1000, round(err_filt))))
        ang_out = 0
        valid = 0

    vx_cmd = 0
    wz_cmd = 0

    # --- arming gate from STM32 Key1 / PC `exec start` ---------------------
    is_armed = link.armed if REQUIRE_KEY1 else True
    if FIELD_TEST:
        is_armed = link_ok and field_seen_disarmed and link.armed
    if not is_armed:
        # Disarming via Key1 clears any stop latch so the next press launches fresh
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        cover_cnt = 0
        field_home_line_frames = 0
        has_prev_err = False
        reset_tracker()
        if time.ticks_diff(now_ms, t_last_ping) >= 250:
            send_sys(1, 0)
            t_last_ping = now_ms
    else:
        if stop_reason is not None:
            fsm_state = "LATCH_STOP"
        elif fsm_state == "WAIT_KEY1":
            reset_tracker()
            cover_cnt = 0
            has_prev_err = False
            t_run_start = now_ms
            if link.arm_busy:
                fsm_state = "WAIT_ARM_WAKE"
            elif HOME_EXIT_MS > 0:
                t_exit_start = now_ms
                fsm_state = "LEAVE_HOME"
            else:
                fsm_state = "FOLLOW_LOOP"

        if fsm_state == "WAIT_ARM_WAKE":
            send_vel(0, 0, 0, 0)
            if not link.arm_busy:
                t_run_start = now_ms
                if HOME_EXIT_MS > 0:
                    t_exit_start = now_ms
                    fsm_state = "LEAVE_HOME"
                else:
                    fsm_state = "FOLLOW_LOOP"

        elif fsm_state == "LEAVE_HOME":
            exit_ms = time.ticks_diff(now_ms, t_exit_start)
            if FIELD_TEST == "HOME_EXIT":
                if exit_ms >= 220 and len(pts) >= 2 and not any_wide and abs(err_out) <= 450:
                    field_home_line_frames += 1
                else:
                    field_home_line_frames = 0
                if max_bg <= COVER_STOP_BG:
                    stop_reason = "COVER"
                elif field_home_line_frames >= 3:
                    stop_reason = "LINE SEEN"
                elif exit_ms >= HOME_EXIT_MS:
                    stop_reason = "EXIT LIMIT"
                else:
                    vx_cmd = HOME_EXIT_VX
                    send_vel(vx_cmd, 0, 0, 0)
                if stop_reason:
                    fsm_state = "LATCH_STOP"
            elif exit_ms >= HOME_EXIT_MS or (exit_ms >= 220 and valid == 1):
                fsm_state = "FOLLOW_LOOP"
            else:
                vx_cmd = HOME_EXIT_VX
                wz_cmd = 0
                send_vel(vx_cmd, 0, wz_cmd, 0)

        if fsm_state == "FOLLOW_LOOP":
            if max_bg <= COVER_STOP_BG:
                cover_cnt += 1
            else:
                cover_cnt = 0

            run_s = time.ticks_diff(now_ms, t_run_start) / 1000.0
            if cover_cnt >= COVER_STOP_FRAMES:
                stop_reason = "COVER"
            elif (MAX_RUN_SEC > 0) and (run_s >= MAX_RUN_SEC):
                stop_reason = "TIME"
            elif valid == 0 and (t_lost_start is not None) and (
                    time.ticks_diff(now_ms, t_lost_start) / 1000.0 >= LATCH_LOST_SEC):
                stop_reason = "LOST"

            if stop_reason is not None:
                fsm_state = "LATCH_STOP"
                send_vel(0, 0, 0, 0)
            elif valid == 0:
                has_prev_err = False
                lost_ms = time.ticks_diff(now_ms, t_lost_start) if t_lost_start is not None else 0
                if last_dir != 0 and lost_ms <= SEARCH_MS:
                    vx_cmd = 0
                    wz_cmd = SEARCH_WZ if last_dir < 0 else -SEARCH_WZ
                else:
                    vx_cmd = 0
                    wz_cmd = 0
                send_vel(vx_cmd, 0, wz_cmd, 0)
            else:
                if valid == 2:
                    e_comb = max(-400.0, min(400.0, 0.5 * err_out + 0.5 * ang_out * K_ANG))
                else:
                    e_comb = max(-1200.0, min(1200.0, err_out + ang_out * K_ANG))

                d_err = 0.0
                if has_prev_err:
                    d_err = max(-120.0, min(120.0, e_comb - prev_e_comb))
                prev_e_comb = e_comb
                has_prev_err = True

                wz_raw = -(KP * e_comb + KD * d_err)
                wz_cmd = int(max(-WZ_MAX, min(WZ_MAX, round(wz_raw))))

                abs_e = min(1000.0, abs(e_comb))
                vx_cmd = int(round(VX_CRUISE - (abs_e * max(0, VX_CRUISE - VX_MIN)) / 1000.0))
                send_vel(vx_cmd, 0, wz_cmd, 0)

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    # --- HUD + LED status --------------------------------------------------
    img.draw_rectangle(0, 0, IMG_W, 13, color=0, fill=True)
    if FIELD_TEST:
        field_event = (fsm_state, stop_reason, link_ok, is_armed)
        if (field_event != field_last_event or field_last_log is None or
                time.ticks_diff(now_ms, field_last_log) >= 500):
            print("%s t=%d %s stop=%s link=%d arm=%d n=%d e=%d a=%d v=%d w=%d" % (
                FIELD_TEST, now_ms, fsm_state, stop_reason, link_ok, is_armed, len(pts),
                err_out, ang_out, vx_cmd, wz_cmd))
            field_last_event = field_event
            field_last_log = now_ms
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s (KEY1)" % stop_reason, color=255)
        set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        if not link_ok:
            img.draw_string(2, 2, "NO STM32 LINK e=%d" % err_out, color=255)
            set_led(1, (now_ms // 300) % 2, 0)
        elif FIELD_TEST and link.armed:
            img.draw_string(2, 2, "DISARM WITH KEY1", color=255)
            set_led(1, 0, 0)
        else:
            img.draw_string(2, 2, "READY KEY1 e=%d v%d" % (err_out, valid), color=255)
            set_led(0, 0, (now_ms // 250) % 2)
    elif FIELD_TEST == "HOME_EXIT" and fsm_state == "LEAVE_HOME":
        img.draw_string(2, 2, "HOME EXIT n%d" % len(pts), color=255)
        set_led(0, 1, 0)
    elif valid == 2:
        img.draw_string(2, 2, "JUNC v%d w%d" % (vx_cmd, wz_cmd), color=255)
        set_led(0, 1, 1)
    elif valid == 1:
        tag = "D" if LANE_MODE == "dual" else "L"
        img.draw_string(2, 2, "%s e%d a%d w%d" % (tag, err_out, ang_out, wz_cmd), color=255)
        set_led(0, 1, 0)
    else:
        img.draw_string(2, 2, "SRCH w%d (%s)" % (wz_cmd, fsm_state[:4]), color=255)
        set_led(1, 1, 0)
