# Standalone visual cylinder detect -> align -> @J grab test for OpenMV Cam H7 Plus.
# Put a cylinder 20-45 cm in front of the car, preview alignment first, then press Key1.
#
# Uses the frozen STM32 UART3 @V / @J / @S / $E protocol (robot.hex untouched).
# Target pixels & red LAB bounds calibrated directly from the real QVGA (320x240)
# grab-reach frame on this robot (j4=165.0 horizontal wrist, clamp j5=115.0).

import time
import sensor
import image
from pyb import UART, LED

# ===========================================================================
# TUNABLE CYLINDER ALIGNMENT & PICK PARAMETERS
# ===========================================================================

REQUIRE_KEY1        = True     # True = preview only until Key1 arms STM32 ($E armed=1)
DRY_PREVIEW_ONLY    = False    # True = never send @V/@J even if armed (HUD debug only)

IMG_W               = 320
IMG_H               = 240
FLOOR_ROI           = (0, 95, 320, 145)   # Excludes upper shoes/chair/horizon clutter

# Sweet-spot cylinder position in QVGA when sitting inside open 'grab' jaws:
# Cylinder bottom should just touch or be clipped slightly by bottom edge (y=236..240),
# while staying centered between the two open jaws (TARGET_CX ~ 136).
TARGET_CX           = 136      # Centered between left/right green jaw tips in QVGA
TARGET_BOT_Y        = 239      # Bottom edge of QVGA (236..240 = touching/slightly clipped)
TOL_X               = 12       # Lateral alignment window (+/- px around TARGET_CX)
TOL_Y_NEAR          = 236      # Must at least reach bottom edge (never grab unclipped!)
TOL_Y_FAR           = 240      # Clamped at bottom border of 320x240 image
MAX_CLIP_TOP_Y      = 192      # If bot_y>=238 and top_y>192, drove too far over cylinder -> back up

# Cylinder color thresholds in CIE LAB (calibrated for dim indoor room & bright mat):
#   R: Red cylinder (measured median L=22, a=33, b=21)
#   B: Blue cylinder
#   K: Black cylinder (enable only on bright white mat; keep False on dark indoor tiles)
ENABLE_RED          = True
ENABLE_BLUE         = True
ENABLE_BLACK        = False

THR_RED             = (10, 95, 18, 127, 0, 127)
THR_BLUE            = (10, 90, -35, 35, -128, -18)
THR_BLACK           = (0, 22, -16, 16, -16, 16)

MIN_PIXELS          = 140
MIN_AREA            = 180
MIN_W               = 16
MAX_W               = 135
MIN_H               = 22
MAX_H               = 165
MIN_BOT_Y           = 115

# Mecanum alignment speeds & pulse sizing (tuned for 7.4V battery)
CRUISE_VX           = 560      # Approach speed when far (bot_y < FINE_ENTER_Y)
CRUISE_KP_WZ        = 3.5      # Turn gain toward TARGET_CX while approaching
CRUISE_MAX_WZ       = 460
FINE_ENTER_Y        = 212      # Switch from continuous cruise to crisp timed pulses
PULSE_VX_FWD        = 580
PULSE_VX_REV        = -560
PULSE_WZ_MAG        = 560
PULSE_COOLDOWN_MS   = 170      # Wait after pulse stops before reading next sharp frame

ALIGN_SETTLE_FRAMES = 3        # Consecutive aligned frames before triggering arm pick
STOP_BEFORE_PICK_MS = 180
LOST_TIMEOUT_MS     = 1800
MAX_RUN_SEC         = 45.0
COVER_STOP_L        = 10       # Mean L* <= 10 for 8 frames stops safely
COVER_STOP_FRAMES   = 8

AUTO_PUT_AFTER_PICK = True     # True = hold briefly at HOME, then auto run 7-step put-down
PUT_PAUSE_MS        = 900      # Hold duration at HOME before putting cylinder back down

FLIP_Y              = False
MIRROR_X            = False

# Calibrated 6-axis poses (0.1 deg, exact match to tools/grab_poses.json):
#   (name, j0, j1, j2, j3, j4, j5, dur_ms, dwell_ms)
PICK_STEPS = (
    ("PRE",     450, 1120, 820, 1060, 1650,  900, 750,  80),
    ("ABOVE",   450, 1600, 680, 1130, 1650,  900, 700,  80),
    ("GRAB",    450, 1799, 792, 1060, 1650,  900, 650, 100),
    ("CLAMP",   450, 1799, 792, 1060, 1650, 1150, 550, 260),
    ("LIFT",    450, 1600, 680, 1130, 1650, 1150, 650,  80),
    ("RETRACT", 450, 1120, 820, 1060, 1650, 1150, 750,  80),
    ("HOME",    450,  918, 900,  990, 1035, 1150, 800,  60),
)

PLACE_STEPS = (
    ("PRE",     450, 1120, 820, 1060, 1650, 1150, 750,  80),
    ("ABOVE",   450, 1600, 680, 1130, 1650, 1150, 700,  80),
    ("PLACE",   450, 1780, 792, 1060, 1650, 1150, 650, 120),
    ("RELEASE", 450, 1780, 792, 1060, 1650,  900, 500, 160),
    ("LIFT",    450, 1600, 680, 1130, 1650,  900, 650,  80),
    ("RETRACT", 450, 1120, 820, 1060, 1650,  900, 750,  80),
    ("HOME",    450,  918, 900,  990, 1035,  900, 800,  60),
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
if FLIP_Y:
    sensor.set_vflip(True)
if MIRROR_X:
    sensor.set_hmirror(True)
sensor.skip_frames(time=900)


def _send_packet(body):
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    uart.write("@%s*%02X\n" % (body, csum))


def send_vel(vx, vy=0, wz=0, ms=0):
    _send_packet("V,%d,%d,%d,%d" % (int(vx), int(vy), int(wz), int(ms)))


def send_joints(j0, j1, j2, j3, j4, j5, dur_ms=0):
    # Exec service reads dur_ms from 8th CSV field (j[7]); 7th field is reserved 0.
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


def build_thresholds():
    thrs = []
    labels = []
    if ENABLE_RED:
        thrs.append(THR_RED)
        labels.append("R")
    if ENABLE_BLUE:
        thrs.append(THR_BLUE)
        labels.append("B")
    if ENABLE_BLACK:
        thrs.append(THR_BLACK)
        labels.append("K")
    return thrs, labels


THRESHOLDS, COLOR_LABELS = build_thresholds()


def detect_cylinder(img):
    if not THRESHOLDS:
        return None
    blobs = img.find_blobs(
        THRESHOLDS,
        roi=FLOOR_ROI,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
        margin=6,
    )
    best = None
    best_score = -1
    for b in blobs:
        bx, by, bw, bh = b.rect()
        bot_y = by + bh
        if bw < MIN_W or bw > MAX_W or bh < MIN_H or bh > MAX_H:
            continue
        if bot_y < MIN_BOT_Y:
            continue
        ratio = float(bh) / float(max(1, bw))
        min_ratio = 0.25 if bot_y >= 236 else 0.50
        if ratio < min_ratio or ratio > 3.20:
            continue
        # Decode single-color blob label when code is a power of two
        code = b.code()
        tag = "C"
        for idx, lbl in enumerate(COLOR_LABELS):
            if code == (1 << idx):
                tag = lbl
                break
        # Prefer closest foreground cylinder (highest bot_y, tie-break by area)
        score = bot_y * 1000 + min(999, b.pixels() // 20)
        if score > best_score:
            best_score = score
            best = (b.cx(), by, bot_y, bw, bh, b.pixels(), tag, (bx, by, bw, bh))
    return best


def compute_align_cmd(cx, top_y, bot_y):
    """Return (aligned, cmd_type, vx, wz, pulse_ms, ex, ey).
    cmd_type in {'STOP', 'CRUISE', 'PULSE'}.
    """
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
        wz = int(max(-CRUISE_MAX_WZ, min(CRUISE_MAX_WZ, round(-CRUISE_KP_WZ * ex))))
        return False, "CRUISE", CRUISE_VX, wz, 0, ex, ey

    # Close-range crisp pulses: correct yaw first if far off-center, else step forward/back
    if not in_x:
        wz = -PULSE_WZ_MAG if ex > 0 else PULSE_WZ_MAG
        ms = 135 if abs(ex) >= 28 else 95
        return False, "PULSE", 0, wz, ms, ex, ey

    if too_close:
        return False, "PULSE", PULSE_VX_REV, 0, 110, ex, ey

    ms = 155 if ey >= 16 else 115
    return False, "PULSE", PULSE_VX_FWD, 0, ms, ex, ey


# ===========================================================================
# MAIN LOOP
# ===========================================================================

link = Stm32Link()
clock = time.clock()
fsm_state = "WAIT_KEY1"
stop_reason = None
seen_disarmed = False
t_run_start = 0
t_lost_start = None
t_pulse_done = 0
t_pick_pause_until = 0
t_step_start = 0
t_step_wait_until = 0
pick_idx = 0
pick_sent_seq = -1
arm_phase = "PICK"
active_steps = PICK_STEPS
t_put_pause_until = 0
settle_cnt = 0
cover_cnt = 0
t_last_ping = 0
t_last_log = 0
last_log_key = None

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

    hit = detect_cylinder(img)

    # Cover-lens safety check
    st_roi = img.get_statistics(roi=(40, 80, 240, 120))
    if st_roi.l_mean() <= COVER_STOP_L:
        cover_cnt += 1
    else:
        cover_cnt = 0

    # Draw visual overlay
    img.draw_rectangle(FLOOR_ROI, color=(90, 90, 90))
    img.draw_rectangle(
        TARGET_CX - TOL_X,
        TOL_Y_NEAR,
        TOL_X * 2,
        IMG_H - TOL_Y_NEAR,
        color=(0, 180, 0),
    )
    img.draw_cross(TARGET_CX, TARGET_BOT_Y, color=(0, 255, 0), size=8)

    if hit is not None:
        cx, top_y, bot_y, bw, bh, pix, ctag, rect = hit
        img.draw_rectangle(rect, color=(0, 255, 255))
        img.draw_cross(cx, bot_y, color=(255, 255, 0), size=6)
        img.draw_line(cx, bot_y, TARGET_CX, TARGET_BOT_Y, color=(255, 160, 0))
        img.draw_string(
            rect[0], max(14, rect[1] - 12),
            "%s c%d b%d w%d" % (ctag, cx, bot_y, bw),
            color=(0, 255, 255),
        )
    else:
        cx = 0
        top_y = 0
        bot_y = 0
        bw = 0
        bh = 0

    is_armed = (link_ok and seen_disarmed and link.armed) if REQUIRE_KEY1 else True
    vx_cmd = 0
    wz_cmd = 0

    if not is_armed:
        if fsm_state != "WAIT_KEY1":
            send_vel(0, 0, 0, 0)
        fsm_state = "WAIT_KEY1"
        stop_reason = None
        settle_cnt = 0
        t_lost_start = None
        pick_idx = 0
        arm_phase = "PICK"
        active_steps = PICK_STEPS
        if time.ticks_diff(now_ms, t_last_ping) >= 250:
            send_sys(1, 0)
            t_last_ping = now_ms
    else:
        if stop_reason is not None:
            fsm_state = "LATCH_STOP"
        elif fsm_state == "WAIT_KEY1":
            t_run_start = now_ms
            settle_cnt = 0
            t_lost_start = None
            t_pulse_done = 0
            cover_cnt = 0
            pick_idx = 0
            arm_phase = "PICK"
            active_steps = PICK_STEPS
            fsm_state = "WAIT_ARM_WAKE" if link.arm_busy else "ALIGN"
        else:
            if cover_cnt >= COVER_STOP_FRAMES:
                stop_reason = "COVER"
            elif (MAX_RUN_SEC > 0) and (fsm_state == "ALIGN") and (
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
                fsm_state = "ALIGN"

        elif fsm_state == "ALIGN":
            if hit is None:
                send_vel(0, 0, 0, 0)
                settle_cnt = 0
                if t_lost_start is None:
                    t_lost_start = now_ms
                elif time.ticks_diff(now_ms, t_lost_start) >= LOST_TIMEOUT_MS:
                    stop_reason = "NO CYL"
                    fsm_state = "LATCH_STOP"
            else:
                t_lost_start = None
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
                    if DRY_PREVIEW_ONLY:
                        vx_cmd, wz_cmd = vx, wz
                    elif cmd_t == "CRUISE":
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, 0)
                    elif time.ticks_diff(now_ms, t_pulse_done) >= PULSE_COOLDOWN_MS:
                        vx_cmd, wz_cmd = vx, wz
                        send_vel(vx, 0, wz, p_ms)
                        t_pulse_done = time.ticks_add(now_ms, p_ms)

        elif fsm_state == "SETTLE_PICK":
            send_vel(0, 0, 0, 0)
            if time.ticks_diff(now_ms, t_pick_pause_until) >= 0:
                if DRY_PREVIEW_ONLY:
                    stop_reason = "DRY ALIGN"
                    fsm_state = "LATCH_STOP"
                else:
                    arm_phase = "PICK"
                    active_steps = PICK_STEPS
                    pick_idx = 0
                    fsm_state = "ARM_SEND"

        elif fsm_state == "HOLD_BEFORE_PUT":
            send_vel(0, 0, 0, 0)
            if time.ticks_diff(now_ms, t_put_pause_until) >= 0:
                arm_phase = "PUT"
                active_steps = PLACE_STEPS
                pick_idx = 0
                fsm_state = "ARM_SEND"

        elif fsm_state == "ARM_SEND":
            send_vel(0, 0, 0, 0)
            if pick_idx >= len(active_steps):
                if (arm_phase == "PICK") and AUTO_PUT_AFTER_PICK:
                    t_put_pause_until = time.ticks_add(now_ms, PUT_PAUSE_MS)
                    fsm_state = "HOLD_BEFORE_PUT"
                else:
                    stop_reason = "DONE" if arm_phase == "PUT" else "PICKED"
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
            seq_bumped = (link.arm_seq != pick_sent_seq)
            step_done = False
            if seq_bumped and (not link.arm_busy) and (elapsed_ms >= int(dur_ms * 0.7)):
                step_done = True
            elif elapsed_ms >= max(dur_ms + 900, 1800):
                step_done = True

            if step_done:
                t_step_wait_until = time.ticks_add(now_ms, dwell_ms)
                fsm_state = "ARM_DWELL"

        elif fsm_state == "ARM_DWELL":
            if time.ticks_diff(now_ms, t_step_wait_until) >= 0:
                pick_idx += 1
                fsm_state = "ARM_SEND"

        elif fsm_state == "LATCH_STOP":
            send_vel(0, 0, 0, 0)

    # HUD bar & status LEDs
    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s (KEY1)" % stop_reason, color=(255, 255, 255))
        set_led(0, 1, 0) if stop_reason in ("PICKED", "DONE") else set_led(1, 0, (now_ms // 180) % 2)
    elif fsm_state == "WAIT_KEY1":
        if not link_ok:
            msg = "NO STM32"
            set_led(1, (now_ms // 300) % 2, 0)
        elif link.armed:
            msg = "DISARM KEY1"
            set_led(1, 0, 0)
        else:
            msg = "READY KEY1"
            set_led(0, 0, (now_ms // 250) % 2)
        if hit is not None:
            msg += " dx=%d dy=%d" % (cx - TARGET_CX, TARGET_BOT_Y - bot_y)
        img.draw_string(2, 2, msg, color=(255, 255, 255))
    elif fsm_state.startswith("ARM_"):
        sname = active_steps[min(pick_idx, len(active_steps) - 1)][0]
        img.draw_string(2, 2, "%s %d/7 %s" % (arm_phase, pick_idx + 1, sname), color=(255, 255, 0))
        set_led(0, 1, 1)
    elif fsm_state == "HOLD_BEFORE_PUT":
        img.draw_string(2, 2, "HOLD -> PUT", color=(0, 255, 255))
        set_led(0, 1, 1)
    else:
        info = "%s v%d w%d" % (fsm_state[:5], vx_cmd, wz_cmd)
        if hit is not None:
            info += " dx=%d dy=%d" % (cx - TARGET_CX, TARGET_BOT_Y - bot_y)
        img.draw_string(2, 2, info, color=(255, 255, 255))
        set_led(0, 1, 0)

    log_key = (fsm_state, stop_reason, link_ok, is_armed, hit is not None)
    if log_key != last_log_key or time.ticks_diff(now_ms, t_last_log) >= 400:
        if hit is not None:
            print("CYL %s link=%d arm=%d cx=%d bot=%d w=%d h=%d dx=%d dy=%d v=%d wz=%d" % (
                fsm_state, link_ok, is_armed, cx, bot_y, bw, bh,
                cx - TARGET_CX, TARGET_BOT_Y - bot_y, vx_cmd, wz_cmd))
        else:
            print("CYL %s link=%d arm=%d stop=%s no_cyl" % (
                fsm_state, link_ok, is_armed, stop_reason))
        last_log_key = log_key
        t_last_log = now_ms
