# OpenMV autonomous cube finder & close-range visual servo sender for Carbot(C5).
# Works with STM32 cube_pick.c over USART3 (@O frames @ 115200 8N1).
#
# Output packet (never starts with '#' so ZMotorD ignores it):
#   @O,<dx>,<dy>,<w>,<valid>,<arm>*<XX>\n
#     dx    : -1000..+1000  (<0 cube left of gripper center, >0 right)
#     dy    : -1000..+1000  (>0 cube too far ahead, <0 too close)
#     w     : detected cube pixel width in QVGA (0..320)
#     valid : 0=not found, 1=compact black cube, 2=AprilTag verified
#     arm   : 0=bench/dry preview / cover-stop, 1=armed for STM32 auto pick
#
# Target pixel constants calibrated directly from VID_20261121_004058_grab_screen.mp4:
#   At exact floor pickup reach, the 3cm black cube sits near (x=160, bottom_y=218)
#   inside QVGA (320x240) GRAYSCALE.

import time
import sensor
import image
from pyb import UART, LED

# --- user setup ------------------------------------------------------------

AUTO_ARM            = True    # True -> arms STM32 auto pick after countdown
WAIT_BEFORE_ARM_SEC = 3.0     # Blue LED countdown before arm=1
MAX_RUN_SEC         = 35.0    # Max run window (covers search + align + grab)
COVER_STOP_BG       = 14      # Pitch-black covered lens stop threshold
COVER_STOP_FRAMES   = 12      # ~0.4s dark cover (avoids false trip during grab)

FLIP_Y              = False
MIRROR_X            = False

IMG_W               = 320
IMG_H               = 240
FLOOR_ROI           = (0, 124, 320, 116)   # Below door/chair horizon

# Sweet-spot pickup coordinates in QVGA (320x240)
TARGET_CX           = 160     # Dead-center between green jaws
TARGET_BOT_Y        = 218     # ~22px above bottom border (deep jaw pocket)
NOMINAL_CUBE_H      = 32      # Used if cube clips past bottom edge y=239

# Candidate filter bounds
MIN_BOX_PX          = 12
MAX_BOX_PX          = 82
MAX_CUBE_GRAY       = 115     # Cube interior must stay genuinely dark
MIN_RING_CONTRAST   = 28      # Surrounding floor must be brighter than cube

TAG_FAMILIES        = image.TAG36H11 | image.TAG25H9 | image.TAG16H5

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
sensor.set_framesize(sensor.QVGA)
try:
    sensor.set_brightness(0)
    sensor.set_contrast(1)
except Exception:
    pass
if FLIP_Y:
    sensor.set_vflip(True)
if MIRROR_X:
    sensor.set_hmirror(True)
sensor.skip_frames(time=700)

# --- helpers ---------------------------------------------------------------

def send_cube_pkt(dx, dy, w, valid, arm):
    body = "O,%d,%d,%d,%d,%d" % (dx, dy, w, valid, arm)
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    uart.write("@%s*%02X\n" % (body, csum))

def ring_ok(img, b):
    bx, by, bw, bh = b.rect()
    # Reject dark clutter flush against left/right borders
    if bx <= 2 or (bx + bw) >= (IMG_W - 2):
        return False

    # Use lower quartile so a bright white AprilTag in the center never fails darkness check
    cube_dark = img.get_statistics(roi=(bx, by, bw, bh)).l_lq()
    if cube_dark > MAX_CUBE_GRAY:
        return False

    drops = []
    if by >= 10:
        top_m = img.get_statistics(roi=(bx, max(0, by - 8), bw, 6)).l_mean()
        drops.append(top_m - cube_dark)
    if bx >= 10:
        left_m = img.get_statistics(roi=(bx - 8, by, 6, bh)).l_mean()
        drops.append(left_m - cube_dark)
    if (bx + bw) <= (IMG_W - 10):
        right_m = img.get_statistics(roi=(bx + bw + 2, by, 6, bh)).l_mean()
        drops.append(right_m - cube_dark)

    if len(drops) < 2:
        return False
    return min(drops) >= MIN_RING_CONTRAST

def find_best_cube(img):
    st = img.get_statistics(roi=FLOOR_ROI)
    bg_med = st.l_median()
    dk = st.l_min()
    if (bg_med - dk) < MIN_RING_CONTRAST:
        return None, bg_med

    # Scale threshold from floor median so dark corner shadows cannot wash out the cube
    thr_hi = max(55, min(MAX_CUBE_GRAY, int(bg_med * 0.55)))

    blobs = img.find_blobs([(0, thr_hi)], roi=FLOOR_ROI,
                           pixels_threshold=100, area_threshold=120,
                           merge=True, margin=4)
    cands = []
    for b in blobs:
        bw = b.w()
        bh = b.h()
        if bw < MIN_BOX_PX or bw > MAX_BOX_PX or bh < MIN_BOX_PX or bh > MAX_BOX_PX:
            continue
        ratio = float(bw) / float(max(1, bh))
        if ratio < 0.55 or ratio > 1.80:
            continue
        if (b.pixels() * 100) < (bw * bh * 30):
            continue
        if not ring_ok(img, b):
            continue
        score = b.pixels() - int(2.0 * abs(b.cx() - TARGET_CX))
        cands.append((score, b))

    if not cands:
        return None, bg_med

    cands.sort(key=lambda item: item[0], reverse=True)
    best_b = cands[0][1]
    bx, by, bw, bh = best_b.rect()
    cx = best_b.cx()
    top_y = by
    bot_y = by + bh
    valid = 1

    # Confirm AprilTag inside tight candidate ROI when lighting permits
    tx0 = max(0, bx - 6)
    ty0 = max(0, by - 6)
    tw = min(IMG_W - tx0, bw + 12)
    th = min(IMG_H - ty0, bh + 12)
    try:
        tags = img.find_apriltags(roi=(tx0, ty0, tw, th), families=TAG_FAMILIES)
    except Exception:
        tags = []
    if tags:
        t = tags[0]
        r = t.rect() if callable(t.rect) else t.rect
        cx = t.cx() if callable(t.cx) else t.cx
        top_y = r[1]
        bot_y = r[1] + r[3]
        bw = r[2]
        valid = 2

    return (cx, top_y, bot_y, bw, bh, valid, (bx, by, bw, bh)), bg_med

# --- main loop -------------------------------------------------------------

t_boot = time.ticks_ms()
t_armed = None
clock = time.clock()
dx_filt = 0.0
dy_filt = 0.0
has_filt = False
cover_cnt = 0
stop_reason = None

while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, t_boot) / 1000.0

    hit, bg_med = find_best_cube(img)

    # Draw floor ROI and calibrated target grab crosshair
    img.draw_rectangle(FLOOR_ROI, color=180)
    img.draw_cross(TARGET_CX, TARGET_BOT_Y, color=255, size=6)

    if hit is not None:
        cx, top_y, bot_y, bw, bh, valid, box_rect = hit
        eff_bot = bot_y if bot_y < (IMG_H - 3) else (top_y + max(bh, NOMINAL_CUBE_H))
        dx_raw = ((cx - TARGET_CX) * 1000.0) / float(IMG_W // 2)
        dy_raw = ((TARGET_BOT_Y - eff_bot) * 1000.0) / float(IMG_H // 2)

        if not has_filt:
            dx_filt = dx_raw
            dy_filt = dy_raw
            has_filt = True
        else:
            dx_filt = 0.70 * dx_raw + 0.30 * dx_filt
            dy_filt = 0.70 * dy_raw + 0.30 * dy_filt

        dx_out = int(max(-1000, min(1000, round(dx_filt))))
        dy_out = int(max(-1000, min(1000, round(dy_filt))))
        w_out = int(bw)

        img.draw_rectangle(box_rect, color=255)
        img.draw_cross(int(cx), int(eff_bot), color=255, size=5)
        img.draw_line(int(cx), int(eff_bot), TARGET_CX, TARGET_BOT_Y, color=200)
    else:
        has_filt = False
        dx_out = 0
        dy_out = 0
        w_out = 0
        valid = 0

    # --- safety latch checks -----------------------------------------------
    if not AUTO_ARM:
        cover_cnt = 0
        arm_flag = 0
    elif stop_reason is not None:
        arm_flag = 0
    elif elapsed_s < WAIT_BEFORE_ARM_SEC:
        cover_cnt = 0
        arm_flag = 0
    else:
        if bg_med <= COVER_STOP_BG:
            cover_cnt += 1
        else:
            cover_cnt = 0

        if t_armed is None:
            t_armed = now_ms
        run_s = time.ticks_diff(now_ms, t_armed) / 1000.0
        if cover_cnt >= COVER_STOP_FRAMES:
            stop_reason = "COVER"
            arm_flag = 0
        elif (MAX_RUN_SEC > 0) and (run_s >= MAX_RUN_SEC):
            stop_reason = "TIME"
            arm_flag = 0
        else:
            arm_flag = 1

    # --- HUD + LED ---------------------------------------------------------
    if stop_reason is not None:
        img.draw_string(2, 2, "STOP:%s" % stop_reason, color=255)
        set_led(1, 0, int(elapsed_s * 6) % 2)
    elif AUTO_ARM and elapsed_s < WAIT_BEFORE_ARM_SEC:
        img.draw_string(2, 2, "WAIT %.1fs" % (WAIT_BEFORE_ARM_SEC - elapsed_s), color=255)
        set_led(0, 0, int(elapsed_s * 5) % 2)
    elif valid == 2:
        img.draw_string(2, 2, "TAG dx=%d dy=%d" % (dx_out, dy_out), color=255)
        set_led(0, 1, 1)
    elif valid == 1:
        img.draw_string(2, 2, "CUBE dx=%d dy=%d" % (dx_out, dy_out), color=255)
        set_led(0, 1, 0)
    else:
        img.draw_string(2, 2, "SEARCH", color=255)
        set_led(1, 0, 0)

    send_cube_pkt(dx_out, dy_out, w_out, valid, arm_flag)
