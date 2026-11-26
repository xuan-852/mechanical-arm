# Venue-ready passive multi-probe viewer for OpenMV Cam H7 Plus (NEVER drives robot).
# Run in OpenMV IDE on the real white competition mat to inspect in 2 minutes:
#   1. 20mm black line error & '+' cross candidates (X count)
#   2. Red / Blue / Black cylinder blob detection in RGB565 QVGA
#   3. Live CIE LAB stats inside CENTER sample box vs MAT reference box
#      (place white cylinder in yellow center box to read exact white-vs-mat LAB delta!)

import time
import sensor
import image
from pyb import LED

IMG_W               = 320
IMG_H               = 240
FLOOR_ROI           = (0, 110, 320, 130)
BLACK_ROI           = (0, 128, 320, 112)   # Isolates black cylinder below dark horizon wall

# Sampling boxes for rapid on-mat calibration (white cylinder vs white mat)
CYL_SAMPLE_ROI      = (124, 165, 28, 36)   # Yellow box: put target cylinder here
MAT_SAMPLE_ROI      = (250, 185, 32, 32)   # Gray box: clean white mat beside line

# Cylinder thresholds (verified across all 10 real mat photos: R/B/G/K + W flank)
THR_RED             = (10, 95, 18, 127, 0, 127)
THR_BLUE            = (10, 90, -35, 35, -128, -18)
THR_GREEN           = (10, 85, -128, -14, 5, 127)
THR_BLACK           = (0, 15, -14, 14, -14, 14)
THR_CROSS           = (0, 44, -18, 18, -18, 18)

# Line scan & '+' cross thresholds (160x120 coordinates * 2 inside QVGA)
MAX_BLACK_GRAY      = 155
MIN_VALLEY_DROP     = 20
MIN_TAPE_W          = 3
MAX_TAPE_W          = 24
WIDE_JUNC_W         = 58
SCAN_BANDS          = (96, 86, 76)         # In 160x120 space -> y = 192, 172, 152

led_b = LED(3)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
try:
    sensor.set_auto_whitebal(False)
except Exception:
    pass
sensor.skip_frames(time=900)


def px_gray(img, x, y):
    r, g, b = img.get_pixel(x, y)
    return (r * 77 + g * 150 + b * 29) >> 8


def scan_line_row(img, cy_q):
    y = cy_q * 2
    row = []
    for xq in range(160):
        row.append(px_gray(img, xq * 2, y))
    s_sorted = sorted(row)
    bg_med = s_sorted[80]
    tapes = []
    m = 3
    while m <= 156:
        v = row[m]
        if (v <= MAX_BLACK_GRAY and v <= row[m - 1] and v <= row[m + 1] and
                v <= row[m - 2] and v <= row[m + 2]):
            lb = max(row[max(0, m - 14) : m - 1])
            rb = max(row[m + 2 : min(160, m + 15)])
            ref = lb if lb < rb else rb
            if (ref - v) >= MIN_VALLEY_DROP:
                half = 0.5 * (ref + v)
                x0 = m
                while x0 > 0 and row[x0 - 1] <= half:
                    x0 -= 1
                x1 = m
                while x1 < 159 and row[x1 + 1] <= half:
                    x1 += 1
                bw = x1 - x0 + 1
                if MIN_TAPE_W <= bw <= MAX_TAPE_W and x0 >= 1 and x1 <= 158:
                    tapes.append((0.5 * (x0 + x1), x0, x1))
                    m = x1 + 3
                    continue
        m += 1
    return tapes, bg_med


def detect_crosses_qvga(img):
    roi = (0, 145, 320, 95)
    blobs = img.find_blobs([THR_CROSS], roi=roi, x_stride=2, y_stride=2,
                           pixels_threshold=80, area_threshold=110)
    hits = []
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
        arm_c = px_gray(img, cx, cy)
        arm_l = min(px_gray(img, left, max(0, min(IMG_H - 1, cy + dy))) for dy in (-4, 0, 4))
        arm_r = min(px_gray(img, right, max(0, min(IMG_H - 1, cy + dy))) for dy in (-4, 0, 4))
        arm_t = min(px_gray(img, max(0, min(IMG_W - 1, cx + dx)), top) for dx in (-4, 0, 4))
        arm_b = min(px_gray(img, max(0, min(IMG_W - 1, cx + dx)), bottom) for dx in (-4, 0, 4))
        arms_max = max(arm_c, arm_l, arm_r, arm_t, arm_b)
        corners = sorted([
            px_gray(img, left, top), px_gray(img, right, top),
            px_gray(img, left, bottom), px_gray(img, right, bottom),
        ])
        if arms_max <= 110 and (corners[1] - arms_max) >= 22:
            hits.append((x, y, w, h))
    return hits


def detect_white_cyl_hints(img):
    """Detect 3D white cylinder via shaded flank vs adjacent bright highlight column."""
    hits = []
    for y_mid in (155, 175, 195):
        col_g = []
        col_min = []
        for x in range(0, IMG_W, 4):
            s = 0
            mn = 255
            for dy in (-8, -4, 0, 4, 8):
                v = px_gray(img, x, y_mid + dy)
                s += v
                if v < mn:
                    mn = v
            col_g.append(s // 5)
            col_min.append(mn)
        ref = sorted(col_g)[(len(col_g) * 3) // 4]
        i = 2
        n = len(col_g)
        while i < n - 6:
            if col_min[i] >= 95 and 18 <= (ref - col_g[i]) <= 68:
                pk = max(col_g[max(0, i - 9) : min(n, i + 10)])
                if (pk - col_g[i]) >= 42 and pk >= max(175, ref + 10):
                    cx = i * 4
                    top_ok = min(px_gray(img, cx, y_mid - 16), px_gray(img, cx, y_mid - 10)) >= 95
                    if top_ok:
                        hits.append((cx, y_mid))
                        i += 6
                        continue
            i += 1
    return hits


clock = time.clock()
t_last_log = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    now_ms = time.ticks_ms()
    if (now_ms // 400) % 2:
        led_b.on()
    else:
        led_b.off()

    cyl_blobs = img.find_blobs(
        [THR_RED, THR_BLUE, THR_GREEN],
        roi=FLOOR_ROI,
        pixels_threshold=140,
        area_threshold=180,
        merge=False,
    )
    blk_blobs = img.find_blobs(
        [THR_BLACK],
        roi=BLACK_ROI,
        pixels_threshold=110,
        area_threshold=150,
        merge=False,
    )
    cyls = []
    for tag_set, blist in ((("R", "B", "G"), cyl_blobs), (("K",), blk_blobs)):
        for b in blist:
            bx, by, bw, bh = b.rect()
            bot_y = by + bh
            if bw < 14 or bw > 135 or bh < 18 or bh > 165:
                continue
            sol = float(b.pixels()) / float(max(1, bw * bh))
            if sol < 0.45:
                continue
            if len(tag_set) == 1:
                tag = "K"
            else:
                code = b.code()
                tag = "R" if code == 1 else ("B" if code == 2 else "G")
            ratio = float(bh) / float(max(1, bw))
            min_ratio = 0.70 if tag == "K" else (0.25 if bot_y >= 236 else 0.50)
            if (tag == "K" and bw > 95) or ratio < min_ratio or ratio > 3.2:
                continue
            cyls.append((tag, b.cx(), bot_y, bw, bh, (bx, by, bw, bh)))

    white_hints = detect_white_cyl_hints(img)
    crosses = detect_crosses_qvga(img)

    line_err = None
    for cy_q in SCAN_BANDS:
        tapes, _ = scan_line_row(img, cy_q)
        img.draw_line(0, cy_q * 2, IMG_W - 1, cy_q * 2, color=(90, 90, 90))
        if tapes:
            best_t = min(tapes, key=lambda t: abs(t[0] - 80.0))
            cx_px = int(best_t[0] * 2)
            img.draw_cross(cx_px, cy_q * 2, color=(0, 255, 0), size=5)
            if line_err is None:
                line_err = int(round((best_t[0] - 80.0) * 1000.0 / 80.0))

    st_cyl = img.get_statistics(roi=CYL_SAMPLE_ROI)
    st_mat = img.get_statistics(roi=MAT_SAMPLE_ROI)

    # Draw overlays
    img.draw_rectangle(FLOOR_ROI, color=(80, 80, 80))
    img.draw_rectangle(CYL_SAMPLE_ROI, color=(255, 255, 0))
    img.draw_rectangle(MAT_SAMPLE_ROI, color=(180, 180, 180))
    for rect in crosses:
        img.draw_rectangle(rect, color=(255, 0, 255))
    for wcx, wcy in white_hints:
        img.draw_cross(wcx, wcy, color=(255, 255, 255), size=7)
    for tag, cx, bot_y, bw, bh, rect in cyls:
        if tag == "R":
            col = (255, 80, 80)
        elif tag == "B":
            col = (80, 160, 255)
        elif tag == "G":
            col = (80, 255, 120)
        else:
            col = (255, 200, 0)
        img.draw_rectangle(rect, color=col)
        img.draw_string(rect[0], max(15, rect[1] - 12), "%s c%d b%d" % (tag, cx, bot_y), color=col)

    img.draw_rectangle(0, 0, IMG_W, 14, color=(0, 0, 0), fill=True)
    img.draw_string(
        2, 2,
        "PROBE C=%d X=%d e=%s | C(%d,%d,%d)" % (
            len(cyls), len(crosses), str(line_err),
            st_cyl.l_mean(), st_cyl.a_mean(), st_cyl.b_mean()
        ),
        color=(255, 255, 255),
    )

    if time.ticks_diff(now_ms, t_last_log) >= 500:
        print("PROBE cyl=%s cross=%d line_e=%s | C_LAB=(%d,%d,%d) MAT_LAB=(%d,%d,%d)" % (
            [(c[0], c[1], c[2], c[3]) for c in cyls],
            len(crosses), str(line_err),
            st_cyl.l_mean(), st_cyl.a_mean(), st_cyl.b_mean(),
            st_mat.l_mean(), st_mat.a_mean(), st_mat.b_mean(),
        ))
        t_last_log = now_ms
