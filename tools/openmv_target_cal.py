# OpenMV read-only calibration viewer for small pickup cube + large placement pad.
# Run directly in OpenMV IDE (does NOT overwrite F:/main.py or move any motors).
#
# Use this while the robot sits parked at HOME over:
#   1) the small black AprilTag cube placed at your verified grab spot, and
#   2) the large bright/blue target placement pad placed where 'put' drops it.
# Watch the OpenMV IDE Serial Terminal for exact pixel coordinates & sizes!

import time
import sensor
import image

# QVGA floor ROI below room horizon (door/wall stays above y=120)
FLOOR_ROI = (0, 120, 320, 120)

# Small black cube threshold in LAB (low lightness L, tight neutral/cool box)
CUBE_LAB = (0, 34, -25, 25, -25, 25)

# Large bright/cool white-blue target pad threshold in LAB (distinguishes white/blue
# pad from warmer wood floor; tune L_min/B_max using printed floor_stats if needed)
PAD_LAB = (52, 100, -35, 18, -50, 8)

TAG_FAMILIES = 0
for _fam in ("TAG36H11", "TAG25H9", "TAG25H7", "TAG16H5"):
    TAG_FAMILIES |= getattr(image, _fam, 0)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1200)
# Keep auto gain/white-balance steady enough to see tag contrast
clock = time.clock()
last_print = time.ticks_ms()

while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_rectangle(FLOOR_ROI, color=(80, 180, 255))
    img.draw_line(160, 120, 160, 239, color=(60, 60, 60))

    tag_msg = "none"
    tags = []
    if TAG_FAMILIES:
        try:
            tags = img.find_apriltags(roi=FLOOR_ROI, families=TAG_FAMILIES)
        except Exception:
            tags = []
    if tags:
        def _tag_rect(t):
            return t.rect() if callable(t.rect) else t.rect

        def _val(attr):
            return attr() if callable(attr) else attr

        t = max(tags, key=lambda a: _tag_rect(a)[2] * _tag_rect(a)[3])
        tx, ty, tw, th = _tag_rect(t)
        tcx = _val(t.cx)
        tcy = _val(t.cy)
        tid = _val(t.id)
        tfam = _val(t.family)
        img.draw_rectangle((tx, ty, tw, th), color=(0, 255, 0), thickness=2)
        img.draw_cross(tcx, tcy, color=(0, 255, 0))
        tag_msg = "fam=%d id=%d cx=%d cy=%d bot=%d w=%d h=%d" % (
            tfam, tid, tcx, tcy, ty + th, tw, th
        )

    cube_msg = "none"
    c_blobs = img.find_blobs([CUBE_LAB], roi=FLOOR_ROI,
                             pixels_threshold=120, area_threshold=160,
                             merge=True, margin=4)
    c_cands = []
    for b in c_blobs:
        if 14 <= b.w() <= 75 and 14 <= b.h() <= 75:
            ratio = float(b.w()) / float(max(1, b.h()))
            if 0.55 <= ratio <= 1.80:
                c_cands.append(b)
    if c_cands:
        cb = max(c_cands, key=lambda b: b.pixels())
        img.draw_rectangle(cb.rect(), color=(255, 255, 0), thickness=2)
        img.draw_cross(cb.cx(), cb.y() + cb.h(), color=(255, 255, 0))
        cube_msg = "cx=%d top=%d bot=%d w=%d h=%d px=%d" % (
            cb.cx(), cb.y(), cb.y() + cb.h(), cb.w(), cb.h(), cb.pixels()
        )

    pad_msg = "none"
    p_blobs = img.find_blobs([PAD_LAB], roi=FLOOR_ROI,
                             pixels_threshold=1200, area_threshold=1800,
                             merge=True, margin=8)
    p_cands = [b for b in p_blobs if b.w() >= 65 and b.h() >= 25]
    if p_cands:
        pb = max(p_cands, key=lambda b: b.pixels())
        img.draw_rectangle(pb.rect(), color=(0, 140, 255), thickness=2)
        img.draw_cross(pb.cx(), pb.cy(), color=(0, 140, 255))
        pad_msg = "cx=%d cy=%d top=%d bot=%d w=%d h=%d" % (
            pb.cx(), pb.cy(), pb.y(), pb.y() + pb.h(), pb.w(), pb.h()
        )

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 250:
        last_print = now
        st = img.get_statistics(roi=(40, 160, 240, 70))
        print("FPS=%.1f | TAG:[%s] | CUBE:[%s] | PAD:[%s] | LAB=(L%d,A%d,B%d)" % (
            clock.fps(), tag_msg, cube_msg, pad_msg, st.l_mean(), st.a_mean(), st.b_mean()
        ))
