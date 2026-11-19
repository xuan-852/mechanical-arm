# OpenMV Run History (`openmv/`)

Local test videos (`*.mp4`, gitignored) are archived here alongside the corresponding firmware/vision findings so regressions stay easy to compare:

| Video | Symptom Observed | Root Cause | Fix Applied |
| :--- | :--- | :--- | :--- |
| `VID_20261120_064052.mp4` | OpenMV IDE preview immediately latched `STOP:END` right at `3.0 s` countdown end | `t_lost_start` / `cover_cnt` accumulated during `WAIT_BEFORE_ARM_SEC`, and `y=72..88` ROI caught dark horizon above floor | Restricted `ROIS` to `y=81..118`, cleared latch timers during 3s wait, removed blank `any_wide` |
| `VID_20261120_070832.mp4` | Robot started off-center toward right rail (`t=3s`), drifted left across center (`t=4..7s`), then whipped ~90 deg right over the right rail (`t=7..9s`) | (1) Controller underdamped & over-aggressive at `vx=600` (`kp=65, kd=25, k_ang=45, wz_max=520, search_wz=420`); (2) single-rail fallback flipped left rail -> right rail once `cx` crossed center | Tuned PD + turn clamp (`kp=38, kd=160, k_ang=20, wz_max=320, search_wz=320, TRACK_SEARCH_MS=350ms`); prioritized dual-rail ROIs and added continuous single-rail side lock in `tools/openmv_line_track.py` |
| `VID_20261120_073052_pass.mp4` | Clean straight tracking & smooth right-curve turn on dorm dual-rail floor track, buzzer silent | — | Verified baseline (`brightness=0/contrast=2`, scanline shoulder-valley detector `y=(103,99,95)`, damped PD `@600`, `PB5` output LOW) |
