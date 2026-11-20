# Carbot (KPZ-STM32) 6-DOF Arm & OpenMV Mecanum Robot

STM32F103C8T6 (`KPZ-STM32`) firmware, OpenMV H7 Plus vision scripts, and Python host tools for a 4-wheel Mecanum chassis (`ZMotorD`) equipped with a 6-axis MG996R robotic arm.

> **Branch status**
> - `main` — **Hardware-verified** codebase: 6-axis motion planner, calibrated limits, Mecanum drive, OpenMV dual-rail line tracking, and the interactive 7-step cube pickup suite (`tools/arm_grab_test.py`).
> - `feat/auto-cube-pick` — **Work-in-progress** autonomous cube discovery + Mecanum micro-jog alignment + automatic 7-step pickup (`Core/Src/cube_pick.c`, `tools/openmv_cube_pick.py`); builds cleanly and passes host simulation unit tests, pending floor verification.

---

## Hardware Architecture (`KPZ-STM32`)

Full measured pin assignments, wheel polarities, and per-joint pulse limits live in [`CALIBRATION.md`](CALIBRATION.md).

| Subsystem | MCU Pins / Bus | Configuration |
| :--- | :--- | :--- |
| Host Link & ISP Flash | `USART1` (`PA9` TX / `PA10` RX) | `115200 8N1`, CRC8 binary protocol & `FlyMcu` (`MDK-ARM/robot/robot.hex`; J-Link disabled on carrier) |
| Chassis Motors & OpenMV | `USART3` (`PB10` TX / `PB11` RX) | `115200 8N1`; `ZMotorD` IDs `#006` (FR inv), `#007` (FL inv), `#008` (RR fwd), `#009` (RL fwd), millisecond timing; `OpenMV P4 -> RX (PB11)` |
| 6-Axis Arm PWM (`TIM3`) | `PB3, PB8, PB9, PB6, PB7, PB4` | 50 Hz non-overlapping 3 ms slots (`SWJ_NOJTAG`); starts relaxed (`0 us`) at boot, staggered `HOME` wake |
| Onboard Buzzer | `PB5` | Active-HIGH buzzer; initialized `OUTPUT_PP` LOW in `Core/Src/gpio.c` |
| Spare UART (Future Use) | `S3 SA3 (PA2)` / `S4 SA4 (PA3)` | Unallocated STM32 `USART2_TX / USART2_RX` (`3.3V` TTL) |

---

## Repository Layout

```text
Core/
  Inc/, Src/
    bsp_pwm.c       TIM3 6-channel sequential-slot 50Hz servo PWM driver
    servo.c         Per-joint angle <-> pulse mapping, hard limits, Home poses
    robot.c         5ms multi-axis synchronous planner, E-stop, speed override
    cfg_store.c     STM32 flash calibration persistence with magic/CRC
    zmotord.c       ZMotorD packet formatter, Mecanum kinematics, USART3 driver
    line_track.c    OpenMV `@L` line parser, damped PD follower & RX failsafe
    protocol.c      USART1 CRC8 frame parser & command dispatcher
tools/
  robot_host.py     Host CLI (`pose`, `home`, `move`, `drive`, `track`, `--check`)
  arm_grab_test.py  Single-session interactive 6-axis teach/grab/place console
  grab_poses.json   Hardware-verified floor pickup & placement pose presets
  openmv_line_track.py  OpenMV Cam H7 Plus dual-rail line-tracking vision script
  test_core.c       Host-compiled C unit test suite (compiles real Core/Src modules)
  fake/             Minimal HAL/CMSIS stubs for offline host C tests
MDK-ARM/            Keil uVision 5 (`robot.uvprojx`) & EIDE (`eide.yml`) projects
CALIBRATION.md      Verified pinouts, motor IDs, servo limits & assembly notes
openmv/             Local test run recordings ledger (`openmv/README.md`, `.mp4` gitignored)
```

---

## Verified Capabilities

### 1. 6-Axis Arm Motion & Safe Power-On (`Core/Src/servo.c`, `Core/Src/robot.c`)
- Boots with all PWM channels relaxed (`0 us`); `CMD_HOME` wakes servos staggered 60 ms apart to upright `HOME`:
  `[45.0, 91.8, 90.0, 99.0, 103.5, 90.0]` deg (`1000, 1520, 1500, 1600, 1650, 1500` us).
- Enforces rear stop floor on lower arm (`j1 >= 1200 us` / `63.0°`) and gripper range (`j5 in [1500, 2150]` us / `[90.0°, 148.5°]`).
- Hardware-verified 7-step cube pickup (`tools/grab_poses.json`): rotates horizontal (`j4 = 180.0°`), clears front OpenMV via `pre`, hovers at `above`, lowers to `grab` (`[45.0, 171.6, 75.9, 113.0, 180.0, 90.0]`), clamps cube at `j5 = 131.9° (1966 us)`, lifts vertically, and carries upright back to `HOME` (`openmv/VID_20261121_004106.mp4`).

### 2. Dual-Rail Black-Tape Line Tracking (`Core/Src/line_track.c`, `tools/openmv_line_track.py`)
- OpenMV (`160×120 GRAYSCALE`, `brightness=0`, `contrast=2`) runs a 3-row scanline local-shoulder valley detector at `y = (103, 99, 95)` prioritizing dual-rail pairs with single-rail side lock.
- Transmits `@L,<err>,<ang>,<valid>,<arm>*<XX>\n` over `UART3 @ 115200`.
- Built-in safeties: 3 s blue LED arming countdown, hand-cover dark-lens stop latch (`STOP:COVER`), `0.8 s` lost-line latch (`STOP:END`), `15 s` max run timer (`STOP:TIME`), and `350 ms` STM32 RX timeout stop.
- STM32 defaults to `TRACK_MODE_AUTO` with damped PD control (`vx_cruise=600, vx_min=420, kp=38, kd=160, k_ang=20, wz_max=320, search_wz=320`), verified smoothly on curved floor tracks (`openmv/VID_20261120_073052_pass.mp4`).

---

## Quick Start

### Build & Offline Tests
```powershell
# Compile and run C firmware core unit tests (MinGW/MSYS gcc)
gcc -std=c99 -Wall -Wextra -Werror -O1 -DZM_HOST_TEST -Itools/fake -ICore/Inc `
    -o tools/test_core.exe tools/test_core.c Core/Src/servo.c Core/Src/robot.c `
    Core/Src/protocol.c Core/Src/cfg_store.c Core/Src/zmotord.c Core/Src/line_track.c -lm
.\tools\test_core.exe

# Host Python protocol & preset bounds checks
python tools/robot_host.py --check
python tools/arm_grab_test.py --check
python -m py_compile tools/openmv_line_track.py
```

- **Keil Build**: Open `MDK-ARM/robot.uvprojx` (or run `UV4.exe -b MDK-ARM/robot.uvprojx`) to generate `MDK-ARM/robot/robot.hex`, then flash over USB-TTL (`USART1`) using `FlyMcu`.

### Common Robot Commands
```powershell
# Inspect line tracking live over USB without spinning wheels
python tools/robot_host.py --port COM10 track dry
python tools/robot_host.py --port COM10 track monitor

# Drive chassis directly (wheels lifted or clear floor; vx, vy, wz in [-1000, 1000])
python tools/robot_host.py --port COM10 drive 600 0 0 --time 500

# Interactive 6-axis arm grab/place teach console (single COM session prevents MCU reset drop)
python tools/arm_grab_test.py --port COM22
```
