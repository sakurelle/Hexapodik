# RP2040RobotController

Real-time C++17 firmware for a six-legged robot on Waveshare RP2040-Plus using Raspberry Pi Pico SDK, CMake, RC PWM input, contact sensors, analytical kinematics, continuous Cartesian tripod gait, and PIO-owned servo pins.

## Build and Flash

Firmware build with the Raspberry Pi Pico SDK extension or CMake presets:

```powershell
cmake --preset pico-release
cmake --build --preset pico-release --target RP2040RobotController
```

The default board is `pico`, which is appropriate for Waveshare RP2040-Plus because this firmware only depends on the standard RP2040 peripherals used by Pico SDK.

Manual firmware build:

```powershell
$env:PICO_SDK_PATH="C:\path\to\pico-sdk"
cmake -S . -B build-pico -G "Ninja"
cmake --build build-pico
```

Flash by copying `build/RP2040RobotController.uf2` to the RP2040 BOOTSEL drive, or use the Pico extension's Run Project workflow.

## GPIO

`GP0` is RC forward/back PWM. `GP1` is RC yaw PWM. Servo outputs are the contiguous PIO bus `GP2..GP19`.

Servo and contact mapping:

```text
FL: GP2 Coxa,  GP3 Femur,  GP4 Tibia,  GP28 Contact
ML: GP5 Coxa,  GP6 Femur,  GP7 Tibia,  GP27 Contact
RL: GP8 Coxa,  GP9 Femur,  GP10 Tibia, GP26 Contact
RR: GP11 Coxa, GP12 Femur, GP13 Tibia, GP22 Contact
MR: GP14 Coxa, GP15 Femur, GP16 Tibia, GP21 Contact
FR: GP17 Coxa, GP18 Femur, GP19 Tibia, GP20 Contact
```

Contact inputs are `HIGH = contact`. External pull-up hardware is assumed; firmware does not enable an internal pull-down.

## Calibration

Servo calibration is centralized in `src/config/servo_config.cpp`.

Initial values:

```text
center_us = 1500
min_pulse_us = 1000
max_pulse_us = 2000
us_per_degree = 1000 / 150
FL/ML/RL direction = +1
FR/MR/RR direction = -1
```

Calibrate every servo's `center_us`, `direction`, `us_per_degree`, `min_pulse_us`, and `max_pulse_us` on the physical robot before loading the legs.

## RC

Default PWM ranges are in `src/config/robot_config.h`:

```text
MIN 1000 us
CENTER 1500 us
MAX 2000 us
DEADBAND 50 us
FAILSAFE 100 ms
vx max 80 mm/s
yaw max 0.45 rad/s
```

`CommandSource` keeps locomotion independent from RC input so UART0 command control can be added later.

## Gait

Tripod A is `FR + ML + RR`; Tripod B is `FL + MR + RL`, with phase offset `0.5`.

Key parameters:

```text
control loop 200 Hz
servo frame 50 Hz / 20000 us
stance duty 0.65
cycle frequency 0.6..1.5 Hz
swing height 30 mm
ground search 15 mm
contact debounce 12 ms
support margin 5 mm
```

The stance model integrates Cartesian foot velocity from body twist:

```text
footVx = -vx + yawRate * footY
footVy = -vy - yawRate * footX
```

## Faults

```text
NONE
FAULT_NO_GROUND
FAULT_CONTACT_STUCK
FAULT_IK_INVALID
FAULT_SUPPORT_LOST
```

On invalid IK or unsafe workspace, the controller keeps the last valid servo target. On latched contact faults, the gait enters `FAULT` and stops starting new swing phases.

## Tests

Host-side tests cover FK/IK round trips, neutral pose, invalid targets, gait continuity and tripod offset, contact debounce/faults, support polygon, and servo mapping:

```powershell
cmake -S tests -B build-host-tests
cmake --build build-host-tests
ctest --test-dir build-host-tests --output-on-failure
```

The host tests are intentionally separate from the Pico firmware CMake project so native test tooling cannot interfere with firmware configure/build in VS Code.
