# RP2040RobotController

The firmware controls all 18 servos of the hexapod directly on RP2040. Its
only locomotion path is:

```text
RC PWM (GP0 forward/back, GP1 steering)
  -> DriveController -> BodyVelocityCommand
  -> continuous Cartesian tripod gait -> IK
  -> servo calibration -> PIO/DMA at 50 Hz
```

`+X` is forward, `+Y` left, and `+Z` up. Geometry is defined once in
`include/robot/robot_geometry.hpp`; its leg order is `FR, MR, RR, RL, ML, FL`.
The curved Tibia is currently represented by a 125.24 mm effective straight
link from Tibia axis to foot contact.

The two tripods are `FR/ML/RR` and `FL/MR/RL`, separated by phase 0.5. Stance
targets move opposite to body velocity, including the per-leg yaw term
`-omega x r`. Swing targets follow one continuous raised trajectory.

The neutral Cartesian targets are calculated with FK from the current
`STAND_*_DEG` values in `config/robot_params.txt`. User-tuned RC and stand
values are therefore retained.

## Cartesian gait parameters

```text
GAIT_CYCLE_MS
GAIT_DUTY_FACTOR
GAIT_STEP_HEIGHT_MM
GAIT_STRIDE_MM
MAX_FORWARD_SPEED_MM_S
MAX_YAW_RATE_DEG_S
```

The serial dashboard reports `ENGINE : CARTESIAN`, RC/arming state, velocity,
phase, active swing legs, and IK status. If an IK target or logical joint limit
is invalid, the controller holds the last valid pose rather than producing a
servo command from invalid data.

## Build and first test

Use VS Code task **Clean Rebuild Project**, then verify the generated UF2
timestamp before flashing. Do not flash automatically from the build task.

1. Lift the robot above the floor and prepare a servo-power disconnect.
2. Flash the freshly built UF2, power servos, and let startup reach `STAND`.
3. Apply a very small forward command and inspect all six legs.
4. Test steering separately; each foot must follow its own XY arc.
5. Release sticks and wait for `GAIT : IDLE` with all feet neutral/grounded.
6. Only after the suspended test is correct, place the robot on the floor.

PIO programs, DMA scheduling, GPIO assignment, servo calibration, and the
20 ms servo frame are intentionally independent of locomotion and unchanged.
