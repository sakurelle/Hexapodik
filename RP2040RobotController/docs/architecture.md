# RP2040 Locomotion Architecture

The RP2040 firmware is the real-time locomotion controller. ROS 2 or RC supplies high-level motion commands only; the 200 Hz gait loop, contact adaptation, workspace checks, IK, and servo output remain on the MCU.

```text
CommandSource
  |
  v
LocomotionSystem
  |
  +-- ContactSource
  +-- GaitGenerator
  +-- Adaptive landing/contact events
  +-- Workspace scale + IK
  +-- ServoOutput
  +-- Telemetry
```

`main.cpp` owns hardware initialization and timing. It does not contain gait policy.

## Hardware Boundaries

`ICommandSource` produces `MotionCommand` in SI units. Current implementations are:

- `RcInput`: GP0 forward PWM, GP1 yaw PWM, `vy = 0`.
- `UartCommandSource`: UART0 binary packets for future Raspberry Pi control.

`IContactSource` produces debounced `ContactState` values. Raw GPIO is only an input to event generation.

`IServoOutput` accepts 18 logical joint targets. `ServoController` preserves the existing ServoConfig angle mapping and PIO/DMA pulse generation.

## Control States

The public robot state vocabulary is:

```cpp
Disabled, Standing, Walking, Stopping, Recovery, Fault
```

Recoverable conditions such as `NoGround`, `SensorStuckHigh`, `SensorStuckLow`, `EarlyCollision`, and `CommandTimeout` drive controlled stop/recovery telemetry rather than a latched hard fault. Hard faults are reserved for non-finite math, impossible IK/internal state, servo output failure, and fatal PIO/DMA failures.

`BlockReason` is reported whenever a non-zero command is not producing walking motion.

## Contact Rules

Contact sensors are event sources:

- LOW -> HIGH: `pressed_event`
- HIGH -> LOW: `released_event`
- Startup HIGH initializes stable contact without generating a pressed event.

Touchdown is accepted only during descent or ground search after the leg has become airborne. Contact during lift is allowed until release timeout/height checks mark the sensor suspect. Contact during horizontal transfer after release is treated as an early collision.

Support is logical: `Stance` or `LandedHold`, not direct raw GPIO.

## Timing

- Locomotion/control: 200 Hz
- Servo output: 50 Hz
- USB diagnostics: 5-10 Hz
- UART and diagnostics are non-blocking with respect to the control loop

## Command Source Mode

`kCommandSourceMode` is currently `CommandSourceMode::RcPwm`. GP0/GP1 are initialized either for RC capture or UART0, never both.
