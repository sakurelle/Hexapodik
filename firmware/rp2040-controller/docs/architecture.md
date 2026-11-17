# Low-level controller architecture

Raspberry Pi 4 owns RC input, MPU6050 processing, gait generation, inverse
kinematics, servo calibration, and mechanical limits. The Pico W is a small
deterministic I/O controller:

```text
Raspberry Pi 4
  -> UART0: ARM / SET_PWM / DISARM / GET_STATUS
Pico W
  -> PIO0 + DMA: 18 independent 50 Hz servo pulses on GP2..GP19
  -> debounced raw input levels: GP20, GP21, GP22, GP26, GP27, GP28
  -> UART0: STATUS
```

Only the low-level controller sources are linked into the firmware target.
Legacy locomotion, IK/FK, RC input, calibration, test modes, Wi-Fi, and USB
stdio are not part of the target.

## Runtime ownership

- `main.cpp` services UART, input debounce, and the communication timeout.
- `LowLevelController` validates complete commands and owns the state machine.
- `ServoController` builds PIO events and swaps all 18 targets at a 20 ms
  frame boundary from the DMA interrupt.
- `ContactSensors` reports a stable six-bit input-level mask. A set bit means
  the external NC input is open/high; it is not proof of physical contact.
- `ProtocolParser` performs COBS framing and CRC validation without dynamic
  allocation.

## Safety states

The controller starts `DISARMED` with every servo GPIO low. A valid `ARM`
contains all 18 pulse widths and enables PWM without inventing a neutral pose.
`SET_PWM` is accepted only while armed and only with a newer sequence number.

If no valid movement command is accepted for 250 ms, the controller enters
`COMMUNICATION_FAULT`. The PIO output continues holding the last applied pulse
set, but further movement commands are rejected. Recovery is deliberately
two-step: `DISARM`, then a new valid `ARM` with all 18 values.

See `uart_protocol.md` for the byte-level contract.
