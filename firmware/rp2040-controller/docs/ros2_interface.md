# Raspberry Pi 4 integration

Any ROS 2 node runs on Raspberry Pi 4. It calculates gait and IK, applies servo
calibration and limits, then sends complete 18-channel pulse sets to the Pico W.
There is no micro-ROS or gait calculation in the Pico firmware.

The Pi-side client should:

1. Open UART at 115200 8N1 and begin with `GET_STATUS`.
2. Send `ARM` with all 18 pulse widths in the documented channel order.
3. Send complete `SET_PWM` frames faster than the 250 ms timeout and use a
   monotonically increasing 32-bit sequence number.
4. Poll `GET_STATUS` independently to read state, error, output state, the six
   raw debounced input levels, and the last accepted movement sequence.
5. On any communication fault, send `DISARM` before attempting another `ARM`.

Do not retry a partially constructed set and do not treat a status request as a
watchdog keepalive. The full wire format and checked frame examples are in
`uart_protocol.md`.
