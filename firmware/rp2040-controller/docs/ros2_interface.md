# ROS 2 Interface

The future ROS 2 driver is a translation layer between ROS messages and the RP2040 UART protocol. It does not run the gait loop and does not send servo pulse widths.

```text
/cmd_vel
    |
hexapod_driver_node
    |
UART
    |
RP2040 LocomotionSystem
```

## Command Mapping

`geometry_msgs/Twist /cmd_vel` maps to `MotionCommand`:

```text
linear.x  -> vx_mps
linear.y  -> vy_mps
angular.z -> wz_radps
```

Body pose, gait, stand/stop, and mode commands will use dedicated UART command message types. The RP2040 keeps all external command units in SI: meters, meters per second, radians, and radians per second.

Current command limits:

```text
linear.x  = +/-0.440 m/s
linear.y  = +/-0.355 m/s
angular.z = +/-3.0 rad/s
```

## UART Protocol

Frames use COBS with a zero delimiter and CRC16-CCITT over the decoded header and payload:

```text
protocol version
message type
sequence
payload length
payload
CRC16
```

Minimum command messages:

- `CMD_VELOCITY`
- `CMD_BODY_POSE`
- `CMD_MODE`
- `CMD_GAIT`
- `PING`

Minimum telemetry messages:

- `STATE`
- `CONTACTS`
- `JOINT_STATE`
- `DIAGNOSTICS`

The parser is non-blocking, uses fixed-size buffers, ignores corrupted frames, and accepts the next valid frame.

## Published ROS Topics

The future driver should publish:

- `/joint_states`
- `/hexapod/contact_states`
- `/hexapod/state`
- `/diagnostics`

Joint states from the RP2040 are commanded/estimated logical joint angles in radians. They are not measured servo positions because the current servos do not provide position feedback.

## ros2_control

Normal locomotion remains:

```text
/cmd_vel -> high-level UART -> MCU gait
```

`ros2_control::SystemInterface` can be added later for joint state representation, diagnostics, calibration, and explicit `DirectJoint` mode. Normal `Walking` and `DirectJoint` must not be active at the same time.

No micro-ROS runs on the RP2040 in this architecture.
