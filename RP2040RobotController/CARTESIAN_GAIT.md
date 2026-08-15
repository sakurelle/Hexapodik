# Cartesian locomotion commissioning

`ROBOT_GEOMETRY` is compiled into the firmware and validated with a
`static_assert`; invalid geometry stops the build. It uses Coxa/Femur lengths
of 44/80 mm and a 125.24 mm effective Tibia link. The effective Tibia is an
approximation for the curved part and is the single place to refine its
mechanical model or zero offset later.

The current standing angles are never replaced by fabricated foot coordinates:
at startup the gait calculates `neutral_foot_positions = FK(STAND_POSE)`.
Each 20 ms control tick generates six targets, solves IK, validates the logical
joint ranges used by `ServoConfig`, then immediately submits the valid pose to
the existing servo layer.

Start/stop uses an internal 250 ms stride ramp. On a stop request, stride ramps
down while phase continues until the current swings land; targets then return
to neutral and the gait becomes `IDLE`. Unreachable trajectory requests reduce
the complete Cartesian stride uniformly, preserving direction rather than
independently clamping coordinates.

For the first hardware test follow the sequence in the README exactly. Never
change the geometry or gait parameters while the robot is standing on the
floor without first repeating the suspended test.
