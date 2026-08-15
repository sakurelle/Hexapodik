#pragma once

#include "robot/robot_geometry.hpp"
#include "robot/robot_pose.hpp"

namespace robot {

using FootTarget = Vector3;

struct IkResult {
    bool reachable = false;
    LegPose pose{};
};

// Rigid body/local-leg transform.  Exposed for regression tests and to make
// the mount-yaw convention explicit: body -> subtract mount -> rotate -yaw.
Vector3 body_to_leg_frame(const RobotGeometry &geometry, servo::Leg leg,
                          const FootTarget &target);
FootTarget leg_to_body_frame(const RobotGeometry &geometry, servo::Leg leg,
                             const Vector3 &local);

// Angles are logical joint angles in degrees; this layer deliberately has no
// knowledge of servo pulse calibration or GPIOs.
IkResult solve_leg_ik(const RobotGeometry &geometry, servo::Leg leg,
                      const FootTarget &target);
FootTarget forward_kinematics(const RobotGeometry &geometry, servo::Leg leg,
                              const LegPose &pose);

} // namespace robot
