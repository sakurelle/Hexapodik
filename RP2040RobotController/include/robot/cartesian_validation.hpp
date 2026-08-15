#pragma once

#include "robot/kinematics.hpp"
#include "servo/servo_config.hpp"

#include <array>

namespace robot {

enum class CartesianLimitReason { None, Unreachable, NonFinite, JointLimit };

struct CartesianLimitInfo {
    CartesianLimitReason reason = CartesianLimitReason::None;
    servo::Leg leg = servo::Leg::FR;
    servo::Joint joint = servo::Joint::Coxa;
    float angle_deg = 0.0f;
    float limit_deg = 0.0f;

    bool active() const { return reason != CartesianLimitReason::None; }
};

struct CartesianPoseValidation {
    bool reachable = false;
    bool finite = false;
    bool inside_joint_limits = false;
    CartesianLimitInfo limit{};
    RobotPose pose{};

    bool valid() const { return reachable && finite && inside_joint_limits; }
};

const char *cartesian_limit_reason_name(CartesianLimitReason reason);

CartesianPoseValidation validate_cartesian_targets(
    const RobotGeometry &geometry,
    const std::array<FootTarget, servo::LEG_COUNT> &targets,
    const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);

} // namespace robot
