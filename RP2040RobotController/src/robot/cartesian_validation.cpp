#include "robot/cartesian_validation.hpp"
#include "robot/robot_model.hpp"

#include <cmath>

namespace robot {

CartesianPoseValidation validate_cartesian_targets(
    const RobotGeometry &geometry,
    const std::array<FootTarget, servo::LEG_COUNT> &targets,
    const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    CartesianPoseValidation validation{};
    validation.reachable = true;
    validation.finite = true;
    validation.inside_joint_limits = true;
    for (size_t leg_index = 0; leg_index < servo::LEG_COUNT; ++leg_index) {
        const auto leg = static_cast<servo::Leg>(leg_index);
        const auto result = solve_leg_ik(geometry, leg, targets[leg_index]);
        validation.reachable = validation.reachable && result.reachable;
        if (!result.reachable && !validation.limit.active()) {
            validation.limit.reason = CartesianLimitReason::Unreachable;
            validation.limit.leg = leg;
        }
        validation.pose.legs[leg_index] = result.pose;
        for (const auto joint : {servo::Joint::Coxa, servo::Joint::Femur, servo::Joint::Tibia}) {
            const float angle = pose_angle(validation.pose, leg, joint);
            const bool finite = std::isfinite(angle);
            validation.finite = validation.finite && finite;
            if (!finite && !validation.limit.active()) {
                validation.limit.reason = CartesianLimitReason::NonFinite;
                validation.limit.leg = leg;
                validation.limit.joint = joint;
                validation.limit.angle_deg = angle;
            }
            const auto &servo_config = servos[servo::servo_index(leg, joint)];
            const bool inside_limits = angle >= servo_config.min_angle_deg &&
                                       angle <= servo_config.max_angle_deg;
            validation.inside_joint_limits = validation.inside_joint_limits && inside_limits;
            if (!inside_limits && !validation.limit.active()) {
                validation.limit.reason = CartesianLimitReason::JointLimit;
                validation.limit.leg = leg;
                validation.limit.joint = joint;
                validation.limit.angle_deg = angle;
                validation.limit.limit_deg = angle < servo_config.min_angle_deg
                                                ? servo_config.min_angle_deg
                                                : servo_config.max_angle_deg;
            }
        }
    }
    return validation;
}

const char *cartesian_limit_reason_name(CartesianLimitReason reason) {
    switch (reason) {
    case CartesianLimitReason::None: return "NONE";
    case CartesianLimitReason::Unreachable: return "UNREACHABLE";
    case CartesianLimitReason::NonFinite: return "NONFINITE";
    case CartesianLimitReason::JointLimit: return "JOINT_LIMIT";
    }
    return "?";
}

} // namespace robot
