#pragma once

#include "robot/robot_pose.hpp"
#include "servo/servo_types.hpp"

namespace robot {

inline float pose_angle(const RobotPose &pose, servo::Leg leg, servo::Joint joint) {
    const auto &leg_pose = pose.legs[servo::leg_index(leg)];
    switch (joint) {
    case servo::Joint::Coxa: return leg_pose.coxa_deg;
    case servo::Joint::Femur: return leg_pose.femur_deg;
    case servo::Joint::Tibia: return leg_pose.tibia_deg;
    }
    return 0.0f;
}

inline void set_pose_angle(RobotPose &pose, servo::Leg leg, servo::Joint joint, float angle_deg) {
    auto &leg_pose = pose.legs[servo::leg_index(leg)];
    switch (joint) {
    case servo::Joint::Coxa: leg_pose.coxa_deg = angle_deg; break;
    case servo::Joint::Femur: leg_pose.femur_deg = angle_deg; break;
    case servo::Joint::Tibia: leg_pose.tibia_deg = angle_deg; break;
    }
}

} // namespace robot
