#pragma once

#include "robot_params.hpp"

#include <array>

namespace robot {

struct LegPose {
    float coxa_deg;
    float femur_deg;
    float tibia_deg;
};

struct RobotPose {
    std::array<LegPose, 6> legs;
};

constexpr LegPose ZERO_LEG_POSE{0.0f, 0.0f, 0.0f};
constexpr LegPose CENTER_LEG_POSE = ZERO_LEG_POSE;
constexpr LegPose STAND_LEG_POSE{config::STAND_COXA_DEG,
                                 config::STAND_FEMUR_DEG,
                                 config::STAND_TIBIA_DEG};

constexpr RobotPose ZERO_POSE{{
    ZERO_LEG_POSE, ZERO_LEG_POSE, ZERO_LEG_POSE,
    ZERO_LEG_POSE, ZERO_LEG_POSE, ZERO_LEG_POSE
}};

constexpr RobotPose CENTER_POSE = ZERO_POSE;

constexpr RobotPose STAND_POSE{{
    STAND_LEG_POSE, STAND_LEG_POSE, STAND_LEG_POSE,
    STAND_LEG_POSE, STAND_LEG_POSE, STAND_LEG_POSE
}};

} // namespace robot
