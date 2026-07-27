#pragma once

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

constexpr LegPose CENTER_LEG_POSE{0.0f, 0.0f, 0.0f};
constexpr LegPose STAND_LEG_POSE{0.0f, -15.0f, 20.0f};

constexpr RobotPose CENTER_POSE{{
    CENTER_LEG_POSE, CENTER_LEG_POSE, CENTER_LEG_POSE,
    CENTER_LEG_POSE, CENTER_LEG_POSE, CENTER_LEG_POSE
}};

constexpr RobotPose STAND_POSE{{
    STAND_LEG_POSE, STAND_LEG_POSE, STAND_LEG_POSE,
    STAND_LEG_POSE, STAND_LEG_POSE, STAND_LEG_POSE
}};

} // namespace robot
