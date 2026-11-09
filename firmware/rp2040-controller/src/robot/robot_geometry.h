#pragma once

#include <array>

#include "config/robot_config.h"
#include "math/vec3.h"

namespace hexapod {

struct LegMount {
    Vec3 position_body_mm;
    float yaw_rad = 0.0f;
};

const std::array<LegMount, kLegCount>& legMounts();
LegMount legMount(LegId leg);
Vec3 bodyToLeg(LegId leg, Vec3 body_point_mm);
Vec3 legToBody(LegId leg, Vec3 leg_point_mm);
std::array<Vec3, kLegCount> neutralFeetBody();

}  // namespace hexapod
