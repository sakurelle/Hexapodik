#include "robot/robot_geometry.h"

#include <cmath>

#include "robot/kinematics.h"

namespace hexapod {
namespace {

constexpr std::array<LegMount, kLegCount> kMounts = {{
    {{+82.93f, +62.79f, 0.0f}, degToRad(+45.0f)},   // FL
    {{+0.25f, +80.25f, 0.0f}, degToRad(+90.0f)},    // ML
    {{-82.95f, +62.81f, 0.0f}, degToRad(+135.0f)},  // RL
    {{-82.95f, -62.81f, 0.0f}, degToRad(-135.0f)},  // RR
    {{+0.25f, -80.25f, 0.0f}, degToRad(-90.0f)},    // MR
    {{+82.93f, -62.79f, 0.0f}, degToRad(-45.0f)},   // FR
}};

Vec3 rotateZ(Vec3 p, float yaw_rad) {
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    return {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
}

}  // namespace

const std::array<LegMount, kLegCount>& legMounts() {
    return kMounts;
}

LegMount legMount(LegId leg) {
    return kMounts[indexOf(leg)];
}

Vec3 bodyToLeg(LegId leg, Vec3 body_point_mm) {
    const LegMount mount = legMount(leg);
    return rotateZ(body_point_mm - mount.position_body_mm, -mount.yaw_rad);
}

Vec3 legToBody(LegId leg, Vec3 leg_point_mm) {
    const LegMount mount = legMount(leg);
    return rotateZ(leg_point_mm, mount.yaw_rad) + mount.position_body_mm;
}

std::array<Vec3, kLegCount> neutralFeetBody() {
    std::array<Vec3, kLegCount> feet{};
    const Vec3 neutral_leg = forwardKinematics({});
    for (std::size_t i = 0; i < kLegCount; ++i) {
        feet[i] = legToBody(static_cast<LegId>(i), neutral_leg);
    }
    return feet;
}

}  // namespace hexapod
