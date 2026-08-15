#pragma once

#include "servo/servo_types.hpp"

#include <array>

namespace robot {

// Body coordinates: +X forward, +Y left, +Z up. mount_yaw_rad points from
// body +X to a leg's local +X direction when its Coxa angle is zero.
struct Vector3 {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
};

struct LegGeometry {
    Vector3 body_mount;
    float mount_yaw_rad = 0.0f;
};

struct RobotGeometry {
    float coxa_length_mm = 0.0f;
    float femur_length_mm = 0.0f;
    // Effective straight axis-to-contact distance. The real Tibia is curved;
    // add a central mechanical offset here if later measurements require it.
    float tibia_length_mm = 0.0f;
    // Logical servo zero is not the same as the CAD link zero.  These are
    // mechanical angles, shared by every leg; servo direction remains in
    // ServoConfig, outside Cartesian kinematics.
    float femur_zero_offset_deg = 0.0f;
    float tibia_zero_offset_deg = 0.0f;
    std::array<LegGeometry, servo::LEG_COUNT> legs{};
};

// servo::Leg order is FR, MR, RR, RL, ML, FL; this array uses that order.
constexpr RobotGeometry ROBOT_GEOMETRY{
    44.0f,
    80.0f,
    125.24f,
    35.27f,
    -104.65f,
    {{
        {{82.93f, -62.79f, 0.0f}, -0.785398163f},  // FR
        {{0.25f, -80.25f, 0.0f}, -1.570796327f},   // MR
        {{-82.95f, -62.81f, 0.0f}, -2.356194490f}, // RR
        {{-82.95f, 62.81f, 0.0f}, 2.356194490f},   // RL
        {{0.25f, 80.25f, 0.0f}, 1.570796327f},     // ML
        {{82.93f, 62.79f, 0.0f}, 0.785398163f},    // FL
    }}
};

constexpr bool geometry_is_valid(const RobotGeometry &geometry) {
    if (geometry.coxa_length_mm <= 0.0f || geometry.femur_length_mm <= 0.0f ||
        geometry.tibia_length_mm <= 0.0f) {
        return false;
    }
    bool has_mount = false;
    for (const auto &leg : geometry.legs) {
        has_mount = has_mount || leg.body_mount.x_mm != 0.0f ||
                    leg.body_mount.y_mm != 0.0f || leg.body_mount.z_mm != 0.0f;
    }
    return has_mount;
}

static_assert(geometry_is_valid(ROBOT_GEOMETRY),
              "ROBOT_GEOMETRY must contain valid measured dimensions");

} // namespace robot
