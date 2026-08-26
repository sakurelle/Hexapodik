#pragma once

#include <algorithm>
#include <cmath>

#include "config/robot_config.h"

namespace hexapod {

struct BodyCommand {
    float vx_mm_s = 0.0f;
    float vy_mm_s = 0.0f;
    float yaw_rad_s = 0.0f;
};

inline float commandMagnitude(BodyCommand command) {
    return std::sqrt(command.vx_mm_s * command.vx_mm_s + command.vy_mm_s * command.vy_mm_s) +
           std::fabs(command.yaw_rad_s) * 100.0f;
}

inline BodyCommand operator*(BodyCommand command, float scale) {
    return {command.vx_mm_s * scale, command.vy_mm_s * scale, command.yaw_rad_s * scale};
}

inline bool commandIsZero(BodyCommand command) {
    return std::fabs(command.vx_mm_s) < 0.5f && std::fabs(command.vy_mm_s) < 0.5f &&
           std::fabs(command.yaw_rad_s) < 0.005f;
}

inline float slewAxis(float current, float target, float max_rate, float dt_s) {
    const float step = std::max(0.0f, max_rate * dt_s);
    const float delta = std::clamp(target - current, -step, step);
    return current + delta;
}

inline BodyCommand slewLimit(BodyCommand current, BodyCommand target, float dt_s) {
    return {
        slewAxis(current.vx_mm_s, target.vx_mm_s, kMaxVxAccelMmS2, dt_s),
        slewAxis(current.vy_mm_s, target.vy_mm_s, kMaxVyAccelMmS2, dt_s),
        slewAxis(current.yaw_rad_s, target.yaw_rad_s, kMaxYawAccelRadS2, dt_s),
    };
}

}  // namespace hexapod
