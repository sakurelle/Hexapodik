#pragma once

#include <algorithm>
#include <cstdint>

#include "locomotion/body_command.h"

namespace hexapod {

struct MotionCommand {
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float wz_radps = 0.0f;

    float body_height_m = 0.0f;
    float body_roll_rad = 0.0f;
    float body_pitch_rad = 0.0f;
    float body_yaw_rad = 0.0f;

    std::uint32_t sequence = 0;
    bool valid = false;
};

inline BodyCommand toBodyCommandMm(MotionCommand command) {
    if (!command.valid) {
        return {};
    }
    return {
        std::clamp(command.vx_mps * 1000.0f, -kMaxVxMmS, kMaxVxMmS),
        std::clamp(command.vy_mps * 1000.0f, -kMaxVyMmS, kMaxVyMmS),
        std::clamp(command.wz_radps, -kMaxYawRadS, kMaxYawRadS),
    };
}

inline MotionCommand toMotionCommandSi(BodyCommand command, std::uint32_t sequence, bool valid = true) {
    return {
        command.vx_mm_s * 0.001f,
        command.vy_mm_s * 0.001f,
        command.yaw_rad_s,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        sequence,
        valid,
    };
}

}  // namespace hexapod
