#pragma once

#include "robot_params.hpp"
#include "servo/servo_types.hpp"

#include <array>
#include <cstdint>

namespace robot::gait {

constexpr float GAIT_LIFT_FEMUR_DELTA_DEG = config::GAIT_LIFT_FEMUR_DELTA_DEG;
constexpr float GAIT_LIFT_TIBIA_DELTA_DEG = config::GAIT_LIFT_TIBIA_DELTA_DEG;
constexpr float GAIT_COXA_SWING_MIN_DEG = config::GAIT_COXA_SWING_MIN_DEG;
constexpr float GAIT_COXA_SWING_MAX_DEG = config::GAIT_COXA_SWING_MAX_DEG;

constexpr uint32_t MARCH_LIFT_TIME_MS = 300;
constexpr uint32_t MARCH_HOLD_TIME_MS = 200;
constexpr uint32_t MARCH_LOWER_TIME_MS = 300;
constexpr uint8_t MARCH_CYCLES = 2;

constexpr uint32_t WALK_PREPARE_MS = 500;
constexpr uint32_t WALK_LIFT_MS = 250;
constexpr uint32_t WALK_TRANSFER_MS = 500;
constexpr uint32_t WALK_LOWER_MS = 250;
constexpr uint32_t WALK_FINISH_MS = 600;
constexpr uint8_t WALK_DEMO_CYCLES = 3;

constexpr bool AUTO_DEMO_GAIT_ON_BOOT = false;
constexpr uint32_t AUTO_DEMO_DELAY_MS = 3000;
constexpr uint32_t AUTO_DEMO_BETWEEN_MS = 2000;

constexpr uint32_t GAIT_CYCLE_SLOW_MS = config::GAIT_CYCLE_SLOW_MS;
constexpr uint32_t GAIT_CYCLE_FAST_MS = config::GAIT_CYCLE_FAST_MS;

constexpr std::array<servo::Leg, 3> TRIPOD_A = {{
    servo::Leg::FR,
    servo::Leg::ML,
    servo::Leg::RR,
}};

constexpr std::array<servo::Leg, 3> TRIPOD_B = {{
    servo::Leg::FL,
    servo::Leg::MR,
    servo::Leg::RL,
}};

constexpr std::array<float, servo::LEG_COUNT> COXA_FORWARD_SIGN = {{
    1.0f, // FR
    1.0f, // MR
    1.0f, // RR
    1.0f, // RL
    1.0f, // ML
    1.0f, // FL
}};

constexpr float forwardCoxa(servo::Leg leg, float swing_deg) {
    return swing_deg * COXA_FORWARD_SIGN[servo::leg_index(leg)];
}

constexpr float backwardCoxa(servo::Leg leg, float swing_deg) {
    return -swing_deg * COXA_FORWARD_SIGN[servo::leg_index(leg)];
}

constexpr float forwardCoxa(servo::Leg leg) {
    return forwardCoxa(leg, GAIT_COXA_SWING_MAX_DEG);
}

constexpr float backwardCoxa(servo::Leg leg) {
    return backwardCoxa(leg, GAIT_COXA_SWING_MAX_DEG);
}

constexpr bool leg_in_tripod(servo::Leg leg, const std::array<servo::Leg, 3> &tripod) {
    return tripod[0] == leg || tripod[1] == leg || tripod[2] == leg;
}

} // namespace robot::gait
