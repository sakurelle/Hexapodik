#pragma once

#include "board/pins.hpp"
#include "servo/servo_types.hpp"

#include <array>
#include <cstdint>

namespace servo {

constexpr uint32_t FRAME_PERIOD_US = 20000;
constexpr uint16_t SAFE_MIN_PULSE_US = 500;
constexpr uint16_t SAFE_MAX_PULSE_US = 2500;
constexpr uint16_t DEFAULT_CENTER_US = 1500;

struct ServoConfig {
    Leg leg;
    Joint joint;
    uint8_t gpio;

    uint16_t center_us;
    uint16_t pulse_minus_45_us;
    uint16_t pulse_plus_45_us;

    float min_angle_deg;
    float max_angle_deg;

    bool enabled;
};

// Temporary bench calibration. Direction is encoded by these values; there is
// intentionally no separate inversion flag.
constexpr ServoConfig make_temp_servo(Leg leg, Joint joint, uint8_t gpio) {
    return ServoConfig{leg, joint, gpio, 1500, 2000, 1000, -45.0f, 45.0f, true};
}

constexpr std::array<ServoConfig, SERVO_COUNT> DEFAULT_SERVOS = {{
    make_temp_servo(Leg::FR, Joint::Coxa, 2),
    make_temp_servo(Leg::FR, Joint::Femur, 3),
    make_temp_servo(Leg::FR, Joint::Tibia, 4),
    make_temp_servo(Leg::MR, Joint::Coxa, 5),
    make_temp_servo(Leg::MR, Joint::Femur, 6),
    make_temp_servo(Leg::MR, Joint::Tibia, 7),
    make_temp_servo(Leg::RR, Joint::Coxa, 8),
    make_temp_servo(Leg::RR, Joint::Femur, 9),
    make_temp_servo(Leg::RR, Joint::Tibia, 10),
    make_temp_servo(Leg::RL, Joint::Coxa, 11),
    make_temp_servo(Leg::RL, Joint::Femur, 12),
    make_temp_servo(Leg::RL, Joint::Tibia, 13),
    make_temp_servo(Leg::ML, Joint::Coxa, 14),
    make_temp_servo(Leg::ML, Joint::Femur, 15),
    make_temp_servo(Leg::ML, Joint::Tibia, 26),
    make_temp_servo(Leg::FL, Joint::Coxa, 27),
    make_temp_servo(Leg::FL, Joint::Femur, 28),
    make_temp_servo(Leg::FL, Joint::Tibia, 29),
}};

constexpr std::array<uint32_t, LEG_COUNT> LEG_PHASE_OFFSET_US = {{
    0, 3333, 6667, 10000, 13333, 16667
}};

constexpr size_t servo_index(Leg leg, Joint joint) {
    return leg_index(leg) * JOINT_COUNT + joint_index(joint);
}

constexpr bool gpio_is_unique() {
    for (size_t i = 0; i < DEFAULT_SERVOS.size(); ++i) {
        for (size_t j = i + 1; j < DEFAULT_SERVOS.size(); ++j) {
            if (DEFAULT_SERVOS[i].gpio == DEFAULT_SERVOS[j].gpio) {
                return false;
            }
        }
    }
    return true;
}

constexpr bool each_leg_has_three_joints() {
    for (size_t leg = 0; leg < LEG_COUNT; ++leg) {
        bool seen[JOINT_COUNT] = {};
        for (const auto &servo : DEFAULT_SERVOS) {
            if (leg_index(servo.leg) == leg) {
                seen[joint_index(servo.joint)] = true;
            }
        }
        for (bool value : seen) {
            if (!value) {
                return false;
            }
        }
    }
    return true;
}

constexpr bool reserved_gpios_are_not_servos() {
    for (const auto &servo : DEFAULT_SERVOS) {
        if (servo.gpio == board::UART0_TX_GPIO || servo.gpio == board::UART0_RX_GPIO ||
            servo.gpio == board::WS2812_GPIO) {
            return false;
        }
    }
    return true;
}

constexpr bool group_a_is_gp2_to_gp15() {
    for (uint8_t gpio = 2; gpio <= 15; ++gpio) {
        bool found = false;
        for (const auto &servo : DEFAULT_SERVOS) {
            found = found || servo.gpio == gpio;
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

constexpr bool group_b_is_gp26_to_gp29() {
    for (uint8_t gpio = 26; gpio <= 29; ++gpio) {
        bool found = false;
        for (const auto &servo : DEFAULT_SERVOS) {
            found = found || servo.gpio == gpio;
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

constexpr bool phase_offsets_are_valid() {
    for (uint32_t phase : LEG_PHASE_OFFSET_US) {
        if (phase >= FRAME_PERIOD_US || phase + SAFE_MAX_PULSE_US > FRAME_PERIOD_US) {
            return false;
        }
    }
    return true;
}

static_assert(DEFAULT_SERVOS.size() == 18, "Exactly 18 servos are required");
static_assert(each_leg_has_three_joints(), "Each leg must have exactly three configured joints");
static_assert(gpio_is_unique(), "Servo GPIO pins must be unique");
static_assert(reserved_gpios_are_not_servos(), "GP0, GP1, and GP16 are reserved");
static_assert(group_a_is_gp2_to_gp15(), "PIO group A must contain GP2-GP15");
static_assert(group_b_is_gp26_to_gp29(), "PIO group B must contain GP26-GP29");
static_assert(phase_offsets_are_valid(), "Leg phase offsets must fit inside the 20 ms frame");

} // namespace servo
