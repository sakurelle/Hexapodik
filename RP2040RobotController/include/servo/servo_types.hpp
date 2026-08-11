#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace servo {

enum class Leg : uint8_t {
    FR,
    MR,
    RR,
    RL,
    ML,
    FL
};

enum class Joint : uint8_t {
    Coxa,
    Femur,
    Tibia
};

constexpr size_t LEG_COUNT = 6;
constexpr size_t JOINT_COUNT = 3;
constexpr size_t SERVO_COUNT = LEG_COUNT * JOINT_COUNT;

constexpr size_t leg_index(Leg leg) {
    return static_cast<size_t>(leg);
}

constexpr size_t joint_index(Joint joint) {
    return static_cast<size_t>(joint);
}

constexpr const char *leg_name(Leg leg) {
    switch (leg) {
    case Leg::FR: return "FR";
    case Leg::MR: return "MR";
    case Leg::RR: return "RR";
    case Leg::RL: return "RL";
    case Leg::ML: return "ML";
    case Leg::FL: return "FL";
    }
    return "?";
}

constexpr const char *joint_name(Joint joint) {
    switch (joint) {
    case Joint::Coxa: return "COXA";
    case Joint::Femur: return "FEMUR";
    case Joint::Tibia: return "TIBIA";
    }
    return "?";
}

} // namespace servo
