#pragma once

#include "robot_params.hpp"

#include <cstdint>

namespace control {

struct RcChannelSnapshot {
    uint16_t pulse_us = 1500;
    uint64_t updated_us = 0;
    bool has_pulse = false;
};

struct RcPwmSnapshot {
    RcChannelSnapshot forward;
    RcChannelSnapshot steer;
};

struct DriveCommand {
    float forward = 0.0f;
    float turn = 0.0f;
    float speed = 0.0f;
    bool active = false;
};

struct DriveMix {
    float left = 0.0f;
    float right = 0.0f;
};

struct DriveControllerState {
    uint16_t raw_forward_us = 1500;
    uint16_t raw_steer_us = 1500;
    uint32_t forward_age_ms = 0;
    uint32_t steer_age_ms = 0;
    bool signal_valid = false;
    bool forward_neutral = false;
    bool steer_neutral = false;
    bool armed = false;
    bool waiting_neutral = true;
    uint32_t arm_elapsed_ms = 0;
    float x = 0.0f;
    float y = 0.0f;
    DriveCommand command{};
    DriveMix mix{};
};

constexpr uint64_t rc_age_us(uint64_t now_us, uint64_t updated_us) {
    return now_us >= updated_us ? now_us - updated_us : 0;
}

float normalize_rc_channel_fixed(uint16_t pulse_us, bool reversed);
bool rc_pulse_neutral(uint16_t pulse_us);
DriveCommand apply_deadzone_and_speed(float x, float y, float deadzone,
                                      float min_speed, float max_speed);
DriveMix differential_mix(float forward, float turn, float turn_gain);

class DriveController {
public:
    DriveControllerState update(const RcPwmSnapshot &snapshot, uint64_t now_us);
    const DriveControllerState &state() const { return state_; }

private:
    bool signal_valid(const RcPwmSnapshot &snapshot, uint64_t now_us) const;
    void update_arming(bool valid, bool neutral, uint64_t now_us);

    DriveControllerState state_{};
    float filtered_x_ = 0.0f;
    float filtered_y_ = 0.0f;
    uint64_t last_update_us_ = 0;
    uint64_t neutral_since_us_ = 0;
    bool neutral_seen_ = false;
};

} // namespace control
