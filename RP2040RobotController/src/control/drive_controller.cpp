#include "control/drive_controller.hpp"

#include <algorithm>
#include <cmath>

namespace control {

namespace {

constexpr uint64_t US_PER_MS = 1000;

float clamp_unit(float value) {
    return std::max(-1.0f, std::min(1.0f, value));
}

float interp_speed(float normalized_stick, float min_speed, float max_speed) {
    return min_speed + normalized_stick * (max_speed - min_speed);
}

} // namespace

float normalize_rc_channel_fixed(uint16_t pulse_us, bool reversed) {
    const uint16_t clamped = static_cast<uint16_t>(
        std::max<uint16_t>(config::RC_MIN_US, std::min<uint16_t>(config::RC_MAX_US, pulse_us)));
    const uint16_t low_neutral = static_cast<uint16_t>(config::RC_CENTER_US - config::RC_DEADBAND_US);
    const uint16_t high_neutral = static_cast<uint16_t>(config::RC_CENTER_US + config::RC_DEADBAND_US);
    float value = 0.0f;
    if (clamped < low_neutral) {
        const float span = static_cast<float>(low_neutral - config::RC_MIN_US);
        value = span > 0.0f ? -static_cast<float>(low_neutral - clamped) / span : 0.0f;
    } else if (clamped > high_neutral) {
        const float span = static_cast<float>(config::RC_MAX_US - high_neutral);
        value = span > 0.0f ? static_cast<float>(clamped - high_neutral) / span : 0.0f;
    }
    value = clamp_unit(value);
    return reversed ? -value : value;
}

bool rc_pulse_neutral(uint16_t pulse_us) {
    return pulse_us >= config::RC_CENTER_US - config::RC_DEADBAND_US &&
           pulse_us <= config::RC_CENTER_US + config::RC_DEADBAND_US;
}

DriveCommand apply_deadzone_and_speed(float x, float y, float deadzone,
                                      float min_speed, float max_speed) {
    const float radius = std::sqrt(x * x + y * y);
    if (radius <= deadzone || radius <= 0.0f) {
        return DriveCommand{};
    }

    const float normalized = std::min(1.0f, (radius - deadzone) / (1.0f - deadzone));
    const float scale = normalized / radius;
    return DriveCommand{
        clamp_unit(y * scale),
        clamp_unit(x * scale),
        interp_speed(normalized, min_speed, max_speed),
        true
    };
}

DriveMix differential_mix(float forward, float turn, float turn_gain) {
    DriveMix mix{forward + turn * turn_gain, forward - turn * turn_gain};
    const float max_abs = std::max(std::fabs(mix.left), std::fabs(mix.right));
    if (max_abs > 1.0f) {
        mix.left /= max_abs;
        mix.right /= max_abs;
    }
    mix.left = clamp_unit(mix.left);
    mix.right = clamp_unit(mix.right);
    return mix;
}

bool DriveController::signal_valid(const RcPwmSnapshot &snapshot, uint64_t now_us) const {
    const uint64_t timeout_us = static_cast<uint64_t>(config::RC_SIGNAL_TIMEOUT_MS) * US_PER_MS;
    return snapshot.forward.has_pulse &&
           snapshot.steer.has_pulse &&
           rc_age_us(now_us, snapshot.forward.updated_us) <= timeout_us &&
           rc_age_us(now_us, snapshot.steer.updated_us) <= timeout_us;
}

void DriveController::update_arming(bool valid, bool neutral, uint64_t now_us) {
    if (!valid) {
        state_.armed = false;
        neutral_seen_ = false;
        neutral_since_us_ = 0;
        state_.arm_elapsed_ms = 0;
        return;
    }

    if (state_.armed) {
        state_.arm_elapsed_ms = config::RC_ARM_NEUTRAL_MS;
        return;
    }

    if (!neutral) {
        neutral_seen_ = false;
        neutral_since_us_ = 0;
        state_.arm_elapsed_ms = 0;
        return;
    }

    if (!neutral_seen_) {
        neutral_seen_ = true;
        neutral_since_us_ = now_us;
    }

    const uint64_t arm_elapsed_us = rc_age_us(now_us, neutral_since_us_);
    const uint64_t arm_required_us = static_cast<uint64_t>(config::RC_ARM_NEUTRAL_MS) * US_PER_MS;
    state_.arm_elapsed_ms = static_cast<uint32_t>(std::min<uint64_t>(
        config::RC_ARM_NEUTRAL_MS,
        arm_elapsed_us / US_PER_MS));
    if (arm_elapsed_us >= arm_required_us) {
        state_.armed = true;
        state_.arm_elapsed_ms = config::RC_ARM_NEUTRAL_MS;
    }
}

DriveControllerState DriveController::update(const RcPwmSnapshot &snapshot, uint64_t now_us) {
    state_.raw_forward_us = snapshot.forward.pulse_us;
    state_.raw_steer_us = snapshot.steer.pulse_us;
    state_.forward_age_ms = snapshot.forward.has_pulse
                                ? static_cast<uint32_t>(rc_age_us(now_us, snapshot.forward.updated_us) / US_PER_MS)
                                : 0;
    state_.steer_age_ms = snapshot.steer.has_pulse
                              ? static_cast<uint32_t>(rc_age_us(now_us, snapshot.steer.updated_us) / US_PER_MS)
                              : 0;
    state_.signal_valid = signal_valid(snapshot, now_us);
    state_.forward_neutral = snapshot.forward.has_pulse && rc_pulse_neutral(snapshot.forward.pulse_us);
    state_.steer_neutral = snapshot.steer.has_pulse && rc_pulse_neutral(snapshot.steer.pulse_us);

    float x = 0.0f;
    float y = 0.0f;
    if (state_.signal_valid) {
        y = normalize_rc_channel_fixed(snapshot.forward.pulse_us, config::RC_FORWARD_REVERSED);
        x = normalize_rc_channel_fixed(snapshot.steer.pulse_us, config::RC_STEER_REVERSED);
    }

    if (last_update_us_ == 0 || config::RC_SMOOTHING_MS == 0) {
        filtered_x_ = x;
        filtered_y_ = y;
    } else {
        const float elapsed_ms = static_cast<float>(now_us - last_update_us_) / 1000.0f;
        const float alpha = elapsed_ms / (static_cast<float>(config::RC_SMOOTHING_MS) + elapsed_ms);
        filtered_x_ += (x - filtered_x_) * alpha;
        filtered_y_ += (y - filtered_y_) * alpha;
    }
    last_update_us_ = now_us;

    state_.x = filtered_x_;
    state_.y = filtered_y_;

    update_arming(state_.signal_valid, state_.forward_neutral && state_.steer_neutral, now_us);
    state_.waiting_neutral = state_.signal_valid && !state_.armed;

    DriveCommand command = apply_deadzone_and_speed(state_.x, state_.y,
                                                    0.0f,
                                                    config::RC_MIN_SPEED,
                                                    config::RC_MAX_SPEED);

    if (!state_.signal_valid || !state_.armed) {
        command = DriveCommand{};
    }

    state_.command = command;
    state_.mix = differential_mix(command.forward, command.turn, config::GAIT_TURN_GAIN);
    return state_;
}

} // namespace control
