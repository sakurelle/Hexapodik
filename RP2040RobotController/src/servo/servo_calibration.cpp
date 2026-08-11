#include "servo/servo_calibration.hpp"

#include <cmath>

namespace servo {

float clamp_angle(const ServoConfig &config, float angle_deg) {
    if (angle_deg < config.min_angle_deg) {
        return config.min_angle_deg;
    }
    if (angle_deg > config.max_angle_deg) {
        return config.max_angle_deg;
    }
    if (angle_deg < -45.0f) {
        return -45.0f;
    }
    if (angle_deg > 45.0f) {
        return 45.0f;
    }
    return angle_deg;
}

static bool pulse_reasonable(uint16_t pulse_us) {
    return pulse_us >= SAFE_MIN_PULSE_US && pulse_us <= SAFE_MAX_PULSE_US;
}

PulseConversion angle_to_pulse_us(const ServoConfig &config, float angle_deg) {
    if (!pulse_reasonable(config.center_us) ||
        !pulse_reasonable(config.pulse_minus_45_us) ||
        !pulse_reasonable(config.pulse_plus_45_us) ||
        config.min_angle_deg > config.max_angle_deg) {
        return {PulseResult::ConfigInvalid, DEFAULT_CENTER_US};
    }

    const float angle = clamp_angle(config, angle_deg);
    float pulse = static_cast<float>(config.center_us);

    if (angle < 0.0f) {
        const float t = (angle + 45.0f) / 45.0f;
        pulse = static_cast<float>(config.pulse_minus_45_us) +
                t * (static_cast<float>(config.center_us) - static_cast<float>(config.pulse_minus_45_us));
    } else {
        const float t = angle / 45.0f;
        pulse = static_cast<float>(config.center_us) +
                t * (static_cast<float>(config.pulse_plus_45_us) - static_cast<float>(config.center_us));
    }

    const auto rounded = static_cast<int>(std::lround(pulse));
    if (rounded < SAFE_MIN_PULSE_US || rounded > SAFE_MAX_PULSE_US) {
        return {PulseResult::PulseOutOfRange, DEFAULT_CENTER_US};
    }

    return {PulseResult::Ok, static_cast<uint16_t>(rounded)};
}

} // namespace servo
