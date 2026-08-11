#pragma once

#include "servo/servo_config.hpp"

#include <cstdint>

namespace servo {

enum class PulseResult {
    Ok,
    ConfigInvalid,
    PulseOutOfRange
};

struct PulseConversion {
    PulseResult result;
    uint16_t pulse_us;
};

float clamp_angle(const ServoConfig &config, float angle_deg);
PulseConversion angle_to_pulse_us(const ServoConfig &config, float angle_deg);

} // namespace servo
