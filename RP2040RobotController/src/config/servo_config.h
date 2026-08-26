#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "robot/kinematics.h"

namespace hexapod {

struct ServoChannelConfig {
    std::uint8_t gpio = 0;
    std::uint16_t center_us = 1500;
    std::int8_t direction = 1;
    float us_per_degree = 1000.0f / 150.0f;
    std::uint16_t min_pulse_us = 1000;
    std::uint16_t max_pulse_us = 2000;
};

struct ServoMapResult {
    bool valid = false;
    std::uint16_t pulse_us = 1500;
};

const std::array<ServoChannelConfig, kServoCount>& servoConfigs();
ServoMapResult angleToPulse(LegId leg, JointId joint, float logical_angle_rad);
bool mapRobotAnglesToPulses(const std::array<JointAngles, kLegCount>& joints,
                            std::array<std::uint16_t, kServoCount>* pulses_us);

}  // namespace hexapod
