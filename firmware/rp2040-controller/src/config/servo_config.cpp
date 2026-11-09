#include "config/servo_config.h"

#include <algorithm>
#include <cmath>

namespace hexapod {
namespace {

constexpr float kUsPerDegree = 1000.0f / 150.0f;

constexpr ServoChannelConfig makeServo(std::uint8_t gpio, std::int8_t direction) {
    return {gpio, 1500, direction, kUsPerDegree, 1000, 2000};
}

constexpr std::array<ServoChannelConfig, kServoCount> kConfig = {{
    makeServo(2, +1), makeServo(3, +1), makeServo(4, +1),       // FL
    makeServo(5, +1), makeServo(6, +1), makeServo(7, +1),       // ML
    makeServo(8, +1), makeServo(9, +1), makeServo(10, +1),      // RL
    makeServo(11, -1), makeServo(12, -1), makeServo(13, -1),    // RR
    makeServo(14, -1), makeServo(15, -1), makeServo(16, -1),    // MR
    makeServo(17, -1), makeServo(18, -1), makeServo(19, -1),    // FR
}};

}  // namespace

const std::array<ServoChannelConfig, kServoCount>& servoConfigs() {
    return kConfig;
}

ServoMapResult angleToPulse(LegId leg, JointId joint, float logical_angle_rad) {
    const ServoChannelConfig& cfg = kConfig[servoIndex(leg, joint)];
    if (!std::isfinite(logical_angle_rad)) {
        return {};
    }

    const float logical_angle_deg = radToDeg(logical_angle_rad);
    const float pulse = static_cast<float>(cfg.center_us) +
                        static_cast<float>(cfg.direction) * logical_angle_deg * cfg.us_per_degree;
    if (!std::isfinite(pulse) || pulse < cfg.min_pulse_us || pulse > cfg.max_pulse_us) {
        return {};
    }
    return {true, static_cast<std::uint16_t>(std::lround(pulse))};
}

bool mapRobotAnglesToPulses(const std::array<JointAngles, kLegCount>& joints,
                            std::array<std::uint16_t, kServoCount>* pulses_us) {
    if (pulses_us == nullptr) {
        return false;
    }
    std::array<std::uint16_t, kServoCount> mapped{};
    for (std::size_t leg_i = 0; leg_i < kLegCount; ++leg_i) {
        const LegId leg = static_cast<LegId>(leg_i);
        const JointAngles q = joints[leg_i];
        const std::array<float, kJointsPerLeg> angles = {q.coxa_rad, q.femur_rad, q.tibia_rad};
        for (std::size_t joint_i = 0; joint_i < kJointsPerLeg; ++joint_i) {
            const JointId joint = static_cast<JointId>(joint_i);
            const ServoMapResult result = angleToPulse(leg, joint, angles[joint_i]);
            if (!result.valid) {
                return false;
            }
            mapped[servoIndex(leg, joint)] = result.pulse_us;
        }
    }
    *pulses_us = mapped;
    return true;
}

}  // namespace hexapod
