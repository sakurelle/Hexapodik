#pragma once

#include <cstdint>

#include "command/command_source.h"

namespace hexapod {

struct RcInputSnapshot {
    std::uint16_t forward_pulse_us = kRcCenterUs;
    std::uint16_t yaw_pulse_us = kRcCenterUs;
    bool forward_fresh = false;
    bool yaw_fresh = false;
    bool failsafe = true;
};

class RcInput : public ICommandSource {
public:
    void init();
    void onEdge(std::uint8_t gpio, bool rising, std::uint64_t now_us);
    MotionCommand readMotionCommand(std::uint64_t now_us) override;
    BodyCommand readCommand(std::uint64_t now_us);
    RcInputSnapshot snapshot(std::uint64_t now_us) const;
    bool forwardFresh(std::uint64_t now_us) const;
    bool yawFresh(std::uint64_t now_us) const;
    bool failsafe(std::uint64_t now_us) const;

    std::uint16_t pulseForwardUs() const { return forward_pulse_us_; }
    std::uint16_t pulseYawUs() const { return yaw_pulse_us_; }

private:
    bool channelFresh(std::uint64_t last_edge_us, std::uint64_t now_us) const;
    float normalize(std::uint16_t pulse_us) const;

    std::uint64_t rise_forward_us_ = 0;
    std::uint64_t rise_yaw_us_ = 0;
    std::uint64_t last_forward_us_ = 0;
    std::uint64_t last_yaw_us_ = 0;
    std::uint16_t forward_pulse_us_ = kRcCenterUs;
    std::uint16_t yaw_pulse_us_ = kRcCenterUs;
};

}  // namespace hexapod
