#pragma once

#include <cstdint>

#include "locomotion/body_command.h"

namespace hexapod {

class CommandSource {
public:
    virtual ~CommandSource() = default;
    virtual BodyCommand readCommand(std::uint64_t now_us) = 0;
};

class RcInput : public CommandSource {
public:
    void init();
    void onEdge(std::uint8_t gpio, bool rising, std::uint64_t now_us);
    BodyCommand readCommand(std::uint64_t now_us) override;

    std::uint16_t pulseForwardUs() const { return forward_pulse_us_; }
    std::uint16_t pulseYawUs() const { return yaw_pulse_us_; }

private:
    float normalize(std::uint16_t pulse_us) const;

    std::uint64_t rise_forward_us_ = 0;
    std::uint64_t rise_yaw_us_ = 0;
    std::uint64_t last_forward_us_ = 0;
    std::uint64_t last_yaw_us_ = 0;
    std::uint16_t forward_pulse_us_ = kRcCenterUs;
    std::uint16_t yaw_pulse_us_ = kRcCenterUs;
};

}  // namespace hexapod
