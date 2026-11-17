#pragma once

#include <cstdint>

#include "config/robot_config.h"

namespace hexapod {

class InputDebouncer {
public:
    void reset(std::uint8_t raw_mask, std::uint64_t now_us);
    void update(std::uint8_t raw_mask, std::uint64_t now_us);
    std::uint8_t stableMask() const { return stable_mask_; }

private:
    std::uint8_t stable_mask_ = 0;
    std::uint8_t candidate_mask_ = 0;
    std::array<std::uint64_t, kLegCount> candidate_since_us_{};
};

class ContactSensors {
public:
    void init(std::uint64_t now_us);
    void update(std::uint64_t now_us);
    std::uint8_t inputMask() const { return debouncer_.stableMask(); }

private:
    std::uint8_t readRawMask() const;
    InputDebouncer debouncer_{};
    bool initialized_ = false;
};

}  // namespace hexapod
