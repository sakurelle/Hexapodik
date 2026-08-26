#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"

namespace hexapod {

class ServoController {
public:
    bool init();
    bool submitPulses(const std::array<std::uint16_t, kServoCount>& pulses_us);
    const std::array<std::uint16_t, kServoCount>& lastPulses() const { return last_pulses_us_; }

private:
    struct Event {
        std::uint32_t mask = 0;
        std::uint32_t delay_ticks = 0;
    };

    static constexpr std::size_t kMaxEvents = kServoCount + 2;
    using EventBuffer = std::array<Event, kMaxEvents>;

    std::size_t buildEventBuffer(const std::array<std::uint16_t, kServoCount>& pulses_us,
                                 EventBuffer* buffer) const;
    bool submitBuffer(const EventBuffer& buffer, std::size_t count);

    std::array<std::uint16_t, kServoCount> last_pulses_us_{};
    EventBuffer buffers_[2]{};
    std::uint8_t write_buffer_ = 0;
    int dma_channel_ = -1;
    bool initialized_ = false;
};

}  // namespace hexapod
