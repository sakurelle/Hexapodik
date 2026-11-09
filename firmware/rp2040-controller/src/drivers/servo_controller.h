#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "core/hardware_interfaces.h"

namespace hexapod {

class ServoController : public IServoOutput {
public:
    struct Event {
        std::uint32_t mask = 0;
        std::uint32_t delay_ticks = 0;
    };

    static constexpr std::size_t kMaxEvents = kServoCount + 2;
    static constexpr std::uint32_t kAllServoPinsMask = (1u << kServoGpioSpan) - 1u;
    static constexpr std::uint32_t kPioSegmentOverheadTicks = 5;
    using EventBuffer = std::array<Event, kMaxEvents>;

    bool init();
    bool submitPulses(const std::array<std::uint16_t, kServoCount>& pulses_us);
    bool submitJointTargets(const std::array<JointAngles, kLegCount>& joints) override;
    const std::array<std::uint16_t, kServoCount>& lastPulses() const { return last_pulses_us_; }

    static std::size_t buildEventBuffer(const std::array<std::uint16_t, kServoCount>& pulses_us,
                                        EventBuffer* buffer);

private:
    void startDma(std::uint8_t buffer_index);
    void handleDmaComplete();
    static void dmaIrqHandler();

    std::array<std::uint16_t, kServoCount> last_pulses_us_{};
    EventBuffer buffers_[2]{};
    std::size_t event_counts_[2]{};
    std::uint8_t active_buffer_ = 0;
    std::uint8_t pending_buffer_ = 1;
    int dma_channel_ = -1;
    bool pending_valid_ = false;
    bool initialized_ = false;
};

}  // namespace hexapod
