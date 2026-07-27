#pragma once

#include "servo/servo_frame_scheduler.hpp"

#include <array>
#include <cstdint>

namespace servo {

class PioServoDriver {
public:
    bool init();
    void start();
    void stop();
    void service();
    bool submit_pulses(const std::array<ServoPulse, SERVO_COUNT> &pulses, bool outputs_enabled);

    uint32_t frame_counter() const { return frame_counter_; }
    uint32_t error_count() const { return error_count_; }
    bool outputs_enabled() const { return outputs_enabled_; }

private:
    struct GroupState {
        PioPinGroup pins{};
        uint8_t sm = 0;
        int dma_channel = -1;
        std::array<std::array<uint32_t, MAX_PIO_WORDS_PER_GROUP>, 2> buffers{};
        std::array<size_t, 2> lengths{};
        uint8_t active = 0;
        uint8_t pending = 1;
        bool has_pending = false;
        bool running = false;
    };

    bool load_group(GroupState &state, const PioFrameWords &words);
    void start_dma(GroupState &state);
    void stop_group(GroupState &state);
    void force_low();

    GroupState group_a_{GROUP_A, 0};
    GroupState group_b_{GROUP_B, 1};
    bool initialized_ = false;
    bool outputs_enabled_ = false;
    uint32_t frame_counter_ = 0;
    uint32_t error_count_ = 0;
};

} // namespace servo
