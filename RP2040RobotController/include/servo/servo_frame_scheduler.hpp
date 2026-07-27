#pragma once

#include "servo/servo_config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace servo {

constexpr size_t MAX_EVENTS_PER_GROUP = 2 + (board::PIO_GROUP_A_CHANNELS * 2);
constexpr size_t MAX_PIO_WORDS_PER_GROUP = MAX_EVENTS_PER_GROUP * 2;
constexpr uint32_t PIO_DELAY_COMPENSATION_US = 4;

struct ServoPulse {
    Leg leg;
    uint8_t gpio;
    uint16_t pulse_us;
    bool enabled;
};

struct MaskEvent {
    uint32_t time_us;
    uint32_t mask;
};

struct EventFrame {
    std::array<MaskEvent, MAX_EVENTS_PER_GROUP> events{};
    size_t count = 0;
    bool valid = true;
};

struct PioFrameWords {
    std::array<uint32_t, MAX_PIO_WORDS_PER_GROUP> words{};
    size_t count = 0;
    bool valid = true;
};

struct PioPinGroup {
    uint8_t first_gpio;
    uint8_t channel_count;
};

constexpr PioPinGroup GROUP_A{board::PIO_GROUP_A_FIRST_GPIO, board::PIO_GROUP_A_CHANNELS};
constexpr PioPinGroup GROUP_B{board::PIO_GROUP_B_FIRST_GPIO, board::PIO_GROUP_B_CHANNELS};

EventFrame build_event_frame(const std::array<ServoPulse, SERVO_COUNT> &pulses, PioPinGroup group);
PioFrameWords build_pio_words(const EventFrame &frame);

} // namespace servo
