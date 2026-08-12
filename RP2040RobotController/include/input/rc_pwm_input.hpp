#pragma once

#include "control/drive_controller.hpp"

#include <cstdint>

namespace input {

constexpr bool rc_pulse_width_valid(uint64_t pulse_us, uint16_t valid_min_us, uint16_t valid_max_us) {
    return pulse_us >= valid_min_us && pulse_us <= valid_max_us;
}

class RcPwmInput {
public:
    void init(uint8_t forward_gpio, uint8_t steer_gpio,
              uint16_t valid_min_us, uint16_t valid_max_us);
    control::RcPwmSnapshot read() const;

private:
    struct Channel {
        uint8_t gpio = 0;
        volatile uint64_t rising_us = 0;
        volatile uint64_t updated_us = 0;
        volatile uint16_t pulse_us = 1500;
        volatile bool high = false;
        volatile bool has_pulse = false;
    };

    static void irq_callback(unsigned int gpio, uint32_t events);
    void handle_irq(unsigned int gpio, uint32_t events);
    Channel *channel_for_gpio(unsigned int gpio);
    const Channel *channel_for_gpio(unsigned int gpio) const;

    Channel forward_{};
    Channel steer_{};
    uint16_t valid_min_us_ = 750;
    uint16_t valid_max_us_ = 2700;
};

} // namespace input
