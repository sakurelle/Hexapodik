#include "input/rc_pwm_input.hpp"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

namespace input {

namespace {
RcPwmInput *g_instance = nullptr;
}

void RcPwmInput::init(uint8_t forward_gpio, uint8_t steer_gpio,
                      uint16_t valid_min_us, uint16_t valid_max_us) {
    forward_.gpio = forward_gpio;
    steer_.gpio = steer_gpio;
    valid_min_us_ = valid_min_us;
    valid_max_us_ = valid_max_us;
    g_instance = this;

    gpio_init(forward_gpio);
    gpio_set_dir(forward_gpio, GPIO_IN);
    gpio_pull_down(forward_gpio);

    gpio_init(steer_gpio);
    gpio_set_dir(steer_gpio, GPIO_IN);
    gpio_pull_down(steer_gpio);

    gpio_set_irq_enabled_with_callback(forward_gpio,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &RcPwmInput::irq_callback);
    gpio_set_irq_enabled(steer_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

control::RcPwmSnapshot RcPwmInput::read() const {
    const uint32_t irq_state = save_and_disable_interrupts();
    const control::RcPwmSnapshot snapshot{
        control::RcChannelSnapshot{forward_.pulse_us, forward_.updated_us, forward_.has_pulse},
        control::RcChannelSnapshot{steer_.pulse_us, steer_.updated_us, steer_.has_pulse}
    };
    restore_interrupts(irq_state);
    return snapshot;
}

void RcPwmInput::irq_callback(unsigned int gpio, uint32_t events) {
    if (g_instance) {
        g_instance->handle_irq(gpio, events);
    }
}

void RcPwmInput::handle_irq(unsigned int gpio, uint32_t events) {
    Channel *channel = channel_for_gpio(gpio);
    if (!channel) {
        return;
    }

    const uint64_t now_us = time_us_64();
    if (events & GPIO_IRQ_EDGE_RISE) {
        channel->rising_us = now_us;
        channel->high = true;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) && channel->high) {
        channel->high = false;
        const uint64_t width_us = now_us - channel->rising_us;
        if (rc_pulse_width_valid(width_us, valid_min_us_, valid_max_us_)) {
            channel->pulse_us = static_cast<uint16_t>(width_us);
            channel->updated_us = now_us;
            channel->has_pulse = true;
        }
    }
}

RcPwmInput::Channel *RcPwmInput::channel_for_gpio(unsigned int gpio) {
    if (gpio == forward_.gpio) {
        return &forward_;
    }
    if (gpio == steer_.gpio) {
        return &steer_;
    }
    return nullptr;
}

const RcPwmInput::Channel *RcPwmInput::channel_for_gpio(unsigned int gpio) const {
    if (gpio == forward_.gpio) {
        return &forward_;
    }
    if (gpio == steer_.gpio) {
        return &steer_;
    }
    return nullptr;
}

} // namespace input
