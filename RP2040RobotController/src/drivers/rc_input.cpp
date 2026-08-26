#include "drivers/rc_input.h"

#include <algorithm>
#include <cmath>

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#endif

namespace hexapod {

void RcInput::init() {
#if defined(PICO_ON_DEVICE)
    gpio_init(kRcForwardGpio);
    gpio_init(kRcYawGpio);
    gpio_set_dir(kRcForwardGpio, GPIO_IN);
    gpio_set_dir(kRcYawGpio, GPIO_IN);
    gpio_set_irq_enabled(kRcForwardGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(kRcYawGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
#endif
}

void RcInput::onEdge(std::uint8_t gpio, bool rising, std::uint64_t now_us) {
    std::uint64_t* rise_us = nullptr;
    std::uint64_t* last_us = nullptr;
    std::uint16_t* pulse_us = nullptr;
    if (gpio == kRcForwardGpio) {
        rise_us = &rise_forward_us_;
        last_us = &last_forward_us_;
        pulse_us = &forward_pulse_us_;
    } else if (gpio == kRcYawGpio) {
        rise_us = &rise_yaw_us_;
        last_us = &last_yaw_us_;
        pulse_us = &yaw_pulse_us_;
    } else {
        return;
    }

    if (rising) {
        *rise_us = now_us;
    } else if (*rise_us != 0 && now_us >= *rise_us) {
        const std::uint64_t width = now_us - *rise_us;
        if (width >= 750 && width <= 2250) {
            *pulse_us = static_cast<std::uint16_t>(width);
            *last_us = now_us;
        }
    }
}

BodyCommand RcInput::readCommand(std::uint64_t now_us) {
    const bool fresh = (now_us - last_forward_us_ <= kRcFailsafeUs) &&
                       (now_us - last_yaw_us_ <= kRcFailsafeUs);
    if (!fresh) {
        return {};
    }
    return {normalize(forward_pulse_us_) * kMaxVxMmS, 0.0f, normalize(yaw_pulse_us_) * kMaxYawRadS};
}

float RcInput::normalize(std::uint16_t pulse_us) const {
    const int centered = static_cast<int>(pulse_us) - static_cast<int>(kRcCenterUs);
    if (std::abs(centered) <= kRcDeadbandUs) {
        return 0.0f;
    }
    if (centered > 0) {
        return std::clamp(static_cast<float>(centered - kRcDeadbandUs) /
                              static_cast<float>(kRcMaxUs - kRcCenterUs - kRcDeadbandUs),
                          0.0f, 1.0f);
    }
    return std::clamp(static_cast<float>(centered + kRcDeadbandUs) /
                          static_cast<float>(kRcCenterUs - kRcMinUs - kRcDeadbandUs),
                      -1.0f, 0.0f);
}

}  // namespace hexapod
