#include "drivers/rc_input.h"

#include <algorithm>
#include <cmath>

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#include "hardware/sync.h"
#endif

namespace hexapod {

void RcInput::init() {
#if defined(PICO_ON_DEVICE)
    gpio_init(kRcForwardGpio);
    gpio_init(kRcYawGpio);
    gpio_set_dir(kRcForwardGpio, GPIO_IN);
    gpio_set_dir(kRcYawGpio, GPIO_IN);
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
        if (width >= 750 && width <= 2500) {
            *pulse_us = static_cast<std::uint16_t>(width);
            *last_us = now_us;
        }
    }
}

BodyCommand RcInput::readCommand(std::uint64_t now_us) {
    const RcInputSnapshot rc = snapshot(now_us);
    if (rc.failsafe) {
        return {};
    }
    return {normalize(rc.forward_pulse_us) * kMaxVxMmS, 0.0f, normalize(rc.yaw_pulse_us) * kMaxYawRadS};
}

RcInputSnapshot RcInput::snapshot(std::uint64_t now_us) const {
    RcInputSnapshot result;
    std::uint64_t last_forward_us = 0;
    std::uint64_t last_yaw_us = 0;
#if defined(PICO_ON_DEVICE)
    const std::uint32_t irq_state = save_and_disable_interrupts();
#endif
    result.forward_pulse_us = forward_pulse_us_;
    result.yaw_pulse_us = yaw_pulse_us_;
    last_forward_us = last_forward_us_;
    last_yaw_us = last_yaw_us_;
#if defined(PICO_ON_DEVICE)
    restore_interrupts(irq_state);
#endif
    result.forward_fresh = channelFresh(last_forward_us, now_us);
    result.yaw_fresh = channelFresh(last_yaw_us, now_us);
    result.failsafe = !(result.forward_fresh && result.yaw_fresh);
    return result;
}

bool RcInput::forwardFresh(std::uint64_t now_us) const {
    return snapshot(now_us).forward_fresh;
}

bool RcInput::yawFresh(std::uint64_t now_us) const {
    return snapshot(now_us).yaw_fresh;
}

bool RcInput::failsafe(std::uint64_t now_us) const {
    return snapshot(now_us).failsafe;
}

bool RcInput::channelFresh(std::uint64_t last_edge_us, std::uint64_t now_us) const {
    return last_edge_us != 0 && now_us >= last_edge_us && now_us - last_edge_us <= kRcFailsafeUs;
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
