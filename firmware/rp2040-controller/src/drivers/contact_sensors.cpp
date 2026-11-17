#include "drivers/contact_sensors.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#endif

namespace hexapod {

void InputDebouncer::reset(std::uint8_t raw_mask, std::uint64_t now_us) {
    stable_mask_ = static_cast<std::uint8_t>(raw_mask & 0x3Fu);
    candidate_mask_ = stable_mask_;
    candidate_since_us_.fill(now_us);
}

void InputDebouncer::update(std::uint8_t raw_mask, std::uint64_t now_us) {
    raw_mask = static_cast<std::uint8_t>(raw_mask & 0x3Fu);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const std::uint8_t bit = static_cast<std::uint8_t>(1u << i);
        const bool raw = (raw_mask & bit) != 0;
        const bool candidate = (candidate_mask_ & bit) != 0;
        if (raw != candidate) {
            candidate_mask_ ^= bit;
            candidate_since_us_[i] = now_us;
            continue;
        }

        const bool stable = (stable_mask_ & bit) != 0;
        if (candidate != stable && now_us >= candidate_since_us_[i] &&
            now_us - candidate_since_us_[i] >= kContactDebounceUs) {
            stable_mask_ ^= bit;
        }
    }
}

void ContactSensors::init(std::uint64_t now_us) {
#if defined(PICO_ON_DEVICE)
    for (const std::uint8_t gpio : kContactGpios) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_disable_pulls(gpio);
    }
#endif
    debouncer_.reset(readRawMask(), now_us);
    initialized_ = true;
}

void ContactSensors::update(std::uint64_t now_us) {
    if (!initialized_) {
        init(now_us);
    }
    debouncer_.update(readRawMask(), now_us);
}

std::uint8_t ContactSensors::readRawMask() const {
    std::uint8_t mask = 0;
#if defined(PICO_ON_DEVICE)
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (gpio_get(kContactGpios[i]) != 0) {
            mask |= static_cast<std::uint8_t>(1u << i);
        }
    }
#endif
    return mask;
}

}  // namespace hexapod
