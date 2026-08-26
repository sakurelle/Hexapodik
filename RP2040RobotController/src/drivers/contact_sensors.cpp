#include "drivers/contact_sensors.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#endif

namespace hexapod {

void ContactSensors::init() {
#if defined(PICO_ON_DEVICE)
    for (std::uint8_t gpio : kContactGpios) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
    }
#endif
    debouncer_.reset(readRaw());
    initialized_ = true;
}

void ContactSensors::update(float dt_s) {
    if (!initialized_) {
        init();
    }
    debouncer_.update(readRaw(), dt_s);
}

std::array<bool, kLegCount> ContactSensors::readRaw() const {
    std::array<bool, kLegCount> raw{};
#if defined(PICO_ON_DEVICE)
    for (std::size_t i = 0; i < kLegCount; ++i) {
        raw[i] = gpio_get(kContactGpios[i]) != 0;
    }
#endif
    return raw;
}

}  // namespace hexapod
