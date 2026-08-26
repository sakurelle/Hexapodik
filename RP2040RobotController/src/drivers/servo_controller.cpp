#include "drivers/servo_controller.h"

#include <algorithm>

#if defined(PICO_ON_DEVICE)
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "servo_output.pio.h"
#endif

namespace hexapod {
namespace {

constexpr std::uint32_t kFullServoMask = (1u << kServoGpioSpan) - 1u;
constexpr std::uint32_t kTicksPerUs = 1u;

struct PulseEvent {
    std::uint16_t at_us = 0;
    std::uint32_t clear_mask = 0;
};

}  // namespace

bool ServoController::init() {
    last_pulses_us_.fill(1500);
#if defined(PICO_ON_DEVICE)
    PIO pio = pio0;
    const unsigned int offset = pio_add_program(pio, &servo_output_program);
    pio_sm_config cfg = servo_output_program_get_default_config(offset);
    sm_config_set_out_pins(&cfg, kFirstServoGpio, kServoGpioSpan);
    sm_config_set_set_pins(&cfg, kFirstServoGpio, kServoGpioSpan);
    sm_config_set_out_shift(&cfg, false, true, 32);
    sm_config_set_clkdiv(&cfg, 125.0f);
    for (std::uint8_t pin = kFirstServoGpio; pin < kFirstServoGpio + kServoGpioSpan; ++pin) {
        pio_gpio_init(pio, pin);
    }
    pio_sm_set_consecutive_pindirs(pio, 0, kFirstServoGpio, kServoGpioSpan, true);
    pio_sm_init(pio, 0, offset, &cfg);
    pio_sm_set_enabled(pio, 0, true);
    dma_channel_ = dma_claim_unused_channel(false);
#endif
    initialized_ = true;
    return true;
}

bool ServoController::submitPulses(const std::array<std::uint16_t, kServoCount>& pulses_us) {
    if (!initialized_ && !init()) {
        return false;
    }

    for (const std::uint16_t pulse : pulses_us) {
        if (pulse < 500 || pulse > kServoFrameUs) {
            return false;
        }
    }

    EventBuffer& buffer = buffers_[write_buffer_];
    const std::size_t count = buildEventBuffer(pulses_us, &buffer);
    if (!submitBuffer(buffer, count)) {
        return false;
    }
    write_buffer_ ^= 1u;
    last_pulses_us_ = pulses_us;
    return true;
}

std::size_t ServoController::buildEventBuffer(const std::array<std::uint16_t, kServoCount>& pulses_us,
                                              EventBuffer* buffer) const {
    if (buffer == nullptr) {
        return 0;
    }

    std::array<PulseEvent, kServoCount> events{};
    for (std::size_t i = 0; i < kServoCount; ++i) {
        events[i] = {pulses_us[i], 1u << i};
    }
    std::sort(events.begin(), events.end(), [](PulseEvent a, PulseEvent b) { return a.at_us < b.at_us; });

    std::size_t out = 0;
    std::uint32_t current_mask = kFullServoMask;
    std::uint16_t previous_us = 0;

    for (std::size_t i = 0; i < kServoCount;) {
        const std::uint16_t at_us = events[i].at_us;
        (*buffer)[out++] = {current_mask, static_cast<std::uint32_t>((at_us - previous_us) * kTicksPerUs)};
        std::uint32_t clear_mask = 0;
        while (i < kServoCount && events[i].at_us == at_us) {
            clear_mask |= events[i].clear_mask;
            ++i;
        }
        current_mask &= ~clear_mask;
        previous_us = at_us;
    }

    if (previous_us < kServoFrameUs) {
        (*buffer)[out++] = {current_mask, static_cast<std::uint32_t>((kServoFrameUs - previous_us) * kTicksPerUs)};
    }
    return out;
}

bool ServoController::submitBuffer(const EventBuffer& buffer, std::size_t count) {
    (void)buffer;
    (void)count;
#if defined(PICO_ON_DEVICE)
    if (dma_channel_ < 0 || dma_channel_is_busy(dma_channel_)) {
        return false;
    }
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel_);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, 0, true));
    dma_channel_configure(dma_channel_,
                          &cfg,
                          &pio0->txf[0],
                          buffer.data(),
                          count * 2,
                          true);
#endif
    return count > 0;
}

}  // namespace hexapod
