#include "drivers/servo_controller.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "servo_output.pio.h"
#endif

namespace hexapod {
namespace {

constexpr std::uint32_t kTicksPerUs = 1u;
constexpr std::uint8_t kServoPioSm = 0;

struct PulseEvent {
    std::uint16_t at_us = 0;
    std::uint32_t clear_mask = 0;
};

#if defined(PICO_ON_DEVICE)
ServoController* g_dma_owner = nullptr;
#endif

void sortPulseEvents(std::array<PulseEvent, kServoCount>* events) {
    for (std::size_t i = 1; i < events->size(); ++i) {
        const PulseEvent key = (*events)[i];
        std::size_t j = i;
        while (j > 0 && key.at_us < (*events)[j - 1].at_us) {
            (*events)[j] = (*events)[j - 1];
            --j;
        }
        (*events)[j] = key;
    }
}

std::uint32_t pioDelayTicksForSegmentUs(std::uint32_t segment_us) {
    const std::uint32_t segment_ticks = segment_us * kTicksPerUs;
    if (segment_ticks <= ServoController::kPioSegmentOverheadTicks) {
        return 0;
    }
    return segment_ticks - ServoController::kPioSegmentOverheadTicks;
}

}  // namespace

bool ServoController::init() {
    last_pulses_us_.fill(1500);
    const std::size_t center_count = buildEventBuffer(last_pulses_us_, &buffers_[0]);
    event_counts_[0] = center_count;
    event_counts_[1] = center_count;
    buffers_[1] = buffers_[0];

#if defined(PICO_ON_DEVICE)
    PIO pio = pio0;
    const unsigned int offset = pio_add_program(pio, &servo_output_program);
    pio_sm_config cfg = servo_output_program_get_default_config(offset);

    pio_sm_set_enabled(pio, kServoPioSm, false);
    pio_sm_clear_fifos(pio, kServoPioSm);
    pio_sm_restart(pio, kServoPioSm);

    sm_config_set_out_pins(&cfg, kFirstServoGpio, kServoGpioSpan);
    sm_config_set_out_shift(&cfg, true, false, 32);
    sm_config_set_clkdiv(&cfg, static_cast<float>(clock_get_hz(clk_sys)) / 1000000.0f);

    for (std::uint8_t pin = kFirstServoGpio; pin < kFirstServoGpio + kServoGpioSpan; ++pin) {
        pio_gpio_init(pio, pin);
    }
    pio_sm_set_consecutive_pindirs(pio, kServoPioSm, kFirstServoGpio, kServoGpioSpan, true);
    pio_sm_set_pins_with_mask(pio, kServoPioSm, 0, kAllServoPinsMask << kFirstServoGpio);
    pio_sm_init(pio, kServoPioSm, offset, &cfg);
    pio_sm_clear_fifos(pio, kServoPioSm);
    pio_sm_restart(pio, kServoPioSm);

    dma_channel_ = dma_claim_unused_channel(false);
    if (dma_channel_ < 0) {
        return false;
    }

    g_dma_owner = this;
    dma_channel_set_irq0_enabled(dma_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, &ServoController::dmaIrqHandler);
    irq_set_enabled(DMA_IRQ_0, true);

    active_buffer_ = 0;
    pio_sm_set_enabled(pio, kServoPioSm, true);
    startDma(active_buffer_);
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

    EventBuffer next_buffer{};
    const std::size_t count = buildEventBuffer(pulses_us, &next_buffer);
    if (count == 0) {
        return false;
    }

#if defined(PICO_ON_DEVICE)
    const std::uint32_t irq_state = save_and_disable_interrupts();
    pending_buffer_ = active_buffer_ ^ 1u;
    buffers_[pending_buffer_] = next_buffer;
    event_counts_[pending_buffer_] = count;
    pending_valid_ = true;
    restore_interrupts(irq_state);
#else
    buffers_[active_buffer_] = next_buffer;
    event_counts_[active_buffer_] = count;
#endif

    last_pulses_us_ = pulses_us;
    return true;
}

std::size_t ServoController::buildEventBuffer(const std::array<std::uint16_t, kServoCount>& pulses_us,
                                              EventBuffer* buffer) {
    if (buffer == nullptr) {
        return 0;
    }

    std::array<PulseEvent, kServoCount> events{};
    for (std::size_t i = 0; i < kServoCount; ++i) {
        events[i] = {pulses_us[i], 1u << i};
    }
    sortPulseEvents(&events);

    std::size_t out = 0;
    std::uint32_t current_mask = kAllServoPinsMask;
    std::uint16_t previous_us = 0;

    for (std::size_t i = 0; i < kServoCount;) {
        const std::uint16_t at_us = events[i].at_us;
        (*buffer)[out++] = {current_mask, pioDelayTicksForSegmentUs(at_us - previous_us)};
        std::uint32_t clear_mask = 0;
        while (i < kServoCount && events[i].at_us == at_us) {
            clear_mask |= events[i].clear_mask;
            ++i;
        }
        current_mask &= ~clear_mask;
        previous_us = at_us;
    }

    if (previous_us < kServoFrameUs) {
        (*buffer)[out++] = {current_mask, pioDelayTicksForSegmentUs(kServoFrameUs - previous_us)};
    }
    return out;
}

void ServoController::startDma(std::uint8_t buffer_index) {
#if defined(PICO_ON_DEVICE)
    if (dma_channel_ < 0 || event_counts_[buffer_index] == 0) {
        return;
    }
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel_);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, kServoPioSm, true));
    dma_channel_configure(dma_channel_,
                          &cfg,
                          &pio0->txf[kServoPioSm],
                          buffers_[buffer_index].data(),
                          event_counts_[buffer_index] * 2,
                          true);
#else
    (void)buffer_index;
#endif
}

void ServoController::handleDmaComplete() {
#if defined(PICO_ON_DEVICE)
    if (dma_channel_ < 0 || (dma_hw->ints0 & (1u << dma_channel_)) == 0) {
        return;
    }
    dma_hw->ints0 = 1u << dma_channel_;

    if (pending_valid_) {
        active_buffer_ = pending_buffer_;
        pending_valid_ = false;
    }
    startDma(active_buffer_);
#endif
}

void ServoController::dmaIrqHandler() {
#if defined(PICO_ON_DEVICE)
    if (g_dma_owner != nullptr) {
        g_dma_owner->handleDmaComplete();
    }
#endif
}

}  // namespace hexapod
