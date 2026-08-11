#include "servo/pio_servo_driver.hpp"

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "servo_14ch.pio.h"
#include "servo_4ch.pio.h"

#include <cstring>

namespace servo {

static PIO kPio = pio0;

bool PioServoDriver::init() {
    if (initialized_) {
        return true;
    }

    const uint offset14 = pio_add_program(kPio, &servo_14ch_program);
    const uint offset4 = pio_add_program(kPio, &servo_4ch_program);
    servo_14ch_program_init(kPio, group_a_.sm, offset14, group_a_.pins.first_gpio);
    servo_4ch_program_init(kPio, group_b_.sm, offset4, group_b_.pins.first_gpio);

    group_a_.dma_channel = dma_claim_unused_channel(false);
    group_b_.dma_channel = dma_claim_unused_channel(false);
    if (group_a_.dma_channel < 0 || group_b_.dma_channel < 0) {
        ++error_count_;
        return false;
    }

    force_low();
    initialized_ = true;
    return true;
}

void PioServoDriver::start() {
    if (!initialized_) {
        return;
    }
    pio_sm_set_enabled(kPio, group_a_.sm, true);
    pio_sm_set_enabled(kPio, group_b_.sm, true);
}

void PioServoDriver::stop() {
    stop_group(group_a_);
    stop_group(group_b_);
    outputs_enabled_ = false;
    force_low();
}

void PioServoDriver::stop_group(GroupState &state) {
    if (state.dma_channel >= 0) {
        dma_channel_abort(state.dma_channel);
    }
    pio_sm_set_enabled(kPio, state.sm, false);
    pio_sm_clear_fifos(kPio, state.sm);
    pio_sm_restart(kPio, state.sm);
    state.running = false;
    state.has_pending = false;
}

void PioServoDriver::force_low() {
    const uint32_t mask_a = ((1u << group_a_.pins.channel_count) - 1u) << group_a_.pins.first_gpio;
    const uint32_t mask_b = ((1u << group_b_.pins.channel_count) - 1u) << group_b_.pins.first_gpio;
    pio_sm_set_pins_with_mask(kPio, group_a_.sm, 0, mask_a);
    pio_sm_set_pins_with_mask(kPio, group_b_.sm, 0, mask_b);
}

bool PioServoDriver::load_group(GroupState &state, const PioFrameWords &words) {
    if (!words.valid || words.count == 0 || words.count > MAX_PIO_WORDS_PER_GROUP) {
        ++error_count_;
        return false;
    }
    auto &buffer = state.buffers[state.pending];
    memset(buffer.data(), 0, buffer.size() * sizeof(uint32_t));
    memcpy(buffer.data(), words.words.data(), words.count * sizeof(uint32_t));
    state.lengths[state.pending] = words.count;
    state.has_pending = true;
    return true;
}

bool PioServoDriver::submit_pulses(const std::array<ServoPulse, SERVO_COUNT> &pulses, bool outputs_enabled) {
    outputs_enabled_ = outputs_enabled;
    if (!outputs_enabled) {
        stop();
        return true;
    }

    const auto events_a = build_event_frame(pulses, GROUP_A);
    const auto events_b = build_event_frame(pulses, GROUP_B);
    const bool ok_a = load_group(group_a_, build_pio_words(events_a));
    const bool ok_b = load_group(group_b_, build_pio_words(events_b));
    return ok_a && ok_b;
}

void PioServoDriver::start_dma(GroupState &state) {
    if (state.has_pending) {
        const uint8_t old_active = state.active;
        state.active = state.pending;
        state.pending = old_active;
        state.has_pending = false;
    }

    dma_channel_config config = dma_channel_get_default_config(state.dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(kPio, state.sm, true));

    dma_channel_configure(state.dma_channel, &config,
                          &kPio->txf[state.sm],
                          state.buffers[state.active].data(),
                          state.lengths[state.active],
                          true);
    state.running = true;
}

void PioServoDriver::service() {
    if (!initialized_ || !outputs_enabled_) {
        return;
    }

    if (!group_a_.running || !dma_channel_is_busy(group_a_.dma_channel)) {
        start_dma(group_a_);
    }
    if (!group_b_.running || !dma_channel_is_busy(group_b_.dma_channel)) {
        start_dma(group_b_);
        ++frame_counter_;
    }
}

} // namespace servo
