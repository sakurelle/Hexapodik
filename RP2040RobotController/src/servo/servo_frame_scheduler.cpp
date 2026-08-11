#include "servo/servo_frame_scheduler.hpp"

namespace servo {

struct Edge {
    uint32_t time_us;
    uint32_t set_bits;
    uint32_t clear_bits;
};

static bool gpio_in_group(uint8_t gpio, PioPinGroup group) {
    return gpio >= group.first_gpio && gpio < group.first_gpio + group.channel_count;
}

static void add_edge(std::array<Edge, MAX_EVENTS_PER_GROUP> &edges, size_t &count,
                     uint32_t time_us, uint32_t set_bits, uint32_t clear_bits) {
    for (size_t i = 0; i < count; ++i) {
        if (edges[i].time_us == time_us) {
            edges[i].set_bits |= set_bits;
            edges[i].clear_bits |= clear_bits;
            return;
        }
    }
    if (count < edges.size()) {
        edges[count++] = Edge{time_us, set_bits, clear_bits};
    }
}

static void sort_edges(std::array<Edge, MAX_EVENTS_PER_GROUP> &edges, size_t count) {
    for (size_t i = 1; i < count; ++i) {
        Edge key = edges[i];
        size_t j = i;
        while (j > 0 && edges[j - 1].time_us > key.time_us) {
            edges[j] = edges[j - 1];
            --j;
        }
        edges[j] = key;
    }
}

EventFrame build_event_frame(const std::array<ServoPulse, SERVO_COUNT> &pulses, PioPinGroup group) {
    std::array<Edge, MAX_EVENTS_PER_GROUP> edges{};
    size_t edge_count = 0;
    bool valid = true;

    add_edge(edges, edge_count, 0, 0, 0xffffffffu);

    for (const auto &pulse : pulses) {
        if (!pulse.enabled || !gpio_in_group(pulse.gpio, group)) {
            continue;
        }
        const uint32_t phase = LEG_PHASE_OFFSET_US[leg_index(pulse.leg)];
        const uint32_t end = phase + pulse.pulse_us;
        if (pulse.pulse_us < SAFE_MIN_PULSE_US || pulse.pulse_us > SAFE_MAX_PULSE_US || end > FRAME_PERIOD_US) {
            valid = false;
            continue;
        }
        const uint32_t bit = 1u << (pulse.gpio - group.first_gpio);
        add_edge(edges, edge_count, phase, bit, 0);
        add_edge(edges, edge_count, end, 0, bit);
    }

    add_edge(edges, edge_count, FRAME_PERIOD_US, 0, 0xffffffffu);
    sort_edges(edges, edge_count);

    EventFrame frame{};
    frame.valid = valid;
    uint32_t mask = 0;
    for (size_t i = 0; i < edge_count && frame.count < frame.events.size(); ++i) {
        mask &= ~edges[i].clear_bits;
        mask |= edges[i].set_bits;
        if (frame.count > 0 && frame.events[frame.count - 1].time_us == edges[i].time_us) {
            frame.events[frame.count - 1].mask = mask;
        } else {
            frame.events[frame.count++] = MaskEvent{edges[i].time_us, mask};
        }
    }

    if (edge_count > frame.events.size()) {
        frame.valid = false;
    }
    return frame;
}

PioFrameWords build_pio_words(const EventFrame &frame) {
    PioFrameWords words{};
    words.valid = frame.valid && frame.count >= 2;

    for (size_t i = 0; i < frame.count && words.count + 2 <= words.words.size(); ++i) {
        const uint32_t now = frame.events[i].time_us;
        const uint32_t next = (i + 1 < frame.count) ? frame.events[i + 1].time_us : FRAME_PERIOD_US;
        if (next < now || next > FRAME_PERIOD_US) {
            words.valid = false;
            break;
        }
        const uint32_t delta = next - now;
        const uint32_t pio_delay = delta > PIO_DELAY_COMPENSATION_US ? delta - PIO_DELAY_COMPENSATION_US : 0;
        words.words[words.count++] = frame.events[i].mask;
        words.words[words.count++] = pio_delay;
    }

    return words;
}

} // namespace servo
