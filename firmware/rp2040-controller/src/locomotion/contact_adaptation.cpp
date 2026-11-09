#include "locomotion/contact_adaptation.h"

#include <algorithm>

namespace hexapod {

void ContactManager::reset(const std::array<bool, kLegCount>& raw_contacts) {
    const std::uint32_t startup_stable_ms = std::max(kContactPressDebounceMs, kContactReleaseDebounceMs);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        states_[i] = {};
        states_[i].raw = raw_contacts[i];
        states_[i].stable = raw_contacts[i];
        states_[i].stable_ms = startup_stable_ms;
        states_[i].healthy = true;
        states_[i].health = SensorHealth::Ok;
        raw_stable_ms_[i] = static_cast<float>(startup_stable_ms);
    }
}

void ContactManager::update(const std::array<bool, kLegCount>& raw_contacts, float dt_s) {
    const float dt_ms = std::max(0.0f, dt_s * 1000.0f);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        ContactState& state = states_[i];
        state.pressed_event = false;
        state.released_event = false;

        if (raw_contacts[i] == state.raw) {
            raw_stable_ms_[i] += dt_ms;
        } else {
            state.raw = raw_contacts[i];
            raw_stable_ms_[i] = dt_ms;
        }
        state.stable_ms = static_cast<std::uint32_t>(raw_stable_ms_[i] + 0.5f);

        const std::uint32_t threshold_ms = state.raw ? kContactPressDebounceMs : kContactReleaseDebounceMs;
        if (state.raw != state.stable && raw_stable_ms_[i] >= static_cast<float>(threshold_ms)) {
            const bool previous_stable = state.stable;
            state.stable = state.raw;
            state.pressed_event = !previous_stable && state.stable;
            state.released_event = previous_stable && !state.stable;
        }
    }
}

std::array<bool, kLegCount> ContactManager::rawContacts() const {
    std::array<bool, kLegCount> result{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        result[i] = states_[i].raw;
    }
    return result;
}

std::array<bool, kLegCount> ContactManager::stableContacts() const {
    std::array<bool, kLegCount> result{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        result[i] = states_[i].stable;
    }
    return result;
}

void ContactManager::markSensorHealth(LegId leg, SensorHealth health) {
    ContactState& state = states_[indexOf(leg)];
    if (health != SensorHealth::Ok) {
        ++state.stuck_count;
    }
    state.health = health;
    state.healthy = health == SensorHealth::Ok;
}

}  // namespace hexapod
