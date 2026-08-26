#include "locomotion/contact_adaptation.h"

#include <algorithm>

namespace hexapod {

void ContactDebouncer::reset(const std::array<bool, kLegCount>& raw_contacts) {
    for (std::size_t i = 0; i < kLegCount; ++i) {
        samples_[i].raw_contact = raw_contacts[i];
        samples_[i].debounced_contact = raw_contacts[i];
        samples_[i].stable_ms = kContactDebounceMs;
    }
}

void ContactDebouncer::update(const std::array<bool, kLegCount>& raw_contacts, float dt_s) {
    const float dt_ms = std::max(0.0f, dt_s * 1000.0f);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        ContactSample& sample = samples_[i];
        if (raw_contacts[i] == sample.raw_contact) {
            sample.stable_ms += dt_ms;
        } else {
            sample.raw_contact = raw_contacts[i];
            sample.stable_ms = 0.0f;
        }
        if (sample.stable_ms >= kContactDebounceMs) {
            sample.debounced_contact = sample.raw_contact;
        }
    }
}

std::array<bool, kLegCount> ContactDebouncer::contacts() const {
    std::array<bool, kLegCount> result{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        result[i] = samples_[i].debounced_contact;
    }
    return result;
}

ContactAdaptationResult applyTouchdownAdaptation(LegId leg,
                                                 bool is_swing,
                                                 bool contact,
                                                 float expected_ground_z_mm,
                                                 Vec3* foot_target_body_mm,
                                                 ContactDebouncer* contacts) {
    if (foot_target_body_mm == nullptr || contacts == nullptr || !is_swing || !contact) {
        return {};
    }
    contacts->setLocalGroundZ(leg, foot_target_body_mm->z);
    if (foot_target_body_mm->z < expected_ground_z_mm - kGroundSearchMm) {
        foot_target_body_mm->z = expected_ground_z_mm - kGroundSearchMm;
    }
    return {};
}

}  // namespace hexapod
