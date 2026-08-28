#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"

namespace hexapod {

struct ContactState {
    bool raw = false;
    bool stable = false;
    bool pressed_event = false;
    bool released_event = false;
    std::uint32_t stable_ms = 0;
    bool healthy = true;
    std::uint32_t stuck_count = 0;
    SensorHealth health = SensorHealth::Ok;
    float local_ground_z_mm = 0.0f;
};

class ContactManager {
public:
    void reset(const std::array<bool, kLegCount>& raw_contacts);
    void update(const std::array<bool, kLegCount>& raw_contacts, float dt_s);

    const std::array<ContactState, kLegCount>& states() const { return states_; }
    const ContactState& state(LegId leg) const { return states_[indexOf(leg)]; }
    std::array<bool, kLegCount> rawContacts() const;
    std::array<bool, kLegCount> stableContacts() const;
    bool stableContact(LegId leg) const { return states_[indexOf(leg)].stable; }
    void setLocalGroundZ(LegId leg, float z_mm) { states_[indexOf(leg)].local_ground_z_mm = z_mm; }
    void markSensorHealth(LegId leg, SensorHealth health);

private:
    std::array<ContactState, kLegCount> states_{};
    std::array<float, kLegCount> raw_stable_ms_{};
};

}  // namespace hexapod
