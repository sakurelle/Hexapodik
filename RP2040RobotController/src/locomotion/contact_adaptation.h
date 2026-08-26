#pragma once

#include <array>

#include "config/robot_config.h"
#include "math/vec3.h"

namespace hexapod {

struct ContactSample {
    bool raw_contact = false;
    bool debounced_contact = false;
    float stable_ms = 0.0f;
    float local_ground_z_mm = 0.0f;
};

class ContactDebouncer {
public:
    void reset(const std::array<bool, kLegCount>& raw_contacts);
    void update(const std::array<bool, kLegCount>& raw_contacts, float dt_s);

    bool contact(LegId leg) const { return samples_[indexOf(leg)].debounced_contact; }
    std::array<bool, kLegCount> contacts() const;
    const ContactSample& sample(LegId leg) const { return samples_[indexOf(leg)]; }
    void setLocalGroundZ(LegId leg, float z_mm) { samples_[indexOf(leg)].local_ground_z_mm = z_mm; }

private:
    std::array<ContactSample, kLegCount> samples_{};
};

struct ContactAdaptationResult {
    bool fault = false;
    FaultCode fault_code = FaultCode::None;
};

ContactAdaptationResult applyTouchdownAdaptation(LegId leg,
                                                 bool is_swing,
                                                 bool contact,
                                                 float expected_ground_z_mm,
                                                 Vec3* foot_target_body_mm,
                                                 ContactDebouncer* contacts);

}  // namespace hexapod
