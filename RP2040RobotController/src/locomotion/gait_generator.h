#pragma once

#include <array>

#include "config/robot_config.h"
#include "locomotion/body_command.h"
#include "math/vec3.h"

namespace hexapod {

struct LegGaitState {
    bool in_swing = false;
    bool touchdown_locked = false;
    Vec3 target_body_mm;
    Vec3 swing_start_body_mm;
    Vec3 swing_end_body_mm;
    float expected_ground_z_mm = 0.0f;
};

struct GaitOutput {
    LocomotionMode mode = LocomotionMode::Idle;
    FaultCode fault = FaultCode::None;
    float phase = 0.0f;
    float cycle_hz = kMinCycleHz;
    std::array<Vec3, kLegCount> feet_body_mm{};
    std::array<bool, kLegCount> swing{};
};

class GaitGenerator {
public:
    void reset(const std::array<Vec3, kLegCount>& neutral_feet_body_mm);
    GaitOutput update(BodyCommand command,
                      const std::array<bool, kLegCount>& contacts,
                      float dt_s);

    const std::array<LegGaitState, kLegCount>& legStates() const { return legs_; }
    LocomotionMode mode() const { return mode_; }
    FaultCode fault() const { return fault_; }
    float phase() const { return phase_; }

private:
    bool legInTripodA(LegId leg) const;
    bool canStartSwing(LegId leg, const std::array<bool, kLegCount>& contacts) const;
    Vec3 stanceVelocityForFoot(BodyCommand command, Vec3 foot_body_mm) const;
    Vec3 landingTarget(LegId leg, BodyCommand command, float cycle_hz) const;
    void enterSwing(LegId leg, BodyCommand command, float cycle_hz);
    void updateSwing(LegId leg, float local_phase, const std::array<bool, kLegCount>& contacts);
    void latchFault(FaultCode fault);

    std::array<Vec3, kLegCount> neutral_{};
    std::array<LegGaitState, kLegCount> legs_{};
    LocomotionMode mode_ = LocomotionMode::Idle;
    FaultCode fault_ = FaultCode::None;
    float phase_ = 0.0f;
};

float smootherstep(float t);
float remapClamped(float value, float in_min, float in_max);

}  // namespace hexapod
