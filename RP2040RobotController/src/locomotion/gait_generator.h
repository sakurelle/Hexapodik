#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "locomotion/body_command.h"
#include "locomotion/contact_adaptation.h"
#include "math/vec3.h"

namespace hexapod {

enum class LegPhase : std::uint8_t {
    Stance = 0,
    Lift,
    Transfer,
    Descend,
    GroundSearch,
    LandedHold,
};

struct LegGaitState {
    bool in_swing = false;
    bool touchdown_locked = false;
    bool released_this_swing = false;
    bool sensor_stuck_high_this_swing = false;
    LegPhase phase = LegPhase::Stance;
    SensorHealth sensor_health = SensorHealth::Ok;
    Vec3 target_body_mm;
    Vec3 swing_start_body_mm;
    Vec3 swing_end_body_mm;
    float expected_ground_z_mm = 0.0f;
    float local_ground_z_mm = 0.0f;
    float swing_elapsed_ms = 0.0f;
};

struct GaitOutput {
    LocomotionMode mode = LocomotionMode::Idle;
    FaultCode fault = FaultCode::None;
    LegId fault_leg = LegId::Count;
    StopReason stop_reason = StopReason::None;
    LegId stop_leg = LegId::Count;
    float phase = 0.0f;
    float cycle_hz = kMinCycleHz;
    float command_level = 0.0f;
    bool support_ok = true;
    bool waiting_for_support = false;
    std::array<Vec3, kLegCount> feet_body_mm{};
    std::array<bool, kLegCount> swing{};
    std::array<LegPhase, kLegCount> leg_phase{};
    std::array<bool, kLegCount> released_this_swing{};
    std::array<SensorHealth, kLegCount> sensor_health{};
    std::array<float, kLegCount> local_ground_z_mm{};
};

class GaitGenerator {
public:
    void reset(const std::array<Vec3, kLegCount>& neutral_feet_body_mm);
    GaitOutput update(BodyCommand command,
                      const std::array<ContactState, kLegCount>& contacts,
                      float dt_s);

    const std::array<LegGaitState, kLegCount>& legStates() const { return legs_; }
    LocomotionMode mode() const { return mode_; }
    FaultCode fault() const { return fault_; }
    LegId faultLeg() const { return fault_leg_; }
    StopReason stopReason() const { return stop_reason_; }
    LegId stopLeg() const { return stop_leg_; }
    float phase() const { return phase_; }

private:
    bool legInTripodA(LegId leg) const;
    bool canStartSwing(LegId leg, const std::array<ContactState, kLegCount>& contacts) const;
    bool swingWouldNeedSupport(float phase, const std::array<ContactState, kLegCount>& contacts) const;
    Vec3 stanceVelocityForFoot(BodyCommand command, Vec3 foot_body_mm) const;
    Vec3 landingTarget(LegId leg, BodyCommand command, float cycle_hz) const;
    void enterSwing(LegId leg, BodyCommand command, float cycle_hz);
    void updateSwing(LegId leg, float local_phase, const ContactState& contact, float dt_s);
    void enterGroundSearch(LegGaitState* state);
    void finishSwingAtCurrentTarget(LegGaitState* state);
    void finishOpenLoopSwing(LegId leg);
    void requestStop(StopReason reason, LegId leg);
    GaitOutput makeOutput(float cycle_hz, float command_level, bool support_ok, bool waiting_for_support) const;
    void latchFault(FaultCode fault, LegId leg);

    std::array<Vec3, kLegCount> neutral_{};
    std::array<LegGaitState, kLegCount> legs_{};
    LocomotionMode mode_ = LocomotionMode::Idle;
    FaultCode fault_ = FaultCode::None;
    LegId fault_leg_ = LegId::Count;
    StopReason stop_reason_ = StopReason::None;
    LegId stop_leg_ = LegId::Count;
    float phase_ = 0.0f;
};

float smootherstep(float t);
float remapClamped(float value, float in_min, float in_max);
const char* legPhaseName(LegPhase phase);

}  // namespace hexapod
