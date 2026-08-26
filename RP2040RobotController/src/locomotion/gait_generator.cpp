#include "locomotion/gait_generator.h"

#include <algorithm>
#include <cmath>

#include "locomotion/support_polygon.h"

namespace hexapod {

float smootherstep(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float remapClamped(float value, float in_min, float in_max) {
    if (in_max <= in_min) {
        return value >= in_max ? 1.0f : 0.0f;
    }
    return std::clamp((value - in_min) / (in_max - in_min), 0.0f, 1.0f);
}

void GaitGenerator::reset(const std::array<Vec3, kLegCount>& neutral_feet_body_mm) {
    neutral_ = neutral_feet_body_mm;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        legs_[i] = {};
        legs_[i].target_body_mm = neutral_[i];
        legs_[i].swing_start_body_mm = neutral_[i];
        legs_[i].swing_end_body_mm = neutral_[i];
        legs_[i].expected_ground_z_mm = neutral_[i].z;
    }
    phase_ = 0.0f;
    mode_ = LocomotionMode::Idle;
    fault_ = FaultCode::None;
}

GaitOutput GaitGenerator::update(BodyCommand command,
                                 const std::array<bool, kLegCount>& contacts,
                                 float dt_s) {
    if (fault_ != FaultCode::None) {
        mode_ = LocomotionMode::Fault;
    } else if (mode_ == LocomotionMode::Idle && !commandIsZero(command)) {
        mode_ = LocomotionMode::Running;
    } else if (mode_ == LocomotionMode::Running && commandIsZero(command)) {
        mode_ = LocomotionMode::Stopping;
    }

    const float command_level = std::clamp(commandMagnitude(command) / 125.0f, 0.0f, 1.0f);
    const float cycle_hz = kMinCycleHz + (kMaxCycleHz - kMinCycleHz) * command_level;

    if (mode_ == LocomotionMode::Running || mode_ == LocomotionMode::Stopping) {
        phase_ += std::max(0.0f, dt_s) * cycle_hz;
        phase_ -= std::floor(phase_);
    }

    bool any_swing = false;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId leg = static_cast<LegId>(i);
        const float offset = legInTripodA(leg) ? 0.0f : 0.5f;
        float local_phase = phase_ + offset;
        local_phase -= std::floor(local_phase);
        const bool wants_swing = (mode_ == LocomotionMode::Running || mode_ == LocomotionMode::Stopping) &&
                                 local_phase >= kStanceDuty;

        if (!legs_[i].in_swing && wants_swing) {
            if (canStartSwing(leg, contacts)) {
                enterSwing(leg, command, cycle_hz);
            }
        }

        if (legs_[i].in_swing) {
            updateSwing(leg, local_phase, contacts);
            if (!wants_swing) {
                legs_[i].in_swing = false;
                legs_[i].touchdown_locked = false;
                legs_[i].target_body_mm = legs_[i].swing_end_body_mm;
            }
        } else if (mode_ == LocomotionMode::Running) {
            legs_[i].target_body_mm += stanceVelocityForFoot(command, legs_[i].target_body_mm) * dt_s;
        } else if (mode_ == LocomotionMode::Stopping || mode_ == LocomotionMode::Idle) {
            legs_[i].target_body_mm += (neutral_[i] - legs_[i].target_body_mm) *
                                       std::clamp(dt_s * 2.5f, 0.0f, 1.0f);
        }
        any_swing = any_swing || legs_[i].in_swing;
    }

    if (mode_ == LocomotionMode::Stopping && !any_swing) {
        bool near_neutral = true;
        for (std::size_t i = 0; i < kLegCount; ++i) {
            const Vec3 d = legs_[i].target_body_mm - neutral_[i];
            near_neutral = near_neutral && std::fabs(d.x) < 1.0f && std::fabs(d.y) < 1.0f && std::fabs(d.z) < 1.0f;
        }
        if (near_neutral) {
            mode_ = LocomotionMode::Idle;
            phase_ = 0.0f;
        }
    }

    GaitOutput out;
    out.mode = mode_;
    out.fault = fault_;
    out.phase = phase_;
    out.cycle_hz = cycle_hz;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        out.feet_body_mm[i] = legs_[i].target_body_mm;
        out.swing[i] = legs_[i].in_swing;
    }
    return out;
}

bool GaitGenerator::legInTripodA(LegId leg) const {
    return leg == LegId::FR || leg == LegId::ML || leg == LegId::RR;
}

bool GaitGenerator::canStartSwing(LegId leg, const std::array<bool, kLegCount>& contacts) const {
    std::array<bool, kLegCount> support{};
    std::array<Vec3, kLegCount> feet{};
    const bool lifting_a = legInTripodA(leg);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId other = static_cast<LegId>(i);
        support[i] = contacts[i] && (legInTripodA(other) != lifting_a);
        feet[i] = legs_[i].target_body_mm;
    }
    return hasStableSupport(feet, support, kSupportMarginMm);
}

Vec3 GaitGenerator::stanceVelocityForFoot(BodyCommand command, Vec3 foot_body_mm) const {
    return {
        -command.vx_mm_s + command.yaw_rad_s * foot_body_mm.y,
        -command.vy_mm_s - command.yaw_rad_s * foot_body_mm.x,
        0.0f,
    };
}

Vec3 GaitGenerator::landingTarget(LegId leg, BodyCommand command, float cycle_hz) const {
    const Vec3 neutral = neutral_[indexOf(leg)];
    const float stance_time_s = kStanceDuty / std::max(kMinCycleHz, cycle_hz);
    const Vec3 foot_velocity = stanceVelocityForFoot(command, neutral);
    return neutral - foot_velocity * (0.5f * stance_time_s);
}

void GaitGenerator::enterSwing(LegId leg, BodyCommand command, float cycle_hz) {
    LegGaitState& state = legs_[indexOf(leg)];
    state.in_swing = true;
    state.touchdown_locked = false;
    state.swing_start_body_mm = state.target_body_mm;
    state.swing_end_body_mm = landingTarget(leg, command, cycle_hz);
    state.expected_ground_z_mm = neutral_[indexOf(leg)].z;
}

void GaitGenerator::updateSwing(LegId leg, float local_phase, const std::array<bool, kLegCount>& contacts) {
    LegGaitState& state = legs_[indexOf(leg)];
    const float t = remapClamped(local_phase, kStanceDuty, 1.0f);
    const float xy_t = smootherstep(remapClamped(t, 0.12f, 0.82f));
    Vec3 target = state.swing_start_body_mm + (state.swing_end_body_mm - state.swing_start_body_mm) * xy_t;

    const float ground_z = state.expected_ground_z_mm;
    if (t <= 0.35f) {
        const float z_t = smootherstep(remapClamped(t, 0.0f, 0.35f));
        target.z = state.swing_start_body_mm.z + ((ground_z + kSwingHeightMm) - state.swing_start_body_mm.z) * z_t;
    } else if (t <= 0.82f) {
        const float z_t = smootherstep(remapClamped(t, 0.35f, 0.82f));
        target.z = (ground_z + kSwingHeightMm) + (ground_z - (ground_z + kSwingHeightMm)) * z_t;
    } else {
        const float z_t = smootherstep(remapClamped(t, 0.82f, 1.0f));
        target.z = ground_z - kGroundSearchMm * z_t;
    }

    if (contacts[indexOf(leg)]) {
        if (target.z > ground_z + kStuckContactLiftMm) {
            latchFault(FaultCode::ContactStuck);
        } else {
            state.touchdown_locked = true;
            state.swing_end_body_mm.z = target.z;
        }
    }

    if (!state.touchdown_locked && target.z <= ground_z - kGroundSearchMm + 0.2f) {
        latchFault(FaultCode::NoGround);
    }

    if (state.touchdown_locked && target.z < state.swing_end_body_mm.z) {
        target.z = state.swing_end_body_mm.z;
    }
    state.target_body_mm = target;
}

void GaitGenerator::latchFault(FaultCode fault) {
    if (fault_ == FaultCode::None) {
        fault_ = fault;
        mode_ = LocomotionMode::Fault;
    }
}

}  // namespace hexapod
