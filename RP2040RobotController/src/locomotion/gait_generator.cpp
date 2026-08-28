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

const char* legPhaseName(LegPhase phase) {
    switch (phase) {
        case LegPhase::Stance: return "S";
        case LegPhase::Lift: return "L";
        case LegPhase::Transfer: return "T";
        case LegPhase::Descend: return "D";
        case LegPhase::GroundSearch: return "G";
    }
    return "?";
}

void GaitGenerator::reset(const std::array<Vec3, kLegCount>& neutral_feet_body_mm) {
    neutral_ = neutral_feet_body_mm;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        legs_[i] = {};
        legs_[i].target_body_mm = neutral_[i];
        legs_[i].swing_start_body_mm = neutral_[i];
        legs_[i].swing_end_body_mm = neutral_[i];
        legs_[i].expected_ground_z_mm = neutral_[i].z;
        legs_[i].local_ground_z_mm = neutral_[i].z;
    }
    phase_ = 0.0f;
    mode_ = LocomotionMode::Idle;
    fault_ = FaultCode::None;
    fault_leg_ = LegId::Count;
    stop_reason_ = StopReason::None;
    stop_leg_ = LegId::Count;
}

GaitOutput GaitGenerator::update(BodyCommand command,
                                 const std::array<ContactState, kLegCount>& contacts,
                                 float dt_s) {
    if (fault_ != FaultCode::None) {
        mode_ = LocomotionMode::Fault;
    } else if (stop_reason_ != StopReason::None) {
        mode_ = LocomotionMode::Stopping;
        command = {};
    } else if (mode_ == LocomotionMode::Idle && !commandIsZero(command)) {
        mode_ = LocomotionMode::Running;
    } else if (mode_ == LocomotionMode::Running && commandIsZero(command)) {
        mode_ = LocomotionMode::Stopping;
    }

    const float command_level = std::clamp(commandMagnitude(command) / 125.0f, 0.0f, 1.0f);
    const float cycle_hz = kMinCycleHz + (kMaxCycleHz - kMinCycleHz) * command_level;

    if (mode_ == LocomotionMode::Fault) {
        return makeOutput(cycle_hz, true, false);
    }

    bool support_ok = true;
    bool waiting_for_support = false;
    float next_phase = phase_;
    if (mode_ == LocomotionMode::Running || mode_ == LocomotionMode::Stopping) {
        next_phase += std::max(0.0f, dt_s) * cycle_hz;
        next_phase -= std::floor(next_phase);
        if (swingWouldNeedSupport(next_phase, contacts)) {
            support_ok = false;
            waiting_for_support = true;
        } else {
            phase_ = next_phase;
        }
    }

    bool any_swing = false;
    const bool allow_new_swing = mode_ == LocomotionMode::Running &&
                                 stop_reason_ == StopReason::None &&
                                 !waiting_for_support;
    const float motion_dt_s = waiting_for_support ? 0.0f : dt_s;

    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId leg = static_cast<LegId>(i);
        const float offset = legInTripodA(leg) ? 0.0f : 0.5f;
        float local_phase = phase_ + offset;
        local_phase -= std::floor(local_phase);
        const bool wants_swing = allow_new_swing && local_phase >= kStanceDuty;

        if (!legs_[i].in_swing && wants_swing) {
            if (canStartSwing(leg, contacts)) {
                enterSwing(leg, command, cycle_hz);
            } else {
                support_ok = false;
                waiting_for_support = true;
            }
        }

        if (legs_[i].in_swing) {
            updateSwing(leg, local_phase, contacts[i], motion_dt_s);
        } else if (mode_ == LocomotionMode::Running) {
            legs_[i].target_body_mm += stanceVelocityForFoot(command, legs_[i].target_body_mm) * motion_dt_s;
        } else if (mode_ == LocomotionMode::Stopping || mode_ == LocomotionMode::Idle) {
            legs_[i].target_body_mm += (neutral_[i] - legs_[i].target_body_mm) *
                                       std::clamp(motion_dt_s * 2.5f, 0.0f, 1.0f);
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

    return makeOutput(cycle_hz, support_ok, waiting_for_support);
}

bool GaitGenerator::legInTripodA(LegId leg) const {
    return leg == LegId::FR || leg == LegId::ML || leg == LegId::RR;
}

bool GaitGenerator::canStartSwing(LegId leg, const std::array<ContactState, kLegCount>& contacts) const {
    if constexpr (kContactMode == ContactMode::Disabled || kContactMode == ContactMode::TouchdownOnly) {
        (void)leg;
        (void)contacts;
        return true;
    }

    std::array<bool, kLegCount> support{};
    std::array<Vec3, kLegCount> feet{};
    const bool lifting_a = legInTripodA(leg);
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId other = static_cast<LegId>(i);
        support[i] = contacts[i].stable && legs_[i].phase == LegPhase::Stance && (legInTripodA(other) != lifting_a);
        feet[i] = legs_[i].target_body_mm;
    }
    return hasStableSupport(feet, support, kSupportMarginMm);
}

bool GaitGenerator::swingWouldNeedSupport(float phase, const std::array<ContactState, kLegCount>& contacts) const {
    if constexpr (kContactMode == ContactMode::Disabled || kContactMode == ContactMode::TouchdownOnly) {
        (void)phase;
        (void)contacts;
        return false;
    }

    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId leg = static_cast<LegId>(i);
        if (legs_[i].in_swing) {
            continue;
        }
        const float offset = legInTripodA(leg) ? 0.0f : 0.5f;
        float local_phase = phase + offset;
        local_phase -= std::floor(local_phase);
        if (local_phase >= kStanceDuty && !canStartSwing(leg, contacts)) {
            return true;
        }
    }
    return false;
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
    state.released_this_swing = false;
    state.sensor_stuck_high_this_swing = false;
    state.phase = LegPhase::Lift;
    state.swing_start_body_mm = state.target_body_mm;
    state.swing_end_body_mm = landingTarget(leg, command, cycle_hz);
    state.expected_ground_z_mm = neutral_[indexOf(leg)].z;
    state.local_ground_z_mm = state.expected_ground_z_mm;
    state.swing_elapsed_ms = 0.0f;
}

void GaitGenerator::updateSwing(LegId leg, float local_phase, const ContactState& contact, float dt_s) {
    LegGaitState& state = legs_[indexOf(leg)];
    state.swing_elapsed_ms += std::max(0.0f, dt_s * 1000.0f);

    if (contact.released_event) {
        state.released_this_swing = true;
    }

    if (state.phase == LegPhase::GroundSearch) {
        if constexpr (kContactMode != ContactMode::Disabled) {
            if (state.released_this_swing && contact.pressed_event) {
                state.local_ground_z_mm = state.target_body_mm.z;
                finishSwingAtCurrentTarget(&state);
                return;
            }
        }

        const float min_z = state.expected_ground_z_mm - kGroundSearchMm;
        state.target_body_mm.z = std::max(min_z, state.target_body_mm.z - kGroundSearchSpeedMmS * std::max(0.0f, dt_s));
        if (state.target_body_mm.z <= min_z + 0.01f) {
            state.sensor_health = SensorHealth::SuspectStuckLow;
            requestStop(StopReason::NoGround, leg);
            finishSwingAtCurrentTarget(&state);
        }
        return;
    }

    if (local_phase < kStanceDuty) {
        if constexpr (kContactMode == ContactMode::Disabled) {
            finishOpenLoopSwing(leg);
        } else if (state.released_this_swing) {
            enterGroundSearch(&state);
            const float min_z = state.expected_ground_z_mm - kGroundSearchMm;
            state.target_body_mm.z = std::max(min_z, state.target_body_mm.z - kGroundSearchSpeedMmS * std::max(0.0f, dt_s));
            if (state.target_body_mm.z <= min_z + 0.01f) {
                state.sensor_health = SensorHealth::SuspectStuckLow;
                requestStop(StopReason::NoGround, leg);
                finishSwingAtCurrentTarget(&state);
            }
        } else {
            state.sensor_health = SensorHealth::SuspectStuckHigh;
            finishOpenLoopSwing(leg);
            requestStop(StopReason::SensorStuckHigh, leg);
        }
        return;
    }

    const float t = remapClamped(local_phase, kStanceDuty, 1.0f);
    const float xy_t = smootherstep(remapClamped(t, 0.12f, 0.82f));
    Vec3 target = state.swing_start_body_mm + (state.swing_end_body_mm - state.swing_start_body_mm) * xy_t;

    const float ground_z = state.expected_ground_z_mm;
    if (t <= 0.35f) {
        state.phase = LegPhase::Lift;
        const float z_t = smootherstep(remapClamped(t, 0.0f, 0.35f));
        target.z = state.swing_start_body_mm.z + ((ground_z + kSwingHeightMm) - state.swing_start_body_mm.z) * z_t;
        if (!state.released_this_swing &&
            (target.z >= ground_z + kContactReleaseCheckHeightMm ||
             state.swing_elapsed_ms >= static_cast<float>(kContactReleaseTimeoutMs))) {
            state.sensor_stuck_high_this_swing = true;
            state.sensor_health = SensorHealth::SuspectStuckHigh;
        }
    } else if (t <= 0.82f) {
        state.phase = LegPhase::Transfer;
        target.z = ground_z + kSwingHeightMm;
        if constexpr (kContactMode != ContactMode::Disabled) {
            if (state.released_this_swing && contact.pressed_event) {
                state.target_body_mm = target;
                state.local_ground_z_mm = target.z;
                requestStop(StopReason::EarlyCollision, leg);
                finishSwingAtCurrentTarget(&state);
                return;
            }
        }
    } else {
        state.phase = LegPhase::Descend;
        const float z_t = smootherstep(remapClamped(t, 0.82f, 1.0f));
        target.z = (ground_z + kSwingHeightMm) + (ground_z - (ground_z + kSwingHeightMm)) * z_t;
        if constexpr (kContactMode != ContactMode::Disabled) {
            if (state.released_this_swing && contact.pressed_event) {
                state.target_body_mm = target;
                state.local_ground_z_mm = target.z;
                finishSwingAtCurrentTarget(&state);
                return;
            }
        }
    }

    state.target_body_mm = target;
}

void GaitGenerator::enterGroundSearch(LegGaitState* state) {
    state->phase = LegPhase::GroundSearch;
    state->target_body_mm.x = state->swing_end_body_mm.x;
    state->target_body_mm.y = state->swing_end_body_mm.y;
    state->target_body_mm.z = state->expected_ground_z_mm;
}

void GaitGenerator::finishSwingAtCurrentTarget(LegGaitState* state) {
    state->in_swing = false;
    state->touchdown_locked = true;
    state->phase = LegPhase::Stance;
    state->swing_end_body_mm = state->target_body_mm;
}

void GaitGenerator::finishOpenLoopSwing(LegId leg) {
    LegGaitState& state = legs_[indexOf(leg)];
    state.target_body_mm = state.swing_end_body_mm;
    state.target_body_mm.z = state.expected_ground_z_mm;
    state.local_ground_z_mm = state.target_body_mm.z;
    finishSwingAtCurrentTarget(&state);
}

void GaitGenerator::requestStop(StopReason reason, LegId leg) {
    if (stop_reason_ == StopReason::None) {
        stop_reason_ = reason;
        stop_leg_ = leg;
        mode_ = LocomotionMode::Stopping;
    }
}

GaitOutput GaitGenerator::makeOutput(float cycle_hz, bool support_ok, bool waiting_for_support) const {
    GaitOutput out;
    out.mode = mode_;
    out.fault = fault_;
    out.fault_leg = fault_leg_;
    out.stop_reason = stop_reason_;
    out.stop_leg = stop_leg_;
    out.phase = phase_;
    out.cycle_hz = cycle_hz;
    out.support_ok = support_ok;
    out.waiting_for_support = waiting_for_support;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        out.feet_body_mm[i] = legs_[i].target_body_mm;
        out.swing[i] = legs_[i].in_swing;
        out.leg_phase[i] = legs_[i].phase;
        out.released_this_swing[i] = legs_[i].released_this_swing;
        out.sensor_health[i] = legs_[i].sensor_health;
        out.local_ground_z_mm[i] = legs_[i].local_ground_z_mm;
    }
    return out;
}

void GaitGenerator::latchFault(FaultCode fault, LegId leg) {
    if (fault_ == FaultCode::None) {
        fault_ = fault;
        fault_leg_ = leg;
        mode_ = LocomotionMode::Fault;
    }
}

}  // namespace hexapod
