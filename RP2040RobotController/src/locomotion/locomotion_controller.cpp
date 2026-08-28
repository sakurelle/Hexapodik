#include "locomotion/locomotion_controller.h"

#include "robot/robot_geometry.h"
#include "robot/workspace.h"

namespace hexapod {

void LocomotionController::reset() {
    const auto neutral = neutralFeetBody();
    gait_.reset(neutral);
    filtered_command_ = {};
    last_valid_feet_ = neutral;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const LegId leg = static_cast<LegId>(i);
        const IkResult ik = inverseKinematics(bodyToLeg(leg, neutral[i]), {});
        last_valid_joints_[i] = ik.valid ? ik.angles : JointAngles{};
    }
    initialized_ = true;
    ik_error_counter_ = 0;
}

LocomotionStepResult LocomotionController::update(BodyCommand requested_command,
                                                  const std::array<ContactState, kLegCount>& contacts,
                                                  float dt_s) {
    if (!initialized_) {
        reset();
    }

    filtered_command_ = slewLimit(filtered_command_, requested_command, dt_s);

    float accepted_scale = 1.0f;
    GaitOutput accepted_output{};
    std::array<JointAngles, kLegCount> accepted_joints{};

    auto candidateValid = [&](BodyCommand scaled_command) {
        GaitGenerator candidate = gait_;
        const GaitOutput output = candidate.update(scaled_command, contacts, dt_s);
        std::array<JointAngles, kLegCount> joints{};
        return validateAllFootTargets(output.feet_body_mm, last_valid_joints_, &joints);
    };

    if (!candidateValid(filtered_command_)) {
        accepted_scale = findWorkspaceScale(filtered_command_, candidateValid);
    }

    filtered_command_ = filtered_command_ * accepted_scale;
    accepted_output = gait_.update(filtered_command_, contacts, dt_s);
    if (accepted_output.stop_reason != StopReason::None) {
        filtered_command_ = {};
    }

    const bool valid = validateAllFootTargets(accepted_output.feet_body_mm, last_valid_joints_, &accepted_joints);
    if (valid) {
        last_valid_joints_ = accepted_joints;
        last_valid_feet_ = accepted_output.feet_body_mm;
    } else {
        ++ik_error_counter_;
    }

    LocomotionStepResult result;
    result.mode = accepted_output.mode;
    result.fault = valid ? accepted_output.fault : FaultCode::IkInvalid;
    result.fault_leg = accepted_output.fault_leg;
    result.stop_reason = accepted_output.stop_reason;
    result.stop_leg = accepted_output.stop_leg;
    result.phase = accepted_output.phase;
    result.workspace_scale = accepted_scale;
    result.ik_error_counter = ik_error_counter_;
    result.support_ok = accepted_output.support_ok;
    result.waiting_for_support = accepted_output.waiting_for_support;
    result.feet_body_mm = valid ? accepted_output.feet_body_mm : last_valid_feet_;
    result.joints = last_valid_joints_;
    result.swing = accepted_output.swing;
    result.leg_phase = accepted_output.leg_phase;
    result.released_this_swing = accepted_output.released_this_swing;
    result.sensor_health = accepted_output.sensor_health;
    result.local_ground_z_mm = accepted_output.local_ground_z_mm;
    result.joints_valid = valid;
    return result;
}

}  // namespace hexapod
