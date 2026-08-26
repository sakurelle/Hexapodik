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
                                                  const std::array<bool, kLegCount>& contacts,
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
    result.phase = accepted_output.phase;
    result.workspace_scale = accepted_scale;
    result.ik_error_counter = ik_error_counter_;
    result.feet_body_mm = valid ? accepted_output.feet_body_mm : last_valid_feet_;
    result.joints = last_valid_joints_;
    result.joints_valid = valid;
    return result;
}

}  // namespace hexapod
