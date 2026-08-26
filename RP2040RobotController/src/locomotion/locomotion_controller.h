#pragma once

#include <array>

#include "config/robot_config.h"
#include "locomotion/contact_adaptation.h"
#include "locomotion/gait_generator.h"
#include "robot/kinematics.h"

namespace hexapod {

struct LocomotionStepResult {
    LocomotionMode mode = LocomotionMode::Idle;
    FaultCode fault = FaultCode::None;
    float phase = 0.0f;
    float workspace_scale = 1.0f;
    std::uint32_t ik_error_counter = 0;
    std::array<Vec3, kLegCount> feet_body_mm{};
    std::array<JointAngles, kLegCount> joints{};
    bool joints_valid = false;
};

class LocomotionController {
public:
    void reset();
    LocomotionStepResult update(BodyCommand requested_command,
                                const std::array<bool, kLegCount>& contacts,
                                float dt_s);

    BodyCommand filteredCommand() const { return filtered_command_; }
    std::uint32_t ikErrorCounter() const { return ik_error_counter_; }
    const std::array<JointAngles, kLegCount>& lastValidJoints() const { return last_valid_joints_; }

private:
    GaitGenerator gait_;
    BodyCommand filtered_command_{};
    std::array<JointAngles, kLegCount> last_valid_joints_{};
    std::array<Vec3, kLegCount> last_valid_feet_{};
    bool initialized_ = false;
    std::uint32_t ik_error_counter_ = 0;
};

}  // namespace hexapod
