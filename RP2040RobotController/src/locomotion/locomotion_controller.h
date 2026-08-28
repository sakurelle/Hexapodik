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
    LegId fault_leg = LegId::Count;
    StopReason stop_reason = StopReason::None;
    LegId stop_leg = LegId::Count;
    float phase = 0.0f;
    float workspace_scale = 1.0f;
    std::uint32_t ik_error_counter = 0;
    bool support_ok = true;
    bool waiting_for_support = false;
    std::array<Vec3, kLegCount> feet_body_mm{};
    std::array<JointAngles, kLegCount> joints{};
    std::array<bool, kLegCount> swing{};
    std::array<LegPhase, kLegCount> leg_phase{};
    std::array<bool, kLegCount> released_this_swing{};
    std::array<SensorHealth, kLegCount> sensor_health{};
    std::array<float, kLegCount> local_ground_z_mm{};
    bool joints_valid = false;
};

class LocomotionController {
public:
    void reset();
    LocomotionStepResult update(BodyCommand requested_command,
                                const std::array<ContactState, kLegCount>& contacts,
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
