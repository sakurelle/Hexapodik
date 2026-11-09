#pragma once

#include <cstdint>

#include "command/command_source.h"
#include "core/hardware_interfaces.h"
#include "diagnostics/telemetry.h"
#include "locomotion/locomotion_controller.h"

namespace hexapod {

class LocomotionSystem {
public:
    LocomotionSystem(ICommandSource& commands,
                     IContactSource& contacts,
                     IServoOutput& servos,
                     CommandSourceMode source_mode = kCommandSourceMode);

    void reset();
    const Telemetry& update(std::uint64_t now_us, float dt_s);
    void resetJointVelocityDiagnostics();

    const Telemetry& telemetry() const { return telemetry_; }
    RobotState state() const { return telemetry_.robot_state; }
    BodyCommand filteredCommand() const { return locomotion_.filteredCommand(); }

private:
    RobotState mapState(const LocomotionStepResult& step, BodyCommand command, bool command_valid) const;
    BlockReason mapBlockReason(const LocomotionStepResult& step, BodyCommand command, bool command_valid) const;
    RecoveryReason mapRecoveryReason(StopReason reason, bool command_valid) const;
    std::uint32_t faultFlags(const LocomotionStepResult& step) const;
    float maxJointVelocityRadS(const std::array<JointAngles, kLegCount>& joints, float dt_s);

    ICommandSource& commands_;
    IContactSource& contacts_;
    IServoOutput& servos_;
    CommandSourceMode source_mode_;
    ControlMode control_mode_ = kDefaultControlMode;
    LocomotionController locomotion_{};
    Telemetry telemetry_{};
    std::uint64_t last_valid_command_us_ = 0;
    bool servo_fault_latched_ = false;
    bool have_previous_joints_ = false;
    std::array<JointAngles, kLegCount> previous_joints_{};
    float interval_max_joint_velocity_rad_s_ = 0.0f;
};

}  // namespace hexapod
