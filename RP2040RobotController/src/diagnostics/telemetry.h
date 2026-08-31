#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "command/motion_command.h"
#include "config/robot_config.h"
#include "locomotion/locomotion_controller.h"

namespace hexapod {

struct Telemetry {
    RobotState robot_state = RobotState::Standing;
    BlockReason block_reason = BlockReason::None;
    RecoveryReason recovery_reason = RecoveryReason::None;
    CommandSourceMode command_source = CommandSourceMode::RcPwm;
    ControlMode control_mode = ControlMode::Locomotion;

    std::uint32_t last_command_sequence = 0;
    std::uint32_t command_age_ms = 0;
    MotionCommand requested_command{};
    BodyCommand filtered_command{};

    float phase = 0.0f;
    float cycle_hz = kMinCycleHz;
    float command_level = 0.0f;
    float stride_x_mm = 0.0f;
    float stride_y_mm = 0.0f;
    float workspace_scale = 1.0f;
    float max_requested_joint_velocity_rad_s = 0.0f;
    float max_requested_joint_velocity_deg_s = 0.0f;
    std::uint32_t ik_error_counter = 0;
    std::uint32_t fault_flags = 0;

    std::array<Vec3, kLegCount> foot_targets_mm{};
    std::array<JointAngles, kLegCount> joint_targets_rad{};
    std::array<LegPhase, kLegCount> leg_phase{};

    std::uint32_t raw_contact_mask = 0;
    std::uint32_t stable_contact_mask = 0;
    std::uint32_t healthy_sensor_mask = 0;
    std::uint32_t support_mask = 0;
};

std::uint32_t contactRawMask(const std::array<ContactState, kLegCount>& contacts);
std::uint32_t contactStableMask(const std::array<ContactState, kLegCount>& contacts);
std::uint32_t contactHealthyMask(const std::array<ContactState, kLegCount>& contacts);
std::uint32_t supportMask(const LocomotionStepResult& step);
int formatTelemetryLine(const Telemetry& telemetry, char* buffer, std::size_t buffer_size);

}  // namespace hexapod
