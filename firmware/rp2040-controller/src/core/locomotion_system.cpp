#include "core/locomotion_system.h"

#include <algorithm>
#include <cmath>

namespace hexapod {

LocomotionSystem::LocomotionSystem(ICommandSource& commands,
                                   IContactSource& contacts,
                                   IServoOutput& servos,
                                   CommandSourceMode source_mode)
    : commands_(commands), contacts_(contacts), servos_(servos), source_mode_(source_mode) {}

void LocomotionSystem::reset() {
    locomotion_.reset();
    telemetry_ = {};
    telemetry_.robot_state = RobotState::Standing;
    telemetry_.command_source = source_mode_;
    telemetry_.control_mode = control_mode_;
    last_valid_command_us_ = 0;
    servo_fault_latched_ = false;
    have_previous_joints_ = false;
    interval_max_joint_velocity_rad_s_ = 0.0f;
}

const Telemetry& LocomotionSystem::update(std::uint64_t now_us, float dt_s) {
    const float safe_dt_s = dt_s > 0.0f ? dt_s : kControlPeriodS;
    contacts_.update(safe_dt_s);
    const auto& contact_states = contacts_.states();

    MotionCommand requested = commands_.readMotionCommand(now_us);
    if (requested.valid) {
        last_valid_command_us_ = now_us;
        telemetry_.last_command_sequence = requested.sequence;
    }

    const BodyCommand body_command = toBodyCommandMm(requested);
    const LocomotionStepResult step = locomotion_.update(body_command, contact_states, safe_dt_s);
    interval_max_joint_velocity_rad_s_ = std::max(interval_max_joint_velocity_rad_s_,
                                                  maxJointVelocityRadS(step.joints, safe_dt_s));
    if (step.joints_valid && !servos_.submitJointTargets(step.joints)) {
        servo_fault_latched_ = true;
    }

    telemetry_.command_source = source_mode_;
    telemetry_.control_mode = control_mode_;
    telemetry_.requested_command = requested;
    telemetry_.filtered_command = locomotion_.filteredCommand();
    telemetry_.command_age_ms = last_valid_command_us_ == 0 || now_us < last_valid_command_us_
                                    ? kCommandWatchdogTimeoutUs / 1000u
                                    : static_cast<std::uint32_t>((now_us - last_valid_command_us_) / 1000u);
    telemetry_.robot_state = servo_fault_latched_ ? RobotState::Fault : mapState(step, body_command, requested.valid);
    telemetry_.block_reason = servo_fault_latched_ ? BlockReason::HardFault
                                                   : mapBlockReason(step, body_command, requested.valid);
    telemetry_.recovery_reason = servo_fault_latched_ ? RecoveryReason::None
                                                      : mapRecoveryReason(step.stop_reason, requested.valid);
    telemetry_.phase = step.phase;
    telemetry_.cycle_hz = step.cycle_hz;
    telemetry_.command_level = step.command_level;
    telemetry_.stride_x_mm = step.stride_x_mm;
    telemetry_.stride_y_mm = step.stride_y_mm;
    telemetry_.workspace_scale = step.workspace_scale;
    telemetry_.max_requested_joint_velocity_rad_s = interval_max_joint_velocity_rad_s_;
    telemetry_.max_requested_joint_velocity_deg_s = radToDeg(interval_max_joint_velocity_rad_s_);
    telemetry_.ik_error_counter = step.ik_error_counter;
    telemetry_.fault_flags = faultFlags(step) | (servo_fault_latched_ ? (1u << 6u) : 0u);
    telemetry_.foot_targets_mm = step.feet_body_mm;
    telemetry_.joint_targets_rad = step.joints;
    telemetry_.leg_phase = step.leg_phase;
    telemetry_.raw_contact_mask = contactRawMask(contact_states);
    telemetry_.stable_contact_mask = contactStableMask(contact_states);
    telemetry_.healthy_sensor_mask = contactHealthyMask(contact_states);
    telemetry_.support_mask = supportMask(step);
    return telemetry_;
}

void LocomotionSystem::resetJointVelocityDiagnostics() {
    interval_max_joint_velocity_rad_s_ = 0.0f;
}

RobotState LocomotionSystem::mapState(const LocomotionStepResult& step, BodyCommand command, bool command_valid) const {
    if (step.fault != FaultCode::None) {
        return RobotState::Fault;
    }
    if (step.stop_reason == StopReason::NoGround ||
        step.stop_reason == StopReason::SensorStuckHigh ||
        step.stop_reason == StopReason::SensorStuckLow ||
        step.stop_reason == StopReason::EarlyCollision ||
        (!command_valid && !commandIsZero(command))) {
        return RobotState::Recovery;
    }
    if (step.mode == LocomotionMode::Running) {
        return RobotState::Walking;
    }
    if (step.mode == LocomotionMode::Stopping) {
        return RobotState::Stopping;
    }
    return RobotState::Standing;
}

BlockReason LocomotionSystem::mapBlockReason(const LocomotionStepResult& step,
                                             BodyCommand command,
                                             bool command_valid) const {
    if (step.fault == FaultCode::IkInvalid) {
        return BlockReason::IkInvalid;
    }
    if (step.fault != FaultCode::None) {
        return BlockReason::HardFault;
    }
    if (!command_valid) {
        return BlockReason::CommandTimeout;
    }
    if (step.waiting_for_support) {
        return BlockReason::LandingWait;
    }
    for (LegPhase phase : step.leg_phase) {
        if (phase == LegPhase::GroundSearch) {
            return BlockReason::GroundSearch;
        }
    }
    if (step.stop_reason != StopReason::None) {
        return BlockReason::Recovery;
    }
    return commandIsZero(command) ? BlockReason::ZeroCommand : BlockReason::None;
}

RecoveryReason LocomotionSystem::mapRecoveryReason(StopReason reason, bool command_valid) const {
    if (!command_valid) {
        return RecoveryReason::CommandTimeout;
    }
    switch (reason) {
        case StopReason::NoGround: return RecoveryReason::NoGround;
        case StopReason::SensorStuckHigh: return RecoveryReason::SensorStuckHigh;
        case StopReason::SensorStuckLow: return RecoveryReason::SensorStuckLow;
        case StopReason::EarlyCollision: return RecoveryReason::EarlyCollision;
        case StopReason::CommandTimeout: return RecoveryReason::CommandTimeout;
        case StopReason::None:
        case StopReason::SupportLost:
            return RecoveryReason::None;
    }
    return RecoveryReason::None;
}

std::uint32_t LocomotionSystem::faultFlags(const LocomotionStepResult& step) const {
    if (step.fault == FaultCode::None) {
        return 0;
    }
    return 1u << static_cast<std::uint8_t>(step.fault);
}

float LocomotionSystem::maxJointVelocityRadS(const std::array<JointAngles, kLegCount>& joints, float dt_s) {
    if (dt_s <= 0.0f) {
        return 0.0f;
    }
    if (!have_previous_joints_) {
        previous_joints_ = joints;
        have_previous_joints_ = true;
        return 0.0f;
    }

    float max_velocity = 0.0f;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        max_velocity = std::max(max_velocity, std::fabs(joints[i].coxa_rad - previous_joints_[i].coxa_rad) / dt_s);
        max_velocity = std::max(max_velocity, std::fabs(joints[i].femur_rad - previous_joints_[i].femur_rad) / dt_s);
        max_velocity = std::max(max_velocity, std::fabs(joints[i].tibia_rad - previous_joints_[i].tibia_rad) / dt_s);
    }
    previous_joints_ = joints;
    return max_velocity;
}

}  // namespace hexapod
