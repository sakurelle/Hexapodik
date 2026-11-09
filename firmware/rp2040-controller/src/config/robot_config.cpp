#include "config/robot_config.h"

namespace hexapod {

const char* legName(LegId leg) {
    switch (leg) {
        case LegId::FL: return "FL";
        case LegId::ML: return "ML";
        case LegId::RL: return "RL";
        case LegId::RR: return "RR";
        case LegId::MR: return "MR";
        case LegId::FR: return "FR";
        case LegId::Count: break;
    }
    return "?";
}

const char* faultName(FaultCode fault) {
    switch (fault) {
        case FaultCode::None: return "NONE";
        case FaultCode::IkInvalid: return "FAULT_IK_INVALID";
        case FaultCode::WorkspaceInvalid: return "FAULT_WORKSPACE_INVALID";
        case FaultCode::InternalState: return "FAULT_INTERNAL_STATE";
        case FaultCode::ServoOutput: return "FAULT_SERVO_OUTPUT";
        case FaultCode::PioDmaFatal: return "FAULT_PIO_DMA_FATAL";
    }
    return "FAULT_UNKNOWN";
}

const char* locomotionModeName(LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Idle: return "IDLE";
        case LocomotionMode::Running: return "RUNNING";
        case LocomotionMode::Stopping: return "STOPPING";
        case LocomotionMode::Fault: return "FAULT";
    }
    return "?";
}

const char* contactModeName(ContactMode mode) {
    switch (mode) {
        case ContactMode::Disabled: return "DISABLED";
        case ContactMode::TouchdownOnly: return "TOUCHDOWN_ONLY";
        case ContactMode::FullTerrain: return "FULL_TERRAIN";
    }
    return "?";
}

const char* stopReasonName(StopReason reason) {
    switch (reason) {
        case StopReason::None: return "NONE";
        case StopReason::SensorStuckHigh: return "SENSOR_STUCK_HIGH";
        case StopReason::SensorStuckLow: return "SENSOR_STUCK_LOW";
        case StopReason::NoGround: return "NO_GROUND";
        case StopReason::EarlyCollision: return "EARLY_COLLISION";
        case StopReason::SupportLost: return "SUPPORT_LOST";
        case StopReason::CommandTimeout: return "COMMAND_TIMEOUT";
    }
    return "?";
}

const char* sensorHealthName(SensorHealth health) {
    switch (health) {
        case SensorHealth::Ok: return "OK";
        case SensorHealth::SuspectStuckHigh: return "SUSPECT_STUCK_HIGH";
        case SensorHealth::SuspectStuckLow: return "SUSPECT_STUCK_LOW";
        case SensorHealth::Unhealthy: return "UNHEALTHY";
    }
    return "?";
}

const char* robotStateName(RobotState state) {
    switch (state) {
        case RobotState::Disabled: return "DISABLED";
        case RobotState::Standing: return "STANDING";
        case RobotState::Walking: return "WALKING";
        case RobotState::Stopping: return "STOPPING";
        case RobotState::Recovery: return "RECOVERY";
        case RobotState::Fault: return "FAULT";
    }
    return "?";
}

const char* blockReasonName(BlockReason reason) {
    switch (reason) {
        case BlockReason::None: return "NONE";
        case BlockReason::CommandTimeout: return "COMMAND_TIMEOUT";
        case BlockReason::ZeroCommand: return "ZERO_COMMAND";
        case BlockReason::LandingWait: return "LANDING_WAIT";
        case BlockReason::GroundSearch: return "GROUND_SEARCH";
        case BlockReason::Recovery: return "RECOVERY";
        case BlockReason::IkInvalid: return "IK_INVALID";
        case BlockReason::WorkspaceInvalid: return "WORKSPACE_INVALID";
        case BlockReason::HardFault: return "HARD_FAULT";
    }
    return "?";
}

const char* recoveryReasonName(RecoveryReason reason) {
    switch (reason) {
        case RecoveryReason::None: return "NONE";
        case RecoveryReason::NoGround: return "NO_GROUND";
        case RecoveryReason::SensorStuckHigh: return "SENSOR_STUCK_HIGH";
        case RecoveryReason::SensorStuckLow: return "SENSOR_STUCK_LOW";
        case RecoveryReason::EarlyCollision: return "EARLY_COLLISION";
        case RecoveryReason::CommandTimeout: return "COMMAND_TIMEOUT";
    }
    return "?";
}

const char* commandSourceModeName(CommandSourceMode mode) {
    switch (mode) {
        case CommandSourceMode::RcPwm: return "RC";
        case CommandSourceMode::Uart: return "UART";
    }
    return "?";
}

const char* controlModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::Locomotion: return "LOCOMOTION";
        case ControlMode::DirectJoint: return "DIRECT_JOINT";
    }
    return "?";
}

}  // namespace hexapod
