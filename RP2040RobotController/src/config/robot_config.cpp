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
        case FaultCode::NoGround: return "FAULT_NO_GROUND";
        case FaultCode::ContactStuck: return "FAULT_CONTACT_STUCK";
        case FaultCode::IkInvalid: return "FAULT_IK_INVALID";
        case FaultCode::SupportLost: return "FAULT_SUPPORT_LOST";
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
        case StopReason::NoGround: return "NO_GROUND";
        case StopReason::EarlyCollision: return "EARLY_COLLISION";
        case StopReason::SupportLost: return "SUPPORT_LOST";
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

}  // namespace hexapod
