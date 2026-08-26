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

}  // namespace hexapod
