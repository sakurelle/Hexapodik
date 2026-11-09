#include "robot/workspace.h"

#include "robot/robot_geometry.h"

namespace hexapod {

WorkspaceCheck validateFootTarget(LegId leg, Vec3 foot_body_mm, JointAngles previous_joints) {
    const Vec3 foot_leg_mm = bodyToLeg(leg, foot_body_mm);
    const IkResult ik = inverseKinematics(foot_leg_mm, previous_joints);
    if (!ik.valid) {
        return {};
    }
    return {true, ik.angles};
}

bool validateAllFootTargets(const std::array<Vec3, kLegCount>& feet_body_mm,
                            const std::array<JointAngles, kLegCount>& previous_joints,
                            std::array<JointAngles, kLegCount>* out_joints) {
    std::array<JointAngles, kLegCount> joints{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        const auto check = validateFootTarget(static_cast<LegId>(i), feet_body_mm[i], previous_joints[i]);
        if (!check.valid) {
            return false;
        }
        joints[i] = check.joints;
    }
    if (out_joints != nullptr) {
        *out_joints = joints;
    }
    return true;
}

}  // namespace hexapod
