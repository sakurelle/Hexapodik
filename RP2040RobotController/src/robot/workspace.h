#pragma once

#include <array>

#include "config/robot_config.h"
#include "locomotion/body_command.h"
#include "math/vec3.h"
#include "robot/kinematics.h"

namespace hexapod {

struct WorkspaceCheck {
    bool valid = false;
    JointAngles joints;
};

WorkspaceCheck validateFootTarget(LegId leg, Vec3 foot_body_mm, JointAngles previous_joints);
bool validateAllFootTargets(const std::array<Vec3, kLegCount>& feet_body_mm,
                            const std::array<JointAngles, kLegCount>& previous_joints,
                            std::array<JointAngles, kLegCount>* out_joints);

template <typename Generator>
float findWorkspaceScale(BodyCommand command, Generator&& generator) {
    float lo = 0.0f;
    float hi = 1.0f;
    for (int i = 0; i < 8; ++i) {
        const float mid = (lo + hi) * 0.5f;
        if (generator(command * mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

}  // namespace hexapod
