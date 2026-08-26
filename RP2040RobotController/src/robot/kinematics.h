#pragma once

#include "config/robot_config.h"
#include "math/vec3.h"

namespace hexapod {

struct JointAngles {
    float coxa_rad = 0.0f;
    float femur_rad = 0.0f;
    float tibia_rad = 0.0f;
};

struct IkResult {
    bool valid = false;
    JointAngles angles;
};

JointAngles logicalToPhysical(JointAngles logical);
JointAngles physicalToLogical(JointAngles physical);
Vec3 forwardKinematics(JointAngles logical_angles);
bool jointsWithinLimits(JointAngles logical_angles, float margin_rad = kJointSafetyMarginRad);
bool isSingular(JointAngles logical_angles);
IkResult inverseKinematics(Vec3 leg_target_mm, JointAngles previous_logical);

}  // namespace hexapod
