#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "locomotion/contact_adaptation.h"
#include "robot/kinematics.h"

namespace hexapod {

class IContactSource {
public:
    virtual ~IContactSource() = default;
    virtual void update(float dt_s) = 0;
    virtual const std::array<ContactState, kLegCount>& states() const = 0;
};

class IServoOutput {
public:
    virtual ~IServoOutput() = default;
    virtual bool submitJointTargets(const std::array<JointAngles, kLegCount>& joints) = 0;
};

}  // namespace hexapod
