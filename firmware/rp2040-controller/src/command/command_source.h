#pragma once

#include <cstdint>

#include "command/motion_command.h"
#include "config/robot_config.h"

namespace hexapod {

class ICommandSource {
public:
    virtual ~ICommandSource() = default;
    virtual MotionCommand readMotionCommand(std::uint64_t now_us) = 0;
};

}  // namespace hexapod
