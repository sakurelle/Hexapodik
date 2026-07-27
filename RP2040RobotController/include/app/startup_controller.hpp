#pragma once

#include "robot/motion_controller.hpp"

#include <cstdint>

namespace app {

constexpr bool AUTO_STAND_ON_BOOT = true;
constexpr uint32_t OUTPUTS_DISABLED_MS = 1000;
constexpr uint32_t CENTER_HOLD_MS = 1000;
constexpr uint32_t STAND_MOVE_MS = 3000;

enum class StartupState {
    Boot,
    OutputsDisabled,
    Centering,
    MovingToStand,
    HoldingStand
};

class StartupController {
public:
    void begin(uint64_t now_us, robot::MotionController &motion);
    void update(uint64_t now_us, robot::MotionController &motion);
    void force_center(uint64_t now_us, robot::MotionController &motion);
    void force_stand(uint64_t now_us, robot::MotionController &motion);

    StartupState state() const { return state_; }
    bool outputs_enabled() const { return outputs_enabled_; }

private:
    void transition(StartupState next, uint64_t now_us);

    StartupState state_ = StartupState::Boot;
    uint64_t state_entered_us_ = 0;
    bool outputs_enabled_ = false;
};

const char *startup_state_name(StartupState state);

} // namespace app
