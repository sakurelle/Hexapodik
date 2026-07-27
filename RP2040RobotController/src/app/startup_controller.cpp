#include "app/startup_controller.hpp"

namespace app {

void StartupController::begin(uint64_t now_us, robot::MotionController &motion) {
    motion.reset(robot::CENTER_POSE);
    outputs_enabled_ = false;
    transition(StartupState::OutputsDisabled, now_us);
}

void StartupController::force_center(uint64_t now_us, robot::MotionController &motion) {
    outputs_enabled_ = true;
    motion.set_target(robot::CENTER_POSE, 30.0f);
    transition(StartupState::Centering, now_us);
}

void StartupController::force_stand(uint64_t now_us, robot::MotionController &motion) {
    outputs_enabled_ = true;
    motion.set_target_timed(robot::STAND_POSE, STAND_MOVE_MS);
    transition(StartupState::MovingToStand, now_us);
}

void StartupController::update(uint64_t now_us, robot::MotionController &motion) {
    const uint64_t elapsed_ms = (now_us - state_entered_us_) / 1000u;

    switch (state_) {
    case StartupState::Boot:
        begin(now_us, motion);
        break;
    case StartupState::OutputsDisabled:
        if (elapsed_ms >= OUTPUTS_DISABLED_MS) {
            outputs_enabled_ = true;
            motion.reset(robot::CENTER_POSE);
            transition(StartupState::Centering, now_us);
        }
        break;
    case StartupState::Centering:
        if (elapsed_ms >= CENTER_HOLD_MS) {
            if (AUTO_STAND_ON_BOOT) {
                motion.set_target_timed(robot::STAND_POSE, STAND_MOVE_MS);
                transition(StartupState::MovingToStand, now_us);
            } else {
                transition(StartupState::HoldingStand, now_us);
            }
        }
        break;
    case StartupState::MovingToStand:
        if (!motion.moving()) {
            transition(StartupState::HoldingStand, now_us);
        }
        break;
    case StartupState::HoldingStand:
        break;
    }
}

void StartupController::transition(StartupState next, uint64_t now_us) {
    state_ = next;
    state_entered_us_ = now_us;
}

const char *startup_state_name(StartupState state) {
    switch (state) {
    case StartupState::Boot: return "BOOT";
    case StartupState::OutputsDisabled: return "OUTPUTS_DISABLED";
    case StartupState::Centering: return "CENTERING";
    case StartupState::MovingToStand: return "MOVING_TO_STAND";
    case StartupState::HoldingStand: return "HOLDING_STAND";
    }
    return "?";
}

} // namespace app
