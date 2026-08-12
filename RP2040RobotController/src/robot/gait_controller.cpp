#include "robot/gait_controller.hpp"
#include "robot/robot_model.hpp"

#include <cstdio>

namespace robot {

namespace {

constexpr uint32_t US_PER_MS = 1000;

void set_tripod_lift(RobotPose &pose, const std::array<servo::Leg, 3> &tripod, bool lifted) {
    for (const auto leg : tripod) {
        auto &leg_pose = pose.legs[servo::leg_index(leg)];
        leg_pose.femur_deg = robot::STAND_LEG_POSE.femur_deg +
                             (lifted ? gait::GAIT_LIFT_FEMUR_DELTA_DEG : 0.0f);
        leg_pose.tibia_deg = robot::STAND_LEG_POSE.tibia_deg +
                             (lifted ? gait::GAIT_LIFT_TIBIA_DELTA_DEG : 0.0f);
    }
}

void set_tripod_coxa(RobotPose &pose, const std::array<servo::Leg, 3> &tripod, bool forward) {
    for (const auto leg : tripod) {
        pose.legs[servo::leg_index(leg)].coxa_deg =
            forward ? gait::forwardCoxa(leg) : gait::backwardCoxa(leg);
    }
}

bool phase_ready(uint64_t now_us, uint64_t entered_us, uint32_t min_ms, const MotionController &motion) {
    return !motion.moving() && (now_us - entered_us) >= static_cast<uint64_t>(min_ms) * US_PER_MS;
}

} // namespace

RobotPose lifted_pose(const std::array<servo::Leg, 3> &tripod) {
    RobotPose pose = STAND_POSE;
    set_tripod_lift(pose, tripod, true);
    return pose;
}

RobotPose walk_cycle_start_pose() {
    RobotPose pose = STAND_POSE;
    set_tripod_coxa(pose, gait::TRIPOD_A, false);
    set_tripod_coxa(pose, gait::TRIPOD_B, true);
    return pose;
}

RobotPose walk_transfer_pose(const std::array<servo::Leg, 3> &swing_tripod) {
    RobotPose pose = STAND_POSE;
    const bool a_is_swing = gait::leg_in_tripod(servo::Leg::FR, swing_tripod);
    set_tripod_coxa(pose, gait::TRIPOD_A, a_is_swing);
    set_tripod_coxa(pose, gait::TRIPOD_B, !a_is_swing);
    set_tripod_lift(pose, swing_tripod, true);
    return pose;
}

RobotPose walk_lift_pose(const std::array<servo::Leg, 3> &tripod) {
    RobotPose pose = walk_cycle_start_pose();
    if (gait::leg_in_tripod(servo::Leg::FL, tripod)) {
        set_tripod_coxa(pose, gait::TRIPOD_A, true);
        set_tripod_coxa(pose, gait::TRIPOD_B, false);
    }
    set_tripod_lift(pose, tripod, true);
    return pose;
}

RobotPose walk_ground_pose() {
    return walk_cycle_start_pose();
}

RobotPose grounded_pose_from(const RobotPose &pose) {
    RobotPose grounded = pose;
    for (auto &leg_pose : grounded.legs) {
        leg_pose.femur_deg = STAND_LEG_POSE.femur_deg;
        leg_pose.tibia_deg = STAND_LEG_POSE.tibia_deg;
    }
    return grounded;
}

bool pose_within_servo_limits(const RobotPose &pose,
                              const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    for (const auto &config : servos) {
        const float angle = pose_angle(pose, config.leg, config.joint);
        if (angle < config.min_angle_deg || angle > config.max_angle_deg ||
            angle < -45.0f || angle > 45.0f) {
            return false;
        }
    }
    return true;
}

bool GaitController::start_march(uint64_t now_us, MotionController &motion,
                                 const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    return begin(now_us, motion, servos, GaitMode::March);
}

bool GaitController::start_walk_demo(uint64_t now_us, MotionController &motion,
                                     const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    return begin(now_us, motion, servos, GaitMode::WalkDemo);
}

bool GaitController::begin(uint64_t now_us, MotionController &motion,
                           const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                           GaitMode mode) {
    bool poses_safe = pose_within_servo_limits(STAND_POSE, servos) &&
                      pose_within_servo_limits(lifted_pose(gait::TRIPOD_A), servos) &&
                      pose_within_servo_limits(lifted_pose(gait::TRIPOD_B), servos);
    if (mode == GaitMode::WalkDemo) {
        poses_safe = poses_safe &&
                     pose_within_servo_limits(walk_cycle_start_pose(), servos) &&
                     pose_within_servo_limits(walk_lift_pose(gait::TRIPOD_A), servos) &&
                     pose_within_servo_limits(walk_lift_pose(gait::TRIPOD_B), servos) &&
                     pose_within_servo_limits(walk_transfer_pose(gait::TRIPOD_A), servos) &&
                     pose_within_servo_limits(walk_transfer_pose(gait::TRIPOD_B), servos);
    }
    if (!poses_safe) {
        fail_to_stand(now_us, motion, "ERR GAIT CONFIG_INVALID");
        return false;
    }

    mode_ = mode;
    cycle_ = 0;
    stop_requested_ = false;
    diagnostic_ = "OK";

    if (mode == GaitMode::March) {
        begin_phase(now_us, motion, servos, GaitState::MarchStand, STAND_POSE, gait::MARCH_LOWER_TIME_MS);
    } else {
        begin_phase(now_us, motion, servos, GaitState::WalkPrepare, walk_cycle_start_pose(), gait::WALK_PREPARE_MS);
    }
    return true;
}

void GaitController::stop(uint64_t now_us, MotionController &motion,
                          const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    if (mode_ == GaitMode::Idle) {
        begin_phase(now_us, motion, servos, GaitState::StopStand, STAND_POSE, gait::WALK_FINISH_MS);
        mode_ = GaitMode::Stopping;
        return;
    }
    stop_requested_ = true;
    mode_ = GaitMode::Stopping;
    begin_phase(now_us, motion, servos, GaitState::StopGround, grounded_pose_from(motion.target_pose()), gait::WALK_LOWER_MS);
}

void GaitController::abort() {
    mode_ = GaitMode::Idle;
    state_ = GaitState::Idle;
    cycle_ = 0;
    stop_requested_ = false;
    diagnostic_ = "OK";
}

void GaitController::begin_phase(uint64_t now_us, MotionController &motion,
                                 const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                                 GaitState state, const RobotPose &pose, uint32_t duration_ms) {
    if (!pose_within_servo_limits(pose, servos)) {
        fail_to_stand(now_us, motion, "ERR GAIT ANGLE_OUT_OF_RANGE");
        return;
    }
    state_ = state;
    state_entered_us_ = now_us;
    motion.set_target_timed(pose, duration_ms);
}

void GaitController::fail_to_stand(uint64_t now_us, MotionController &motion, const char *diagnostic) {
    mode_ = GaitMode::Error;
    state_ = GaitState::ErrorReturnStand;
    state_entered_us_ = now_us;
    stop_requested_ = false;
    diagnostic_ = diagnostic;
    printf("%s\r\n", diagnostic_);
    motion.set_target_timed(STAND_POSE, gait::WALK_FINISH_MS);
}

void GaitController::finish_idle(MotionController &motion) {
    mode_ = GaitMode::Idle;
    state_ = GaitState::Idle;
    cycle_ = 0;
    stop_requested_ = false;
    diagnostic_ = "OK";
    motion.set_target(STAND_POSE, 30.0f);
}

void GaitController::update(uint64_t now_us, MotionController &motion,
                            const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                            bool startup_holding_stand) {
    update_auto_demo(now_us, motion, servos, startup_holding_stand);

    if (mode_ == GaitMode::Idle) {
        return;
    }

    if (mode_ == GaitMode::Error) {
        if (!motion.moving()) {
            finish_idle(motion);
        }
        return;
    }

    if (stop_requested_ && state_ != GaitState::StopGround && state_ != GaitState::StopStand) {
        begin_phase(now_us, motion, servos, GaitState::StopGround, grounded_pose_from(motion.target_pose()), gait::WALK_LOWER_MS);
        return;
    }

    switch (state_) {
    case GaitState::MarchStand:
        if (phase_ready(now_us, state_entered_us_, 0, motion)) {
            begin_phase(now_us, motion, servos, GaitState::MarchLiftA, lifted_pose(gait::TRIPOD_A),
                        gait::MARCH_LIFT_TIME_MS);
        }
        break;
    case GaitState::MarchLiftA:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_LIFT_TIME_MS, motion)) {
            state_ = GaitState::MarchHoldA;
            state_entered_us_ = now_us;
        }
        break;
    case GaitState::MarchHoldA:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_HOLD_TIME_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::MarchLowerA, STAND_POSE, gait::MARCH_LOWER_TIME_MS);
        }
        break;
    case GaitState::MarchLowerA:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_LOWER_TIME_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::MarchLiftB, lifted_pose(gait::TRIPOD_B),
                        gait::MARCH_LIFT_TIME_MS);
        }
        break;
    case GaitState::MarchLiftB:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_LIFT_TIME_MS, motion)) {
            state_ = GaitState::MarchHoldB;
            state_entered_us_ = now_us;
        }
        break;
    case GaitState::MarchHoldB:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_HOLD_TIME_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::MarchLowerB, STAND_POSE, gait::MARCH_LOWER_TIME_MS);
        }
        break;
    case GaitState::MarchLowerB:
        if (phase_ready(now_us, state_entered_us_, gait::MARCH_LOWER_TIME_MS, motion)) {
            ++cycle_;
            if (cycle_ >= gait::MARCH_CYCLES) {
                finish_idle(motion);
            } else {
                begin_phase(now_us, motion, servos, GaitState::MarchLiftA, lifted_pose(gait::TRIPOD_A),
                            gait::MARCH_LIFT_TIME_MS);
            }
        }
        break;
    case GaitState::WalkPrepare:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_PREPARE_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::WalkALift, walk_lift_pose(gait::TRIPOD_A),
                        gait::WALK_LIFT_MS);
        }
        break;
    case GaitState::WalkALift:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LIFT_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::WalkATransfer, walk_transfer_pose(gait::TRIPOD_A),
                        gait::WALK_TRANSFER_MS);
        }
        break;
    case GaitState::WalkATransfer:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_TRANSFER_MS, motion)) {
            RobotPose pose = walk_transfer_pose(gait::TRIPOD_A);
            set_tripod_lift(pose, gait::TRIPOD_A, false);
            begin_phase(now_us, motion, servos, GaitState::WalkALower, pose, gait::WALK_LOWER_MS);
        }
        break;
    case GaitState::WalkALower:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LOWER_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::WalkBLift, walk_lift_pose(gait::TRIPOD_B),
                        gait::WALK_LIFT_MS);
        }
        break;
    case GaitState::WalkBLift:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LIFT_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::WalkBTransfer, walk_transfer_pose(gait::TRIPOD_B),
                        gait::WALK_TRANSFER_MS);
        }
        break;
    case GaitState::WalkBTransfer:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_TRANSFER_MS, motion)) {
            RobotPose pose = walk_transfer_pose(gait::TRIPOD_B);
            set_tripod_lift(pose, gait::TRIPOD_B, false);
            begin_phase(now_us, motion, servos, GaitState::WalkBLower, pose, gait::WALK_LOWER_MS);
        }
        break;
    case GaitState::WalkBLower:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LOWER_MS, motion)) {
            ++cycle_;
            if (cycle_ >= gait::WALK_DEMO_CYCLES) {
                begin_phase(now_us, motion, servos, GaitState::WalkFinishGround, walk_ground_pose(),
                            gait::WALK_LOWER_MS);
            } else {
                begin_phase(now_us, motion, servos, GaitState::WalkALift, walk_lift_pose(gait::TRIPOD_A),
                            gait::WALK_LIFT_MS);
            }
        }
        break;
    case GaitState::WalkFinishGround:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LOWER_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::WalkFinishStand, STAND_POSE, gait::WALK_FINISH_MS);
        }
        break;
    case GaitState::WalkFinishStand:
    case GaitState::StopStand:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_FINISH_MS, motion)) {
            finish_idle(motion);
        }
        break;
    case GaitState::StopGround:
        if (phase_ready(now_us, state_entered_us_, gait::WALK_LOWER_MS, motion)) {
            begin_phase(now_us, motion, servos, GaitState::StopStand, STAND_POSE, gait::WALK_FINISH_MS);
        }
        break;
    case GaitState::Idle:
    case GaitState::ErrorReturnStand:
        break;
    }
}

void GaitController::update_auto_demo(uint64_t now_us, MotionController &motion,
                                      const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                                      bool startup_holding_stand) {
    if constexpr (!gait::AUTO_DEMO_GAIT_ON_BOOT) {
        return;
    }

    switch (auto_state_) {
    case AutoDemoState::WaitingForStand:
        if (startup_holding_stand && mode_ == GaitMode::Idle) {
            auto_state_ = AutoDemoState::DelayBeforeMarch;
            auto_entered_us_ = now_us;
        }
        break;
    case AutoDemoState::DelayBeforeMarch:
        if (mode_ == GaitMode::Idle &&
            now_us - auto_entered_us_ >= static_cast<uint64_t>(gait::AUTO_DEMO_DELAY_MS) * US_PER_MS) {
            start_march(now_us, motion, servos);
            auto_state_ = AutoDemoState::MarchRunning;
        }
        break;
    case AutoDemoState::MarchRunning:
        if (mode_ == GaitMode::Idle) {
            auto_state_ = AutoDemoState::DelayBeforeWalk;
            auto_entered_us_ = now_us;
        }
        break;
    case AutoDemoState::DelayBeforeWalk:
        if (mode_ == GaitMode::Idle &&
            now_us - auto_entered_us_ >= static_cast<uint64_t>(gait::AUTO_DEMO_BETWEEN_MS) * US_PER_MS) {
            start_walk_demo(now_us, motion, servos);
            auto_state_ = AutoDemoState::WalkRunning;
        }
        break;
    case AutoDemoState::WalkRunning:
        if (mode_ == GaitMode::Idle) {
            auto_state_ = AutoDemoState::Done;
        }
        break;
    case AutoDemoState::Done:
        break;
    }
}

const char *gait_mode_name(GaitMode mode) {
    switch (mode) {
    case GaitMode::Idle: return "IDLE";
    case GaitMode::March: return "MARCH";
    case GaitMode::WalkDemo: return "WALK";
    case GaitMode::Stopping: return "STOPPING";
    case GaitMode::Error: return "ERROR";
    }
    return "?";
}

const char *gait_state_name(GaitState state) {
    switch (state) {
    case GaitState::Idle: return "IDLE";
    case GaitState::MarchStand: return "MARCH_STAND";
    case GaitState::MarchLiftA: return "MARCH_LIFT_A";
    case GaitState::MarchHoldA: return "MARCH_HOLD_A";
    case GaitState::MarchLowerA: return "MARCH_LOWER_A";
    case GaitState::MarchLiftB: return "MARCH_LIFT_B";
    case GaitState::MarchHoldB: return "MARCH_HOLD_B";
    case GaitState::MarchLowerB: return "MARCH_LOWER_B";
    case GaitState::WalkPrepare: return "PREPARE";
    case GaitState::WalkALift: return "A_LIFT";
    case GaitState::WalkATransfer: return "A_TRANSFER";
    case GaitState::WalkALower: return "A_LOWER";
    case GaitState::WalkBLift: return "B_LIFT";
    case GaitState::WalkBTransfer: return "B_TRANSFER";
    case GaitState::WalkBLower: return "B_LOWER";
    case GaitState::WalkFinishGround: return "FINISH_GROUND";
    case GaitState::WalkFinishStand: return "FINISH_STAND";
    case GaitState::StopGround: return "STOP_GROUND";
    case GaitState::StopStand: return "STOP_STAND";
    case GaitState::ErrorReturnStand: return "ERROR_RETURN_STAND";
    }
    return "?";
}

} // namespace robot
