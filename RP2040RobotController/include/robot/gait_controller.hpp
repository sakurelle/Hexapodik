#pragma once

#include "control/drive_controller.hpp"
#include "robot/gait_config.hpp"
#include "robot/motion_controller.hpp"
#include "servo/servo_config.hpp"

#include <array>
#include <cstdint>

namespace robot {

enum class GaitMode {
    Idle,
    March,
    WalkDemo,
    RCDrive,
    Stopping,
    Error
};

enum class GaitState {
    Idle,
    MarchStand,
    MarchPreliftA,
    MarchLiftA,
    MarchHoldA,
    MarchLowerA,
    MarchPreliftB,
    MarchLiftB,
    MarchHoldB,
    MarchLowerB,
    WalkPrepare,
    WalkAPrelift,
    WalkALift,
    WalkATransfer,
    WalkALower,
    WalkBPrelift,
    WalkBLift,
    WalkBTransfer,
    WalkBLower,
    WalkFinishGround,
    WalkFinishStand,
    StopGround,
    StopStand,
    ErrorReturnStand
};

class GaitController {
public:
    bool start_march(uint64_t now_us, MotionController &motion,
                     const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);
    bool start_walk_demo(uint64_t now_us, MotionController &motion,
                         const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);
    void stop(uint64_t now_us, MotionController &motion,
              const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);
    void abort();
    void update(uint64_t now_us, MotionController &motion,
                const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                bool startup_holding_stand);
    void update_drive(uint64_t now_us, MotionController &motion,
                      const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                      const control::DriveCommand &command,
                      bool startup_holding_stand);

    GaitMode mode() const { return mode_; }
    GaitState state() const { return state_; }
    uint8_t cycle() const { return cycle_; }
    bool active() const { return mode_ != GaitMode::Idle; }
    bool error() const { return mode_ == GaitMode::Error; }
    const char *diagnostic() const { return diagnostic_; }
    control::DriveMix drive_mix() const { return drive_mix_; }
    float drive_command_magnitude() const;
    float drive_step_swing_deg() const;
    float drive_step_speed() const;

private:
    enum class AutoDemoState {
        WaitingForStand,
        DelayBeforeMarch,
        MarchRunning,
        DelayBeforeWalk,
        WalkRunning,
        Done
    };

    bool begin(uint64_t now_us, MotionController &motion,
               const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
               GaitMode mode);
    void begin_phase(uint64_t now_us, MotionController &motion,
                     const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                     GaitState state, const RobotPose &pose, uint32_t duration_ms);
    void fail_to_stand(uint64_t now_us, MotionController &motion, const char *diagnostic);
    void finish_idle(MotionController &motion);
    void update_auto_demo(uint64_t now_us, MotionController &motion,
                          const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                          bool startup_holding_stand);

    GaitMode mode_ = GaitMode::Idle;
    GaitState state_ = GaitState::Idle;
    uint64_t state_entered_us_ = 0;
    uint8_t cycle_ = 0;
    bool stop_requested_ = false;
    bool rc_finish_stop_requested_ = false;
    const char *diagnostic_ = "OK";
    control::DriveCommand drive_command_{};
    control::DriveMix drive_mix_{};
    uint32_t rc_lift_ms_ = 250;
    uint32_t rc_prelift_ms_ = 120;
    uint32_t rc_transfer_ms_ = 500;
    uint32_t rc_lower_ms_ = 250;
    AutoDemoState auto_state_ = AutoDemoState::WaitingForStand;
    uint64_t auto_entered_us_ = 0;
};

RobotPose lifted_pose(const std::array<servo::Leg, 3> &tripod);
RobotPose prelifted_pose(const std::array<servo::Leg, 3> &tripod);
RobotPose walk_cycle_start_pose();
RobotPose walk_prelift_pose(const std::array<servo::Leg, 3> &tripod);
RobotPose walk_transfer_pose(const std::array<servo::Leg, 3> &swing_tripod);
RobotPose walk_lift_pose(const std::array<servo::Leg, 3> &tripod);
RobotPose walk_ground_pose();
RobotPose grounded_pose_from(const RobotPose &pose);
bool pose_within_servo_limits(const RobotPose &pose,
                              const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);

const char *gait_mode_name(GaitMode mode);
const char *gait_state_name(GaitState state);

} // namespace robot
