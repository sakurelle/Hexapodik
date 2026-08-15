#pragma once

#include "control/drive_controller.hpp"
#include "robot/cartesian_validation.hpp"
#include "robot/kinematics.hpp"

#include <array>

namespace robot {

struct BodyVelocityCommand {
    float vx_mm_s = 0.0f;
    float vy_mm_s = 0.0f;
    float yaw_rad_s = 0.0f;
};

BodyVelocityCommand body_velocity_from_drive(const control::DriveCommand &command);
float stance_displacement_mm(float velocity_mm_s);
// Body-frame position during the stance fraction (0..1).  Its time
// derivative is exactly -velocity_mm_s when phase progresses at the gait
// stance rate.
float stance_position_offset_mm(float velocity_mm_s, float normalized_stance_phase);
FootTarget stance_velocity_for_body_command(const BodyVelocityCommand &command,
                                            const FootTarget &body_position);

enum class CartesianGaitState { Idle, Walking, Stopping, Limited };

struct CartesianGaitStatus {
    CartesianGaitState state = CartesianGaitState::Idle;
    float phase = 0.0f;
    float stride_scale = 0.0f;
    float workspace_scale = 1.0f;
    float requested_stride_mm = 0.0f;
    float actual_stride_mm = 0.0f;
    bool ik_limited = false;
    CartesianLimitInfo limit{};
    std::array<bool, servo::LEG_COUNT> swing{};
};

// Continuous tripod trajectory generator.  It owns no hardware resource and
// emits one body-frame XYZ target for every leg on every control tick.
class GaitGenerator {
public:
    GaitGenerator(const RobotGeometry &geometry,
                  const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);

    bool ready() const { return ready_; }
    void reset();
    std::array<FootTarget, servo::LEG_COUNT> update(float dt_s,
                                                    const BodyVelocityCommand &command);
    const std::array<FootTarget, servo::LEG_COUNT> &neutral_targets() const { return neutral_; }
    const CartesianGaitStatus &status() const { return status_; }

private:
    std::array<FootTarget, servo::LEG_COUNT> targets_for_scale(
        const BodyVelocityCommand &command, float scale, float phase) const;
    float requested_stride_mm(const BodyVelocityCommand &command) const;
    float workspace_scale(const BodyVelocityCommand &command) const;
    bool targets_reachable(const std::array<FootTarget, servo::LEG_COUNT> &targets) const;
    bool command_is_zero(const BodyVelocityCommand &command) const;

    RobotGeometry geometry_{};
    const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos_;
    std::array<FootTarget, servo::LEG_COUNT> neutral_{};
    std::array<FootTarget, servo::LEG_COUNT> targets_{};
    CartesianGaitStatus status_{};
    float phase_ = 0.0f;
    float stride_scale_ = 0.0f;
    float stop_elapsed_s_ = 0.0f;
    BodyVelocityCommand active_command_{};
    bool ready_ = false;
};

constexpr std::array<float, servo::LEG_COUNT> TRIPOD_PHASE_OFFSETS = {{
    0.0f, // FR, tripod A
    0.5f, // MR, tripod B
    0.0f, // RR, tripod A
    0.5f, // RL, tripod B
    0.0f, // ML, tripod A
    0.5f, // FL, tripod B
}};

const char *cartesian_gait_state_name(CartesianGaitState state);

} // namespace robot
