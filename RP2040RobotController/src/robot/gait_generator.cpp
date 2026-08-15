#include "robot/gait_generator.hpp"

#include "robot_params.hpp"

#include <algorithm>
#include <cmath>

namespace robot {
namespace {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float EPSILON = 0.0001f;

float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

float wrap01(float value) {
    value -= std::floor(value);
    return value;
}

float smoothstep(float value) {
    const float t = clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

FootTarget add(const FootTarget &a, const FootTarget &b) {
    return FootTarget{a.x_mm + b.x_mm, a.y_mm + b.y_mm, a.z_mm + b.z_mm};
}

FootTarget multiply(const FootTarget &value, float factor) {
    return FootTarget{value.x_mm * factor, value.y_mm * factor, value.z_mm * factor};
}

} // namespace

BodyVelocityCommand body_velocity_from_drive(const control::DriveCommand &command) {
    return BodyVelocityCommand{
        command.forward * config::MAX_FORWARD_SPEED_MM_S,
        0.0f,
        command.turn * config::MAX_YAW_RATE_DEG_S * DEG_TO_RAD,
    };
}

float stance_displacement_mm(float velocity_mm_s) {
    return velocity_mm_s * (static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f) *
           config::GAIT_DUTY_FACTOR;
}

float stance_position_offset_mm(float velocity_mm_s, float normalized_stance_phase) {
    return stance_displacement_mm(velocity_mm_s) *
           (0.5f - clamp(normalized_stance_phase, 0.0f, 1.0f));
}

FootTarget stance_velocity_for_body_command(const BodyVelocityCommand &command,
                                            const FootTarget &body_position) {
    // Foot velocity in body coordinates during stance: -v_body - omega x r.
    // For +Z yaw, omega x r = (-w*y, +w*x, 0).
    return FootTarget{-command.vx_mm_s + command.yaw_rad_s * body_position.y_mm,
                      -command.vy_mm_s - command.yaw_rad_s * body_position.x_mm,
                      0.0f};
}

GaitGenerator::GaitGenerator(
    const RobotGeometry &geometry,
    const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos)
    : geometry_(geometry), servos_(servos) {
    ready_ = geometry_is_valid(geometry_);
    if (!ready_) {
        return;
    }
    for (size_t i = 0; i < neutral_.size(); ++i) {
        neutral_[i] = forward_kinematics(geometry_, static_cast<servo::Leg>(i),
                                         STAND_POSE.legs[i]);
    }
    targets_ = neutral_;
}

void GaitGenerator::reset() {
    phase_ = 0.0f;
    stride_scale_ = 0.0f;
    stop_elapsed_s_ = 0.0f;
    active_command_ = BodyVelocityCommand{};
    status_ = CartesianGaitStatus{};
    targets_ = neutral_;
}

bool GaitGenerator::command_is_zero(const BodyVelocityCommand &command) const {
    return std::fabs(command.vx_mm_s) < EPSILON && std::fabs(command.vy_mm_s) < EPSILON &&
           std::fabs(command.yaw_rad_s) < EPSILON;
}

std::array<FootTarget, servo::LEG_COUNT> GaitGenerator::targets_for_scale(
    const BodyVelocityCommand &command, float scale, float phase) const {
    std::array<FootTarget, servo::LEG_COUNT> targets{};
    std::array<FootTarget, servo::LEG_COUNT> strides{};
    const float cycle_s = static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f;
    const float duty = config::GAIT_DUTY_FACTOR;
    for (size_t index = 0; index < strides.size(); ++index) {
        const FootTarget &neutral = neutral_[index];
        // Velocity of the body at this particular foot.  Stance moves the
        // body-frame target in the opposite direction: -v - omega x r.
        const FootTarget stance_velocity = stance_velocity_for_body_command(command, neutral);
        const float body_x = -stance_velocity.x_mm;
        const float body_y = -stance_velocity.y_mm;
        strides[index] = FootTarget{body_x * cycle_s * duty, body_y * cycle_s * duty, 0.0f};
    }
    const float horizontal_scale = workspace_scale(command);
    for (size_t index = 0; index < targets.size(); ++index) {
        const float local_phase = wrap01(phase + TRIPOD_PHASE_OFFSETS[index]);
        const FootTarget &neutral = neutral_[index];
        const FootTarget stance_velocity = stance_velocity_for_body_command(command, neutral);
        const float body_x = -stance_velocity.x_mm;
        const float body_y = -stance_velocity.y_mm;
        const FootTarget stride = multiply(strides[index], horizontal_scale * scale);

        if (local_phase < duty) {
            const float stance = local_phase / duty;
            targets[index] = neutral;
            targets[index].x_mm += stance_position_offset_mm(body_x, stance) * horizontal_scale * scale;
            targets[index].y_mm += stance_position_offset_mm(body_y, stance) * horizontal_scale * scale;
        } else {
            const float swing = (local_phase - duty) / (1.0f - duty);
            const float horizontal = -0.5f + smoothstep(swing);
            // sin(pi*s)^0.7 rises earlier than a plain sinusoid, preserving
            // the useful pre-lift behaviour without discrete joint phases.
            const float lift = config::GAIT_STEP_HEIGHT_MM * std::pow(std::sin(PI * swing), 0.7f);
            targets[index] = add(neutral, multiply(stride, horizontal));
            targets[index].z_mm += lift;
        }
    }
    return targets;
}

float GaitGenerator::requested_stride_mm(const BodyVelocityCommand &command) const {
    const float cycle_s = static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f;
    float largest_stride = 0.0f;
    for (const FootTarget &neutral : neutral_) {
        const FootTarget stance_velocity = stance_velocity_for_body_command(command, neutral);
        const float body_x = -stance_velocity.x_mm;
        const float body_y = -stance_velocity.y_mm;
        const float x = body_x * cycle_s * config::GAIT_DUTY_FACTOR;
        const float y = body_y * cycle_s * config::GAIT_DUTY_FACTOR;
        largest_stride = std::max(largest_stride, std::sqrt(x * x + y * y));
    }
    return largest_stride;
}

float GaitGenerator::workspace_scale(const BodyVelocityCommand &command) const {
    const float requested = requested_stride_mm(command);
    return requested > config::GAIT_STRIDE_MM && requested > EPSILON
               ? config::GAIT_STRIDE_MM / requested
               : 1.0f;
}

bool GaitGenerator::targets_reachable(
    const std::array<FootTarget, servo::LEG_COUNT> &targets) const {
    return validate_cartesian_targets(geometry_, targets, servos_).valid();
}

std::array<FootTarget, servo::LEG_COUNT> GaitGenerator::update(
    float dt_s, const BodyVelocityCommand &command) {
    if (!ready_) {
        status_.state = CartesianGaitState::Limited;
        status_.ik_limited = true;
        return targets_;
    }

    dt_s = clamp(dt_s, 0.0f, 0.1f);
    const bool zero_command = command_is_zero(command);
    if (!zero_command) {
        active_command_ = command;
    }
    const BodyVelocityCommand &trajectory_command = zero_command ? active_command_ : command;
    const float ramp_s = 0.25f;
    const float target_scale = zero_command ? 0.0f : 1.0f;
    const float max_change = ramp_s > 0.0f ? dt_s / ramp_s : 1.0f;
    stride_scale_ += clamp(target_scale - stride_scale_, -max_change, max_change);

    if (!zero_command || stride_scale_ > EPSILON) {
        phase_ = wrap01(phase_ + dt_s / (static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f));
    }
    if (zero_command) {
        stop_elapsed_s_ += dt_s;
    } else {
        stop_elapsed_s_ = 0.0f;
    }

    status_.ik_limited = false;
    status_.limit = CartesianLimitInfo{};
    auto candidate = targets_for_scale(trajectory_command, stride_scale_, phase_);
    // Preserve Cartesian direction by uniformly reducing the complete motion
    // vector until every leg is reachable; never clamp XYZ axes independently.
    const auto requested_validation = validate_cartesian_targets(geometry_, candidate, servos_);
    if (!requested_validation.valid()) {
        status_.limit = requested_validation.limit;
        float low = 0.0f;
        float high = stride_scale_;
        for (int iteration = 0; iteration < 12; ++iteration) {
            const float middle = (low + high) * 0.5f;
            if (targets_reachable(targets_for_scale(trajectory_command, middle, phase_))) {
                low = middle;
            } else {
                high = middle;
            }
        }
        stride_scale_ = low;
        candidate = targets_for_scale(trajectory_command, stride_scale_, phase_);
        status_.ik_limited = true;
    }

    if (zero_command && stride_scale_ <= EPSILON &&
        stop_elapsed_s_ >= static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f) {
        targets_ = neutral_;
        status_.state = CartesianGaitState::Idle;
    } else {
        targets_ = candidate;
        status_.state = zero_command ? CartesianGaitState::Stopping :
                                      (status_.ik_limited ? CartesianGaitState::Limited : CartesianGaitState::Walking);
    }
    status_.phase = phase_;
    status_.stride_scale = stride_scale_;
    status_.requested_stride_mm = requested_stride_mm(trajectory_command);
    status_.workspace_scale = workspace_scale(trajectory_command);
    status_.actual_stride_mm = status_.requested_stride_mm * status_.workspace_scale * stride_scale_;
    for (size_t index = 0; index < status_.swing.size(); ++index) {
        status_.swing[index] = wrap01(phase_ + TRIPOD_PHASE_OFFSETS[index]) >= config::GAIT_DUTY_FACTOR;
    }
    return targets_;
}

const char *cartesian_gait_state_name(CartesianGaitState state) {
    switch (state) {
    case CartesianGaitState::Idle: return "IDLE";
    case CartesianGaitState::Walking: return "WALKING";
    case CartesianGaitState::Stopping: return "STOPPING";
    case CartesianGaitState::Limited: return "LIMITED";
    }
    return "?";
}

} // namespace robot
