#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

namespace {

float apply_interpolation(float t, MotionInterpolation interpolation) {
    t = std::max(0.0f, std::min(1.0f, t));
    switch (interpolation) {
    case MotionInterpolation::Linear:
        return t;
    case MotionInterpolation::SmoothStep:
        return t * t * (3.0f - 2.0f * t);
    case MotionInterpolation::SmootherStep:
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }
    return t;
}

float lerp_value(float start, float target, float t) {
    return start + (target - start) * t;
}

RobotPose lerp_pose(const RobotPose &start, const RobotPose &target, float t) {
    RobotPose pose = start;
    for (size_t leg = 0; leg < pose.legs.size(); ++leg) {
        pose.legs[leg].coxa_deg = lerp_value(start.legs[leg].coxa_deg, target.legs[leg].coxa_deg, t);
        pose.legs[leg].femur_deg = lerp_value(start.legs[leg].femur_deg, target.legs[leg].femur_deg, t);
        pose.legs[leg].tibia_deg = lerp_value(start.legs[leg].tibia_deg, target.legs[leg].tibia_deg, t);
    }
    return pose;
}

bool pose_equal(const RobotPose &a, const RobotPose &b) {
    for (size_t leg = 0; leg < a.legs.size(); ++leg) {
        if (a.legs[leg].coxa_deg != b.legs[leg].coxa_deg ||
            a.legs[leg].femur_deg != b.legs[leg].femur_deg ||
            a.legs[leg].tibia_deg != b.legs[leg].tibia_deg) {
            return false;
        }
    }
    return true;
}

} // namespace

void MotionController::reset(const RobotPose &pose) {
    current_ = pose;
    start_ = pose;
    target_ = pose;
    duration_us_ = 0;
    elapsed_us_ = 0;
    mode_ = MotionMode::SpeedLimited;
    interpolation_ = MotionInterpolation::Linear;
    moving_ = false;
}

void MotionController::set_target(const RobotPose &pose, float speed_deg_per_sec) {
    target_ = pose;
    max_speed_deg_per_sec_ = speed_deg_per_sec > 0.0f ? speed_deg_per_sec : 30.0f;
    duration_us_ = 0;
    elapsed_us_ = 0;
    mode_ = MotionMode::SpeedLimited;
    interpolation_ = MotionInterpolation::Linear;
    moving_ = !pose_equal(current_, target_);
}

void MotionController::set_target_timed(const RobotPose &pose,
                                        uint32_t duration_ms,
                                        MotionInterpolation interpolation) {
    start_ = current_;
    target_ = pose;
    duration_us_ = static_cast<uint64_t>(duration_ms) * 1000u;
    elapsed_us_ = 0;
    mode_ = MotionMode::Timed;
    interpolation_ = interpolation;
    if (duration_us_ == 0 || pose_equal(start_, target_)) {
        current_ = target_;
        moving_ = false;
    } else {
        moving_ = true;
    }
}

void MotionController::set_servo_target(servo::Leg leg, servo::Joint joint, float angle_deg) {
    set_pose_angle(target_, leg, joint, angle_deg);
    set_target(target_, max_speed_deg_per_sec_);
}

void MotionController::set_leg_target(servo::Leg leg, const LegPose &pose) {
    target_.legs[servo::leg_index(leg)] = pose;
    set_target(target_, max_speed_deg_per_sec_);
}

void MotionController::set_immediate_pose(const RobotPose &pose) {
    current_ = pose;
    start_ = pose;
    target_ = pose;
    moving_ = false;
    elapsed_us_ = 0;
    duration_us_ = 0;
}

static bool step_value(float &current, float target, float step) {
    const float delta = target - current;
    if (fabsf(delta) <= step) {
        current = target;
        return false;
    }
    current += delta > 0.0f ? step : -step;
    return true;
}

void MotionController::update(uint32_t elapsed_us) {
    if (!moving_) {
        return;
    }
    if (mode_ == MotionMode::Timed) {
        if (duration_us_ == 0) {
            current_ = target_;
            moving_ = false;
            return;
        }
        elapsed_us_ = std::min(duration_us_, elapsed_us_ + elapsed_us);
        const float raw_t = static_cast<float>(elapsed_us_) / static_cast<float>(duration_us_);
        current_ = lerp_pose(start_, target_, apply_interpolation(raw_t, interpolation_));
        if (elapsed_us_ >= duration_us_) {
            current_ = target_;
            moving_ = false;
        }
        return;
    }

    const float step = max_speed_deg_per_sec_ * (static_cast<float>(elapsed_us) / 1000000.0f);
    bool still_moving = false;
    for (size_t i = 0; i < current_.legs.size(); ++i) {
        still_moving |= step_value(current_.legs[i].coxa_deg, target_.legs[i].coxa_deg, step);
        still_moving |= step_value(current_.legs[i].femur_deg, target_.legs[i].femur_deg, step);
        still_moving |= step_value(current_.legs[i].tibia_deg, target_.legs[i].tibia_deg, step);
    }
    moving_ = still_moving;
}

uint8_t MotionController::progress_percent() const {
    if (mode_ != MotionMode::Timed || duration_us_ == 0) {
        return moving_ ? 0u : 100u;
    }
    const uint64_t percent = std::min<uint64_t>(100u, (elapsed_us_ * 100u) / duration_us_);
    return static_cast<uint8_t>(percent);
}

const char *motion_interpolation_name(MotionInterpolation interpolation) {
    switch (interpolation) {
    case MotionInterpolation::Linear: return "LINEAR";
    case MotionInterpolation::SmoothStep: return "SMOOTHSTEP";
    case MotionInterpolation::SmootherStep: return "SMOOTHERSTEP";
    }
    return "?";
}

} // namespace robot
