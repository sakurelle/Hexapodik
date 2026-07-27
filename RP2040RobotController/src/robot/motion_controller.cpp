#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"

#include <cmath>

namespace robot {

void MotionController::reset(const RobotPose &pose) {
    current_ = pose;
    target_ = pose;
    moving_ = false;
}

void MotionController::set_target(const RobotPose &pose, float speed_deg_per_sec) {
    target_ = pose;
    max_speed_deg_per_sec_ = speed_deg_per_sec > 0.0f ? speed_deg_per_sec : 30.0f;
    moving_ = true;
}

void MotionController::set_target_timed(const RobotPose &pose, uint32_t duration_ms) {
    float max_delta = 0.0f;
    for (size_t leg = 0; leg < current_.legs.size(); ++leg) {
        const auto &a = current_.legs[leg];
        const auto &b = pose.legs[leg];
        max_delta = fmaxf(max_delta, fabsf(b.coxa_deg - a.coxa_deg));
        max_delta = fmaxf(max_delta, fabsf(b.femur_deg - a.femur_deg));
        max_delta = fmaxf(max_delta, fabsf(b.tibia_deg - a.tibia_deg));
    }
    const float seconds = duration_ms > 0 ? static_cast<float>(duration_ms) / 1000.0f : 1.0f;
    set_target(pose, max_delta > 0.0f ? max_delta / seconds : 30.0f);
}

void MotionController::set_servo_target(servo::Leg leg, servo::Joint joint, float angle_deg) {
    set_pose_angle(target_, leg, joint, angle_deg);
    moving_ = true;
}

void MotionController::set_leg_target(servo::Leg leg, const LegPose &pose) {
    target_.legs[servo::leg_index(leg)] = pose;
    moving_ = true;
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
    const float step = max_speed_deg_per_sec_ * (static_cast<float>(elapsed_us) / 1000000.0f);
    bool still_moving = false;
    for (size_t i = 0; i < current_.legs.size(); ++i) {
        still_moving |= step_value(current_.legs[i].coxa_deg, target_.legs[i].coxa_deg, step);
        still_moving |= step_value(current_.legs[i].femur_deg, target_.legs[i].femur_deg, step);
        still_moving |= step_value(current_.legs[i].tibia_deg, target_.legs[i].tibia_deg, step);
    }
    moving_ = still_moving;
}

} // namespace robot
