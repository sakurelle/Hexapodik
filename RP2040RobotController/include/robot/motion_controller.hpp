#pragma once

#include "robot/robot_pose.hpp"
#include "servo/servo_config.hpp"

#include <array>
#include <cstdint>

namespace robot {

enum class MotionInterpolation {
    Linear,
    SmoothStep,
    SmootherStep
};

class MotionController {
public:
    void reset(const RobotPose &pose);
    void set_target(const RobotPose &pose, float speed_deg_per_sec = 30.0f);
    void set_target_timed(const RobotPose &pose,
                          uint32_t duration_ms,
                          MotionInterpolation interpolation = MotionInterpolation::SmootherStep);
    void set_servo_target(servo::Leg leg, servo::Joint joint, float angle_deg);
    void set_leg_target(servo::Leg leg, const LegPose &pose);
    void update(uint32_t elapsed_us);

    const RobotPose &current_pose() const { return current_; }
    const RobotPose &target_pose() const { return target_; }
    bool moving() const { return moving_; }
    MotionInterpolation interpolation() const { return interpolation_; }
    uint8_t progress_percent() const;

private:
    enum class MotionMode {
        SpeedLimited,
        Timed
    };

    RobotPose current_ = CENTER_POSE;
    RobotPose start_ = CENTER_POSE;
    RobotPose target_ = CENTER_POSE;
    float max_speed_deg_per_sec_ = 30.0f;
    uint64_t duration_us_ = 0;
    uint64_t elapsed_us_ = 0;
    MotionMode mode_ = MotionMode::SpeedLimited;
    MotionInterpolation interpolation_ = MotionInterpolation::Linear;
    bool moving_ = false;
};

const char *motion_interpolation_name(MotionInterpolation interpolation);

} // namespace robot
