#include "app/startup_controller.hpp"
#include "control/drive_controller.hpp"
#include "input/rc_pwm_input.hpp"
#include "protocol/uart_protocol.hpp"
#include "robot/gait_generator.hpp"
#include "robot/cartesian_validation.hpp"
#include "robot/kinematics.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_geometry.hpp"
#include "robot/robot_model.hpp"
#include "robot/robot_pose.hpp"
#include "robot_params.hpp"
#include "servo/servo_calibration.hpp"
#include "servo/servo_frame_scheduler.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace {

bool near(float a, float b, float epsilon = 0.01f) {
    return std::fabs(a - b) <= epsilon;
}

control::RcPwmSnapshot rc_snapshot(uint16_t forward, uint16_t steer, uint64_t timestamp) {
    return {{forward, timestamp, true}, {steer, timestamp, true}};
}

void test_geometry_and_mount_order() {
    static_assert(servo::leg_index(servo::Leg::FR) == 0);
    static_assert(servo::leg_index(servo::Leg::MR) == 1);
    static_assert(servo::leg_index(servo::Leg::RR) == 2);
    static_assert(servo::leg_index(servo::Leg::RL) == 3);
    static_assert(servo::leg_index(servo::Leg::ML) == 4);
    static_assert(servo::leg_index(servo::Leg::FL) == 5);
    assert(robot::geometry_is_valid(robot::ROBOT_GEOMETRY));
    assert(near(robot::ROBOT_GEOMETRY.coxa_length_mm, 44.0f));
    assert(near(robot::ROBOT_GEOMETRY.tibia_length_mm, 125.24f));
    assert(near(robot::ROBOT_GEOMETRY.femur_zero_offset_deg, 35.27f));
    assert(near(robot::ROBOT_GEOMETRY.tibia_zero_offset_deg, -104.65f));
    assert(robot::ROBOT_GEOMETRY.legs[0].body_mount.y_mm < 0.0f);
    assert(robot::ROBOT_GEOMETRY.legs[5].body_mount.y_mm > 0.0f);
    assert(robot::ROBOT_GEOMETRY.legs[0].mount_yaw_rad < 0.0f);
    assert(robot::ROBOT_GEOMETRY.legs[5].mount_yaw_rad > 0.0f);
}

void test_zero_pose_mechanical_geometry() {
    for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
        const auto leg = static_cast<servo::Leg>(index);
        const auto target = robot::forward_kinematics(robot::ROBOT_GEOMETRY, leg, {});
        const auto local = robot::body_to_leg_frame(robot::ROBOT_GEOMETRY, leg, target);
        assert(near(std::sqrt(local.x_mm * local.x_mm + local.y_mm * local.y_mm), 153.4f, 1.0f));
        assert(near(local.z_mm, -71.0f, 1.0f));
        const auto result = robot::solve_leg_ik(robot::ROBOT_GEOMETRY, leg, target);
        assert(result.reachable);
        assert(near(result.pose.coxa_deg, 0.0f));
        assert(near(result.pose.femur_deg, 0.0f));
        assert(near(result.pose.tibia_deg, 0.0f));
    }
}

void test_body_leg_transform_round_trip() {
    for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
        const auto leg = static_cast<servo::Leg>(index);
        const robot::Vector3 local{151.0f, -27.0f, -62.0f};
        const auto body = robot::leg_to_body_frame(robot::ROBOT_GEOMETRY, leg, local);
        const auto reconstructed = robot::body_to_leg_frame(robot::ROBOT_GEOMETRY, leg, body);
        assert(near(reconstructed.x_mm, local.x_mm));
        assert(near(reconstructed.y_mm, local.y_mm));
        assert(near(reconstructed.z_mm, local.z_mm));
    }
}

void test_current_stand_is_neutral_fk_ik() {
    for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
        const auto leg = static_cast<servo::Leg>(index);
        const auto target = robot::forward_kinematics(robot::ROBOT_GEOMETRY, leg,
                                                       robot::STAND_POSE.legs[index]);
        const auto result = robot::solve_leg_ik(robot::ROBOT_GEOMETRY, leg, target);
        assert(result.reachable);
        assert(near(result.pose.coxa_deg, robot::STAND_POSE.legs[index].coxa_deg));
        assert(near(result.pose.femur_deg, robot::STAND_POSE.legs[index].femur_deg));
        assert(near(result.pose.tibia_deg, robot::STAND_POSE.legs[index].tibia_deg));
    }
}

void test_ik_fk_round_trip() {
    const std::array<robot::LegPose, 3> poses{{
        {5.0f, 2.0f, 32.0f},
        {10.0f, 15.0f, -10.0f},
        {-10.0f, -8.0f, 25.0f},
    }};
    for (const auto &pose : poses) {
        for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
            const auto leg = static_cast<servo::Leg>(index);
            const auto target = robot::forward_kinematics(robot::ROBOT_GEOMETRY, leg, pose);
            const auto result = robot::solve_leg_ik(robot::ROBOT_GEOMETRY, leg, target);
            assert(result.reachable);
            assert(near(result.pose.coxa_deg, pose.coxa_deg));
            assert(near(result.pose.femur_deg, pose.femur_deg));
            assert(near(result.pose.tibia_deg, pose.tibia_deg));
            const auto reconstructed = robot::forward_kinematics(robot::ROBOT_GEOMETRY, leg, result.pose);
            assert(near(target.x_mm, reconstructed.x_mm));
            assert(near(target.y_mm, reconstructed.y_mm));
            assert(near(target.z_mm, reconstructed.z_mm));
        }
    }
}

void test_cartesian_tripod_and_stop() {
    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    assert(gait.ready());
    const robot::BodyVelocityCommand forward{config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f};
    const auto first = gait.update(0.02f, forward);
    std::array<robot::FootTarget, servo::LEG_COUNT> later{};
    for (int i = 0; i < 8; ++i) later = gait.update(0.02f, forward);
    const size_t fr = servo::leg_index(servo::Leg::FR);
    assert(later[fr].x_mm < first[fr].x_mm); // forward stance moves backward in body frame
    assert(!gait.status().swing[fr]);
    for (int i = 0; i < 12; ++i) later = gait.update(0.02f, forward);
    assert(gait.status().swing[fr]);
    assert(later[fr].z_mm > gait.neutral_targets()[fr].z_mm); // continuous swing lift

    robot::GaitGenerator reverse(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    const auto reverse_first = reverse.update(0.02f, {-config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f});
    std::array<robot::FootTarget, servo::LEG_COUNT> reverse_later{};
    for (int i = 0; i < 8; ++i) reverse_later = reverse.update(0.02f, {-config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f});
    assert(reverse_later[fr].x_mm > reverse_first[fr].x_mm);

    robot::GaitGenerator yaw(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    const auto yaw_targets = yaw.update(0.02f, {0.0f, 0.0f, 0.4f});
    const auto &neutral = yaw.neutral_targets();
    const size_t mr = servo::leg_index(servo::Leg::MR);
    assert(!near(yaw_targets[fr].x_mm - neutral[fr].x_mm, yaw_targets[mr].x_mm - neutral[mr].x_mm) ||
           !near(yaw_targets[fr].y_mm - neutral[fr].y_mm, yaw_targets[mr].y_mm - neutral[mr].y_mm));

    for (int i = 0; i < 80; ++i) later = gait.update(0.02f, {});
    assert(gait.status().state == robot::CartesianGaitState::Idle);
    for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
        assert(near(later[index].x_mm, gait.neutral_targets()[index].x_mm));
        assert(near(later[index].y_mm, gait.neutral_targets()[index].y_mm));
        assert(near(later[index].z_mm, gait.neutral_targets()[index].z_mm));
    }
}

void test_pure_translation_and_yaw_velocities() {
    const robot::BodyVelocityCommand forward{80.0f, 0.0f, 0.0f};
    const robot::BodyVelocityCommand reverse{-80.0f, 0.0f, 0.0f};
    const robot::BodyVelocityCommand yaw{0.0f, 0.0f, 0.5f};
    const auto &geometry = robot::ROBOT_GEOMETRY;
    const auto fr_target = robot::forward_kinematics(geometry, servo::Leg::FR, robot::STAND_POSE.legs[0]);
    const auto fl_target = robot::forward_kinematics(geometry, servo::Leg::FL, robot::STAND_POSE.legs[5]);
    const auto fr_forward = robot::stance_velocity_for_body_command(forward, fr_target);
    const auto fr_reverse = robot::stance_velocity_for_body_command(reverse, fr_target);
    assert(near(fr_forward.x_mm, -80.0f) && near(fr_forward.y_mm, 0.0f));
    assert(near(fr_reverse.x_mm, 80.0f) && near(fr_reverse.y_mm, 0.0f));

    const auto fr_yaw = robot::stance_velocity_for_body_command(yaw, fr_target);
    const auto fl_yaw = robot::stance_velocity_for_body_command(yaw, fl_target);
    // -omega x r: left/right pair has mirrored X and the same Y at equal X.
    assert(near(fr_yaw.x_mm, -fl_yaw.x_mm, 0.1f));
    assert(near(fr_yaw.y_mm, fl_yaw.y_mm, 0.1f));
    assert(!near(fr_yaw.x_mm, 0.0f) || !near(fr_yaw.y_mm, 0.0f));

    const std::array<std::pair<servo::Leg, servo::Leg>, 3> mirrored_pairs{{
        {servo::Leg::FR, servo::Leg::FL},
        {servo::Leg::MR, servo::Leg::ML},
        {servo::Leg::RR, servo::Leg::RL},
    }};
    for (const auto &[right, left] : mirrored_pairs) {
        const auto right_target = robot::forward_kinematics(
            geometry, right, robot::STAND_POSE.legs[servo::leg_index(right)]);
        const auto left_target = robot::forward_kinematics(
            geometry, left, robot::STAND_POSE.legs[servo::leg_index(left)]);
        const auto right_velocity = robot::stance_velocity_for_body_command(yaw, right_target);
        const auto left_velocity = robot::stance_velocity_for_body_command(yaw, left_target);
        assert(near(right_velocity.x_mm, -left_velocity.x_mm, 0.1f));
        assert(near(right_velocity.y_mm, left_velocity.y_mm, 0.1f));
    }
}

void test_swing_height_is_not_stride_scaled() {
    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    const auto &neutral = gait.neutral_targets();
    const size_t fr = servo::leg_index(servo::Leg::FR);
    float highest = neutral[fr].z_mm;
    for (int i = 0; i < 25; ++i) {
        const auto targets = gait.update(0.02f, {1.0f, 0.0f, 0.0f});
        highest = std::max(highest, targets[fr].z_mm);
    }
    assert(highest - neutral[fr].z_mm > config::GAIT_STEP_HEIGHT_MM * 0.90f);
}

void test_phase_boundary_continuity() {
    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    const robot::BodyVelocityCommand command{config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f};
    gait.update(0.10f, command); // phase 0.20
    gait.update(0.10f, command); // phase 0.40
    gait.update(0.08f, command); // phase 0.56
    const auto before = gait.update(0.009f, command); // phase 0.578, stance
    const auto after = gait.update(0.002f, command);  // phase 0.582, swing
    const size_t fr = servo::leg_index(servo::Leg::FR);
    const float dx = after[fr].x_mm - before[fr].x_mm;
    const float dy = after[fr].y_mm - before[fr].y_mm;
    const float dz = after[fr].z_mm - before[fr].z_mm;
    assert(std::sqrt(dx * dx + dy * dy + dz * dz) < 3.0f);
}

void print_joint_ranges(const char *label, const robot::BodyVelocityCommand &command) {
    struct Range { float low = std::numeric_limits<float>::infinity(); float high = -std::numeric_limits<float>::infinity(); };
    std::array<Range, servo::SERVO_COUNT> ranges{};
    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    for (int sample = 0; sample < 70; ++sample) {
        const auto targets = gait.update(0.01f, command);
        const auto validation = robot::validate_cartesian_targets(robot::ROBOT_GEOMETRY, targets,
                                                                    servo::DEFAULT_SERVOS);
        assert(validation.valid());
        for (size_t leg = 0; leg < servo::LEG_COUNT; ++leg) {
            for (const auto joint : {servo::Joint::Coxa, servo::Joint::Femur, servo::Joint::Tibia}) {
                const size_t index = servo::servo_index(static_cast<servo::Leg>(leg), joint);
                const float angle = robot::pose_angle(validation.pose, static_cast<servo::Leg>(leg), joint);
                ranges[index].low = std::min(ranges[index].low, angle);
                ranges[index].high = std::max(ranges[index].high, angle);
            }
        }
    }
    std::cout << label << " joint ranges (deg):\n";
    for (size_t index = 0; index < ranges.size(); ++index) {
        const auto &config = servo::DEFAULT_SERVOS[index];
        std::cout << servo::leg_name(config.leg) << ' ' << servo::joint_name(config.joint)
                  << ": " << ranges[index].low << " ... " << ranges[index].high << '\n';
    }
}

void test_joint_ranges_for_full_forward_and_yaw() {
    print_joint_ranges("FORWARD", {config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f});
    print_joint_ranges("YAW", {0.0f, 0.0f, config::MAX_YAW_RATE_DEG_S * 0.01745329252f});
}

void test_minimum_rc_speed_remap() {
    const auto forward = control::apply_deadzone_and_speed(0.0f, 0.001f, 0.0f,
                                                            config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    const auto reverse = control::apply_deadzone_and_speed(0.0f, -0.001f, 0.0f,
                                                            config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    const auto turn = control::apply_deadzone_and_speed(0.001f, 0.0f, 0.0f,
                                                         config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    assert(near(forward.forward, config::RC_MIN_SPEED, 0.001f));
    assert(near(reverse.forward, -config::RC_MIN_SPEED, 0.001f));
    assert(near(turn.turn, config::RC_MIN_SPEED, 0.001f));
}

void test_rc_and_cartesian_velocity() {
    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    state = drive.update(rc_snapshot(1500, 1500, 500000), 500000);
    assert(state.armed && !state.command.active);
    state = drive.update(rc_snapshot(2100, 1500, 520000), 520000);
    assert(state.command.active && state.command.forward > 0.0f);
    const auto velocity = robot::body_velocity_from_drive(state.command);
    assert(velocity.vx_mm_s > 0.0f && near(velocity.vy_mm_s, 0.0f));
    state = drive.update(rc_snapshot(2100, 1500, 520000), 650001);
    assert(!state.signal_valid && !state.command.active);
}

void test_startup_reaches_holding_stand_on_50hz_ticks() {
    robot::MotionController motion;
    app::StartupController startup;
    startup.begin(0, motion);

    for (uint64_t now_us = 20000; now_us < 5000000; now_us += 20000) {
        motion.update(20000);
        startup.update(now_us, motion);
    }
    assert(startup.state() == app::StartupState::MovingToStand);
    assert(motion.moving());

    for (uint64_t now_us = 5000000; now_us <= 5100000; now_us += 20000) {
        motion.update(20000);
        startup.update(now_us, motion);
    }
    assert(startup.state() == app::StartupState::HoldingStand);
    assert(!motion.moving());
    for (size_t index = 0; index < servo::LEG_COUNT; ++index) {
        assert(near(motion.current_pose().legs[index].coxa_deg, robot::STAND_POSE.legs[index].coxa_deg));
        assert(near(motion.current_pose().legs[index].femur_deg, robot::STAND_POSE.legs[index].femur_deg));
        assert(near(motion.current_pose().legs[index].tibia_deg, robot::STAND_POSE.legs[index].tibia_deg));
    }
}

void test_rc_timestamp_race_is_saturated() {
    assert(control::rc_age_us(1000000, 1000004) == 0);
    control::DriveController drive;
    const auto state = drive.update(rc_snapshot(1500, 1500, 1000004), 1000000);
    assert(state.signal_valid);
    assert(state.forward_age_ms == 0 && state.steer_age_ms == 0);
}

void test_stance_displacement_and_velocity() {
    const float displacement = robot::stance_displacement_mm(60.0f);
    assert(near(displacement, 17.4f, 0.001f));

    constexpr float velocity_mm_s = 60.0f;
    constexpr float dt_s = 0.001f;
    const float stance_time_s = static_cast<float>(config::GAIT_CYCLE_MS) / 1000.0f *
                                config::GAIT_DUTY_FACTOR;
    const float phase_step = dt_s / stance_time_s;
    const float before = robot::stance_position_offset_mm(velocity_mm_s, 0.30f);
    const float after = robot::stance_position_offset_mm(velocity_mm_s, 0.30f + phase_step);
    assert(near((after - before) / dt_s, -velocity_mm_s, 0.01f));
}

void test_joint_limit_stride_scaling() {
    auto narrow_servos = servo::DEFAULT_SERVOS;
    for (size_t index = 0; index < narrow_servos.size(); ++index) {
        const auto leg = narrow_servos[index].leg;
        const auto joint = narrow_servos[index].joint;
        const float neutral = robot::pose_angle(robot::STAND_POSE, leg, joint);
        narrow_servos[index].min_angle_deg = neutral - 0.05f;
        narrow_servos[index].max_angle_deg = neutral + 0.05f;
    }
    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, narrow_servos);
    const auto targets = gait.update(0.02f, {config::MAX_FORWARD_SPEED_MM_S, 0.0f, 0.0f});
    const auto validation = robot::validate_cartesian_targets(robot::ROBOT_GEOMETRY, targets, narrow_servos);
    assert(gait.status().ik_limited);
    assert(gait.status().stride_scale < 0.08f);
    assert(validation.valid());
}

void test_startup_rc_gait_integration() {
    robot::MotionController motion;
    app::StartupController startup;
    startup.begin(0, motion);
    for (uint64_t now_us = 20000; now_us <= 5100000; now_us += 20000) {
        motion.update(20000);
        startup.update(now_us, motion);
    }
    assert(startup.state() == app::StartupState::HoldingStand);

    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    state = drive.update(rc_snapshot(1500, 1500, 500000), 500000);
    assert(state.armed);
    state = drive.update(rc_snapshot(2100, 1500, 520000), 520000);
    const auto command = robot::body_velocity_from_drive(state.command);
    assert(command.vx_mm_s > 0.0f);

    robot::GaitGenerator gait(robot::ROBOT_GEOMETRY, servo::DEFAULT_SERVOS);
    const auto first = gait.update(0.02f, command);
    std::array<robot::FootTarget, servo::LEG_COUNT> second{};
    for (int i = 0; i < 15; ++i) second = gait.update(0.02f, command);
    const auto validation = robot::validate_cartesian_targets(robot::ROBOT_GEOMETRY, second,
                                                                servo::DEFAULT_SERVOS);
    assert(gait.status().state == robot::CartesianGaitState::Walking ||
           gait.status().state == robot::CartesianGaitState::Limited);
    assert(gait.status().phase > 0.0f);
    assert(gait.status().workspace_scale > 0.99f);
    assert(gait.status().stride_scale > 0.99f);
    assert(validation.valid());
    const size_t fr = servo::leg_index(servo::Leg::FR);
    assert(!near(first[fr].x_mm, gait.neutral_targets()[fr].x_mm) ||
           !near(second[fr].x_mm, gait.neutral_targets()[fr].x_mm));
    motion.set_immediate_pose(validation.pose);
    assert(!near(motion.current_pose().legs[fr].femur_deg, robot::STAND_POSE.legs[fr].femur_deg) ||
           !near(motion.current_pose().legs[fr].tibia_deg, robot::STAND_POSE.legs[fr].tibia_deg));
}

void test_protocol_and_servo_layer() {
    assert(protocol::parse_command("STAND").command.type == protocol::CommandType::Stand);
    assert(protocol::parse_command("OLD_COMMAND").error == protocol::ParseError::UnknownCommand);
    const auto pulse = servo::angle_to_pulse_us(servo::DEFAULT_SERVOS[0], 0.0f);
    assert(pulse.result == servo::PulseResult::Ok && pulse.pulse_us == 1500);
    std::array<servo::ServoPulse, servo::SERVO_COUNT> pulses{};
    for (size_t index = 0; index < pulses.size(); ++index) {
        pulses[index] = {servo::DEFAULT_SERVOS[index].leg, servo::DEFAULT_SERVOS[index].gpio, 1500, true};
    }
    assert(servo::build_event_frame(pulses, servo::GROUP_A).valid);
    assert(servo::build_event_frame(pulses, servo::GROUP_B).valid);
}

void test_generated_params() {
    assert(config::STAND_FEMUR_DEG == 9.0f);
    assert(config::STAND_TIBIA_DEG == 18.0f);
    assert(config::GAIT_DUTY_FACTOR == 0.58f);
    assert(config::GAIT_CYCLE_MS == 500);
    assert(config::GAIT_STEP_HEIGHT_MM == 30.0f);
    assert(config::GAIT_STRIDE_MM == 35.0f);
    assert(config::MAX_FORWARD_SPEED_MM_S == 80.0f);
    assert(config::MAX_YAW_RATE_DEG_S == 45.0f);
}

} // namespace

int main() {
    test_geometry_and_mount_order();
    test_zero_pose_mechanical_geometry();
    test_body_leg_transform_round_trip();
    test_current_stand_is_neutral_fk_ik();
    test_ik_fk_round_trip();
    test_cartesian_tripod_and_stop();
    test_pure_translation_and_yaw_velocities();
    test_swing_height_is_not_stride_scaled();
    test_phase_boundary_continuity();
    test_joint_ranges_for_full_forward_and_yaw();
    test_minimum_rc_speed_remap();
    test_rc_and_cartesian_velocity();
    test_startup_reaches_holding_stand_on_50hz_ticks();
    test_rc_timestamp_race_is_saturated();
    test_stance_displacement_and_velocity();
    test_joint_limit_stride_scaling();
    test_startup_rc_gait_integration();
    test_protocol_and_servo_layer();
    test_generated_params();
    std::cout << "All Cartesian host tests passed\n";
    return 0;
}
