#include "protocol/uart_protocol.hpp"
#include "control/drive_controller.hpp"
#include "input/rc_pwm_input.hpp"
#include "robot/gait_controller.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"
#include "robot_params.hpp"
#include "servo/servo_calibration.hpp"
#include "servo/servo_frame_scheduler.hpp"
#include "storage/config_storage.hpp"
#include "storage/crc32.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>

namespace {

servo::ServoConfig temp_config(uint16_t minus45 = 2000, uint16_t center = 1500, uint16_t plus45 = 1000) {
    return servo::ServoConfig{
        servo::Leg::FR,
        servo::Joint::Coxa,
        2,
        center,
        minus45,
        plus45,
        -45.0f,
        45.0f,
        true
    };
}

bool near(float a, float b, float epsilon = 0.001f) {
    return std::fabs(a - b) <= epsilon;
}

bool pulse_near(uint16_t actual, uint16_t expected) {
    return std::abs(static_cast<int>(actual) - static_cast<int>(expected)) <= 1;
}

uint16_t pose_pulse_us(const robot::RobotPose &pose, servo::Leg leg, servo::Joint joint) {
    const auto &cfg = servo::DEFAULT_SERVOS[servo::servo_index(leg, joint)];
    return servo::angle_to_pulse_us(cfg, robot::pose_angle(pose, leg, joint)).pulse_us;
}

void test_angle_to_pulse_endpoints() {
    const auto cfg = temp_config();
    assert(servo::angle_to_pulse_us(cfg, -45.0f).pulse_us == 2000);
    assert(servo::angle_to_pulse_us(cfg, 0.0f).pulse_us == 1500);
    assert(servo::angle_to_pulse_us(cfg, 45.0f).pulse_us == 1000);
}

void test_interpolation() {
    const auto cfg = temp_config();
    assert(servo::angle_to_pulse_us(cfg, -22.5f).pulse_us == 1750);
    assert(servo::angle_to_pulse_us(cfg, 22.5f).pulse_us == 1250);
}

void test_increasing_and_decreasing_calibration() {
    const auto increasing = temp_config(1000, 1500, 2000);
    assert(servo::angle_to_pulse_us(increasing, -45.0f).pulse_us == 1000);
    assert(servo::angle_to_pulse_us(increasing, 45.0f).pulse_us == 2000);

    const auto decreasing = temp_config(2000, 1500, 1000);
    assert(servo::angle_to_pulse_us(decreasing, -45.0f).pulse_us == 2000);
    assert(servo::angle_to_pulse_us(decreasing, 45.0f).pulse_us == 1000);
}

void test_default_right_and_left_servo_calibration() {
    const auto &right = servo::DEFAULT_SERVOS[servo::servo_index(servo::Leg::FR, servo::Joint::Femur)];
    assert(servo::angle_to_pulse_us(right, -45.0f).pulse_us == 2000);
    assert(servo::angle_to_pulse_us(right, 0.0f).pulse_us == 1500);
    assert(servo::angle_to_pulse_us(right, 45.0f).pulse_us == 1000);

    const auto &left = servo::DEFAULT_SERVOS[servo::servo_index(servo::Leg::RL, servo::Joint::Femur)];
    assert(servo::angle_to_pulse_us(left, -45.0f).pulse_us == 1000);
    assert(servo::angle_to_pulse_us(left, 0.0f).pulse_us == 1500);
    assert(servo::angle_to_pulse_us(left, 45.0f).pulse_us == 2000);

    const auto right_stand_femur = servo::angle_to_pulse_us(right, -15.0f).pulse_us;
    const auto left_stand_femur = servo::angle_to_pulse_us(left, -15.0f).pulse_us;
    assert(right_stand_femur > servo::DEFAULT_CENTER_US);
    assert(left_stand_femur < servo::DEFAULT_CENTER_US);
}

void test_left_and_right_pulses_are_mirrored() {
    const auto &right = servo::DEFAULT_SERVOS[servo::servo_index(servo::Leg::FR, servo::Joint::Tibia)];
    const auto &left = servo::DEFAULT_SERVOS[servo::servo_index(servo::Leg::FL, servo::Joint::Tibia)];

    assert(servo::angle_to_pulse_us(right, -45.0f).pulse_us == 2000);
    assert(servo::angle_to_pulse_us(right, 0.0f).pulse_us == 1500);
    assert(servo::angle_to_pulse_us(right, 45.0f).pulse_us == 1000);

    assert(servo::angle_to_pulse_us(left, -45.0f).pulse_us == 1000);
    assert(servo::angle_to_pulse_us(left, 0.0f).pulse_us == 1500);
    assert(servo::angle_to_pulse_us(left, 45.0f).pulse_us == 2000);

    const auto right_pulse = servo::angle_to_pulse_us(right, 20.0f).pulse_us;
    const auto left_pulse = servo::angle_to_pulse_us(left, 20.0f).pulse_us;
    assert(right_pulse < servo::DEFAULT_CENTER_US);
    assert(left_pulse > servo::DEFAULT_CENTER_US);
}

void test_stand_pose_default_pulses() {
    constexpr std::array<servo::Leg, 3> right_legs = {{
        servo::Leg::FR,
        servo::Leg::MR,
        servo::Leg::RR,
    }};
    constexpr std::array<servo::Leg, 3> left_legs = {{
        servo::Leg::RL,
        servo::Leg::ML,
        servo::Leg::FL,
    }};

    for (const auto leg : right_legs) {
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Coxa), 1500));
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Femur), 1400));
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Tibia), 1500));
    }

    for (const auto leg : left_legs) {
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Coxa), 1500));
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Femur), 1600));
        assert(pulse_near(pose_pulse_us(robot::STAND_POSE, leg, servo::Joint::Tibia), 1500));
    }

    const auto right_femur = pose_pulse_us(robot::STAND_POSE, servo::Leg::FR, servo::Joint::Femur);
    const auto left_femur = pose_pulse_us(robot::STAND_POSE, servo::Leg::FL, servo::Joint::Femur);
    assert(right_femur < servo::DEFAULT_CENTER_US);
    assert(left_femur > servo::DEFAULT_CENTER_US);
}

void test_angle_clamping() {
    auto cfg = temp_config();
    cfg.min_angle_deg = -20.0f;
    cfg.max_angle_deg = 10.0f;
    assert(servo::angle_to_pulse_us(cfg, -45.0f).pulse_us == servo::angle_to_pulse_us(cfg, -20.0f).pulse_us);
    assert(servo::angle_to_pulse_us(cfg, 45.0f).pulse_us == servo::angle_to_pulse_us(cfg, 10.0f).pulse_us);
}

void test_invalid_pulses() {
    auto cfg = temp_config();
    cfg.pulse_plus_45_us = 3000;
    const auto result = servo::angle_to_pulse_us(cfg, 45.0f);
    assert(result.result == servo::PulseResult::ConfigInvalid);
    assert(result.pulse_us == servo::DEFAULT_CENTER_US);
}

void test_crc32() {
    const char *text = "123456789";
    assert(storage::crc32(text, strlen(text)) == 0xcbf43926u);
}

void test_config_validation() {
    storage::StoredConfigImage image{};
    image.header.magic = storage::CONFIG_MAGIC;
    image.header.version = storage::CONFIG_VERSION;
    image.header.payload_size = sizeof(storage::StoredConfigPayload);
    image.header.crc32 = storage::crc32(&image.payload, sizeof(image.payload));
    assert(storage::validate_config_image(image) == storage::ConfigStatus::Valid);

    image.header.magic = 0;
    assert(storage::validate_config_image(image) == storage::ConfigStatus::InvalidMagic);
    image.header.magic = storage::CONFIG_MAGIC;
    image.header.version = storage::CONFIG_VERSION + 1;
    assert(storage::validate_config_image(image) == storage::ConfigStatus::InvalidVersion);
}

void test_uart_parser_valid() {
    auto parsed = protocol::parse_command("ping");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::Ping);

    parsed = protocol::parse_command("zero");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::Zero);

    parsed = protocol::parse_command("SERVO fr coxa 10");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::Servo);
    assert(parsed.command.leg == servo::Leg::FR);
    assert(parsed.command.joint == servo::Joint::Coxa);

    parsed = protocol::parse_command("LEG FL 0 -15 20");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::Leg);

    parsed = protocol::parse_command("MARCH");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::March);

    parsed = protocol::parse_command("walk demo");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::WalkDemo);

    parsed = protocol::parse_command("WALK STOP");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::WalkStop);

    parsed = protocol::parse_command("GAIT STATUS");
    assert(parsed.error == protocol::ParseError::None);
    assert(parsed.command.type == protocol::CommandType::GaitStatus);
}

void test_uart_parser_invalid() {
    assert(protocol::parse_command("SERVO XX COXA 0").error == protocol::ParseError::InvalidLeg);
    assert(protocol::parse_command("SERVO FR BAD 0").error == protocol::ParseError::InvalidJoint);
    assert(protocol::parse_command("SERVO FR COXA 90").error == protocol::ParseError::AngleOutOfRange);
    assert(protocol::parse_command("WHAT").error == protocol::ParseError::UnknownCommand);
}

std::array<servo::ServoPulse, servo::SERVO_COUNT> default_pulses() {
    std::array<servo::ServoPulse, servo::SERVO_COUNT> pulses{};
    for (size_t i = 0; i < servo::DEFAULT_SERVOS.size(); ++i) {
        const auto &cfg = servo::DEFAULT_SERVOS[i];
        pulses[i] = servo::ServoPulse{cfg.leg, cfg.gpio, 1500, true};
    }
    return pulses;
}

void test_phase_schedule() {
    const auto pulses = default_pulses();
    const auto frame = servo::build_event_frame(pulses, servo::GROUP_A);
    assert(frame.valid);
    assert(frame.count > 0);
    assert(frame.events[0].time_us == 0);
    assert((frame.events[0].mask & 0x7u) == 0x7u);

    bool saw_mr_phase = false;
    for (size_t i = 0; i < frame.count; ++i) {
        if (frame.events[i].time_us == 3333) {
            saw_mr_phase = true;
        }
    }
    assert(saw_mr_phase);
}

void test_no_events_outside_frame() {
    const auto pulses = default_pulses();
    const auto frame_a = servo::build_event_frame(pulses, servo::GROUP_A);
    const auto frame_b = servo::build_event_frame(pulses, servo::GROUP_B);
    for (size_t i = 0; i < frame_a.count; ++i) {
        assert(frame_a.events[i].time_us <= servo::FRAME_PERIOD_US);
    }
    for (size_t i = 0; i < frame_b.count; ++i) {
        assert(frame_b.events[i].time_us <= servo::FRAME_PERIOD_US);
    }
}

bool pose_close(const robot::RobotPose &a, const robot::RobotPose &b) {
    constexpr float EPSILON = 0.001f;
    for (size_t i = 0; i < a.legs.size(); ++i) {
        if (std::fabs(a.legs[i].coxa_deg - b.legs[i].coxa_deg) > EPSILON ||
            std::fabs(a.legs[i].femur_deg - b.legs[i].femur_deg) > EPSILON ||
            std::fabs(a.legs[i].tibia_deg - b.legs[i].tibia_deg) > EPSILON) {
            return false;
        }
    }
    return true;
}

bool coxa_close(const robot::RobotPose &a, const robot::RobotPose &b) {
    constexpr float EPSILON = 0.001f;
    for (size_t i = 0; i < a.legs.size(); ++i) {
        if (std::fabs(a.legs[i].coxa_deg - b.legs[i].coxa_deg) > EPSILON) {
            return false;
        }
    }
    return true;
}

control::RcPwmSnapshot rc_snapshot(uint16_t forward_us, uint16_t steer_us, uint64_t now_us) {
    return control::RcPwmSnapshot{
        control::RcChannelSnapshot{forward_us, now_us, true},
        control::RcChannelSnapshot{steer_us, now_us, true}
    };
}

void advance_gait(robot::GaitController &gait, robot::MotionController &motion, uint64_t &now_us) {
    now_us += 20000;
    gait.update(now_us, motion, servo::DEFAULT_SERVOS, true);
    motion.update(20000);
}

robot::RobotPose wait_for_state(robot::GaitController &gait,
                                robot::MotionController &motion,
                                uint64_t &now_us,
                                robot::GaitState state) {
    for (int i = 0; i < 2000; ++i) {
        advance_gait(gait, motion, now_us);
        if (gait.state() == state) {
            return motion.target_pose();
        }
    }
    assert(false && "expected gait state was not reached");
    return motion.target_pose();
}

void run_gait_until_idle(robot::GaitController &gait, robot::MotionController &motion, uint64_t &now_us) {
    for (int i = 0; i < 2000; ++i) {
        advance_gait(gait, motion, now_us);
        if (!gait.active() && !motion.moving()) {
            return;
        }
    }
    assert(false && "gait did not finish");
}

void test_tripod_groups() {
    size_t seen[servo::LEG_COUNT] = {};
    for (const auto leg : robot::gait::TRIPOD_A) {
        ++seen[servo::leg_index(leg)];
        assert(!robot::gait::leg_in_tripod(leg, robot::gait::TRIPOD_B));
    }
    for (const auto leg : robot::gait::TRIPOD_B) {
        ++seen[servo::leg_index(leg)];
        assert(!robot::gait::leg_in_tripod(leg, robot::gait::TRIPOD_A));
    }
    assert(robot::gait::leg_in_tripod(servo::Leg::FR, robot::gait::TRIPOD_A));
    assert(robot::gait::leg_in_tripod(servo::Leg::ML, robot::gait::TRIPOD_A));
    assert(robot::gait::leg_in_tripod(servo::Leg::RR, robot::gait::TRIPOD_A));
    assert(robot::gait::leg_in_tripod(servo::Leg::FL, robot::gait::TRIPOD_B));
    assert(robot::gait::leg_in_tripod(servo::Leg::MR, robot::gait::TRIPOD_B));
    assert(robot::gait::leg_in_tripod(servo::Leg::RL, robot::gait::TRIPOD_B));
    for (const auto count : seen) {
        assert(count == 1);
    }
}

void test_gait_pose_limits() {
    assert(pose_close(robot::ZERO_POSE, robot::CENTER_POSE));
    assert(!pose_close(robot::ZERO_POSE, robot::STAND_POSE));
    assert(robot::STAND_LEG_POSE.coxa_deg == config::STAND_COXA_DEG);
    assert(robot::STAND_LEG_POSE.femur_deg == config::STAND_FEMUR_DEG);
    assert(robot::STAND_LEG_POSE.tibia_deg == config::STAND_TIBIA_DEG);

    const auto prelifted_a = robot::prelifted_pose(robot::gait::TRIPOD_A);
    const auto prelifted_b = robot::prelifted_pose(robot::gait::TRIPOD_B);
    const auto lifted_a = robot::lifted_pose(robot::gait::TRIPOD_A);
    const auto lifted_b = robot::lifted_pose(robot::gait::TRIPOD_B);
    assert(robot::pose_within_servo_limits(prelifted_a, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(prelifted_b, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(lifted_a, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(lifted_b, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_cycle_start_pose(), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_prelift_pose(robot::gait::TRIPOD_A), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_prelift_pose(robot::gait::TRIPOD_B), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_lift_pose(robot::gait::TRIPOD_A), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_lift_pose(robot::gait::TRIPOD_B), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_transfer_pose(robot::gait::TRIPOD_A), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_transfer_pose(robot::gait::TRIPOD_B), servo::DEFAULT_SERVOS));

    for (const auto leg : robot::gait::TRIPOD_A) {
        const auto &pose = lifted_a.legs[servo::leg_index(leg)];
        const auto &prelift = prelifted_a.legs[servo::leg_index(leg)];
        assert(prelift.coxa_deg == robot::STAND_LEG_POSE.coxa_deg);
        assert(prelift.femur_deg == robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_PRELIFT_FEMUR_DELTA_DEG);
        assert(prelift.tibia_deg == robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_PRELIFT_TIBIA_DELTA_DEG);
        assert(pose.coxa_deg == robot::STAND_LEG_POSE.coxa_deg);
        assert(pose.femur_deg == robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_LIFT_FEMUR_DELTA_DEG);
        assert(pose.tibia_deg == robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_LIFT_TIBIA_DELTA_DEG);
    }
    for (const auto leg : robot::gait::TRIPOD_B) {
        const auto &pose = lifted_b.legs[servo::leg_index(leg)];
        const auto &prelift = prelifted_b.legs[servo::leg_index(leg)];
        assert(prelift.coxa_deg == robot::STAND_LEG_POSE.coxa_deg);
        assert(prelift.femur_deg == robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_PRELIFT_FEMUR_DELTA_DEG);
        assert(prelift.tibia_deg == robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_PRELIFT_TIBIA_DELTA_DEG);
        assert(pose.coxa_deg == robot::STAND_LEG_POSE.coxa_deg);
        assert(pose.femur_deg == robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_LIFT_FEMUR_DELTA_DEG);
        assert(pose.tibia_deg == robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_LIFT_TIBIA_DELTA_DEG);
    }

    for (const auto &config : servo::DEFAULT_SERVOS) {
        if (config.joint != servo::Joint::Coxa) {
            continue;
        }
        assert(robot::gait::forwardCoxa(config.leg) >= config.min_angle_deg);
        assert(robot::gait::forwardCoxa(config.leg) <= config.max_angle_deg);
        assert(robot::gait::backwardCoxa(config.leg) >= config.min_angle_deg);
        assert(robot::gait::backwardCoxa(config.leg) <= config.max_angle_deg);
        assert(near(std::fabs(robot::gait::forwardCoxa(config.leg)), robot::gait::GAIT_COXA_SWING_MAX_DEG));
        assert(near(std::fabs(robot::gait::backwardCoxa(config.leg)), robot::gait::GAIT_COXA_SWING_MAX_DEG));
    }
}

void test_robot_params_generated_values() {
    assert(config::CONTROL_MODE == config::ControlInputMode::RC_PWM);
    assert(config::RC_MIN_US == 900);
    assert(config::RC_CENTER_US == 1500);
    assert(config::RC_MAX_US == 2100);
    assert(config::RC_VALID_MIN_US == 800);
    assert(config::RC_VALID_MAX_US == 2200);
    assert(config::RC_DEADBAND_US == 80);
    assert(config::RC_ACTIVE_THRESHOLD == 0.12f);
    assert(!config::RC_FORWARD_REVERSED);
    assert(!config::RC_STEER_REVERSED);
    assert(config::RC_MIN_SPEED == 0.45f);
    assert(config::RC_ARM_NEUTRAL_MS == 500);
    assert(config::SERIAL_DASHBOARD);
    assert(config::SERIAL_DASHBOARD_RATE_HZ == 10);
    assert(config::SERIAL_DASHBOARD_ANSI);
    assert(config::STAND_COXA_DEG == 0.0f);
    assert(config::STAND_FEMUR_DEG == 9.0f);
    assert(config::STAND_TIBIA_DEG == 0.0f);
    assert(config::GAIT_PRELIFT_FEMUR_DELTA_DEG == 6.0f);
    assert(config::GAIT_PRELIFT_TIBIA_DELTA_DEG == 0.0f);
    assert(config::GAIT_LIFT_FEMUR_DELTA_DEG == 14.0f);
    assert(config::GAIT_LIFT_TIBIA_DELTA_DEG == -18.0f);
    assert(config::GAIT_COXA_SWING_MIN_DEG == 8.0f);
    assert(config::GAIT_COXA_SWING_MAX_DEG == 16.0f);
    assert(config::GAIT_PRELIFT_MIN_MS == 80);
    assert(config::GAIT_LIFT_MIN_MS == 100);
    assert(config::GAIT_TRANSFER_MIN_MS == 160);
    assert(config::GAIT_LOWER_MIN_MS == 120);
}

void test_rc_normalization() {
    assert(near(control::normalize_rc_channel_fixed(900, false), -1.0f));
    assert(near(control::normalize_rc_channel_fixed(1500, false), 0.0f));
    assert(near(control::normalize_rc_channel_fixed(2100, false), 1.0f));
    assert(near(control::normalize_rc_channel_fixed(800, false), -1.0f));
    assert(near(control::normalize_rc_channel_fixed(2200, false), 1.0f));

    assert(near(control::normalize_rc_channel_fixed(1420, false), 0.0f));
    assert(near(control::normalize_rc_channel_fixed(1580, false), 0.0f));
    assert(near(control::normalize_rc_channel_fixed(1470, false), 0.0f));
    assert(near(control::normalize_rc_channel_fixed(1535, false), 0.0f));
    assert(control::normalize_rc_channel_fixed(1419, false) < 0.0f);
    assert(control::normalize_rc_channel_fixed(1581, false) > 0.0f);

    assert(near(control::normalize_rc_channel_fixed(2100, true), -1.0f));
    assert(near(control::normalize_rc_channel_fixed(900, true), 1.0f));
    assert(near(control::normalize_rc_channel_fixed(1538, false), 0.0f));
    assert(near(control::normalize_rc_channel_fixed(1660, false), 0.153846f));

    assert(control::rc_pulse_neutral(1420));
    assert(control::rc_pulse_neutral(1580));
    assert(control::rc_pulse_neutral(1470));
    assert(control::rc_pulse_neutral(1535));
    assert(!control::rc_pulse_neutral(1419));
    assert(!control::rc_pulse_neutral(1581));
}

void test_no_radial_deadzone_and_min_speed() {
    auto command = control::apply_deadzone_and_speed(0.0f, 0.0f, 0.0f, config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    assert(!command.active);
    assert(command.speed == 0.0f);

    command = control::apply_deadzone_and_speed(0.0f, 0.001f, 0.0f, config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    assert(command.active);
    assert(command.forward > 0.0f);
    assert(command.speed >= config::RC_MIN_SPEED);

    command = control::apply_deadzone_and_speed(0.0f, 1.0f, 0.0f, config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    assert(command.active);
    assert(near(command.forward, 1.0f));
    assert(near(command.speed, 1.0f));
}

void test_diagonal_and_differential_mixing() {
    auto command = control::apply_deadzone_and_speed(1.0f, 1.0f, 0.0f, config::RC_MIN_SPEED, config::RC_MAX_SPEED);
    assert(command.active);
    assert(command.forward <= 1.0f);
    assert(command.turn <= 1.0f);
    assert(near(std::sqrt(command.forward * command.forward + command.turn * command.turn), 1.0f));

    auto mix = control::differential_mix(1.0f, 0.0f, 1.0f);
    assert(near(mix.left, 1.0f));
    assert(near(mix.right, 1.0f));

    mix = control::differential_mix(-1.0f, 0.0f, 1.0f);
    assert(near(mix.left, -1.0f));
    assert(near(mix.right, -1.0f));

    mix = control::differential_mix(0.0f, -1.0f, 1.0f);
    assert(near(mix.left, -1.0f));
    assert(near(mix.right, 1.0f));

    mix = control::differential_mix(0.0f, 1.0f, 1.0f);
    assert(near(mix.left, 1.0f));
    assert(near(mix.right, -1.0f));

    mix = control::differential_mix(1.0f, 1.0f, 1.0f);
    assert(near(mix.left, 1.0f));
    assert(near(mix.right, 0.0f));
}

void test_rc_failsafe_and_arming() {
    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    assert(state.signal_valid);
    assert(state.forward_neutral);
    assert(state.steer_neutral);
    assert(!state.armed);
    assert(state.waiting_neutral);
    assert(state.arm_elapsed_ms == 0);
    assert(!state.command.active);

    state = drive.update(rc_snapshot(1500, 1500, 499000), 499000);
    assert(state.signal_valid);
    assert(!state.armed);
    assert(state.waiting_neutral);
    assert(state.arm_elapsed_ms == 499);

    state = drive.update(rc_snapshot(1500, 1500, 500000), 500000);
    assert(state.signal_valid);
    assert(state.armed);
    assert(state.arm_elapsed_ms == 500);
    assert(!state.command.active);

    state = drive.update(rc_snapshot(2100, 1500, 520000), 520000);
    assert(state.signal_valid);
    assert(!state.forward_neutral);
    assert(state.steer_neutral);
    assert(state.armed);
    assert(state.arm_elapsed_ms == 500);
    assert(state.command.active);
    assert(state.command.forward > 0.0f);

    state = drive.update(rc_snapshot(2100, 1500, 520000), 650001);
    assert(!state.signal_valid);
    assert(!state.armed);
    assert(!state.command.active);

    state = drive.update(rc_snapshot(1500, 1500, 660000), 660000);
    assert(state.signal_valid);
    assert(!state.armed);
    assert(state.waiting_neutral);
    assert(state.arm_elapsed_ms == 0);

    state = drive.update(rc_snapshot(1500, 1500, 1159000), 1159000);
    assert(!state.armed);
    assert(state.arm_elapsed_ms == 499);

    state = drive.update(rc_snapshot(1500, 1500, 1160000), 1160000);
    assert(state.armed);
    assert(state.arm_elapsed_ms == 500);
}

void test_arming_timer_resets_when_stick_leaves_neutral() {
    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    assert(!state.armed);

    state = drive.update(rc_snapshot(1581, 1500, 250000), 250000);
    assert(state.signal_valid);
    assert(!state.armed);
    assert(state.waiting_neutral);

    state = drive.update(rc_snapshot(1500, 1500, 260000), 260000);
    assert(!state.armed);

    state = drive.update(rc_snapshot(1500, 1500, 759000), 759000);
    assert(!state.armed);

    state = drive.update(rc_snapshot(1500, 1500, 760000), 760000);
    assert(state.armed);
}

void test_active_threshold_blocks_micro_command() {
    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    state = drive.update(rc_snapshot(1500, 1500, 500000), 500000);
    assert(state.armed);

    for (uint64_t i = 1; i <= 20; ++i) {
        state = drive.update(rc_snapshot(1640, 1500, 500000 + i * 20000), 500000 + i * 20000);
        assert(state.signal_valid);
        assert(state.armed);
        assert(!state.command.active);
        assert(state.command_magnitude == 0.0f);
    }

    for (uint64_t i = 21; i <= 35; ++i) {
        state = drive.update(rc_snapshot(1660, 1500, 500000 + i * 20000), 500000 + i * 20000);
    }
    assert(state.command.active);
    assert(state.command_magnitude >= config::RC_ACTIVE_THRESHOLD);
}

void test_release_to_neutral_stops_drive_command() {
    control::DriveController drive;
    auto state = drive.update(rc_snapshot(1500, 1500, 0), 0);
    state = drive.update(rc_snapshot(1500, 1500, 500000), 500000);
    assert(state.armed);

    state = drive.update(rc_snapshot(2100, 1500, 520000), 520000);
    assert(state.command.active);

    state = drive.update(rc_snapshot(1500, 1500, 540000), 540000);
    assert(state.armed);
    assert(state.forward_neutral);
    assert(state.steer_neutral);
    assert(!state.command.active);
    assert(state.command_magnitude == 0.0f);
}

void test_invalid_pulse_rejection_helper() {
    assert(input::rc_pulse_width_valid(800, config::RC_VALID_MIN_US, config::RC_VALID_MAX_US));
    assert(input::rc_pulse_width_valid(2200, config::RC_VALID_MIN_US, config::RC_VALID_MAX_US));
    assert(!input::rc_pulse_width_valid(799, config::RC_VALID_MIN_US, config::RC_VALID_MAX_US));
    assert(!input::rc_pulse_width_valid(2201, config::RC_VALID_MIN_US, config::RC_VALID_MAX_US));
}

void test_rc_age_saturates_future_timestamp() {
    assert(control::rc_age_us(1000000, 1000005) == 0);
    assert(control::rc_age_us(1000000, 999995) == 5);

    control::RcPwmSnapshot snapshot = rc_snapshot(1500, 1500, 1000005);
    snapshot.steer.updated_us = 1000003;

    control::DriveController drive;
    const auto state = drive.update(snapshot, 1000000);
    assert(state.signal_valid);
    assert(state.forward_age_ms == 0);
    assert(state.steer_age_ms == 0);
    assert(state.forward_neutral);
    assert(state.steer_neutral);
    assert(!state.armed);
}

void test_real_pwm_refresh_does_not_reset_neutral_timer() {
    control::DriveController drive;
    control::DriveControllerState state{};
    for (uint64_t now_us = 0; now_us <= 500000; now_us += 20000) {
        const uint16_t forward = (now_us / 20000) % 2 == 0 ? 1500 : 1501;
        const uint16_t steer = (now_us / 20000) % 2 == 0 ? 1500 : 1499;
        state = drive.update(rc_snapshot(forward, steer, now_us), now_us);
        assert(state.signal_valid);
        assert(state.forward_neutral);
        assert(state.steer_neutral);
    }
    assert(state.armed);
    assert(state.arm_elapsed_ms == 500);
}

void test_transient_future_timestamp_race_each_frame() {
    control::DriveController drive;
    control::DriveControllerState state{};
    for (uint64_t i = 0; i <= 25; ++i) {
        const uint64_t now_us = 100000 + i * 20000;
        auto snapshot = rc_snapshot(1504, 1484, now_us + 1 + (i % 5));
        snapshot.steer.updated_us = now_us + 1 + (i % 3);
        state = drive.update(snapshot, now_us);
        assert(state.signal_valid);
        assert(state.forward_age_ms == 0);
        assert(state.steer_age_ms == 0);
        assert(state.forward_neutral);
        assert(state.steer_neutral);
    }
    assert(state.armed);
    assert(state.arm_elapsed_ms == 500);
}

void test_march_returns_to_stand_after_two_cycles() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);
    assert(gait.start_march(now_us, motion, servo::DEFAULT_SERVOS));
    run_gait_until_idle(gait, motion, now_us);
    assert(!gait.active());
    assert(pose_close(motion.target_pose(), robot::STAND_POSE));
    assert(pose_close(motion.current_pose(), robot::STAND_POSE));
}

void test_timed_motion_smootherstep_keeps_joints_synchronized() {
    robot::MotionController motion;
    motion.reset(robot::ZERO_POSE);

    robot::RobotPose target = robot::ZERO_POSE;
    for (auto &leg : target.legs) {
        leg.coxa_deg = 10.0f;
        leg.femur_deg = 20.0f;
        leg.tibia_deg = 5.0f;
    }

    motion.set_target_timed(target, 1000, robot::MotionInterpolation::SmootherStep);
    motion.update(500000);
    assert(motion.moving());
    assert(motion.progress_percent() == 50);
    for (const auto &leg : motion.current_pose().legs) {
        assert(near(leg.coxa_deg, 5.0f));
        assert(near(leg.femur_deg, 10.0f));
        assert(near(leg.tibia_deg, 2.5f));
    }

    motion.update(500000);
    assert(!motion.moving());
    assert(motion.progress_percent() == 100);
    assert(pose_close(motion.current_pose(), target));
}

void test_walk_cycle_returns_to_start_phase() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);
    assert(gait.start_walk_demo(now_us, motion, servo::DEFAULT_SERVOS));

    for (int i = 0; i < 1000; ++i) {
        advance_gait(gait, motion, now_us);
        if (gait.state() == robot::GaitState::WalkBLower && gait.cycle() == 0) {
            assert(pose_close(motion.target_pose(), robot::walk_cycle_start_pose()));
            return;
        }
    }
    assert(false && "walk cycle end phase was not reached");
}

void test_rc_drive_coxa_continuity_across_tripod_phases() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);

    control::DriveCommand command{};
    command.forward = 1.0f;
    command.turn = 0.0f;
    command.speed = 1.0f;
    command.active = true;

    gait.update_drive(now_us, motion, servo::DEFAULT_SERVOS, command, true);
    assert(gait.state() == robot::GaitState::WalkPrepare);

    const auto a_prelift = wait_for_state(gait, motion, now_us, robot::GaitState::WalkAPrelift);
    const auto a_lift = wait_for_state(gait, motion, now_us, robot::GaitState::WalkALift);
    const auto a_transfer = wait_for_state(gait, motion, now_us, robot::GaitState::WalkATransfer);
    const auto a_lower = wait_for_state(gait, motion, now_us, robot::GaitState::WalkALower);
    const auto b_prelift = wait_for_state(gait, motion, now_us, robot::GaitState::WalkBPrelift);
    const auto b_lift = wait_for_state(gait, motion, now_us, robot::GaitState::WalkBLift);
    const auto b_transfer = wait_for_state(gait, motion, now_us, robot::GaitState::WalkBTransfer);
    const auto b_lower = wait_for_state(gait, motion, now_us, robot::GaitState::WalkBLower);
    const auto next_a_prelift = wait_for_state(gait, motion, now_us, robot::GaitState::WalkAPrelift);

    assert(coxa_close(a_prelift, a_lift));
    assert(!coxa_close(a_lift, a_transfer));
    assert(coxa_close(a_transfer, a_lower));
    assert(coxa_close(a_lower, b_prelift));
    assert(coxa_close(b_prelift, b_lift));
    assert(!coxa_close(b_lift, b_transfer));
    assert(coxa_close(b_transfer, b_lower));
    assert(coxa_close(b_lower, next_a_prelift));
}

void test_walk_uses_prelift_before_lift() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);
    assert(gait.start_walk_demo(now_us, motion, servo::DEFAULT_SERVOS));

    for (int i = 0; i < 200; ++i) {
        advance_gait(gait, motion, now_us);
        if (gait.state() == robot::GaitState::WalkAPrelift) {
            const auto &pose = motion.target_pose().legs[servo::leg_index(robot::gait::TRIPOD_A[0])];
            assert(near(pose.femur_deg, robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_PRELIFT_FEMUR_DELTA_DEG));
            assert(near(pose.tibia_deg, robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_PRELIFT_TIBIA_DELTA_DEG));
            break;
        }
    }
    assert(gait.state() == robot::GaitState::WalkAPrelift);

    for (int i = 0; i < 200; ++i) {
        advance_gait(gait, motion, now_us);
        if (gait.state() == robot::GaitState::WalkALift) {
            const auto &pose = motion.target_pose().legs[servo::leg_index(robot::gait::TRIPOD_A[0])];
            assert(near(pose.femur_deg, robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_LIFT_FEMUR_DELTA_DEG));
            assert(near(pose.tibia_deg, robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_LIFT_TIBIA_DELTA_DEG));
            return;
        }
    }
    assert(false && "walk full lift phase was not reached");
}

void test_walk_stop_returns_to_stand() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);
    assert(gait.start_walk_demo(now_us, motion, servo::DEFAULT_SERVOS));
    for (int i = 0; i < 50; ++i) {
        advance_gait(gait, motion, now_us);
    }
    gait.stop(now_us, motion, servo::DEFAULT_SERVOS);
    run_gait_until_idle(gait, motion, now_us);
    assert(pose_close(motion.target_pose(), robot::STAND_POSE));
    assert(pose_close(motion.current_pose(), robot::STAND_POSE));
}

void test_walk_demo_returns_to_stand_after_three_cycles() {
    robot::MotionController motion;
    robot::GaitController gait;
    uint64_t now_us = 0;
    motion.reset(robot::STAND_POSE);
    assert(gait.start_walk_demo(now_us, motion, servo::DEFAULT_SERVOS));
    run_gait_until_idle(gait, motion, now_us);
    assert(!gait.active());
    assert(pose_close(motion.target_pose(), robot::STAND_POSE));
    assert(pose_close(motion.current_pose(), robot::STAND_POSE));
}

} // namespace

int main() {
    test_angle_to_pulse_endpoints();
    test_interpolation();
    test_increasing_and_decreasing_calibration();
    test_default_right_and_left_servo_calibration();
    test_left_and_right_pulses_are_mirrored();
    test_stand_pose_default_pulses();
    test_angle_clamping();
    test_invalid_pulses();
    test_crc32();
    test_config_validation();
    test_uart_parser_valid();
    test_uart_parser_invalid();
    test_phase_schedule();
    test_no_events_outside_frame();
    test_robot_params_generated_values();
    test_rc_normalization();
    test_no_radial_deadzone_and_min_speed();
    test_diagonal_and_differential_mixing();
    test_rc_failsafe_and_arming();
    test_arming_timer_resets_when_stick_leaves_neutral();
    test_active_threshold_blocks_micro_command();
    test_release_to_neutral_stops_drive_command();
    test_invalid_pulse_rejection_helper();
    test_rc_age_saturates_future_timestamp();
    test_real_pwm_refresh_does_not_reset_neutral_timer();
    test_transient_future_timestamp_race_each_frame();
    test_tripod_groups();
    test_gait_pose_limits();
    test_march_returns_to_stand_after_two_cycles();
    test_timed_motion_smootherstep_keeps_joints_synchronized();
    test_walk_cycle_returns_to_start_phase();
    test_rc_drive_coxa_continuity_across_tripod_phases();
    test_walk_uses_prelift_before_lift();
    test_walk_stop_returns_to_stand();
    test_walk_demo_returns_to_stand_after_three_cycles();
    std::cout << "All host logic tests passed\n";
    return 0;
}
