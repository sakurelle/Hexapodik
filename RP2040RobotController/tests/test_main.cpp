#include "protocol/uart_protocol.hpp"
#include "robot/gait_controller.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"
#include "servo/servo_calibration.hpp"
#include "servo/servo_frame_scheduler.hpp"
#include "storage/config_storage.hpp"
#include "storage/crc32.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
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

void advance_gait(robot::GaitController &gait, robot::MotionController &motion, uint64_t &now_us) {
    now_us += 20000;
    gait.update(now_us, motion, servo::DEFAULT_SERVOS, true);
    motion.update(20000);
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
    const auto lifted_a = robot::lifted_pose(robot::gait::TRIPOD_A);
    const auto lifted_b = robot::lifted_pose(robot::gait::TRIPOD_B);
    assert(robot::pose_within_servo_limits(lifted_a, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(lifted_b, servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_cycle_start_pose(), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_lift_pose(robot::gait::TRIPOD_A), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_lift_pose(robot::gait::TRIPOD_B), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_transfer_pose(robot::gait::TRIPOD_A), servo::DEFAULT_SERVOS));
    assert(robot::pose_within_servo_limits(robot::walk_transfer_pose(robot::gait::TRIPOD_B), servo::DEFAULT_SERVOS));

    for (const auto leg : robot::gait::TRIPOD_A) {
        const auto &pose = lifted_a.legs[servo::leg_index(leg)];
        assert(pose.coxa_deg == robot::STAND_LEG_POSE.coxa_deg);
        assert(pose.femur_deg == robot::STAND_LEG_POSE.femur_deg + robot::gait::GAIT_LIFT_FEMUR_DELTA_DEG);
        assert(pose.tibia_deg == robot::STAND_LEG_POSE.tibia_deg + robot::gait::GAIT_LIFT_TIBIA_DELTA_DEG);
    }
    for (const auto leg : robot::gait::TRIPOD_B) {
        const auto &pose = lifted_b.legs[servo::leg_index(leg)];
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
        assert(std::fabs(robot::gait::forwardCoxa(config.leg)) == robot::gait::GAIT_COXA_SWING_DEG);
        assert(std::fabs(robot::gait::backwardCoxa(config.leg)) == robot::gait::GAIT_COXA_SWING_DEG);
    }
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
    test_angle_clamping();
    test_invalid_pulses();
    test_crc32();
    test_config_validation();
    test_uart_parser_valid();
    test_uart_parser_invalid();
    test_phase_schedule();
    test_no_events_outside_frame();
    test_tripod_groups();
    test_gait_pose_limits();
    test_march_returns_to_stand_after_two_cycles();
    test_walk_cycle_returns_to_start_phase();
    test_walk_stop_returns_to_stand();
    test_walk_demo_returns_to_stand_after_three_cycles();
    std::cout << "All host logic tests passed\n";
    return 0;
}
