#include "protocol/uart_protocol.hpp"
#include "servo/servo_calibration.hpp"
#include "servo/servo_frame_scheduler.hpp"
#include "storage/config_storage.hpp"
#include "storage/crc32.hpp"

#include <array>
#include <cassert>
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

} // namespace

int main() {
    test_angle_to_pulse_endpoints();
    test_interpolation();
    test_increasing_and_decreasing_calibration();
    test_angle_clamping();
    test_invalid_pulses();
    test_crc32();
    test_config_validation();
    test_uart_parser_valid();
    test_uart_parser_invalid();
    test_phase_schedule();
    test_no_events_outside_frame();
    std::cout << "All host logic tests passed\n";
    return 0;
}
