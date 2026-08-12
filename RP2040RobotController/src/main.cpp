#include "app/startup_controller.hpp"
#include "board/pins.hpp"
#include "protocol/uart_protocol.hpp"
#include "robot/gait_controller.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"
#include "servo/pio_servo_driver.hpp"
#include "servo/servo_calibration.hpp"
#include "storage/config_storage.hpp"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <array>
#include <cstdio>

namespace {

constexpr const char *FIRMWARE_VERSION = "RP2040RobotController 0.1.0";
constexpr uint32_t MOTION_UPDATE_US = 20000;

std::array<servo::ServoConfig, servo::SERVO_COUNT> g_servos = servo::DEFAULT_SERVOS;
robot::MotionController g_motion;
robot::GaitController g_gait;
app::StartupController g_startup;
servo::PioServoDriver g_driver;
storage::ConfigStorage g_storage;
protocol::UartLineParser g_line_parser;
uint32_t g_uart_errors = 0;
bool g_outputs_allowed = true;

void uart_write_line(const char *text) {
    uart_puts(uart0, text);
    uart_puts(uart0, "\r\n");
}

void init_command_uart() {
    uart_init(uart0, board::UART_BAUD);
    gpio_set_function(board::UART0_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(board::UART0_RX_GPIO, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
}

servo::ServoConfig &servo_config(servo::Leg leg, servo::Joint joint) {
    return g_servos[servo::servo_index(leg, joint)];
}

void build_pulses(std::array<servo::ServoPulse, servo::SERVO_COUNT> &pulses) {
    for (size_t i = 0; i < g_servos.size(); ++i) {
        const auto &config = g_servos[i];
        const float angle = robot::pose_angle(g_motion.current_pose(), config.leg, config.joint);
        const auto pulse = servo::angle_to_pulse_us(config, angle);
        if (pulse.result != servo::PulseResult::Ok) {
            printf("calibration error on %s %s, using safe center\r\n",
                   servo::leg_name(config.leg), servo::joint_name(config.joint));
        }
        pulses[i] = servo::ServoPulse{config.leg, config.gpio, pulse.pulse_us, config.enabled};
    }
}

void print_status() {
    char line[96];
    snprintf(line, sizeof(line), "STATE %s OUTPUTS %s FLASH %s VERSION %u UART_ERRORS %lu PIO_ERRORS %lu FRAME %lu",
             app::startup_state_name(g_startup.state()),
             g_driver.outputs_enabled() ? "ON" : "OFF",
             g_storage.last_load_valid() ? "VALID" : "BUILTIN",
             g_storage.version(),
             static_cast<unsigned long>(g_uart_errors),
             static_cast<unsigned long>(g_driver.error_count()),
             static_cast<unsigned long>(g_driver.frame_counter()));
    uart_write_line(line);
}

void print_gait_status() {
    char line[96];
    snprintf(line, sizeof(line), "OK GAIT MODE %s STATE %s CYCLE %u ACTIVE %u",
             robot::gait_mode_name(g_gait.mode()),
             robot::gait_state_name(g_gait.state()),
             static_cast<unsigned>(g_gait.cycle()),
             g_gait.active() ? 1u : 0u);
    uart_write_line(line);
}

void print_help() {
    uart_write_line("OK COMMANDS PING STATUS ALL CENTER STAND ENABLE ALL DISABLE ALL MARCH WALK DEMO WALK STOP GAIT STATUS SERVO LEG CAL SHOW CAL SET CAL SAVE CAL LOAD CAL RESET HELP");
}

void handle_command(const protocol::Command &command) {
    char line[128];
    switch (command.type) {
    case protocol::CommandType::Ping:
        uart_write_line("OK PONG");
        break;
    case protocol::CommandType::Status:
        print_status();
        break;
    case protocol::CommandType::AllCenter:
        g_gait.abort();
        g_startup.force_center(time_us_64(), g_motion);
        uart_write_line("OK ALL CENTER");
        break;
    case protocol::CommandType::Stand:
        g_gait.abort();
        g_startup.force_stand(time_us_64(), g_motion);
        uart_write_line("OK STAND");
        break;
    case protocol::CommandType::EnableAll:
        g_outputs_allowed = true;
        uart_write_line("OK ENABLE ALL");
        break;
    case protocol::CommandType::DisableAll:
        g_gait.abort();
        g_outputs_allowed = false;
        g_driver.stop();
        uart_write_line("OK DISABLE ALL");
        break;
    case protocol::CommandType::March:
        if (g_gait.start_march(time_us_64(), g_motion, g_servos)) {
            uart_write_line("OK MARCH");
        } else {
            uart_write_line(g_gait.diagnostic());
        }
        break;
    case protocol::CommandType::WalkDemo:
        if (g_gait.start_walk_demo(time_us_64(), g_motion, g_servos)) {
            uart_write_line("OK WALK DEMO");
        } else {
            uart_write_line(g_gait.diagnostic());
        }
        break;
    case protocol::CommandType::WalkStop:
        g_gait.stop(time_us_64(), g_motion, g_servos);
        uart_write_line("OK WALK STOP");
        break;
    case protocol::CommandType::GaitStatus:
        print_gait_status();
        break;
    case protocol::CommandType::Servo:
        g_motion.set_servo_target(command.leg, command.joint, command.angle_a);
        snprintf(line, sizeof(line), "OK SERVO %s %s %.2f",
                 servo::leg_name(command.leg), servo::joint_name(command.joint), command.angle_a);
        uart_write_line(line);
        break;
    case protocol::CommandType::Leg:
        g_motion.set_leg_target(command.leg, robot::LegPose{command.angle_a, command.angle_b, command.angle_c});
        snprintf(line, sizeof(line), "OK LEG %s %.2f %.2f %.2f",
                 servo::leg_name(command.leg), command.angle_a, command.angle_b, command.angle_c);
        uart_write_line(line);
        break;
    case protocol::CommandType::CalShow: {
        const auto &cfg = servo_config(command.leg, command.joint);
        snprintf(line, sizeof(line), "OK CAL %s %s MINUS45 %u CENTER %u PLUS45 %u",
                 servo::leg_name(command.leg), servo::joint_name(command.joint),
                 cfg.pulse_minus_45_us, cfg.center_us, cfg.pulse_plus_45_us);
        uart_write_line(line);
        break;
    }
    case protocol::CommandType::CalSetMinus45:
        servo_config(command.leg, command.joint).pulse_minus_45_us = command.pulse_us;
        uart_write_line("OK CAL SET MINUS45");
        break;
    case protocol::CommandType::CalSetPlus45:
        servo_config(command.leg, command.joint).pulse_plus_45_us = command.pulse_us;
        uart_write_line("OK CAL SET PLUS45");
        break;
    case protocol::CommandType::CalSave:
        g_driver.stop();
        uart_write_line(g_storage.save(g_servos) ? "OK CAL SAVE" : "ERR CONFIG_INVALID");
        g_driver.start();
        break;
    case protocol::CommandType::CalLoad:
        uart_write_line(g_storage.load(g_servos) ? "OK CAL LOAD" : "ERR CONFIG_INVALID");
        break;
    case protocol::CommandType::CalReset:
        g_servos = servo::DEFAULT_SERVOS;
        uart_write_line("OK CAL RESET");
        break;
    case protocol::CommandType::Help:
        print_help();
        break;
    case protocol::CommandType::None:
        break;
    }
}

void poll_uart() {
    char line[protocol::MAX_LINE_LENGTH + 1];
    while (uart_is_readable(uart0)) {
        if (g_line_parser.push_char(static_cast<char>(uart_getc(uart0)), line, sizeof(line))) {
            const auto parsed = protocol::parse_command(line);
            if (parsed.error != protocol::ParseError::None) {
                ++g_uart_errors;
                uart_write_line(protocol::error_text(parsed.error));
            } else {
                handle_command(parsed.command);
            }
        }
        if (g_line_parser.overflowed()) {
            ++g_uart_errors;
            g_line_parser.clear_overflow();
            uart_write_line(protocol::error_text(protocol::ParseError::LineTooLong));
        }
    }
}

} // namespace

int main() {
    stdio_init_all();
    init_command_uart();

    printf("%s\r\n", FIRMWARE_VERSION);
    const bool flash_ok = g_storage.load(g_servos);
    printf("config: %s\r\n", flash_ok ? "flash valid" : "using built-in temporary calibration");

    g_driver.init();
    printf("PIO/DMA initialized\r\n");

    const uint64_t now = time_us_64();
    g_motion.reset(robot::CENTER_POSE);
    g_startup.begin(now, g_motion);

    uint64_t last_motion_us = now;
    uint64_t last_frame_us = now;

    while (true) {
        const uint64_t t = time_us_64();
        poll_uart();
        g_startup.update(t, g_motion);
        g_gait.update(t, g_motion, g_servos, g_startup.state() == app::StartupState::HoldingStand);

        if (t - last_motion_us >= MOTION_UPDATE_US) {
            g_motion.update(static_cast<uint32_t>(t - last_motion_us));
            last_motion_us = t;
        }

        if (t - last_frame_us >= servo::FRAME_PERIOD_US) {
            std::array<servo::ServoPulse, servo::SERVO_COUNT> pulses{};
            build_pulses(pulses);
            g_driver.submit_pulses(pulses, g_startup.outputs_enabled() && g_outputs_allowed);
            last_frame_us = t;
        }

        if (g_startup.outputs_enabled() && g_outputs_allowed && !g_driver.outputs_enabled()) {
            g_driver.start();
            std::array<servo::ServoPulse, servo::SERVO_COUNT> pulses{};
            build_pulses(pulses);
            g_driver.submit_pulses(pulses, true);
        }
        g_driver.service();
        tight_loop_contents();
    }
}
