#include "app/startup_controller.hpp"
#include "board/pins.hpp"
#include "control/drive_controller.hpp"
#include "input/rc_pwm_input.hpp"
#include "protocol/uart_protocol.hpp"
#include "robot/gait_controller.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_model.hpp"
#include "robot_params.hpp"
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
input::RcPwmInput g_rc_input;
control::DriveController g_drive_controller;
servo::PioServoDriver g_driver;
storage::ConfigStorage g_storage;
protocol::UartLineParser g_line_parser;
uint32_t g_uart_errors = 0;
bool g_outputs_allowed = true;
uint64_t g_last_dashboard_us = 0;
bool g_dashboard_initialized = false;
bool g_last_signal_valid = false;
bool g_last_armed = false;
const char *g_last_rc_event = "NONE";
uint64_t g_last_rc_event_us = 0;
bool g_has_last_rc_event = false;

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
    uart_write_line("OK COMMANDS PING STATUS ALL CENTER ZERO STAND ENABLE ALL DISABLE ALL MARCH WALK DEMO WALK STOP GAIT STATUS SERVO LEG CAL SHOW CAL SET CAL SAVE CAL LOAD CAL RESET HELP");
}

void record_rc_event(const char *text, uint64_t now_us) {
    g_last_rc_event = text;
    g_last_rc_event_us = now_us;
    g_has_last_rc_event = true;
}

void print_serial_dashboard(uint64_t now_us, const control::DriveControllerState &drive) {
    if constexpr (!config::SERIAL_DASHBOARD) {
        return;
    }

    constexpr uint64_t DEBUG_INTERVAL_US = 1000000u / config::SERIAL_DASHBOARD_RATE_HZ;
    if (now_us - g_last_dashboard_us < DEBUG_INTERVAL_US) {
        return;
    }
    g_last_dashboard_us = now_us;

    if constexpr (config::SERIAL_DASHBOARD_ANSI) {
        if (!g_dashboard_initialized) {
            printf("\x1b[2J\x1b[H");
            g_dashboard_initialized = true;
        } else {
            printf("\x1b[H");
        }
        printf("RP2040 HEXAPOD RC CONTROL\x1b[K\r\n");
        printf("----------------------------------------\x1b[K\r\n");
        printf("RC FWD   : %4u us   normalized: %+0.2f\x1b[K\r\n",
               drive.raw_forward_us, static_cast<double>(drive.y));
        printf("RC STEER : %4u us   normalized: %+0.2f\x1b[K\r\n",
               drive.raw_steer_us, static_cast<double>(drive.x));
        printf("AGE FWD  : %4lu ms\x1b[K\r\n", static_cast<unsigned long>(drive.forward_age_ms));
        printf("AGE STR  : %4lu ms\x1b[K\r\n", static_cast<unsigned long>(drive.steer_age_ms));
        printf("\x1b[K\r\n");
        printf("SIGNAL      : %s\x1b[K\r\n", drive.signal_valid ? "VALID" : "INVALID");
        printf("NEUTRAL FWD : %s\x1b[K\r\n", drive.forward_neutral ? "YES" : "NO");
        printf("NEUTRAL STR : %s\x1b[K\r\n", drive.steer_neutral ? "YES" : "NO");
        printf("ARM TIMER   : %4lu / %lu ms\x1b[K\r\n",
               static_cast<unsigned long>(drive.arm_elapsed_ms),
               static_cast<unsigned long>(config::RC_ARM_NEUTRAL_MS));
        printf("ARM         : %s\x1b[K\r\n", drive.armed ? "READY" : (drive.waiting_neutral ? "WAIT_NEUTRAL" : "NO_SIGNAL"));
        printf("ACTIVE      : %s\x1b[K\r\n", drive.command.active ? "YES" : "NO");
        printf("SPEED       : %0.2f\x1b[K\r\n", static_cast<double>(drive.command.speed));
        printf("ACTIVE THR  : %0.2f\x1b[K\r\n", static_cast<double>(config::RC_ACTIVE_THRESHOLD));
        printf("CMD MAG     : %0.2f\x1b[K\r\n", static_cast<double>(drive.command_magnitude));
        printf("STEP SWING  : %0.1f deg\x1b[K\r\n", static_cast<double>(g_gait.drive_step_swing_deg()));
        printf("STEP SPEED  : %0.2f\x1b[K\r\n", static_cast<double>(g_gait.drive_step_speed()));
        printf("MOTION      : %s\x1b[K\r\n", robot::motion_interpolation_name(g_motion.interpolation()));
        printf("PHASE       : %s\x1b[K\r\n", robot::gait_state_name(g_gait.state()));
        printf("PHASE %%     : %3u\x1b[K\r\n", static_cast<unsigned>(g_motion.progress_percent()));
        if (g_has_last_rc_event) {
            printf("LAST EVENT  : %s, %0.1f s ago\x1b[K\r\n",
                   g_last_rc_event,
                   static_cast<double>(control::rc_age_us(now_us, g_last_rc_event_us)) / 1000000.0);
        } else {
            printf("LAST EVENT  : NONE\x1b[K\r\n");
        }
        printf("\x1b[K\r\n");
        printf("DRIVE L  : %+0.2f\x1b[K\r\n", static_cast<double>(drive.mix.left));
        printf("DRIVE R  : %+0.2f\x1b[K\r\n", static_cast<double>(drive.mix.right));
        printf("\x1b[K\r\n");
        printf("GAIT     : %-16s\x1b[K\r\n", robot::gait_state_name(g_gait.state()));
        printf("----------------------------------------\x1b[K\r\n");
    } else {
        printf("\rFWD=%4u STR=%4u F=%+.2f T=%+.2f AGEF=%3lu AGES=%3lu NF=%d NS=%d TMR=%3lu/%lu SIG=%d ARM=%d ACT=%d THR=%.2f MAG=%.2f SPD=%.2f SWG=%.1f STEP=%.2f L=%+.2f R=%+.2f MOT=%s PH=%s PCT=%u EVT=%s GAIT=%-16s        ",
               drive.raw_forward_us,
               drive.raw_steer_us,
               static_cast<double>(drive.y),
               static_cast<double>(drive.x),
               static_cast<unsigned long>(drive.forward_age_ms),
               static_cast<unsigned long>(drive.steer_age_ms),
               drive.forward_neutral ? 1 : 0,
               drive.steer_neutral ? 1 : 0,
               static_cast<unsigned long>(drive.arm_elapsed_ms),
               static_cast<unsigned long>(config::RC_ARM_NEUTRAL_MS),
               drive.signal_valid ? 1 : 0,
               drive.armed ? 1 : 0,
               drive.command.active ? 1 : 0,
               static_cast<double>(config::RC_ACTIVE_THRESHOLD),
               static_cast<double>(drive.command_magnitude),
               static_cast<double>(drive.command.speed),
               static_cast<double>(g_gait.drive_step_swing_deg()),
               static_cast<double>(g_gait.drive_step_speed()),
               static_cast<double>(drive.mix.left),
               static_cast<double>(drive.mix.right),
               robot::motion_interpolation_name(g_motion.interpolation()),
               robot::gait_state_name(g_gait.state()),
               static_cast<unsigned>(g_motion.progress_percent()),
               g_has_last_rc_event ? g_last_rc_event : "NONE",
               robot::gait_state_name(g_gait.state()));
    }
}

void print_rc_events(uint64_t now_us, const control::DriveControllerState &drive) {
    if (drive.signal_valid != g_last_signal_valid) {
        record_rc_event(drive.signal_valid ? "RC SIGNAL RESTORED" : "RC SIGNAL LOST", now_us);
        g_last_signal_valid = drive.signal_valid;
    }
    if (drive.armed != g_last_armed) {
        if (drive.armed) {
            record_rc_event("RC ARMED", now_us);
        }
        g_last_armed = drive.armed;
    }
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
    case protocol::CommandType::Zero:
        g_gait.abort();
        g_motion.set_target(robot::ZERO_POSE, 30.0f);
        uart_write_line("OK ZERO");
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
    if constexpr (config::CONTROL_MODE == config::ControlInputMode::UART) {
        init_command_uart();
    } else {
        g_rc_input.init(board::RC_FORWARD_GPIO,
                        board::RC_STEER_GPIO,
                        config::RC_VALID_MIN_US,
                        config::RC_VALID_MAX_US);
    }

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
        if constexpr (config::CONTROL_MODE == config::ControlInputMode::UART) {
            poll_uart();
        }
        g_startup.update(t, g_motion);
        const bool startup_holding_stand = g_startup.state() == app::StartupState::HoldingStand;
        if constexpr (config::CONTROL_MODE == config::ControlInputMode::UART) {
            g_gait.update(t, g_motion, g_servos, startup_holding_stand);
        } else {
            const auto rc_snapshot = g_rc_input.read();
            const uint64_t rc_now_us = time_us_64();
            const auto drive_state = g_drive_controller.update(rc_snapshot, rc_now_us);
            print_rc_events(rc_now_us, drive_state);
            g_gait.update_drive(rc_now_us, g_motion, g_servos, drive_state.command, startup_holding_stand);
            print_serial_dashboard(rc_now_us, drive_state);
        }

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
