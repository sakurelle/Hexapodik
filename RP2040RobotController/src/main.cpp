#include "app/startup_controller.hpp"
#include "board/pins.hpp"
#include "control/drive_controller.hpp"
#include "input/rc_pwm_input.hpp"
#include "protocol/uart_protocol.hpp"
#include "robot/cartesian_validation.hpp"
#include "robot/gait_generator.hpp"
#include "robot/motion_controller.hpp"
#include "robot/robot_geometry.hpp"
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

constexpr uint32_t CONTROL_PERIOD_US = servo::FRAME_PERIOD_US;

std::array<servo::ServoConfig, servo::SERVO_COUNT> g_servos = servo::DEFAULT_SERVOS;
robot::MotionController g_motion;
robot::GaitGenerator g_cartesian_gait(robot::ROBOT_GEOMETRY, g_servos);
app::StartupController g_startup;
input::RcPwmInput g_rc_input;
control::DriveController g_drive_controller;
servo::PioServoDriver g_driver;
storage::ConfigStorage g_storage;
protocol::UartLineParser g_line_parser;
bool g_outputs_allowed = true;
bool g_ik_error = false;
uint32_t g_uart_errors = 0;
uint64_t g_last_dashboard_us = 0;
bool g_dashboard_initialized = false;

void init_command_uart() {
    uart_init(uart0, board::UART_BAUD);
    gpio_set_function(board::UART0_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(board::UART0_RX_GPIO, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
}

void uart_write_line(const char *text) {
    uart_puts(uart0, text);
    uart_puts(uart0, "\r\n");
}

servo::ServoConfig &servo_config(servo::Leg leg, servo::Joint joint) {
    return g_servos[servo::servo_index(leg, joint)];
}

void build_pulses(std::array<servo::ServoPulse, servo::SERVO_COUNT> &pulses) {
    for (size_t index = 0; index < g_servos.size(); ++index) {
        const auto &config = g_servos[index];
        const auto pulse = servo::angle_to_pulse_us(
            config, robot::pose_angle(g_motion.current_pose(), config.leg, config.joint));
        pulses[index] = {config.leg, config.gpio, pulse.pulse_us, config.enabled};
    }
}

void print_dashboard(uint64_t now_us, const control::DriveControllerState &drive) {
    if constexpr (!config::SERIAL_DASHBOARD) return;
    constexpr uint64_t interval = 1000000u / config::SERIAL_DASHBOARD_RATE_HZ;
    if (control::rc_age_us(now_us, g_last_dashboard_us) < interval) return;
    g_last_dashboard_us = now_us;

    const auto velocity = robot::body_velocity_from_drive(drive.command);
    const auto &gait = g_cartesian_gait.status();
    const float effective_scale = gait.stride_scale * gait.workspace_scale;
    const bool limited = gait.ik_limited || effective_scale < 0.999f;
    const char *ik = g_ik_error ? "ERROR" : (limited ? "LIMITED" : "OK");
    const char *motion = g_motion.moving() ? "MOVING" : "DONE";
    if constexpr (config::SERIAL_DASHBOARD_ANSI) {
        printf(g_dashboard_initialized ? "\x1b[H" : "\x1b[2J\x1b[H");
        g_dashboard_initialized = true;
        printf("RP2040 HEXAPOD\x1b[K\r\nENGINE   : CARTESIAN\x1b[K\r\n");
        printf("STARTUP  : %s\x1b[K\r\nMOTION   : %s (%u%%)\x1b[K\r\n",
               app::startup_state_name(g_startup.state()), motion,
               static_cast<unsigned>(g_motion.progress_percent()));
        printf("RC       : %s  ARM: %s\x1b[K\r\n", drive.signal_valid ? "VALID" : "INVALID",
               drive.armed ? "READY" : "WAIT_NEUTRAL");
        printf("RC F/T   : %+0.2f / %+0.2f\x1b[K\r\n", static_cast<double>(drive.y),
               static_cast<double>(drive.x));
        printf("CMD F/T  : %+0.2f / %+0.2f\x1b[K\r\n", static_cast<double>(drive.command.forward),
               static_cast<double>(drive.command.turn));
        printf("VX/YAW   : %+0.1f mm/s / %+0.1f deg/s\x1b[K\r\n",
               static_cast<double>(velocity.vx_mm_s), static_cast<double>(velocity.yaw_rad_s * 57.29578f));
        printf("GAIT     : %s  PHASE: %.3f\x1b[K\r\n",
               robot::cartesian_gait_state_name(gait.state), static_cast<double>(gait.phase));
        printf("STRIDE   : %.1f mm  HEIGHT: %.1f mm\x1b[K\r\n",
               static_cast<double>(gait.actual_stride_mm),
               static_cast<double>(config::GAIT_STEP_HEIGHT_MM));
        printf("SCALE    : %.2f (RAMP %.2f, WORK %.2f)\x1b[K\r\n",
               static_cast<double>(effective_scale), static_cast<double>(gait.stride_scale),
               static_cast<double>(gait.workspace_scale));
        printf("IK       : %s\x1b[K\r\n", ik);
        if (limited && gait.limit.active()) {
            printf("LIMIT    : %s %s %s A=%+.1f L=%+.1f\x1b[K\r\n",
                   robot::cartesian_limit_reason_name(gait.limit.reason),
                   servo::leg_name(gait.limit.leg), servo::joint_name(gait.limit.joint),
                   static_cast<double>(gait.limit.angle_deg), static_cast<double>(gait.limit.limit_deg));
        }
        if constexpr (config::RC_DEBUG) {
            const auto &neutral = g_cartesian_gait.neutral_targets();
            const auto fl = robot::stance_velocity_for_body_command(
                velocity, neutral[servo::leg_index(servo::Leg::FL)]);
            const auto fr = robot::stance_velocity_for_body_command(
                velocity, neutral[servo::leg_index(servo::Leg::FR)]);
            const auto ml = robot::stance_velocity_for_body_command(
                velocity, neutral[servo::leg_index(servo::Leg::ML)]);
            const auto mr = robot::stance_velocity_for_body_command(
                velocity, neutral[servo::leg_index(servo::Leg::MR)]);
            printf("VXY FL %+.1f,%+.1f FR %+.1f,%+.1f ML %+.1f,%+.1f MR %+.1f,%+.1f\x1b[K\r\n",
                   static_cast<double>(fl.x_mm), static_cast<double>(fl.y_mm),
                   static_cast<double>(fr.x_mm), static_cast<double>(fr.y_mm),
                   static_cast<double>(ml.x_mm), static_cast<double>(ml.y_mm),
                   static_cast<double>(mr.x_mm), static_cast<double>(mr.y_mm));
        }
    } else {
        printf("\rENGINE=CARTESIAN STARTUP=%s MOTION=%u RC_F=%+.2f RC_T=%+.2f CMD_F=%+.2f CMD_T=%+.2f VX=%+.1f YAW=%+.1f GAIT=%s PH=%.3f STRIDE=%.1f HEIGHT=%.1f SCALE=%.2f IK=%s     ",
               app::startup_state_name(g_startup.state()), static_cast<unsigned>(g_motion.progress_percent()),
               static_cast<double>(drive.y), static_cast<double>(drive.x),
               static_cast<double>(drive.command.forward), static_cast<double>(drive.command.turn),
               static_cast<double>(velocity.vx_mm_s), static_cast<double>(velocity.yaw_rad_s * 57.29578f),
               robot::cartesian_gait_state_name(gait.state), static_cast<double>(gait.phase),
               static_cast<double>(gait.actual_stride_mm), static_cast<double>(config::GAIT_STEP_HEIGHT_MM),
               static_cast<double>(effective_scale), ik);
    }
}

void print_status() {
    char line[128];
    snprintf(line, sizeof(line), "LOCOMOTION CARTESIAN STARTUP %s PIO_ERRORS %lu FRAME %lu",
             app::startup_state_name(g_startup.state()), static_cast<unsigned long>(g_driver.error_count()),
             static_cast<unsigned long>(g_driver.frame_counter()));
    uart_write_line(line);
}

void handle_command(const protocol::Command &command) {
    switch (command.type) {
    case protocol::CommandType::Ping: uart_write_line("OK PONG"); break;
    case protocol::CommandType::Status: print_status(); break;
    case protocol::CommandType::AllCenter: g_startup.force_center(time_us_64(), g_motion); uart_write_line("OK ALL CENTER"); break;
    case protocol::CommandType::Zero: g_motion.set_target(robot::ZERO_POSE, 30.0f); uart_write_line("OK ZERO"); break;
    case protocol::CommandType::Stand: g_startup.force_stand(time_us_64(), g_motion); uart_write_line("OK STAND"); break;
    case protocol::CommandType::EnableAll: g_outputs_allowed = true; uart_write_line("OK ENABLE ALL"); break;
    case protocol::CommandType::DisableAll: g_outputs_allowed = false; g_driver.stop(); uart_write_line("OK DISABLE ALL"); break;
    case protocol::CommandType::Servo: g_motion.set_servo_target(command.leg, command.joint, command.angle_a); uart_write_line("OK SERVO"); break;
    case protocol::CommandType::Leg: g_motion.set_leg_target(command.leg, {command.angle_a, command.angle_b, command.angle_c}); uart_write_line("OK LEG"); break;
    case protocol::CommandType::CalShow: {
        char line[128];
        const auto &config = servo_config(command.leg, command.joint);
        snprintf(line, sizeof(line), "OK CAL %s %s MINUS45 %u CENTER %u PLUS45 %u",
                 servo::leg_name(command.leg), servo::joint_name(command.joint),
                 config.pulse_minus_45_us, config.center_us, config.pulse_plus_45_us);
        uart_write_line(line);
        break;
    }
    case protocol::CommandType::CalSetMinus45: servo_config(command.leg, command.joint).pulse_minus_45_us = command.pulse_us; uart_write_line("OK CAL SET MINUS45"); break;
    case protocol::CommandType::CalSetPlus45: servo_config(command.leg, command.joint).pulse_plus_45_us = command.pulse_us; uart_write_line("OK CAL SET PLUS45"); break;
    case protocol::CommandType::CalSave: uart_write_line(g_storage.save(g_servos) ? "OK CAL SAVE" : "ERR CONFIG_INVALID"); break;
    case protocol::CommandType::CalLoad: uart_write_line(g_storage.load(g_servos) ? "OK CAL LOAD" : "ERR CONFIG_INVALID"); break;
    case protocol::CommandType::CalReset: g_servos = servo::DEFAULT_SERVOS; uart_write_line("OK CAL RESET"); break;
    case protocol::CommandType::Help: uart_write_line("OK COMMANDS PING STATUS ALL CENTER ZERO STAND ENABLE ALL DISABLE ALL SERVO LEG CAL HELP"); break;
    case protocol::CommandType::None: break;
    }
}

void poll_uart() {
    char line[protocol::MAX_LINE_LENGTH + 1];
    while (uart_is_readable(uart0)) {
        if (g_line_parser.push_char(static_cast<char>(uart_getc(uart0)), line, sizeof(line))) {
            const auto parsed = protocol::parse_command(line);
            if (parsed.error == protocol::ParseError::None) handle_command(parsed.command);
            else { ++g_uart_errors; uart_write_line(protocol::error_text(parsed.error)); }
        }
    }
}

void submit_control_frame() {
    std::array<servo::ServoPulse, servo::SERVO_COUNT> pulses{};
    build_pulses(pulses);
    const bool enabled = g_startup.outputs_enabled() && g_outputs_allowed;
    if (enabled && !g_driver.outputs_enabled()) g_driver.start();
    g_driver.submit_pulses(pulses, enabled);
}

} // namespace

int main() {
    stdio_init_all();
    if constexpr (config::CONTROL_MODE == config::ControlInputMode::UART) init_command_uart();
    else g_rc_input.init(board::RC_FORWARD_GPIO, board::RC_STEER_GPIO,
                         config::RC_VALID_MIN_US, config::RC_VALID_MAX_US);
    printf("FW: RP2040RobotController\r\nBUILD: %s %s\r\nLOCOMOTION: CARTESIAN\r\n", __DATE__, __TIME__);
    g_storage.load(g_servos);
    g_driver.init();
    const uint64_t boot_us = time_us_64();
    g_motion.reset(robot::CENTER_POSE);
    g_startup.begin(boot_us, g_motion);
    uint64_t last_control_us = boot_us;

    while (true) {
        const uint64_t now_us = time_us_64();
        if (now_us - last_control_us >= CONTROL_PERIOD_US) {
            const uint32_t elapsed_us = static_cast<uint32_t>(now_us - last_control_us);
            last_control_us = now_us;
            g_motion.update(elapsed_us);
            g_startup.update(now_us, g_motion);
            if constexpr (config::CONTROL_MODE == config::ControlInputMode::UART) {
                poll_uart();
            } else {
                // Read first, then sample time: an IRQ can publish a newer timestamp at any moment.
                const auto rc_snapshot = g_rc_input.read();
                const uint64_t rc_now_us = time_us_64();
                const auto drive = g_drive_controller.update(rc_snapshot, rc_now_us);
                if (g_startup.state() == app::StartupState::HoldingStand) {
                    const auto targets = g_cartesian_gait.update(
                        static_cast<float>(elapsed_us) / 1000000.0f,
                        robot::body_velocity_from_drive(drive.command));
                    const auto validation = robot::validate_cartesian_targets(
                        robot::ROBOT_GEOMETRY, targets, g_servos);
                    if (validation.valid()) {
                        g_motion.set_immediate_pose(validation.pose);
                        g_ik_error = false;
                    } else {
                        g_ik_error = true; // retain the last valid pose
                    }
                }
                print_dashboard(rc_now_us, drive);
            }
            submit_control_frame();
        }
        g_driver.service();
        tight_loop_contents();
    }
}
