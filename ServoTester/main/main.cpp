#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
}

namespace {

constexpr char TAG[] = "hexapod_leg";

constexpr float SERVO_MIN_ANGLE_DEG = -45.0f;
constexpr float SERVO_MAX_ANGLE_DEG = 45.0f;
constexpr float DEFAULT_SPEED_DEG_PER_SEC = 45.0f;
constexpr float MIN_SPEED_DEG_PER_SEC = 5.0f;
constexpr float MAX_SPEED_DEG_PER_SEC = 180.0f;
constexpr float MOVEMENT_DONE_EPSILON_DEG = 0.05f;

constexpr uint32_t SERVO_FREQUENCY_HZ = 50;
constexpr uint32_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQUENCY_HZ;
constexpr uint32_t MOVEMENT_PERIOD_MS = 20;
constexpr float MOVEMENT_PERIOD_SEC = 0.020f;

constexpr ledc_mode_t SERVO_LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t SERVO_LEDC_RESOLUTION = LEDC_TIMER_16_BIT;
constexpr uint32_t SERVO_LEDC_MAX_DUTY = (1UL << 16) - 1UL;

constexpr size_t MAX_COMMAND_CHARS = 128;

struct ServoCalibration {
    const char *name;
    gpio_num_t gpio;
    ledc_channel_t channel;

    int32_t pulse_minus_45_us;
    int32_t pulse_zero_us;
    int32_t pulse_plus_45_us;

    float current_angle_deg;
    float target_angle_deg;
};

enum ServoIndex : size_t {
    COXA = 0,
    FEMUR = 1,
    TIBIA = 2,
};

std::array<ServoCalibration, 3> s_servos = {{
    {"Coxa", GPIO_NUM_27, LEDC_CHANNEL_0, 1995, 1491, 961, 0.0f, 0.0f},
    {"Femur", GPIO_NUM_26, LEDC_CHANNEL_1, 1976, 1462, 934, 0.0f, 0.0f},
    {"Tibia", GPIO_NUM_25, LEDC_CHANNEL_2, 2017, 1498, 969, 0.0f, 0.0f},
}};

SemaphoreHandle_t s_state_mutex = nullptr;
float s_speed_deg_per_sec = DEFAULT_SPEED_DEG_PER_SEC;
bool s_pwm_enabled = true;

enum class SequenceType : uint8_t {
    None,
    TestCoxa,
    TestFemur,
    TestTibia,
    Demo,
};

struct SequenceState {
    SequenceType type = SequenceType::None;
    size_t step = 0;
    bool target_issued = false;
    int64_t hold_until_us = 0;
    std::array<float, 3> base_angles = {0.0f, 0.0f, 0.0f};
};

SequenceState s_sequence;

float clamp_float(float value, float low, float high)
{
    return std::min(std::max(value, low), high);
}

int32_t min_pulse_us(const ServoCalibration &servo)
{
    return std::min({servo.pulse_minus_45_us, servo.pulse_zero_us, servo.pulse_plus_45_us});
}

int32_t max_pulse_us(const ServoCalibration &servo)
{
    return std::max({servo.pulse_minus_45_us, servo.pulse_zero_us, servo.pulse_plus_45_us});
}

int32_t clamp_pulse_us(const ServoCalibration &servo, int32_t pulse_us)
{
    return std::min(std::max(pulse_us, min_pulse_us(servo)), max_pulse_us(servo));
}

int32_t rounded_lerp(float start, float end, float t)
{
    return static_cast<int32_t>(std::lround(start + ((end - start) * t)));
}

int32_t angle_to_pulse_us(const ServoCalibration &servo, float angle_deg)
{
    const float clamped_angle = clamp_float(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    int32_t pulse_us = servo.pulse_zero_us;
    if (clamped_angle <= 0.0f) {
        const float t = (clamped_angle - SERVO_MIN_ANGLE_DEG) /
                        (0.0f - SERVO_MIN_ANGLE_DEG);
        pulse_us = rounded_lerp(static_cast<float>(servo.pulse_minus_45_us),
                                static_cast<float>(servo.pulse_zero_us),
                                t);
    } else {
        const float t = clamped_angle / SERVO_MAX_ANGLE_DEG;
        pulse_us = rounded_lerp(static_cast<float>(servo.pulse_zero_us),
                                static_cast<float>(servo.pulse_plus_45_us),
                                t);
    }

    return clamp_pulse_us(servo, pulse_us);
}

uint32_t pulse_us_to_duty(int32_t pulse_us)
{
    const uint64_t numerator =
        static_cast<uint64_t>(pulse_us) * SERVO_LEDC_MAX_DUTY;
    return static_cast<uint32_t>(
        (numerator + (SERVO_PERIOD_US / 2U)) / SERVO_PERIOD_US);
}

esp_err_t servo_pwm_init()
{
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = SERVO_LEDC_MODE;
    timer_config.duty_resolution = SERVO_LEDC_RESOLUTION;
    timer_config.timer_num = SERVO_LEDC_TIMER;
    timer_config.freq_hz = SERVO_FREQUENCY_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(
        ledc_timer_config(&timer_config), TAG, "LEDC timer config failed");

    for (const ServoCalibration &servo : s_servos) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = servo.gpio;
        channel_config.speed_mode = SERVO_LEDC_MODE;
        channel_config.channel = servo.channel;
        channel_config.timer_sel = SERVO_LEDC_TIMER;
        channel_config.duty = pulse_us_to_duty(servo.pulse_zero_us);
        channel_config.hpoint = 0;
        ESP_RETURN_ON_ERROR(
            ledc_channel_config(&channel_config), TAG, "LEDC channel config failed");
    }

    ESP_LOGI(TAG, "LEDC initialized: %u Hz, GPIO%d/GPIO%d/GPIO%d",
             SERVO_FREQUENCY_HZ,
             static_cast<int>(s_servos[COXA].gpio),
             static_cast<int>(s_servos[FEMUR].gpio),
             static_cast<int>(s_servos[TIBIA].gpio));
    return ESP_OK;
}

esp_err_t servo_write_pulse_us(const ServoCalibration &servo, int32_t pulse_us)
{
    const int32_t clamped_pulse_us = clamp_pulse_us(servo, pulse_us);
    const uint32_t duty = pulse_us_to_duty(clamped_pulse_us);

    ESP_RETURN_ON_ERROR(
        ledc_set_duty(SERVO_LEDC_MODE, servo.channel, duty),
        TAG,
        "LEDC set duty failed");
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(SERVO_LEDC_MODE, servo.channel),
        TAG,
        "LEDC update duty failed");
    return ESP_OK;
}

void disable_pwm_outputs()
{
    for (const ServoCalibration &servo : s_servos) {
        const esp_err_t result = ledc_stop(SERVO_LEDC_MODE, servo.channel, 0);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to disable %s PWM: %s",
                     servo.name,
                     esp_err_to_name(result));
        }
    }
}

void enable_pwm_outputs_at_current_angles()
{
    std::array<ServoCalibration, 3> snapshot{};

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    snapshot = s_servos;
    s_pwm_enabled = true;
    xSemaphoreGive(s_state_mutex);

    for (const ServoCalibration &servo : snapshot) {
        const esp_err_t result = servo_write_pulse_us(
            servo, angle_to_pulse_us(servo, servo.current_angle_deg));
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable %s PWM: %s",
                     servo.name,
                     esp_err_to_name(result));
        }
    }
}

void cancel_sequence_locked()
{
    s_sequence = SequenceState{};
}

void cancel_sequence()
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    cancel_sequence_locked();
    xSemaphoreGive(s_state_mutex);
}

void set_servo_target(size_t index, float angle_deg, bool cancel_active_sequence = true)
{
    const float clamped = clamp_float(
        angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (cancel_active_sequence) {
        cancel_sequence_locked();
    }
    s_servos[index].target_angle_deg = clamped;
    xSemaphoreGive(s_state_mutex);
}

void set_leg_target(float coxa_deg,
                    float femur_deg,
                    float tibia_deg,
                    bool cancel_active_sequence = true)
{
    const float coxa = clamp_float(
        coxa_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    const float femur = clamp_float(
        femur_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    const float tibia = clamp_float(
        tibia_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (cancel_active_sequence) {
        cancel_sequence_locked();
    }
    s_servos[COXA].target_angle_deg = coxa;
    s_servos[FEMUR].target_angle_deg = femur;
    s_servos[TIBIA].target_angle_deg = tibia;
    xSemaphoreGive(s_state_mutex);
}

bool motion_complete_locked()
{
    for (const ServoCalibration &servo : s_servos) {
        if (std::fabs(servo.target_angle_deg - servo.current_angle_deg) >
            MOVEMENT_DONE_EPSILON_DEG) {
            return false;
        }
    }
    return true;
}

const char *sequence_name(SequenceType type)
{
    switch (type) {
        case SequenceType::TestCoxa:
            return "Coxa test";
        case SequenceType::TestFemur:
            return "Femur test";
        case SequenceType::TestTibia:
            return "Tibia test";
        case SequenceType::Demo:
            return "Demo";
        case SequenceType::None:
        default:
            return "None";
    }
}

size_t sequence_step_count(SequenceType type)
{
    switch (type) {
        case SequenceType::TestCoxa:
        case SequenceType::TestFemur:
        case SequenceType::TestTibia:
        case SequenceType::Demo:
            return 4;
        case SequenceType::None:
        default:
            return 0;
    }
}

uint32_t sequence_hold_ms(SequenceType type)
{
    return type == SequenceType::Demo ? 300U : 250U;
}

void issue_sequence_target_locked()
{
    constexpr std::array<float, 4> test_angles = {
        0.0f, -20.0f, 20.0f, 0.0f};

    struct Pose {
        float coxa;
        float femur;
        float tibia;
    };

    constexpr std::array<Pose, 4> demo_poses = {{
        {0.0f, 0.0f, 0.0f},
        {-15.0f, -15.0f, 20.0f},
        {15.0f, -15.0f, 20.0f},
        {0.0f, 0.0f, 0.0f},
    }};

    if (s_sequence.type == SequenceType::Demo) {
        const Pose &pose = demo_poses[s_sequence.step];
        s_servos[COXA].target_angle_deg = pose.coxa;
        s_servos[FEMUR].target_angle_deg = pose.femur;
        s_servos[TIBIA].target_angle_deg = pose.tibia;
    } else {
        s_servos[COXA].target_angle_deg = s_sequence.base_angles[COXA];
        s_servos[FEMUR].target_angle_deg = s_sequence.base_angles[FEMUR];
        s_servos[TIBIA].target_angle_deg = s_sequence.base_angles[TIBIA];

        size_t servo_index = COXA;
        if (s_sequence.type == SequenceType::TestFemur) {
            servo_index = FEMUR;
        } else if (s_sequence.type == SequenceType::TestTibia) {
            servo_index = TIBIA;
        }
        s_servos[servo_index].target_angle_deg = test_angles[s_sequence.step];
    }

    s_sequence.target_issued = true;
    s_sequence.hold_until_us = 0;
}

bool update_sequence_locked(int64_t now_us)
{
    if (s_sequence.type == SequenceType::None) {
        return false;
    }

    if (!s_sequence.target_issued) {
        issue_sequence_target_locked();
        return false;
    }

    if (!motion_complete_locked()) {
        return false;
    }

    if (s_sequence.hold_until_us == 0) {
        s_sequence.hold_until_us =
            now_us + static_cast<int64_t>(sequence_hold_ms(s_sequence.type)) * 1000LL;
        return false;
    }

    if (now_us < s_sequence.hold_until_us) {
        return false;
    }

    ++s_sequence.step;
    if (s_sequence.step >= sequence_step_count(s_sequence.type)) {
        cancel_sequence_locked();
        return true;
    }

    s_sequence.target_issued = false;
    s_sequence.hold_until_us = 0;
    return false;
}

void movement_task(void *)
{
    const TickType_t delay_ticks = std::max<TickType_t>(
        1, pdMS_TO_TICKS(MOVEMENT_PERIOD_MS));

    while (true) {
        std::array<ServoCalibration, 3> snapshot{};
        bool enabled = false;
        bool sequence_completed = false;

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);

        enabled = s_pwm_enabled;
        if (enabled) {
            const float max_step =
                s_speed_deg_per_sec * MOVEMENT_PERIOD_SEC;

            for (ServoCalibration &servo : s_servos) {
                const float delta =
                    servo.target_angle_deg - servo.current_angle_deg;
                const float abs_delta = std::fabs(delta);

                if (abs_delta <= max_step) {
                    servo.current_angle_deg = servo.target_angle_deg;
                } else {
                    servo.current_angle_deg +=
                        delta > 0.0f ? max_step : -max_step;
                }
            }

            sequence_completed = update_sequence_locked(esp_timer_get_time());
            snapshot = s_servos;
        }

        xSemaphoreGive(s_state_mutex);

        if (enabled) {
            for (const ServoCalibration &servo : snapshot) {
                const int32_t pulse_us =
                    angle_to_pulse_us(servo, servo.current_angle_deg);
                const esp_err_t result =
                    servo_write_pulse_us(servo, pulse_us);

                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to update %s PWM: %s",
                             servo.name,
                             esp_err_to_name(result));
                }
            }
        }

        if (sequence_completed) {
            printf("Sequence complete\r\n> ");
            std::fflush(stdout);
        }

        // Always block the task. This guarantees time for IDLE0/IDLE1 and
        // prevents the Task Watchdog from firing.
        vTaskDelay(delay_ticks);
    }
}

void start_sequence(SequenceType type)
{
    bool already_running = false;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    already_running = s_sequence.type != SequenceType::None;

    if (!already_running) {
        s_sequence = SequenceState{};
        s_sequence.type = type;
        for (size_t i = 0; i < s_servos.size(); ++i) {
            s_sequence.base_angles[i] = s_servos[i].current_angle_deg;
        }
    }

    xSemaphoreGive(s_state_mutex);

    if (already_running) {
        printf("A test or demo is already running. Use stop to cancel it.\r\n");
        return;
    }

    printf("Starting %s sequence\r\n", sequence_name(type));
}

void print_help()
{
    printf("\r\nCommands:\r\n");
    printf("  leg <coxa> <femur> <tibia>   Set all target angles, degrees\r\n");
    printf("  coxa <angle>                  Set Coxa target angle\r\n");
    printf("  femur <angle>                 Set Femur target angle\r\n");
    printf("  tibia <angle>                 Set Tibia target angle\r\n");
    printf("  speed <degrees_per_second>    Set speed, 5..180 deg/s\r\n");
    printf("  center                        Same as leg 0 0 0\r\n");
    printf("  status                        Print angles, pulses and calibration\r\n");
    printf("  stop                          Hold current position and cancel sequence\r\n");
    printf("  test coxa|femur|tibia         Run a joint test\r\n");
    printf("  demo                          Run a safe leg sequence\r\n");
    printf("  disable                       Stop PWM outputs\r\n");
    printf("  enable                        Resume PWM at saved angles\r\n");
    printf("  help                          Show this help\r\n\r\n");
}

void print_status()
{
    std::array<ServoCalibration, 3> snapshot{};
    float speed = DEFAULT_SPEED_DEG_PER_SEC;
    bool enabled = false;
    SequenceType sequence_type = SequenceType::None;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    snapshot = s_servos;
    speed = s_speed_deg_per_sec;
    enabled = s_pwm_enabled;
    sequence_type = s_sequence.type;
    xSemaphoreGive(s_state_mutex);

    printf("\r\nStatus: PWM %s, speed %.1f deg/s, sequence %s\r\n",
           enabled ? "enabled" : "disabled",
           static_cast<double>(speed),
           sequence_name(sequence_type));
    printf("Joint   GPIO  Current   Target    Pulse(us)  Calibration(-45/0/+45 us)\r\n");

    for (const ServoCalibration &servo : snapshot) {
        printf("%-6s  %4d  %7.2f  %7.2f  %9ld  %ld / %ld / %ld\r\n",
               servo.name,
               static_cast<int>(servo.gpio),
               static_cast<double>(servo.current_angle_deg),
               static_cast<double>(servo.target_angle_deg),
               static_cast<long>(
                   angle_to_pulse_us(servo, servo.current_angle_deg)),
               static_cast<long>(servo.pulse_minus_45_us),
               static_cast<long>(servo.pulse_zero_us),
               static_cast<long>(servo.pulse_plus_45_us));
    }
    printf("\r\n");
}

void to_lower_ascii(char *text)
{
    for (; *text != '\0'; ++text) {
        if (*text >= 'A' && *text <= 'Z') {
            *text = static_cast<char>(*text - 'A' + 'a');
        }
    }
}

bool parse_float_arg(const char *text, float *value)
{
    if (text == nullptr || *text == '\0') {
        return false;
    }

    errno = 0;
    char *end = nullptr;
    const float parsed = std::strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

bool tokenize(char *line, std::array<char *, 5> &argv, size_t *argc)
{
    *argc = 0;
    char *save = nullptr;
    char *token = strtok_r(line, " \t", &save);

    while (token != nullptr) {
        if (*argc >= argv.size()) {
            return false;
        }
        argv[*argc] = token;
        ++(*argc);
        token = strtok_r(nullptr, " \t", &save);
    }

    return true;
}

void report_angle_clamp(const char *name, float requested)
{
    const float clamped = clamp_float(
        requested, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    if (std::fabs(clamped - requested) > 0.0001f) {
        printf("%s angle clamped from %.2f to %.2f deg\r\n",
               name,
               static_cast<double>(requested),
               static_cast<double>(clamped));
    }
}

void handle_single_servo_command(size_t servo_index,
                                 size_t argc,
                                 char *argv[])
{
    if (argc != 2) {
        printf("Usage: %s <angle>, example: %s 15\r\n", argv[0], argv[0]);
        return;
    }

    float angle = 0.0f;
    if (!parse_float_arg(argv[1], &angle)) {
        printf("Error: angle must be a finite number. Example: %s 15\r\n",
               argv[0]);
        return;
    }

    report_angle_clamp(s_servos[servo_index].name, angle);
    set_servo_target(servo_index, angle);
    printf("%s target set to %.2f deg\r\n",
           s_servos[servo_index].name,
           static_cast<double>(clamp_float(
               angle, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG)));
}

bool joint_from_name(const char *name, SequenceType *type)
{
    if (std::strcmp(name, "coxa") == 0) {
        *type = SequenceType::TestCoxa;
        return true;
    }
    if (std::strcmp(name, "femur") == 0) {
        *type = SequenceType::TestFemur;
        return true;
    }
    if (std::strcmp(name, "tibia") == 0) {
        *type = SequenceType::TestTibia;
        return true;
    }
    return false;
}

void execute_command(char *line)
{
    std::array<char *, 5> argv{};
    size_t argc = 0;

    if (!tokenize(line, argv, &argc)) {
        printf("Error: too many arguments. Try: help\r\n");
        return;
    }
    if (argc == 0) {
        return;
    }

    to_lower_ascii(argv[0]);

    if (std::strcmp(argv[0], "help") == 0) {
        print_help();
        return;
    }

    if (std::strcmp(argv[0], "status") == 0) {
        print_status();
        return;
    }

    if (std::strcmp(argv[0], "leg") == 0) {
        if (argc != 4) {
            printf("Usage: leg <coxa> <femur> <tibia>, example: leg 10 -20 30\r\n");
            return;
        }

        float coxa = 0.0f;
        float femur = 0.0f;
        float tibia = 0.0f;
        if (!parse_float_arg(argv[1], &coxa) ||
            !parse_float_arg(argv[2], &femur) ||
            !parse_float_arg(argv[3], &tibia)) {
            printf("Error: angles must be finite numbers. Example: leg 10 -20 30\r\n");
            return;
        }

        report_angle_clamp("Coxa", coxa);
        report_angle_clamp("Femur", femur);
        report_angle_clamp("Tibia", tibia);
        set_leg_target(coxa, femur, tibia);
        printf("Leg target set: Coxa %.2f, Femur %.2f, Tibia %.2f deg\r\n",
               static_cast<double>(clamp_float(
                   coxa, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG)),
               static_cast<double>(clamp_float(
                   femur, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG)),
               static_cast<double>(clamp_float(
                   tibia, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG)));
        return;
    }

    if (std::strcmp(argv[0], "coxa") == 0) {
        handle_single_servo_command(COXA, argc, argv.data());
        return;
    }
    if (std::strcmp(argv[0], "femur") == 0) {
        handle_single_servo_command(FEMUR, argc, argv.data());
        return;
    }
    if (std::strcmp(argv[0], "tibia") == 0) {
        handle_single_servo_command(TIBIA, argc, argv.data());
        return;
    }

    if (std::strcmp(argv[0], "speed") == 0) {
        if (argc != 2) {
            printf("Usage: speed <degrees_per_second>, example: speed 30\r\n");
            return;
        }

        float speed = 0.0f;
        if (!parse_float_arg(argv[1], &speed)) {
            printf("Error: speed must be a finite number. Example: speed 30\r\n");
            return;
        }

        const float clamped = clamp_float(
            speed, MIN_SPEED_DEG_PER_SEC, MAX_SPEED_DEG_PER_SEC);

        if (std::fabs(clamped - speed) > 0.0001f) {
            printf("Speed clamped from %.2f to %.2f deg/s\r\n",
                   static_cast<double>(speed),
                   static_cast<double>(clamped));
        }

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_speed_deg_per_sec = clamped;
        xSemaphoreGive(s_state_mutex);
        printf("Speed set to %.2f deg/s\r\n", static_cast<double>(clamped));
        return;
    }

    if (std::strcmp(argv[0], "center") == 0) {
        set_leg_target(0.0f, 0.0f, 0.0f);
        printf("Leg target set to center\r\n");
        return;
    }

    if (std::strcmp(argv[0], "stop") == 0) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        cancel_sequence_locked();
        for (ServoCalibration &servo : s_servos) {
            servo.target_angle_deg = servo.current_angle_deg;
        }
        xSemaphoreGive(s_state_mutex);
        printf("Motion stopped at current position; PWM still holds enabled servos\r\n");
        return;
    }

    if (std::strcmp(argv[0], "test") == 0) {
        if (argc != 2) {
            printf("Usage: test coxa|femur|tibia, example: test coxa\r\n");
            return;
        }
        to_lower_ascii(argv[1]);
        SequenceType type = SequenceType::None;
        if (!joint_from_name(argv[1], &type)) {
            printf("Usage: test coxa|femur|tibia, example: test coxa\r\n");
            return;
        }
        start_sequence(type);
        return;
    }

    if (std::strcmp(argv[0], "demo") == 0) {
        start_sequence(SequenceType::Demo);
        return;
    }

    if (std::strcmp(argv[0], "disable") == 0) {
        cancel_sequence();
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_pwm_enabled = false;
        xSemaphoreGive(s_state_mutex);
        disable_pwm_outputs();
        printf("PWM disabled; saved angles are preserved\r\n");
        return;
    }

    if (std::strcmp(argv[0], "enable") == 0) {
        enable_pwm_outputs_at_current_angles();
        printf("PWM enabled at saved current angles\r\n");
        return;
    }

    printf("Unknown command. Try: help\r\n");
}

void flush_extra_input()
{
    int ch = 0;
    while ((ch = std::getchar()) != '\n' && ch != '\r' && ch != EOF) {
    }
}

void console_task(void *)
{
    char line[MAX_COMMAND_CHARS + 2] = {};

    print_help();
    printf("> ");
    std::fflush(stdout);

    while (true) {
        if (std::fgets(line, sizeof(line), stdin) == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const size_t length = std::strlen(line);
        const bool has_newline =
            length > 0U &&
            (line[length - 1U] == '\n' || line[length - 1U] == '\r');

        if (!has_newline && length > MAX_COMMAND_CHARS) {
            flush_extra_input();
            printf("Error: command is longer than %u characters.\r\n",
                   static_cast<unsigned>(MAX_COMMAND_CHARS));
            printf("> ");
            std::fflush(stdout);
            continue;
        }

        size_t current_length = std::strlen(line);
        while (current_length > 0U) {
            const char last = line[current_length - 1U];
            if (last != '\n' && last != '\r') {
                break;
            }
            line[current_length - 1U] = '\0';
            --current_length;
        }

        execute_command(line);
        printf("> ");
        std::fflush(stdout);
    }
}

}  // namespace

extern "C" void app_main()
{
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    const esp_err_t pwm_result = servo_pwm_init();
    if (pwm_result != ESP_OK) {
        ESP_LOGE(TAG, "PWM init failed; tasks will not start");
        return;
    }

    for (const ServoCalibration &servo : s_servos) {
        const esp_err_t result = servo_write_pulse_us(
            servo, angle_to_pulse_us(servo, 0.0f));
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to center %s: %s",
                     servo.name,
                     esp_err_to_name(result));
            return;
        }
    }

    TaskHandle_t movement_handle = nullptr;
    TaskHandle_t console_handle = nullptr;

    const BaseType_t movement_created = xTaskCreate(
        movement_task,
        "movement_task",
        4096,
        nullptr,
        3,
        &movement_handle);

    if (movement_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create movement_task");
        disable_pwm_outputs();
        return;
    }

    const BaseType_t console_created = xTaskCreate(
        console_task,
        "console_task",
        4096,
        nullptr,
        2,
        &console_handle);

    if (console_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create console_task");
        vTaskDelete(movement_handle);
        disable_pwm_outputs();
        return;
    }

    ESP_LOGI(TAG, "Hexapod leg controller started");
}
