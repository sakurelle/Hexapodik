#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

extern "C" {
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

// ============================================================
// ОБЩАЯ КОНФИГУРАЦИЯ
// ============================================================

constexpr char TAG[] = "hexapod_dual";

// Режимы:
// 1) CALIBRATION — энкодер управляет отдельной сервой GPIO33.
// 2) LEG_CONTROL — стик управляет кончиком ноги через обратную кинематику.
enum class AppMode : uint8_t {
    Calibration,
    LegControl,
};

AppMode s_mode = AppMode::Calibration;

// ============================================================
// РАСПИНОВКА
// ============================================================

// 4-разрядный индикатор 3461AS-1, общий катод.
// Сегменты: A, B, C, D, E, F, G, DP.
constexpr std::array<gpio_num_t, 8> DISPLAY_SEGMENT_PINS = {
    GPIO_NUM_13,
    GPIO_NUM_14,
    GPIO_NUM_21,
    GPIO_NUM_22,
    GPIO_NUM_23,
    GPIO_NUM_32,
    GPIO_NUM_4,
    GPIO_NUM_5,
};

// Разряды слева направо.
constexpr std::array<gpio_num_t, 4> DISPLAY_DIGIT_PINS = {
    GPIO_NUM_16,
    GPIO_NUM_17,
    GPIO_NUM_18,
    GPIO_NUM_19,
};

// Сервы ноги.
constexpr gpio_num_t COXA_SERVO_PIN = GPIO_NUM_27;
constexpr gpio_num_t FEMUR_SERVO_PIN = GPIO_NUM_26;
constexpr gpio_num_t TIBIA_SERVO_PIN = GPIO_NUM_25;

// Отдельная калибруемая серва.
constexpr gpio_num_t CALIBRATION_SERVO_PIN = GPIO_NUM_33;

// Энкодер.
constexpr gpio_num_t ENCODER_CLK_PIN = GPIO_NUM_34;
constexpr gpio_num_t ENCODER_DT_PIN = GPIO_NUM_35;

// Аналоговый стик.
constexpr adc_channel_t JOYSTICK_X_ADC_CHANNEL = ADC_CHANNEL_0;  // GPIO36
constexpr adc_channel_t JOYSTICK_Y_ADC_CHANNEL = ADC_CHANNEL_3;  // GPIO39

// SW энкодера и SW стика соединены параллельно.
// Внешняя подтяжка 10 кОм к 3,3 В, нажатие замыкает на GND.
constexpr gpio_num_t MODE_BUTTON_PIN = GPIO_NUM_15;

// ============================================================
// НАСТРОЙКИ, КОТОРЫЕ МОЖНО БЫСТРО ИЗМЕНИТЬ
// ============================================================

// Оси стика программно поменяны местами:
// VRX управляет координатой Y, VRY управляет координатой X.
// Если после этого направление конкретной оси окажется обратным,
// поменяйте 1.0f на -1.0f.
constexpr float JOYSTICK_X_SIGN = 1.0f;
constexpr float JOYSTICK_Y_SIGN = 1.0f;

// Если конкретная серва в собранной ноге вращает геометрию наоборот,
// поменяйте знак только у неё.
constexpr float COXA_SERVO_TO_GEOMETRY_SIGN = 1.0f;
constexpr float FEMUR_SERVO_TO_GEOMETRY_SIGN = 1.0f;
constexpr float TIBIA_SERVO_TO_GEOMETRY_SIGN = 1.0f;

// Мёртвая зона стика.
constexpr float JOYSTICK_DEAD_ZONE = 0.08f;

// Команды на сервы передаются без программного ограничения скорости.
// Реальная максимальная скорость определяется самими сервоприводами.

// Допустимый командный диапазон всех трёх серв.
constexpr float SERVO_MIN_ANGLE_DEG = -75.0f;
constexpr float SERVO_MAX_ANGLE_DEG = 75.0f;

// Небольшой запас относительно программного предела.
constexpr float IK_ANGLE_LIMIT_DEG = 74.8f;

// Калибровочные точки измерены при -45°, 0° и +45°.
// За пределами ±45° импульс линейно экстраполируется.
constexpr float CALIBRATION_ANGLE_DEG = 45.0f;

// ============================================================
// ГЕОМЕТРИЯ НОГИ
// ============================================================

// A-B.
constexpr float COXA_LENGTH_MM = 45.0f;

// B-C.
constexpr float FEMUR_LENGTH_MM = 80.0f;

// Прямое расстояние C-D.
constexpr float TIBIA_EFFECTIVE_LENGTH_MM = 116.0f;

// При всех сервоприводах в 0°:
// Femur B-C расположен под +45° к горизонтали.
constexpr float FEMUR_ZERO_ABSOLUTE_DEG = 45.0f;

// Направление эффективного вектора C-D определяется измеренными
// проекциями: 52 мм наружу и 106 мм вниз.
// Сам фиксированный изгиб детали Tibia 50° для расчёта положения
// кончика не нужен: вся Tibia является одним жёстким звеном.
constexpr float TIBIA_ZERO_HORIZONTAL_MM = 52.0f;
constexpr float TIBIA_ZERO_DOWN_MM = 106.0f;

constexpr float PI_F = 3.14159265358979323846f;

constexpr float deg_to_rad(float degrees)
{
    return degrees * PI_F / 180.0f;
}

constexpr float rad_to_deg(float radians)
{
    return radians * 180.0f / PI_F;
}

// atan2 нельзя вычислить как constexpr во всех версиях компилятора,
// поэтому эти значения вычисляются один раз при запуске.
float s_tibia_zero_absolute_rad = 0.0f;
float s_tibia_zero_relative_rad = 0.0f;
float s_neutral_radius_mm = 0.0f;
float s_neutral_z_mm = 0.0f;

// ============================================================
// PWM СЕРВОПРИВОДОВ
// ============================================================

constexpr uint32_t SERVO_FREQUENCY_HZ = 50;
constexpr uint32_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQUENCY_HZ;

constexpr ledc_mode_t SERVO_LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t SERVO_LEDC_RESOLUTION = LEDC_TIMER_16_BIT;
constexpr uint32_t SERVO_LEDC_MAX_DUTY = (1UL << 16U) - 1UL;

struct ServoCalibration {
    const char *name;
    gpio_num_t gpio;
    ledc_channel_t channel;
    int32_t pulse_minus_45_us;
    int32_t pulse_zero_us;
    int32_t pulse_plus_45_us;
};

enum ServoIndex : size_t {
    COXA = 0,
    FEMUR = 1,
    TIBIA = 2,
};

constexpr std::array<ServoCalibration, 3> LEG_SERVOS = {{
    {"Coxa", COXA_SERVO_PIN, LEDC_CHANNEL_0, 1995, 1491, 961},
    {"Femur", FEMUR_SERVO_PIN, LEDC_CHANNEL_1, 1976, 1462, 934},
    {"Tibia", TIBIA_SERVO_PIN, LEDC_CHANNEL_2, 2017, 1498, 969},
}};

constexpr ledc_channel_t CALIBRATION_SERVO_CHANNEL = LEDC_CHANNEL_3;
constexpr int32_t CALIBRATION_MIN_PULSE_US = 500;
constexpr int32_t CALIBRATION_MAX_PULSE_US = 2500;

int32_t s_calibration_pulse_us = 1500;

std::array<float, 3> s_current_leg_angles = {0.0f, 0.0f, 0.0f};
std::array<float, 3> s_target_leg_angles = {0.0f, 0.0f, 0.0f};

// ============================================================
// ИНДИКАТОР
// ============================================================

// Биты: A, B, C, D, E, F, G, DP.
constexpr std::array<uint8_t, 10> DIGIT_PATTERNS = {
    0b00111111,  // 0
    0b00000110,  // 1
    0b01011011,  // 2
    0b01001111,  // 3
    0b01100110,  // 4
    0b01101101,  // 5
    0b01111101,  // 6
    0b00000111,  // 7
    0b01111111,  // 8
    0b01101111,  // 9
};

constexpr uint8_t PATTERN_BLANK = 0x00;
constexpr uint8_t PATTERN_C = 0b00111001;
constexpr uint8_t PATTERN_A = 0b01110111;
constexpr uint8_t PATTERN_L = 0b00111000;
constexpr uint8_t PATTERN_E = 0b01111001;
constexpr uint8_t PATTERN_G = 0b00111101;
constexpr uint8_t PATTERN_N = 0b01010100;

// По одному байту на каждый разряд.
// 32-битная запись/чтение для ESP32 атомарна, поэтому отдельный mutex
// для шаблона дисплея не требуется.
volatile uint32_t s_display_packed = 0;

uint32_t pack_display_patterns(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    return static_cast<uint32_t>(d0) |
           (static_cast<uint32_t>(d1) << 8U) |
           (static_cast<uint32_t>(d2) << 16U) |
           (static_cast<uint32_t>(d3) << 24U);
}

void display_set_patterns(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    s_display_packed = pack_display_patterns(d0, d1, d2, d3);
}

void display_show_calibration_label()
{
    display_set_patterns(PATTERN_BLANK, PATTERN_C, PATTERN_A, PATTERN_L);
}

void display_show_leg_label()
{
    display_set_patterns(PATTERN_BLANK, PATTERN_L, PATTERN_E, PATTERN_G);
}

void display_show_centering_label()
{
    display_set_patterns(PATTERN_C, PATTERN_E, PATTERN_N, PATTERN_BLANK);
}

void display_set_number(int32_t value)
{
    value = std::clamp(
        value,
        static_cast<int32_t>(0),
        static_cast<int32_t>(9999));

    std::array<uint8_t, 4> result = {
        PATTERN_BLANK,
        PATTERN_BLANK,
        PATTERN_BLANK,
        PATTERN_BLANK,
    };

    result[3] = DIGIT_PATTERNS[value % 10];
    value /= 10;

    if (value > 0) {
        result[2] = DIGIT_PATTERNS[value % 10];
        value /= 10;
    }
    if (value > 0) {
        result[1] = DIGIT_PATTERNS[value % 10];
        value /= 10;
    }
    if (value > 0) {
        result[0] = DIGIT_PATTERNS[value % 10];
    }

    display_set_patterns(result[0], result[1], result[2], result[3]);
}

void display_disable_all_digits()
{
    for (gpio_num_t pin : DISPLAY_DIGIT_PINS) {
        gpio_set_level(pin, 1);
    }
}

void display_disable_all_segments()
{
    for (gpio_num_t pin : DISPLAY_SEGMENT_PINS) {
        gpio_set_level(pin, 0);
    }
}

// ============================================================
// ЭНКОДЕР
// ============================================================

portMUX_TYPE s_encoder_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t s_encoder_detent_delta = 0;
int8_t s_encoder_accumulator = 0;
uint8_t s_encoder_previous_state = 0;

// Таблица квадратурного декодера.
// Если направление окажется обратным, поменяйте знак
// ENCODER_DIRECTION_SIGN.
constexpr int32_t ENCODER_DIRECTION_SIGN = 1;

constexpr std::array<int8_t, 16> ENCODER_TRANSITION_TABLE = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
};

void sample_encoder()
{
    // Дисплейная задача вызывает эту функцию каждые 1 мс.
    const uint8_t current_state =
        (static_cast<uint8_t>(gpio_get_level(ENCODER_CLK_PIN)) << 1U) |
        static_cast<uint8_t>(gpio_get_level(ENCODER_DT_PIN));

    const uint8_t index =
        static_cast<uint8_t>((s_encoder_previous_state << 2U) | current_state);

    s_encoder_accumulator += ENCODER_TRANSITION_TABLE[index];
    s_encoder_previous_state = current_state;

    if (s_encoder_accumulator >= 4) {
        portENTER_CRITICAL(&s_encoder_mux);
        s_encoder_detent_delta += ENCODER_DIRECTION_SIGN;
        portEXIT_CRITICAL(&s_encoder_mux);
        s_encoder_accumulator = 0;
    } else if (s_encoder_accumulator <= -4) {
        portENTER_CRITICAL(&s_encoder_mux);
        s_encoder_detent_delta -= ENCODER_DIRECTION_SIGN;
        portEXIT_CRITICAL(&s_encoder_mux);
        s_encoder_accumulator = 0;
    }
}

int32_t take_encoder_delta()
{
    portENTER_CRITICAL(&s_encoder_mux);
    const int32_t delta = s_encoder_detent_delta;
    s_encoder_detent_delta = 0;
    portEXIT_CRITICAL(&s_encoder_mux);
    return delta;
}

// ============================================================
// ОТДЕЛЬНАЯ ЗАДАЧА ИНДИКАТОРА
// ============================================================

// Раньше мультиплексирование выполнялось callback-функцией esp_timer.
// В режиме стика ADC и обратная кинематика создавали джиттер выполнения,
// поэтому отдельные разряды получали разную длительность включения.
//
// Теперь дисплей работает в отдельной высокоприоритетной задаче на CPU1,
// а управление ногой — на CPU0. Частота полного кадра: 250 Гц.
void display_task(void *)
{
    static_assert(
        configTICK_RATE_HZ >= 1000,
        "Set CONFIG_FREERTOS_HZ=1000 in sdkconfig.defaults");

    TickType_t last_wake = xTaskGetTickCount();
    uint8_t active_digit = 0;

    while (true) {
        display_disable_all_digits();
        display_disable_all_segments();

        const uint32_t packed = s_display_packed;
        const uint8_t pattern =
            static_cast<uint8_t>(
                (packed >> (active_digit * 8U)) & 0xFFU);

        for (uint8_t segment = 0;
             segment < DISPLAY_SEGMENT_PINS.size();
             ++segment) {
            const bool enabled =
                (pattern & (1U << segment)) != 0U;

            gpio_set_level(
                DISPLAY_SEGMENT_PINS[segment],
                enabled ? 1 : 0);
        }

        if (pattern != PATTERN_BLANK) {
            gpio_set_level(
                DISPLAY_DIGIT_PINS[active_digit],
                0);
        }

        active_digit =
            static_cast<uint8_t>(
                (active_digit + 1U) % DISPLAY_DIGIT_PINS.size());

        sample_encoder();

        // Один разряд удерживается 1 мс:
        // 4 разряда -> полный кадр 4 мс -> 250 Гц.
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(1));
    }
}

// ============================================================
// ADC И СТИК
// ============================================================

adc_oneshot_unit_handle_t s_adc_handle = nullptr;

int32_t s_joystick_center_x = 2048;
int32_t s_joystick_center_y = 2048;

std::array<int32_t, 3> s_joystick_x_samples = {2048, 2048, 2048};
std::array<int32_t, 3> s_joystick_y_samples = {2048, 2048, 2048};
size_t s_joystick_sample_index = 0;

float s_filtered_joystick_x = 2048.0f;
float s_filtered_joystick_y = 2048.0f;

bool s_joystick_centering = false;
int32_t s_center_sum_x = 0;
int32_t s_center_sum_y = 0;
uint32_t s_center_sample_count = 0;

constexpr uint32_t JOYSTICK_CENTER_SAMPLE_COUNT = 32;
constexpr float JOYSTICK_EMA_ALPHA = 0.35f;

int32_t median_of_three(int32_t a, int32_t b, int32_t c)
{
    if (a > b) {
        std::swap(a, b);
    }
    if (b > c) {
        std::swap(b, c);
    }
    if (a > b) {
        std::swap(a, b);
    }
    return b;
}

esp_err_t adc_init()
{
    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = ADC_UNIT_1;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&unit_config, &s_adc_handle),
        TAG,
        "ADC unit init failed");

    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(
            s_adc_handle, JOYSTICK_X_ADC_CHANNEL, &channel_config),
        TAG,
        "ADC X channel config failed");

    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(
            s_adc_handle, JOYSTICK_Y_ADC_CHANNEL, &channel_config),
        TAG,
        "ADC Y channel config failed");

    ESP_LOGI(TAG, "Joystick ADC initialized: GPIO36 and GPIO39");
    return ESP_OK;
}

bool read_joystick_raw(int32_t *raw_x, int32_t *raw_y)
{
    int value_x = 0;
    int value_y = 0;

    if (adc_oneshot_read(s_adc_handle, JOYSTICK_X_ADC_CHANNEL, &value_x) != ESP_OK) {
        return false;
    }
    if (adc_oneshot_read(s_adc_handle, JOYSTICK_Y_ADC_CHANNEL, &value_y) != ESP_OK) {
        return false;
    }

    *raw_x = value_x;
    *raw_y = value_y;
    return true;
}

float normalize_axis(float raw, float center)
{
    if (raw >= center) {
        const float denominator = std::max(1.0f, 4095.0f - center);
        return std::clamp((raw - center) / denominator, 0.0f, 1.0f);
    }

    const float denominator = std::max(1.0f, center);
    return std::clamp((raw - center) / denominator, -1.0f, 0.0f);
}

// ============================================================
// ОБРАТНАЯ КИНЕМАТИКА
// ============================================================

struct LegAngles {
    float coxa_deg;
    float femur_deg;
    float tibia_deg;
};

bool angle_is_safe(float angle)
{
    return std::isfinite(angle) &&
           angle >= -IK_ANGLE_LIMIT_DEG &&
           angle <= IK_ANGLE_LIMIT_DEG;
}

bool solve_leg_ik(float target_x_mm,
                  float target_y_mm,
                  float target_z_mm,
                  LegAngles *result)
{
    // X — вправо при взгляде сверху.
    // Y — направление ноги от оси Coxa наружу.
    // Z — вверх. Нейтральная стопа имеет отрицательный Z.

    const float radius = std::hypot(target_x_mm, target_y_mm);
    if (radius <= COXA_LENGTH_MM + 0.01f) {
        return false;
    }

    const float planar_x = radius - COXA_LENGTH_MM;
    const float planar_z = target_z_mm;

    const float numerator =
        planar_x * planar_x +
        planar_z * planar_z -
        FEMUR_LENGTH_MM * FEMUR_LENGTH_MM -
        TIBIA_EFFECTIVE_LENGTH_MM * TIBIA_EFFECTIVE_LENGTH_MM;

    const float denominator =
        2.0f * FEMUR_LENGTH_MM * TIBIA_EFFECTIVE_LENGTH_MM;

    float cos_delta = numerator / denominator;
    if (cos_delta < -1.0001f || cos_delta > 1.0001f) {
        return false;
    }
    cos_delta = std::clamp(cos_delta, -1.0f, 1.0f);

    const float acos_delta = std::acos(cos_delta);
    const float delta_positive = acos_delta;
    const float delta_negative = -acos_delta;

    // Выбираем ту ветвь, которая ближе к собранному нулевому положению.
    const float delta =
        std::fabs(delta_negative - s_tibia_zero_relative_rad) <
                std::fabs(delta_positive - s_tibia_zero_relative_rad)
            ? delta_negative
            : delta_positive;

    const float femur_absolute =
        std::atan2(planar_z, planar_x) -
        std::atan2(
            TIBIA_EFFECTIVE_LENGTH_MM * std::sin(delta),
            FEMUR_LENGTH_MM +
                TIBIA_EFFECTIVE_LENGTH_MM * std::cos(delta));

    const float coxa_geometry_deg =
        rad_to_deg(std::atan2(target_x_mm, target_y_mm));

    const float femur_geometry_delta_deg =
        rad_to_deg(femur_absolute - deg_to_rad(FEMUR_ZERO_ABSOLUTE_DEG));

    const float tibia_geometry_delta_deg =
        rad_to_deg(delta - s_tibia_zero_relative_rad);

    const float coxa_servo_deg =
        coxa_geometry_deg * COXA_SERVO_TO_GEOMETRY_SIGN;

    const float femur_servo_deg =
        femur_geometry_delta_deg * FEMUR_SERVO_TO_GEOMETRY_SIGN;

    const float tibia_servo_deg =
        tibia_geometry_delta_deg * TIBIA_SERVO_TO_GEOMETRY_SIGN;

    if (!angle_is_safe(coxa_servo_deg) ||
        !angle_is_safe(femur_servo_deg) ||
        !angle_is_safe(tibia_servo_deg)) {
        return false;
    }

    result->coxa_deg = coxa_servo_deg;
    result->femur_deg = femur_servo_deg;
    result->tibia_deg = tibia_servo_deg;
    return true;
}

// Максимальная дальность зависит только от направления стика.
// Раньше она заново искалась каждые 10 мс: до ~85 решений IK за цикл.
// Это перегружало ESP32 и нарушало равномерность мультиплексирования дисплея.
//
// Теперь используется таблица направлений с ленивым кэшированием.
// Для одного нового направления выполняется только двоичный поиск,
// а затем результат переиспользуется.
constexpr size_t REACHABILITY_DIRECTION_BINS = 72;  // шаг 5°
constexpr float REACHABILITY_SEARCH_LIMIT_MM = 220.0f;
constexpr uint32_t REACHABILITY_BINARY_STEPS = 14;

std::array<float, REACHABILITY_DIRECTION_BINS> s_reachability_cache_mm = {};
std::array<bool, REACHABILITY_DIRECTION_BINS> s_reachability_cache_valid = {};

float compute_max_reachable_distance(float direction_x, float direction_y)
{
    LegAngles angles = {};

    const float limit_x =
        direction_x * REACHABILITY_SEARCH_LIMIT_MM;

    const float limit_y =
        s_neutral_radius_mm +
        direction_y * REACHABILITY_SEARCH_LIMIT_MM;

    // Если вся заданная длина достижима, дальнейший поиск не нужен.
    if (solve_leg_ik(limit_x, limit_y, s_neutral_z_mm, &angles)) {
        return REACHABILITY_SEARCH_LIMIT_MM;
    }

    // Нулевая точка соответствует собранной нейтрали и достижима.
    float low = 0.0f;
    float high = REACHABILITY_SEARCH_LIMIT_MM;

    for (uint32_t i = 0; i < REACHABILITY_BINARY_STEPS; ++i) {
        const float middle = 0.5f * (low + high);

        const float x = direction_x * middle;
        const float y =
            s_neutral_radius_mm +
            direction_y * middle;

        if (solve_leg_ik(x, y, s_neutral_z_mm, &angles)) {
            low = middle;
        } else {
            high = middle;
        }
    }

    // Небольшой запас от программного ограничения ±75°.
    return low * 0.995f;
}

float cached_reachable_distance(size_t index)
{
    index %= REACHABILITY_DIRECTION_BINS;

    if (!s_reachability_cache_valid[index]) {
        const float angle =
            (2.0f * PI_F * static_cast<float>(index)) /
            static_cast<float>(REACHABILITY_DIRECTION_BINS);

        const float direction_x = std::cos(angle);
        const float direction_y = std::sin(angle);

        s_reachability_cache_mm[index] =
            compute_max_reachable_distance(
                direction_x,
                direction_y);

        s_reachability_cache_valid[index] = true;
    }

    return s_reachability_cache_mm[index];
}

float find_max_reachable_distance(float direction_x, float direction_y)
{
    float angle = std::atan2(direction_y, direction_x);

    if (angle < 0.0f) {
        angle += 2.0f * PI_F;
    }

    const float table_position =
        angle *
        static_cast<float>(REACHABILITY_DIRECTION_BINS) /
        (2.0f * PI_F);

    const size_t first_index =
        static_cast<size_t>(std::floor(table_position)) %
        REACHABILITY_DIRECTION_BINS;

    const size_t second_index =
        (first_index + 1U) %
        REACHABILITY_DIRECTION_BINS;

    const float fraction =
        table_position -
        std::floor(table_position);

    const float first_distance =
        cached_reachable_distance(first_index);

    const float second_distance =
        cached_reachable_distance(second_index);

    return first_distance +
           (second_distance - first_distance) *
               fraction;
}

// ============================================================
// ПРЕОБРАЗОВАНИЕ УГЛА В ИМПУЛЬС
// ============================================================

float clamp_float(float value, float minimum, float maximum)
{
    return std::min(std::max(value, minimum), maximum);
}

int32_t interpolate_pulse(float start, float end, float t)
{
    return static_cast<int32_t>(
        std::lround(start + (end - start) * t));
}

int32_t angle_to_pulse_us(const ServoCalibration &servo, float angle_deg)
{
    const float angle =
        clamp_float(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

    // Калибровка известна только в точках -45°, 0° и +45°.
    // Для диапазона от -75° до +75° продолжаем те же прямые,
    // сохраняя точное соответствие исходным калибровочным точкам.
    if (angle <= 0.0f) {
        const float t = (-angle) / CALIBRATION_ANGLE_DEG;

        return interpolate_pulse(
            static_cast<float>(servo.pulse_zero_us),
            static_cast<float>(servo.pulse_minus_45_us),
            t);
    }

    const float t = angle / CALIBRATION_ANGLE_DEG;

    return interpolate_pulse(
        static_cast<float>(servo.pulse_zero_us),
        static_cast<float>(servo.pulse_plus_45_us),
        t);
}

uint32_t pulse_to_duty(int32_t pulse_us)
{
    const uint64_t numerator =
        static_cast<uint64_t>(pulse_us) * SERVO_LEDC_MAX_DUTY;

    return static_cast<uint32_t>(
        (numerator + SERVO_PERIOD_US / 2U) / SERVO_PERIOD_US);
}

esp_err_t write_servo_pulse(ledc_channel_t channel, int32_t pulse_us)
{
    pulse_us = std::clamp(
        pulse_us, CALIBRATION_MIN_PULSE_US, CALIBRATION_MAX_PULSE_US);

    const uint32_t duty = pulse_to_duty(pulse_us);

    ESP_RETURN_ON_ERROR(
        ledc_set_duty(SERVO_LEDC_MODE, channel, duty),
        TAG,
        "LEDC set duty failed");

    ESP_RETURN_ON_ERROR(
        ledc_update_duty(SERVO_LEDC_MODE, channel),
        TAG,
        "LEDC update duty failed");

    return ESP_OK;
}

void stop_leg_pwm()
{
    for (const ServoCalibration &servo : LEG_SERVOS) {
        ledc_stop(SERVO_LEDC_MODE, servo.channel, 0);
    }
}

void stop_calibration_pwm()
{
    ledc_stop(SERVO_LEDC_MODE, CALIBRATION_SERVO_CHANNEL, 0);
}

void write_leg_angles(const std::array<float, 3> &angles)
{
    for (size_t i = 0; i < LEG_SERVOS.size(); ++i) {
        const int32_t pulse = angle_to_pulse_us(LEG_SERVOS[i], angles[i]);
        const esp_err_t result =
            write_servo_pulse(LEG_SERVOS[i].channel, pulse);

        if (result != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Failed to update %s: %s",
                LEG_SERVOS[i].name,
                esp_err_to_name(result));
        }
    }
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
        ledc_timer_config(&timer_config),
        TAG,
        "LEDC timer config failed");

    for (const ServoCalibration &servo : LEG_SERVOS) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = servo.gpio;
        channel_config.speed_mode = SERVO_LEDC_MODE;
        channel_config.channel = servo.channel;
        channel_config.timer_sel = SERVO_LEDC_TIMER;
        channel_config.duty = 0;
        channel_config.hpoint = 0;

        ESP_RETURN_ON_ERROR(
            ledc_channel_config(&channel_config),
            TAG,
            "Leg LEDC channel config failed");
    }

    ledc_channel_config_t calibration_channel = {};
    calibration_channel.gpio_num = CALIBRATION_SERVO_PIN;
    calibration_channel.speed_mode = SERVO_LEDC_MODE;
    calibration_channel.channel = CALIBRATION_SERVO_CHANNEL;
    calibration_channel.timer_sel = SERVO_LEDC_TIMER;
    calibration_channel.duty = 0;
    calibration_channel.hpoint = 0;

    ESP_RETURN_ON_ERROR(
        ledc_channel_config(&calibration_channel),
        TAG,
        "Calibration LEDC channel config failed");

    stop_leg_pwm();
    stop_calibration_pwm();

    ESP_LOGI(TAG, "Four servo PWM channels initialized at 50 Hz");
    return ESP_OK;
}

// ============================================================
// GPIO И ТАЙМЕР ИНДИКАТОРА
// ============================================================

esp_err_t gpio_init()
{
    uint64_t output_mask = 0;

    for (gpio_num_t pin : DISPLAY_SEGMENT_PINS) {
        output_mask |= (1ULL << static_cast<uint32_t>(pin));
    }
    for (gpio_num_t pin : DISPLAY_DIGIT_PINS) {
        output_mask |= (1ULL << static_cast<uint32_t>(pin));
    }

    gpio_config_t output_config = {};
    output_config.pin_bit_mask = output_mask;
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(
        gpio_config(&output_config),
        TAG,
        "Display GPIO config failed");

    display_disable_all_digits();
    display_disable_all_segments();

    gpio_config_t encoder_config = {};
    encoder_config.pin_bit_mask =
        (1ULL << static_cast<uint32_t>(ENCODER_CLK_PIN)) |
        (1ULL << static_cast<uint32_t>(ENCODER_DT_PIN));
    encoder_config.mode = GPIO_MODE_INPUT;
    encoder_config.pull_up_en = GPIO_PULLUP_DISABLE;
    encoder_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    encoder_config.intr_type = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(
        gpio_config(&encoder_config),
        TAG,
        "Encoder GPIO config failed");

    gpio_config_t button_config = {};
    button_config.pin_bit_mask =
        1ULL << static_cast<uint32_t>(MODE_BUTTON_PIN);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(
        gpio_config(&button_config),
        TAG,
        "Mode button GPIO config failed");

    s_encoder_previous_state =
        (static_cast<uint8_t>(gpio_get_level(ENCODER_CLK_PIN)) << 1U) |
        static_cast<uint8_t>(gpio_get_level(ENCODER_DT_PIN));

    return ESP_OK;
}

// ============================================================
// РЕЖИМЫ
// ============================================================

int64_t s_mode_banner_until_us = 0;

void start_joystick_centering()
{
    s_joystick_centering = true;
    s_center_sum_x = 0;
    s_center_sum_y = 0;
    s_center_sample_count = 0;
    display_show_centering_label();
}

void enter_calibration_mode()
{
    s_mode = AppMode::Calibration;

    stop_leg_pwm();
    write_servo_pulse(
        CALIBRATION_SERVO_CHANNEL, s_calibration_pulse_us);

    display_show_calibration_label();
    s_mode_banner_until_us = esp_timer_get_time() + 700000;

    ESP_LOGI(
        TAG,
        "Calibration mode: encoder controls GPIO33, pulse=%ld us",
        static_cast<long>(s_calibration_pulse_us));
}

void enter_leg_control_mode()
{
    s_mode = AppMode::LegControl;

    stop_calibration_pwm();

    s_current_leg_angles = {0.0f, 0.0f, 0.0f};
    s_target_leg_angles = {0.0f, 0.0f, 0.0f};
    write_leg_angles(s_current_leg_angles);

    display_show_leg_label();
    s_mode_banner_until_us = esp_timer_get_time() + 700000;

    start_joystick_centering();

    ESP_LOGI(
        TAG,
        "Leg mode: joystick controls the foot tip; centering joystick");
}

void toggle_mode()
{
    if (s_mode == AppMode::Calibration) {
        enter_leg_control_mode();
    } else {
        enter_calibration_mode();
    }
}

// ============================================================
// КНОПКА
// ============================================================

struct DebouncedButton {
    bool raw_pressed = false;
    bool stable_pressed = false;
    int64_t raw_changed_at_us = 0;
};

DebouncedButton s_mode_button;
constexpr int64_t BUTTON_DEBOUNCE_US = 30000;

void update_mode_button(int64_t now_us)
{
    const bool pressed = gpio_get_level(MODE_BUTTON_PIN) == 0;

    if (pressed != s_mode_button.raw_pressed) {
        s_mode_button.raw_pressed = pressed;
        s_mode_button.raw_changed_at_us = now_us;
    }

    if (pressed != s_mode_button.stable_pressed &&
        now_us - s_mode_button.raw_changed_at_us >= BUTTON_DEBOUNCE_US) {
        s_mode_button.stable_pressed = pressed;

        if (pressed) {
            toggle_mode();
        }
    }
}

// ============================================================
// ОБРАБОТКА КАЛИБРОВКИ
// ============================================================

int64_t s_last_encoder_detent_us = 0;

void update_calibration_mode(int64_t now_us)
{
    const int32_t encoder_delta = take_encoder_delta();

    if (encoder_delta != 0) {
        const int64_t elapsed =
            s_last_encoder_detent_us == 0
                ? 1000000
                : now_us - s_last_encoder_detent_us;

        int32_t step_us = 1;
        if (elapsed < 30000) {
            step_us = 20;
        } else if (elapsed < 80000) {
            step_us = 5;
        }

        s_last_encoder_detent_us = now_us;

        s_calibration_pulse_us += encoder_delta * step_us;
        s_calibration_pulse_us = std::clamp(
            s_calibration_pulse_us,
            CALIBRATION_MIN_PULSE_US,
            CALIBRATION_MAX_PULSE_US);

        write_servo_pulse(
            CALIBRATION_SERVO_CHANNEL,
            s_calibration_pulse_us);
    }

    if (now_us >= s_mode_banner_until_us) {
        display_set_number(s_calibration_pulse_us);
    }
}

// ============================================================
// ОБРАБОТКА СТИКА И НОГИ
// ============================================================

void update_joystick_filter(int32_t raw_x, int32_t raw_y)
{
    s_joystick_x_samples[s_joystick_sample_index] = raw_x;
    s_joystick_y_samples[s_joystick_sample_index] = raw_y;
    s_joystick_sample_index =
        (s_joystick_sample_index + 1U) % s_joystick_x_samples.size();

    const int32_t median_x = median_of_three(
        s_joystick_x_samples[0],
        s_joystick_x_samples[1],
        s_joystick_x_samples[2]);

    const int32_t median_y = median_of_three(
        s_joystick_y_samples[0],
        s_joystick_y_samples[1],
        s_joystick_y_samples[2]);

    s_filtered_joystick_x +=
        JOYSTICK_EMA_ALPHA *
        (static_cast<float>(median_x) - s_filtered_joystick_x);

    s_filtered_joystick_y +=
        JOYSTICK_EMA_ALPHA *
        (static_cast<float>(median_y) - s_filtered_joystick_y);
}

void finish_joystick_centering()
{
    s_joystick_center_x =
        s_center_sum_x / static_cast<int32_t>(s_center_sample_count);

    s_joystick_center_y =
        s_center_sum_y / static_cast<int32_t>(s_center_sample_count);

    s_joystick_x_samples.fill(s_joystick_center_x);
    s_joystick_y_samples.fill(s_joystick_center_y);
    s_filtered_joystick_x =
        static_cast<float>(s_joystick_center_x);
    s_filtered_joystick_y =
        static_cast<float>(s_joystick_center_y);

    s_joystick_centering = false;
    display_show_leg_label();

    ESP_LOGI(
        TAG,
        "Joystick centered: X=%ld Y=%ld",
        static_cast<long>(s_joystick_center_x),
        static_cast<long>(s_joystick_center_y));
}

void calculate_leg_target_from_joystick()
{
    // Оси поменяны местами по требованию:
    // физический VRY задаёт координату X,
    // физический VRX задаёт координату Y.
    float x = normalize_axis(
        s_filtered_joystick_y,
        static_cast<float>(s_joystick_center_y));

    float y = normalize_axis(
        s_filtered_joystick_x,
        static_cast<float>(s_joystick_center_x));

    x *= JOYSTICK_X_SIGN;
    y *= JOYSTICK_Y_SIGN;

    const float magnitude = std::hypot(x, y);

    if (magnitude <= JOYSTICK_DEAD_ZONE) {
        s_target_leg_angles = {0.0f, 0.0f, 0.0f};
        return;
    }

    const float direction_x = x / magnitude;
    const float direction_y = y / magnitude;

    const float normalized_magnitude = std::clamp(
        (magnitude - JOYSTICK_DEAD_ZONE) /
            (1.0f - JOYSTICK_DEAD_ZONE),
        0.0f,
        1.0f);

    const float maximum_distance =
        find_max_reachable_distance(direction_x, direction_y);

    const float commanded_distance =
        maximum_distance * normalized_magnitude;

    const float target_x =
        direction_x * commanded_distance;

    const float target_y =
        s_neutral_radius_mm +
        direction_y * commanded_distance;

    LegAngles angles = {};
    if (solve_leg_ik(
            target_x,
            target_y,
            s_neutral_z_mm,
            &angles)) {
        s_target_leg_angles = {
            angles.coxa_deg,
            angles.femur_deg,
            angles.tibia_deg,
        };
    }
}

void update_leg_motion()
{
    // Максимально быстрая программная реакция:
    // новый целевой угол сразу передаётся в PWM без сглаживания
    // и без ограничения градусов в секунду.
    s_current_leg_angles = s_target_leg_angles;
    write_leg_angles(s_current_leg_angles);
}

void update_leg_mode()
{
    int32_t raw_x = 0;
    int32_t raw_y = 0;

    if (!read_joystick_raw(&raw_x, &raw_y)) {
        return;
    }

    if (s_joystick_centering) {
        s_center_sum_x += raw_x;
        s_center_sum_y += raw_y;
        ++s_center_sample_count;

        if (s_center_sample_count >= JOYSTICK_CENTER_SAMPLE_COUNT) {
            finish_joystick_centering();
        }
        return;
    }

    update_joystick_filter(raw_x, raw_y);
    calculate_leg_target_from_joystick();
}

// ============================================================
// ОСНОВНАЯ ЗАДАЧА
// ============================================================

void control_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t leg_update_divider = 0;

    // Не считать нажатую при старте кнопку новым нажатием.
    const bool initially_pressed =
        gpio_get_level(MODE_BUTTON_PIN) == 0;
    s_mode_button.raw_pressed = initially_pressed;
    s_mode_button.stable_pressed = initially_pressed;
    s_mode_button.raw_changed_at_us = esp_timer_get_time();

    while (true) {
        const int64_t now_us = esp_timer_get_time();

        update_mode_button(now_us);

        if (s_mode == AppMode::Calibration) {
            update_calibration_mode(now_us);
        } else {
            // ADC, IK и обновление трёх PWM выполняются раз в 20 мс.
            // Кнопка продолжает проверяться каждые 10 мс.
            ++leg_update_divider;

            if (leg_update_divider >= 2U) {
                leg_update_divider = 0;

                update_leg_mode();
                update_leg_motion();
            }

            // finish_joystick_centering() уже один раз выводит LEG.
            // Постоянно перезаписывать дисплей каждые 10 мс не требуется.
        }

        // 10 мс. Обновление PWM ноги выполняется каждые 20 мс.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ============================================================
// ИНИЦИАЛИЗАЦИЯ ГЕОМЕТРИИ
// ============================================================

void initialize_geometry()
{
    s_tibia_zero_absolute_rad =
        std::atan2(
            -TIBIA_ZERO_DOWN_MM,
            TIBIA_ZERO_HORIZONTAL_MM);

    s_tibia_zero_relative_rad =
        s_tibia_zero_absolute_rad -
        deg_to_rad(FEMUR_ZERO_ABSOLUTE_DEG);

    const float femur_zero_rad =
        deg_to_rad(FEMUR_ZERO_ABSOLUTE_DEG);

    s_neutral_radius_mm =
        COXA_LENGTH_MM +
        FEMUR_LENGTH_MM * std::cos(femur_zero_rad) +
        TIBIA_EFFECTIVE_LENGTH_MM *
            std::cos(s_tibia_zero_absolute_rad);

    s_neutral_z_mm =
        FEMUR_LENGTH_MM * std::sin(femur_zero_rad) +
        TIBIA_EFFECTIVE_LENGTH_MM *
            std::sin(s_tibia_zero_absolute_rad);

    ESP_LOGI(
        TAG,
        "Neutral foot: radius=%.2f mm, Z=%.2f mm, tibia absolute=%.2f deg",
        static_cast<double>(s_neutral_radius_mm),
        static_cast<double>(s_neutral_z_mm),
        static_cast<double>(rad_to_deg(s_tibia_zero_absolute_rad)));
}

}  // namespace

extern "C" void app_main()
{
    initialize_geometry();

    ESP_ERROR_CHECK(gpio_init());
    ESP_ERROR_CHECK(adc_init());
    ESP_ERROR_CHECK(servo_pwm_init());

    TaskHandle_t display_handle = nullptr;
    const BaseType_t display_created =
        xTaskCreatePinnedToCore(
            display_task,
            "display_task",
            3072,
            nullptr,
            12,
            &display_handle,
            1);

    if (display_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display_task");
        stop_leg_pwm();
        stop_calibration_pwm();
        display_disable_all_digits();
        return;
    }

    // Безопасный режим при включении: калибровка отдельной сервы.
    enter_calibration_mode();

    TaskHandle_t control_handle = nullptr;
    const BaseType_t control_created =
        xTaskCreatePinnedToCore(
            control_task,
            "control_task",
            6144,
            nullptr,
            4,
            &control_handle,
            0);

    if (control_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control_task");
        stop_leg_pwm();
        stop_calibration_pwm();
        display_disable_all_digits();
        vTaskDelete(display_handle);
        return;
    }

    ESP_LOGI(
        TAG,
        "Started. Press encoder SW or joystick SW to change mode.");

    ESP_LOGI(
        TAG,
        "Reachability uses a cached 72-direction lookup table.");

    ESP_LOGI(
        TAG,
        "Display task runs on CPU1 at 250 Hz frame rate.");
}