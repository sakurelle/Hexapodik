#pragma once

#include "servo/servo_config.hpp"

#include <array>
#include <cstdint>

namespace storage {

constexpr uint32_t CONFIG_MAGIC = 0x48585044u; // HXPD
constexpr uint16_t CONFIG_VERSION = 1;
constexpr uint32_t FLASH_SECTOR_SIZE_BYTES = 4096;

struct StoredConfigHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t crc32;
};

struct StoredServoCalibration {
    uint16_t center_us;
    uint16_t pulse_minus_45_us;
    uint16_t pulse_plus_45_us;
    float min_angle_deg;
    float max_angle_deg;
    uint8_t enabled;
    uint8_t reserved[3];
};

struct StoredConfigPayload {
    std::array<StoredServoCalibration, servo::SERVO_COUNT> servos;
};

struct StoredConfigImage {
    StoredConfigHeader header;
    StoredConfigPayload payload;
};

static_assert(sizeof(StoredConfigImage) <= FLASH_SECTOR_SIZE_BYTES,
              "Stored configuration must fit in one reserved flash sector");

enum class ConfigStatus {
    Valid,
    InvalidMagic,
    InvalidVersion,
    InvalidSize,
    InvalidCrc
};

ConfigStatus validate_config_image(const StoredConfigImage &image);
void payload_from_servos(const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                         StoredConfigPayload &payload);
void apply_payload_to_servos(const StoredConfigPayload &payload,
                             std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);

class ConfigStorage {
public:
    bool load(std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);
    bool save(const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos);
    bool last_load_valid() const { return last_load_valid_; }
    uint16_t version() const { return CONFIG_VERSION; }

private:
    bool last_load_valid_ = false;
};

} // namespace storage
