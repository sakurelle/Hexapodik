#include "storage/config_storage.hpp"
#include "storage/crc32.hpp"

#if __has_include("hardware/flash.h")
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#endif

#include <cstring>

namespace storage {

ConfigStatus validate_config_image(const StoredConfigImage &image) {
    if (image.header.magic != CONFIG_MAGIC) {
        return ConfigStatus::InvalidMagic;
    }
    if (image.header.version != CONFIG_VERSION) {
        return ConfigStatus::InvalidVersion;
    }
    if (image.header.payload_size != sizeof(StoredConfigPayload)) {
        return ConfigStatus::InvalidSize;
    }
    const uint32_t actual_crc = crc32(&image.payload, sizeof(image.payload));
    if (actual_crc != image.header.crc32) {
        return ConfigStatus::InvalidCrc;
    }
    return ConfigStatus::Valid;
}

void payload_from_servos(const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos,
                         StoredConfigPayload &payload) {
    for (size_t i = 0; i < servos.size(); ++i) {
        payload.servos[i].center_us = servos[i].center_us;
        payload.servos[i].pulse_minus_45_us = servos[i].pulse_minus_45_us;
        payload.servos[i].pulse_plus_45_us = servos[i].pulse_plus_45_us;
        payload.servos[i].min_angle_deg = servos[i].min_angle_deg;
        payload.servos[i].max_angle_deg = servos[i].max_angle_deg;
        payload.servos[i].enabled = servos[i].enabled ? 1 : 0;
    }
}

void apply_payload_to_servos(const StoredConfigPayload &payload,
                             std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    for (size_t i = 0; i < servos.size(); ++i) {
        servos[i].center_us = payload.servos[i].center_us;
        servos[i].pulse_minus_45_us = payload.servos[i].pulse_minus_45_us;
        servos[i].pulse_plus_45_us = payload.servos[i].pulse_plus_45_us;
        servos[i].min_angle_deg = payload.servos[i].min_angle_deg;
        servos[i].max_angle_deg = payload.servos[i].max_angle_deg;
        servos[i].enabled = payload.servos[i].enabled != 0;
    }
}

#if __has_include("hardware/flash.h")
static constexpr uint32_t CONFIG_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE_BYTES;

bool ConfigStorage::load(std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    const auto *image = reinterpret_cast<const StoredConfigImage *>(XIP_BASE + CONFIG_FLASH_OFFSET);
    const ConfigStatus status = validate_config_image(*image);
    last_load_valid_ = status == ConfigStatus::Valid;
    if (last_load_valid_) {
        apply_payload_to_servos(image->payload, servos);
    }
    return last_load_valid_;
}

bool ConfigStorage::save(const std::array<servo::ServoConfig, servo::SERVO_COUNT> &servos) {
    StoredConfigImage image{};
    image.header.magic = CONFIG_MAGIC;
    image.header.version = CONFIG_VERSION;
    image.header.payload_size = sizeof(StoredConfigPayload);
    payload_from_servos(servos, image.payload);
    image.header.crc32 = crc32(&image.payload, sizeof(image.payload));

    uint8_t sector[FLASH_SECTOR_SIZE_BYTES];
    memset(sector, 0xff, sizeof(sector));
    memcpy(sector, &image, sizeof(image));

    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE_BYTES);
    flash_range_program(CONFIG_FLASH_OFFSET, sector, FLASH_SECTOR_SIZE_BYTES);
    restore_interrupts(ints);

    return true;
}
#else
bool ConfigStorage::load(std::array<servo::ServoConfig, servo::SERVO_COUNT> &) {
    last_load_valid_ = false;
    return false;
}

bool ConfigStorage::save(const std::array<servo::ServoConfig, servo::SERVO_COUNT> &) {
    return false;
}
#endif

} // namespace storage
