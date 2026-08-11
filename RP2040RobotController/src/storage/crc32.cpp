#include "storage/crc32.hpp"

namespace storage {

uint32_t crc32(const void *data, size_t length) {
    const auto *bytes = static_cast<const uint8_t *>(data);
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

} // namespace storage
