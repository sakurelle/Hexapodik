#include "protocol/crc.h"

namespace hexapod::protocol {

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8u;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) != 0 ? static_cast<std::uint16_t>((crc << 1u) ^ 0x1021u)
                                       : static_cast<std::uint16_t>(crc << 1u);
        }
    }
    return crc;
}

}  // namespace hexapod::protocol
