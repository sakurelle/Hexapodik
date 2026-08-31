#pragma once

#include <cstddef>
#include <cstdint>

namespace hexapod::protocol {

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t size);

}  // namespace hexapod::protocol
