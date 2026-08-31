#pragma once

#include <cstddef>
#include <cstdint>

namespace hexapod::protocol {

std::size_t cobsEncode(const std::uint8_t* input, std::size_t input_size, std::uint8_t* output, std::size_t output_size);
std::size_t cobsDecode(const std::uint8_t* input, std::size_t input_size, std::uint8_t* output, std::size_t output_size);

}  // namespace hexapod::protocol
