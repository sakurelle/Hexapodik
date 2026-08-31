#pragma once

#include <cstddef>
#include <cstdint>

namespace hexapod {

class UartTransport {
public:
    bool init(std::uint32_t baud = 115200);
    int readByte();
    bool write(const std::uint8_t* data, std::size_t size);
};

}  // namespace hexapod
