#pragma once

#include <cstddef>
#include <cstdint>

namespace storage {

uint32_t crc32(const void *data, size_t length);

} // namespace storage
