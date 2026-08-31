#include "protocol/cobs.h"

namespace hexapod::protocol {

std::size_t cobsEncode(const std::uint8_t* input, std::size_t input_size, std::uint8_t* output, std::size_t output_size) {
    if (output_size == 0) {
        return 0;
    }

    std::size_t read = 0;
    std::size_t write = 1;
    std::size_t code_index = 0;
    std::uint8_t code = 1;

    while (read < input_size) {
        if (write >= output_size) {
            return 0;
        }
        if (input[read] == 0) {
            output[code_index] = code;
            code_index = write++;
            code = 1;
            ++read;
        } else {
            output[write++] = input[read++];
            ++code;
            if (code == 0xFFu) {
                output[code_index] = code;
                code_index = write++;
                code = 1;
            }
        }
    }

    if (code_index >= output_size) {
        return 0;
    }
    output[code_index] = code;
    return write;
}

std::size_t cobsDecode(const std::uint8_t* input, std::size_t input_size, std::uint8_t* output, std::size_t output_size) {
    std::size_t read = 0;
    std::size_t write = 0;

    while (read < input_size) {
        const std::uint8_t code = input[read++];
        if (code == 0 || read + code - 1u > input_size) {
            return 0;
        }
        for (std::uint8_t i = 1; i < code; ++i) {
            if (write >= output_size) {
                return 0;
            }
            output[write++] = input[read++];
        }
        if (code != 0xFFu && read < input_size) {
            if (write >= output_size) {
                return 0;
            }
            output[write++] = 0;
        }
    }
    return write;
}

}  // namespace hexapod::protocol
