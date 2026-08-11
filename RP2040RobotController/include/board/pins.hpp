#pragma once

#include <cstdint>

namespace board {

constexpr uint8_t UART0_TX_GPIO = 0;
constexpr uint8_t UART0_RX_GPIO = 1;
constexpr uint8_t WS2812_GPIO = 16;

constexpr uint8_t PIO_GROUP_A_FIRST_GPIO = 2;
constexpr uint8_t PIO_GROUP_A_CHANNELS = 14;
constexpr uint8_t PIO_GROUP_B_FIRST_GPIO = 26;
constexpr uint8_t PIO_GROUP_B_CHANNELS = 4;

constexpr uint32_t UART_BAUD = 115200;

} // namespace board
