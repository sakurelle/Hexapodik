#include "drivers/uart_transport.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#include "hardware/uart.h"
#endif

namespace hexapod {

bool UartTransport::init(std::uint32_t baud) {
#if defined(PICO_ON_DEVICE)
    uart_init(uart0, baud);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
#else
    (void)baud;
#endif
    return true;
}

int UartTransport::readByte() {
#if defined(PICO_ON_DEVICE)
    if (!uart_is_readable(uart0)) {
        return -1;
    }
    return uart_getc(uart0);
#else
    return -1;
#endif
}

bool UartTransport::write(const std::uint8_t* data, std::size_t size) {
#if defined(PICO_ON_DEVICE)
    if (data == nullptr && size != 0) {
        return false;
    }
    for (std::size_t i = 0; i < size; ++i) {
        uart_putc_raw(uart0, data[i]);
    }
#else
    (void)data;
    (void)size;
#endif
    return true;
}

}  // namespace hexapod
