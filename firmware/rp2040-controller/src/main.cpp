#include <cstdint>
#include <cstdio>

#include "command/uart_command_source.h"
#include "config/robot_config.h"
#include "core/locomotion_system.h"
#include "diagnostics/telemetry.h"
#include "drivers/contact_sensors.h"
#include "drivers/rc_input.h"
#include "drivers/servo_controller.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#endif

namespace hexapod {
namespace {

RcInput* g_rc_input = nullptr;

#if defined(PICO_ON_DEVICE)
void gpioIrq(unsigned int gpio, std::uint32_t events) {
    if (g_rc_input == nullptr) {
        return;
    }
    const std::uint64_t now_us = time_us_64();
    if ((events & GPIO_IRQ_EDGE_RISE) != 0) {
        g_rc_input->onEdge(static_cast<std::uint8_t>(gpio), true, now_us);
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0) {
        g_rc_input->onEdge(static_cast<std::uint8_t>(gpio), false, now_us);
    }
}
#endif

void initSelectedCommandSource(RcInput& rc, UartCommandSource& uart) {
    if constexpr (kCommandSourceMode == CommandSourceMode::RcPwm) {
        rc.init();
        g_rc_input = &rc;
#if defined(PICO_ON_DEVICE)
        gpio_set_irq_callback(&gpioIrq);
        irq_set_enabled(IO_IRQ_BANK0, true);
        gpio_set_irq_enabled(kRcForwardGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(kRcYawGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
#endif
    } else {
        uart.init();
    }
}

ICommandSource& selectedCommandSource(RcInput& rc, UartCommandSource& uart) {
    if constexpr (kCommandSourceMode == CommandSourceMode::RcPwm) {
        (void)uart;
        return rc;
    } else {
        (void)rc;
        return uart;
    }
}

}  // namespace
}  // namespace hexapod

int main() {
    using namespace hexapod;

#if defined(PICO_ON_DEVICE)
    stdio_init_all();
#endif

    RcInput rc;
    UartTransport uart_transport;
    UartCommandSource uart(uart_transport);
    ContactSensors contacts;
    ServoController servos;

    initSelectedCommandSource(rc, uart);
    contacts.init();
    servos.init();

    LocomotionSystem locomotion(selectedCommandSource(rc, uart), contacts, servos, kCommandSourceMode);
    locomotion.reset();

#if defined(PICO_ON_DEVICE)
    std::uint64_t last_us = time_us_64();
    std::uint64_t next_diag_us = last_us + 100000;
    std::uint64_t next_tick_us = last_us;
#else
    std::uint64_t last_us = 0;
    std::uint64_t next_diag_us = 100000;
    std::uint64_t next_tick_us = 0;
#endif

    while (true) {
#if defined(PICO_ON_DEVICE)
        const std::uint64_t now_us = time_us_64();
        if (now_us < next_tick_us) {
            tight_loop_contents();
            continue;
        }
#else
        const std::uint64_t now_us = next_tick_us;
#endif
        const float dt_s = static_cast<float>(now_us - last_us) * 1.0e-6f;
        last_us = now_us;
        next_tick_us += static_cast<std::uint64_t>(kControlPeriodS * 1000000.0f);

        const Telemetry& telemetry = locomotion.update(now_us, dt_s > 0.0f ? dt_s : kControlPeriodS);

        if (now_us >= next_diag_us) {
            char line[512]{};
            formatTelemetryLine(telemetry, line, sizeof(line));
            std::printf("%s\n", line);
            locomotion.resetJointVelocityDiagnostics();
            next_diag_us = now_us + 100000;
        }

#if !defined(PICO_ON_DEVICE)
        break;
#endif
    }

    return 0;
}
