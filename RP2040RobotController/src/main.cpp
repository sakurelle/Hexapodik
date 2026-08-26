#include <array>
#include <cstdint>
#include <cstdio>

#include "config/servo_config.h"
#include "drivers/contact_sensors.h"
#include "drivers/rc_input.h"
#include "drivers/servo_controller.h"
#include "locomotion/locomotion_controller.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/gpio.h"
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
    const bool rising = (events & GPIO_IRQ_EDGE_RISE) != 0;
    g_rc_input->onEdge(static_cast<std::uint8_t>(gpio), rising, time_us_64());
}
#endif

void printDiagnostics(const LocomotionStepResult& step,
                      BodyCommand command,
                      const RcInput& rc,
                      const std::array<bool, kLegCount>& contacts) {
    std::uint32_t contact_mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i]) {
            contact_mask |= (1u << i);
        }
    }
    std::printf("mode=%u vx=%.1f vy=%.1f yaw=%.3f phase=%.3f rc=%u/%u contact=0x%02lx scale=%.2f fault=%s ik_errors=%lu valid=%u\n",
                static_cast<unsigned>(step.mode),
                command.vx_mm_s,
                command.vy_mm_s,
                command.yaw_rad_s,
                step.phase,
                rc.pulseForwardUs(),
                rc.pulseYawUs(),
                static_cast<unsigned long>(contact_mask),
                step.workspace_scale,
                faultName(step.fault),
                static_cast<unsigned long>(step.ik_error_counter),
                step.joints_valid ? 1u : 0u);
}

}  // namespace
}  // namespace hexapod

int main() {
    using namespace hexapod;

#if defined(PICO_ON_DEVICE)
    stdio_init_all();
#endif

    RcInput rc;
    ContactSensors contacts;
    ServoController servos;
    LocomotionController locomotion;

    rc.init();
    contacts.init();
    servos.init();
    locomotion.reset();
    g_rc_input = &rc;

#if defined(PICO_ON_DEVICE)
    gpio_set_irq_enabled_with_callback(kRcForwardGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpioIrq);
    gpio_set_irq_enabled(kRcYawGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
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

        const BodyCommand requested = rc.readCommand(now_us);
        contacts.update(dt_s > 0.0f ? dt_s : kControlPeriodS);
        const auto contact_mask = contacts.contacts();
        const LocomotionStepResult step = locomotion.update(requested, contact_mask, dt_s > 0.0f ? dt_s : kControlPeriodS);

        std::array<std::uint16_t, kServoCount> pulses{};
        if (mapRobotAnglesToPulses(step.joints, &pulses)) {
            servos.submitPulses(pulses);
        }

        if (now_us >= next_diag_us) {
            printDiagnostics(step, locomotion.filteredCommand(), rc, contact_mask);
            next_diag_us = now_us + 100000;
        }

#if !defined(PICO_ON_DEVICE)
        break;
#endif
    }

    return 0;
}
