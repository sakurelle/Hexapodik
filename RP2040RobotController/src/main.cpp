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

char legPhaseChar(LegPhase phase) {
    switch (phase) {
        case LegPhase::Stance: return 'S';
        case LegPhase::Lift: return 'L';
        case LegPhase::Transfer: return 'T';
        case LegPhase::Descend: return 'D';
        case LegPhase::GroundSearch: return 'G';
    }
    return '?';
}

std::array<char, kLegCount + 1> boolMaskString(const std::array<bool, kLegCount>& values) {
    std::array<char, kLegCount + 1> text{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        text[i] = values[i] ? '1' : '0';
    }
    text[kLegCount] = '\0';
    return text;
}

std::array<char, kLegCount + 1> rawMaskString(const std::array<ContactState, kLegCount>& states) {
    std::array<char, kLegCount + 1> text{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        text[i] = states[i].raw ? '1' : '0';
    }
    text[kLegCount] = '\0';
    return text;
}

std::array<char, kLegCount + 1> stableMaskString(const std::array<ContactState, kLegCount>& states) {
    std::array<char, kLegCount + 1> text{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        text[i] = states[i].stable ? '1' : '0';
    }
    text[kLegCount] = '\0';
    return text;
}

std::array<char, kLegCount + 1> eventString(const std::array<ContactState, kLegCount>& states) {
    std::array<char, kLegCount + 1> text{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (states[i].pressed_event && states[i].released_event) {
            text[i] = 'B';
        } else if (states[i].pressed_event) {
            text[i] = 'P';
        } else if (states[i].released_event) {
            text[i] = 'R';
        } else {
            text[i] = '-';
        }
    }
    text[kLegCount] = '\0';
    return text;
}

std::array<char, kLegCount + 1> legPhaseString(const std::array<LegPhase, kLegCount>& phases) {
    std::array<char, kLegCount + 1> text{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        text[i] = legPhaseChar(phases[i]);
    }
    text[kLegCount] = '\0';
    return text;
}

std::array<char, 96> healthString(const std::array<SensorHealth, kLegCount>& health) {
    std::array<char, 96> text{};
    std::snprintf(text.data(),
                  text.size(),
                  "%s,%s,%s,%s,%s,%s",
                  sensorHealthName(health[0]),
                  sensorHealthName(health[1]),
                  sensorHealthName(health[2]),
                  sensorHealthName(health[3]),
                  sensorHealthName(health[4]),
                  sensorHealthName(health[5]));
    return text;
}

std::array<char, 96> contactHealthString(const std::array<ContactState, kLegCount>& states) {
    std::array<char, 96> text{};
    std::snprintf(text.data(),
                  text.size(),
                  "%s,%s,%s,%s,%s,%s",
                  sensorHealthName(states[0].health),
                  sensorHealthName(states[1].health),
                  sensorHealthName(states[2].health),
                  sensorHealthName(states[3].health),
                  sensorHealthName(states[4].health),
                  sensorHealthName(states[5].health));
    return text;
}

std::array<char, kLegCount + 1> releasedString(const std::array<bool, kLegCount>& released) {
    return boolMaskString(released);
}

void printDiagnostics(const LocomotionStepResult& step,
                      BodyCommand command,
                      const RcInput& rc,
                      std::uint64_t now_us,
                      const std::array<ContactState, kLegCount>& contacts) {
    std::uint32_t contact_mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i].stable) {
            contact_mask |= (1u << i);
        }
    }
    const RcInputSnapshot rc_state = rc.snapshot(now_us);
    const char* reason = step.waiting_for_support ? "WAITING_FOR_SUPPORT" : "OK";
    const auto raw_text = rawMaskString(contacts);
    const auto stable_text = stableMaskString(contacts);
    const auto event_text = eventString(contacts);
    const auto swing_text = boolMaskString(step.swing);
    const auto leg_phase_text = legPhaseString(step.leg_phase);
    const auto released_text = releasedString(step.released_this_swing);
    const auto health_text = healthString(step.sensor_health);
    std::printf("mode=%s phase=%.3f rc=%u/%u fresh=%u/%u failsafe=%u vx=%.1f yaw=%.3f contacts raw=%s stable=%s event=%s contact=0x%02lx swing=%s leg_phase=%s released=%s health=%s ground_z=%.1f/%.1f/%.1f/%.1f/%.1f/%.1f contact_mode=%s support_ok=%u waiting_for_support=%u reason=%s stop_reason=%s stop_leg=%s scale=%.2f fault=%s fault_leg=%s ik_errors=%lu valid=%u\n",
                locomotionModeName(step.mode),
                step.phase,
                rc_state.forward_pulse_us,
                rc_state.yaw_pulse_us,
                rc_state.forward_fresh ? 1u : 0u,
                rc_state.yaw_fresh ? 1u : 0u,
                rc_state.failsafe ? 1u : 0u,
                command.vx_mm_s,
                command.yaw_rad_s,
                raw_text.data(),
                stable_text.data(),
                event_text.data(),
                static_cast<unsigned long>(contact_mask),
                swing_text.data(),
                leg_phase_text.data(),
                released_text.data(),
                health_text.data(),
                step.local_ground_z_mm[0],
                step.local_ground_z_mm[1],
                step.local_ground_z_mm[2],
                step.local_ground_z_mm[3],
                step.local_ground_z_mm[4],
                step.local_ground_z_mm[5],
                contactModeName(kContactMode),
                step.support_ok ? 1u : 0u,
                step.waiting_for_support ? 1u : 0u,
                reason,
                stopReasonName(step.stop_reason),
                legName(step.stop_leg),
                step.workspace_scale,
                faultName(step.fault),
                legName(step.fault_leg),
                static_cast<unsigned long>(step.ik_error_counter),
                step.joints_valid ? 1u : 0u);
}

void printContactDiagnostics(const std::array<ContactState, kLegCount>& contacts) {
    const auto raw_text = rawMaskString(contacts);
    const auto stable_text = stableMaskString(contacts);
    const auto event_text = eventString(contacts);
    const auto health_text = contactHealthString(contacts);
    std::printf("contact_diag=1 contacts raw=%s stable=%s event=%s health=%s gpio=FL:%u ML:%u RL:%u RR:%u MR:%u FR:%u contact_mode=%s\n",
                raw_text.data(),
                stable_text.data(),
                event_text.data(),
                health_text.data(),
                kContactGpios[indexOf(LegId::FL)],
                kContactGpios[indexOf(LegId::ML)],
                kContactGpios[indexOf(LegId::RL)],
                kContactGpios[indexOf(LegId::RR)],
                kContactGpios[indexOf(LegId::MR)],
                kContactGpios[indexOf(LegId::FR)],
                contactModeName(kContactMode));
}

std::array<std::uint16_t, kServoCount> centerServoPulses() {
    std::array<std::uint16_t, kServoCount> pulses{};
    pulses.fill(1500);
    return pulses;
}

std::array<std::uint16_t, kServoCount> sequentialServoTestPulses(std::uint64_t now_us) {
    std::array<std::uint16_t, kServoCount> pulses = centerServoPulses();
    if constexpr (!kServoSequentialTestMode) {
        (void)now_us;
        return pulses;
    }

    const std::uint32_t step = static_cast<std::uint32_t>(now_us / (kServoSequentialStepMs * 1000ull));
    const std::size_t channel = (step / 3u) % kServoCount;
    const std::uint32_t phase = step % 3u;
    pulses[channel] = (phase == 1u) ? 1550 : 1500;
    return pulses;
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
    gpio_set_irq_callback(&gpioIrq);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(kRcForwardGpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
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

        if constexpr (kServoDriverTestMode) {
            servos.submitPulses(sequentialServoTestPulses(now_us));
            if (now_us >= next_diag_us) {
                std::printf("servo_test_mode=1 sequential=%u pulse_us=1500 frame_us=%lu\n",
                            kServoSequentialTestMode ? 1u : 0u,
                            static_cast<unsigned long>(kServoFrameUs));
                next_diag_us = now_us + 100000;
            }
#if !defined(PICO_ON_DEVICE)
            break;
#else
            continue;
#endif
        }

        contacts.update(dt_s > 0.0f ? dt_s : kControlPeriodS);
        const auto contact_states = contacts.states();

        if constexpr (kContactDiagnosticMode) {
            servos.submitPulses(centerServoPulses());
            if (now_us >= next_diag_us) {
                printContactDiagnostics(contact_states);
                next_diag_us = now_us + 100000;
            }
#if !defined(PICO_ON_DEVICE)
            break;
#else
            continue;
#endif
        }

        const BodyCommand requested = rc.readCommand(now_us);
        const LocomotionStepResult step = locomotion.update(requested, contact_states, dt_s > 0.0f ? dt_s : kControlPeriodS);

        std::array<std::uint16_t, kServoCount> pulses{};
        if (mapRobotAnglesToPulses(step.joints, &pulses)) {
            servos.submitPulses(pulses);
        }

        if (now_us >= next_diag_us) {
            printDiagnostics(step, locomotion.filteredCommand(), rc, now_us, contact_states);
            next_diag_us = now_us + 100000;
        }

#if !defined(PICO_ON_DEVICE)
        break;
#endif
    }

    return 0;
}
