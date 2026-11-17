#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hexapod {

enum class LegId : std::uint8_t { FL = 0, ML, RL, RR, MR, FR, Count };
enum class JointId : std::uint8_t { Tibia = 0, Femur, Coxa, Count };

constexpr std::size_t kLegCount = static_cast<std::size_t>(LegId::Count);
constexpr std::size_t kJointsPerLeg = static_cast<std::size_t>(JointId::Count);
constexpr std::size_t kServoCount = kLegCount * kJointsPerLeg;

constexpr std::uint8_t kUartTxGpio = 0;
constexpr std::uint8_t kUartRxGpio = 1;
constexpr std::uint32_t kUartBaud = 115200;

// UART/PWM channel order: FL, ML, RL, RR, MR, FR; Tibia, Femur, Coxa.
constexpr std::array<std::uint8_t, kServoCount> kServoGpios = {
    2, 3, 4,
    5, 6, 7,
    8, 9, 10,
    11, 12, 13,
    14, 15, 16,
    17, 18, 19,
};

constexpr std::array<std::uint8_t, kLegCount> kContactGpios = {
    20, 21, 22, 26, 27, 28,
};

constexpr std::uint8_t kFirstServoGpio = 2;
constexpr std::uint8_t kServoGpioSpan = 18;
constexpr std::uint32_t kServoFrameUs = 20000;
constexpr std::uint16_t kMinPulseUs = 1000;
constexpr std::uint16_t kMaxPulseUs = 2000;
constexpr std::uint32_t kCommandTimeoutUs = 250000;
constexpr std::uint32_t kContactDebounceUs = 10000;

constexpr bool servoGpiosAreContiguous() {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (kServoGpios[i] != kFirstServoGpio + i) {
            return false;
        }
    }
    return true;
}

static_assert(kServoCount == 18, "Expected 18 servo channels");
static_assert(kServoGpioSpan == 18, "Expected GP2..GP19 servo span");
static_assert(servoGpiosAreContiguous(), "Servo outputs must be contiguous GP2..GP19");

constexpr std::size_t channelIndex(LegId leg, JointId joint) {
    return static_cast<std::size_t>(leg) * kJointsPerLeg + static_cast<std::size_t>(joint);
}

}  // namespace hexapod
