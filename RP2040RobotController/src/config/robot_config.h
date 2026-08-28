#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hexapod {

constexpr float kPi = 3.14159265358979323846f;

constexpr float degToRad(float deg) { return deg * kPi / 180.0f; }
constexpr float radToDeg(float rad) { return rad * 180.0f / kPi; }

enum class LegId : std::uint8_t { FL = 0, ML, RL, RR, MR, FR, Count };
enum class JointId : std::uint8_t { Coxa = 0, Femur, Tibia, Count };
enum class FaultCode : std::uint8_t { None = 0, NoGround, ContactStuck, IkInvalid, SupportLost };
enum class LocomotionMode : std::uint8_t { Idle = 0, Running, Stopping, Fault };
enum class ContactMode : std::uint8_t { Disabled = 0, TouchdownOnly, FullTerrain };
enum class StopReason : std::uint8_t { None = 0, SensorStuckHigh, NoGround, EarlyCollision, SupportLost };
enum class SensorHealth : std::uint8_t { Ok = 0, SuspectStuckHigh, SuspectStuckLow, Unhealthy };

constexpr std::size_t kLegCount = static_cast<std::size_t>(LegId::Count);
constexpr std::size_t kJointsPerLeg = static_cast<std::size_t>(JointId::Count);
constexpr std::size_t kServoCount = kLegCount * kJointsPerLeg;

constexpr float kControlLoopHz = 200.0f;
constexpr float kControlPeriodS = 1.0f / kControlLoopHz;
constexpr std::uint32_t kServoFrameUs = 20000;

constexpr float kLcoxaMm = 44.0f;
constexpr float kLfemurMm = 80.0f;
constexpr float kLtibiaMm = 125.24f;

constexpr float kCoxaModelZeroRad = degToRad(0.0f);
constexpr float kFemurModelZeroRad = degToRad(35.27f);
constexpr float kTibiaModelZeroRad = degToRad(-104.65f);

constexpr float kCoxaLimitMinRad = degToRad(-60.0f);
constexpr float kCoxaLimitMaxRad = degToRad(60.0f);
constexpr float kFemurLimitMinRad = degToRad(-75.0f);
constexpr float kFemurLimitMaxRad = degToRad(75.0f);
constexpr float kTibiaLimitMinRad = degToRad(-75.0f);
constexpr float kTibiaLimitMaxRad = degToRad(75.0f);
constexpr float kJointSafetyMarginRad = degToRad(5.0f);
constexpr float kSingularitySinMin = 0.15f;

constexpr std::uint8_t kRcForwardGpio = 0;
constexpr std::uint8_t kRcYawGpio = 1;
constexpr std::array<std::uint8_t, kServoCount> kServoGpios = {
    2, 3, 4,
    5, 6, 7,
    8, 9, 10,
    11, 12, 13,
    14, 15, 16,
    17, 18, 19,
};
constexpr std::array<std::uint8_t, kLegCount> kContactGpios = {
    28, 27, 26, 22, 21, 20,
};
constexpr std::uint8_t kFirstServoGpio = 2;
constexpr std::uint8_t kServoGpioSpan = 18;

constexpr bool servoGpiosAreContiguous() {
    if (kServoCount != kServoGpioSpan) {
        return false;
    }
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (kServoGpios[i] != kFirstServoGpio + i) {
            return false;
        }
    }
    return true;
}

static_assert(kServoCount == 18, "Expected exactly 18 servo channels");
static_assert(kServoGpioSpan == 18, "Expected GP2..GP19 servo GPIO span");
static_assert(servoGpiosAreContiguous(), "Servo GPIOs must be contiguous GP2..GP19");

constexpr std::uint16_t kRcMinUs = 1000;
constexpr std::uint16_t kRcCenterUs = 1500;
constexpr std::uint16_t kRcMaxUs = 2000;
constexpr std::uint16_t kRcDeadbandUs = 50;
constexpr std::uint32_t kRcFailsafeUs = 100000;
constexpr float kMaxVxMmS = 80.0f;
constexpr float kMaxVyMmS = 0.0f;
constexpr float kMaxYawRadS = 0.45f;
constexpr float kMaxVxAccelMmS2 = 200.0f;
constexpr float kMaxVyAccelMmS2 = 200.0f;
constexpr float kMaxYawAccelRadS2 = 1.5f;

constexpr float kStanceDuty = 0.65f;
constexpr float kMinCycleHz = 0.6f;
constexpr float kMaxCycleHz = 1.5f;
constexpr float kSwingHeightMm = 30.0f;
constexpr float kGroundSearchMm = 15.0f;
constexpr float kContactDebounceMs = 12.0f;
constexpr float kStuckContactLiftMm = 8.0f;
constexpr float kSupportMarginMm = 5.0f;
constexpr std::uint32_t kContactPressDebounceMs = 10;
constexpr std::uint32_t kContactReleaseDebounceMs = 15;
constexpr float kContactReleaseCheckHeightMm = 10.0f;
constexpr std::uint32_t kContactReleaseTimeoutMs = 150;
constexpr float kGroundSearchSpeedMmS = 15.0f;
constexpr ContactMode kContactMode = ContactMode::TouchdownOnly;

constexpr bool kServoDriverTestMode = false;
constexpr bool kServoSequentialTestMode = false;
constexpr bool kContactDiagnosticMode = false;
constexpr std::uint32_t kServoSequentialStepMs = 600;

const char* legName(LegId leg);
const char* faultName(FaultCode fault);
const char* locomotionModeName(LocomotionMode mode);
const char* contactModeName(ContactMode mode);
const char* stopReasonName(StopReason reason);
const char* sensorHealthName(SensorHealth health);

constexpr std::size_t indexOf(LegId leg) { return static_cast<std::size_t>(leg); }
constexpr std::size_t indexOf(JointId joint) { return static_cast<std::size_t>(joint); }
constexpr std::size_t servoIndex(LegId leg, JointId joint) {
    return indexOf(leg) * kJointsPerLeg + indexOf(joint);
}

}  // namespace hexapod
