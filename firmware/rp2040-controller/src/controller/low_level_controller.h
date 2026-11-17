#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "protocol/protocol.h"

namespace hexapod {

class IPwmOutput {
public:
    virtual ~IPwmOutput() = default;
    virtual bool enable(const std::array<std::uint16_t, kServoCount>& pulses_us) = 0;
    virtual bool update(const std::array<std::uint16_t, kServoCount>& pulses_us) = 0;
    virtual void disable() = 0;
    virtual bool enabled() const = 0;
};

struct ProcessResult {
    bool send_status = false;
    std::uint32_t status_sequence = 0;
};

class LowLevelController {
public:
    explicit LowLevelController(IPwmOutput& pwm) : pwm_(pwm) {}

    void reset();
    ProcessResult processPacket(const protocol::Packet& packet, std::uint64_t now_us);
    void tick(std::uint64_t now_us);
    protocol::StatusPayload status(std::uint8_t input_mask, std::uint32_t dropped_frames) const;

    protocol::ControllerState state() const { return state_; }
    protocol::ErrorCode error() const { return error_; }
    std::uint32_t lastCommandSequence() const { return last_command_sequence_; }
    const std::array<std::uint16_t, kServoCount>& lastPulses() const { return last_pulses_us_; }

private:
    static bool sequenceIsNewer(std::uint32_t candidate, std::uint32_t previous);
    static bool pulsesValid(const std::array<std::uint16_t, kServoCount>& pulses_us);
    bool decodePwm(const protocol::Packet& packet,
                   std::array<std::uint16_t, kServoCount>* pulses_us);
    void handleArm(const protocol::Packet& packet, std::uint64_t now_us);
    void handleSetPwm(const protocol::Packet& packet, std::uint64_t now_us);
    void handleDisarm(const protocol::Packet& packet);

    IPwmOutput& pwm_;
    protocol::ControllerState state_ = protocol::ControllerState::Disarmed;
    protocol::ErrorCode error_ = protocol::ErrorCode::None;
    std::array<std::uint16_t, kServoCount> last_pulses_us_{};
    std::uint32_t last_command_sequence_ = 0;
    std::uint64_t last_motion_command_us_ = 0;
    bool have_motion_command_ = false;
};

}  // namespace hexapod
