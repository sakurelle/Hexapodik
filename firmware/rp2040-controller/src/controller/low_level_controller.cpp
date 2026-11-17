#include "controller/low_level_controller.h"

namespace hexapod {

void LowLevelController::reset() {
    pwm_.disable();
    state_ = protocol::ControllerState::Disarmed;
    error_ = protocol::ErrorCode::None;
    last_pulses_us_.fill(0);
    last_command_sequence_ = 0;
    last_motion_command_us_ = 0;
    have_motion_command_ = false;
}

ProcessResult LowLevelController::processPacket(const protocol::Packet& packet, std::uint64_t now_us) {
    ProcessResult result;
    switch (packet.type) {
        case protocol::MessageType::Arm:
            handleArm(packet, now_us);
            break;
        case protocol::MessageType::SetPwm:
            handleSetPwm(packet, now_us);
            break;
        case protocol::MessageType::Disarm:
            handleDisarm(packet);
            break;
        case protocol::MessageType::GetStatus:
            if (packet.payload_size != 0) {
                error_ = protocol::ErrorCode::InvalidLength;
            }
            result.send_status = true;
            result.status_sequence = packet.sequence;
            break;
        case protocol::MessageType::Status:
        default:
            error_ = protocol::ErrorCode::UnsupportedMessage;
            break;
    }
    return result;
}

void LowLevelController::tick(std::uint64_t now_us) {
    if (state_ != protocol::ControllerState::Armed || !have_motion_command_) {
        return;
    }
    if (now_us >= last_motion_command_us_ && now_us - last_motion_command_us_ >= kCommandTimeoutUs) {
        state_ = protocol::ControllerState::CommunicationFault;
        error_ = protocol::ErrorCode::CommandTimeout;
        // Intentionally keep PWM enabled and hold the last applied frame.
    }
}

protocol::StatusPayload LowLevelController::status(std::uint8_t input_mask,
                                                   std::uint32_t dropped_frames) const {
    protocol::StatusPayload value;
    value.state = state_;
    value.error = error_;
    value.outputs_enabled = pwm_.enabled();
    value.input_mask = static_cast<std::uint8_t>(input_mask & 0x3Fu);
    value.last_command_sequence = last_command_sequence_;
    value.dropped_frames = dropped_frames;
    return value;
}

bool LowLevelController::sequenceIsNewer(std::uint32_t candidate, std::uint32_t previous) {
    return static_cast<std::int32_t>(candidate - previous) > 0;
}

bool LowLevelController::pulsesValid(const std::array<std::uint16_t, kServoCount>& pulses_us) {
    for (const std::uint16_t pulse : pulses_us) {
        if (pulse < kMinPulseUs || pulse > kMaxPulseUs) {
            return false;
        }
    }
    return true;
}

bool LowLevelController::decodePwm(const protocol::Packet& packet,
                                   std::array<std::uint16_t, kServoCount>* pulses_us) {
    if (packet.payload_size != protocol::kPwmPayloadSize) {
        error_ = protocol::ErrorCode::InvalidLength;
        return false;
    }
    if (!protocol::pwmCommandFromPacket(packet, pulses_us)) {
        error_ = protocol::ErrorCode::InvalidLength;
        return false;
    }
    if (!pulsesValid(*pulses_us)) {
        error_ = protocol::ErrorCode::InvalidPwm;
        return false;
    }
    return true;
}

void LowLevelController::handleArm(const protocol::Packet& packet, std::uint64_t now_us) {
    if (state_ != protocol::ControllerState::Disarmed) {
        error_ = protocol::ErrorCode::RequiresDisarm;
        return;
    }

    std::array<std::uint16_t, kServoCount> pulses{};
    if (!decodePwm(packet, &pulses)) {
        return;
    }
    if (!pwm_.enable(pulses)) {
        state_ = protocol::ControllerState::OutputFault;
        error_ = protocol::ErrorCode::OutputFailure;
        return;
    }

    last_pulses_us_ = pulses;
    last_command_sequence_ = packet.sequence;
    last_motion_command_us_ = now_us;
    have_motion_command_ = true;
    state_ = protocol::ControllerState::Armed;
    error_ = protocol::ErrorCode::None;
}

void LowLevelController::handleSetPwm(const protocol::Packet& packet, std::uint64_t now_us) {
    if (state_ != protocol::ControllerState::Armed) {
        error_ = state_ == protocol::ControllerState::Disarmed
                     ? protocol::ErrorCode::NotArmed
                     : protocol::ErrorCode::RequiresDisarm;
        return;
    }
    if (!sequenceIsNewer(packet.sequence, last_command_sequence_)) {
        error_ = protocol::ErrorCode::StaleSequence;
        return;
    }

    std::array<std::uint16_t, kServoCount> pulses{};
    if (!decodePwm(packet, &pulses)) {
        return;
    }
    if (!pwm_.update(pulses)) {
        state_ = protocol::ControllerState::OutputFault;
        error_ = protocol::ErrorCode::OutputFailure;
        return;
    }

    last_pulses_us_ = pulses;
    last_command_sequence_ = packet.sequence;
    last_motion_command_us_ = now_us;
    have_motion_command_ = true;
    error_ = protocol::ErrorCode::None;
}

void LowLevelController::handleDisarm(const protocol::Packet& packet) {
    if (packet.payload_size != 0) {
        error_ = protocol::ErrorCode::InvalidLength;
        return;
    }
    pwm_.disable();
    state_ = protocol::ControllerState::Disarmed;
    error_ = protocol::ErrorCode::None;
    last_motion_command_us_ = 0;
    have_motion_command_ = false;
}

}  // namespace hexapod
