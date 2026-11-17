#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "config/robot_config.h"

namespace hexapod::protocol {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kPwmPayloadSize = kServoCount * sizeof(std::uint16_t);
constexpr std::size_t kStatusPayloadSize = 12;
constexpr std::size_t kMaxPayloadSize = 64;
constexpr std::size_t kMaxFrameSize = kMaxPayloadSize + 16;

enum class MessageType : std::uint8_t {
    Arm = 0x01,
    SetPwm = 0x02,
    Disarm = 0x03,
    GetStatus = 0x04,
    Status = 0x80,
};

enum class ControllerState : std::uint8_t {
    Disarmed = 0,
    Armed = 1,
    CommunicationFault = 2,
    OutputFault = 3,
};

enum class ErrorCode : std::uint8_t {
    None = 0,
    InvalidLength = 1,
    InvalidPwm = 2,
    StaleSequence = 3,
    NotArmed = 4,
    RequiresDisarm = 5,
    CommandTimeout = 6,
    OutputFailure = 7,
    UnsupportedMessage = 8,
};

struct Packet {
    MessageType type = MessageType::GetStatus;
    std::uint32_t sequence = 0;
    std::array<std::uint8_t, kMaxPayloadSize> payload{};
    std::size_t payload_size = 0;
};

struct StatusPayload {
    ControllerState state = ControllerState::Disarmed;
    ErrorCode error = ErrorCode::None;
    bool outputs_enabled = false;
    std::uint8_t input_mask = 0;
    std::uint32_t last_command_sequence = 0;
    std::uint32_t dropped_frames = 0;
};

class Parser {
public:
    bool push(std::uint8_t byte, Packet* out_packet);
    void reset();
    std::uint32_t droppedFrames() const { return dropped_frames_; }

private:
    std::array<std::uint8_t, kMaxFrameSize> frame_{};
    std::size_t frame_size_ = 0;
    std::uint32_t dropped_frames_ = 0;
    bool discarding_ = false;
};

std::size_t encodePacket(const Packet& packet, std::uint8_t* output, std::size_t output_size);
bool decodePacket(const std::uint8_t* data, std::size_t size, Packet* packet);

bool makePwmCommandPacket(MessageType type,
                          std::uint32_t sequence,
                          const std::array<std::uint16_t, kServoCount>& pulses_us,
                          Packet* packet);
bool pwmCommandFromPacket(const Packet& packet, std::array<std::uint16_t, kServoCount>* pulses_us);
bool makeStatusPacket(std::uint32_t sequence, const StatusPayload& status, Packet* packet);
bool statusFromPacket(const Packet& packet, StatusPayload* status);

}  // namespace hexapod::protocol
