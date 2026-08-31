#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "command/motion_command.h"
#include "config/robot_config.h"

namespace hexapod::protocol {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaxPayloadSize = 96;
constexpr std::size_t kMaxFrameSize = kMaxPayloadSize + 16;

enum class MessageType : std::uint8_t {
    CmdVelocity = 1,
    CmdBodyPose = 2,
    CmdMode = 3,
    CmdGait = 4,
    Ping = 5,
    State = 64,
    Contacts = 65,
    JointState = 66,
    Diagnostics = 67,
};

struct Packet {
    MessageType type = MessageType::Ping;
    std::uint32_t sequence = 0;
    std::array<std::uint8_t, kMaxPayloadSize> payload{};
    std::size_t payload_size = 0;
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
};

std::size_t encodePacket(const Packet& packet, std::uint8_t* output, std::size_t output_size);
bool decodePacket(const std::uint8_t* data, std::size_t size, Packet* packet);
bool makeVelocityCommandPacket(const MotionCommand& command, Packet* packet);
bool motionCommandFromPacket(const Packet& packet, MotionCommand* command);

}  // namespace hexapod::protocol
