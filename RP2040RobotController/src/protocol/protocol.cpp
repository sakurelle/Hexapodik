#include "protocol/protocol.h"

#include <cstring>

#include "protocol/cobs.h"
#include "protocol/crc.h"

namespace hexapod::protocol {
namespace {

constexpr std::size_t kHeaderSize = 7;
constexpr std::size_t kCrcSize = 2;

void writeU32Le(std::uint8_t* out, std::uint32_t value) {
    out[0] = static_cast<std::uint8_t>(value & 0xFFu);
    out[1] = static_cast<std::uint8_t>((value >> 8u) & 0xFFu);
    out[2] = static_cast<std::uint8_t>((value >> 16u) & 0xFFu);
    out[3] = static_cast<std::uint8_t>((value >> 24u) & 0xFFu);
}

std::uint32_t readU32Le(const std::uint8_t* in) {
    return static_cast<std::uint32_t>(in[0]) |
           (static_cast<std::uint32_t>(in[1]) << 8u) |
           (static_cast<std::uint32_t>(in[2]) << 16u) |
           (static_cast<std::uint32_t>(in[3]) << 24u);
}

void writeFloat(std::uint8_t* out, float value) {
    static_assert(sizeof(float) == 4, "Expected 32-bit float");
    std::memcpy(out, &value, sizeof(float));
}

float readFloat(const std::uint8_t* in) {
    float value = 0.0f;
    std::memcpy(&value, in, sizeof(float));
    return value;
}

}  // namespace

void Parser::reset() {
    frame_size_ = 0;
}

bool Parser::push(std::uint8_t byte, Packet* out_packet) {
    if (byte == 0) {
        if (frame_size_ == 0) {
            return false;
        }
        const bool ok = decodePacket(frame_.data(), frame_size_, out_packet);
        if (!ok) {
            ++dropped_frames_;
        }
        frame_size_ = 0;
        return ok;
    }

    if (frame_size_ >= frame_.size()) {
        ++dropped_frames_;
        frame_size_ = 0;
        return false;
    }
    frame_[frame_size_++] = byte;
    return false;
}

std::size_t encodePacket(const Packet& packet, std::uint8_t* output, std::size_t output_size) {
    if (packet.payload_size > kMaxPayloadSize || output_size < 2) {
        return 0;
    }

    std::array<std::uint8_t, kMaxFrameSize> raw{};
    const std::size_t raw_size = kHeaderSize + packet.payload_size + kCrcSize;
    if (raw_size > raw.size()) {
        return 0;
    }
    raw[0] = kProtocolVersion;
    raw[1] = static_cast<std::uint8_t>(packet.type);
    writeU32Le(&raw[2], packet.sequence);
    raw[6] = static_cast<std::uint8_t>(packet.payload_size);
    for (std::size_t i = 0; i < packet.payload_size; ++i) {
        raw[kHeaderSize + i] = packet.payload[i];
    }
    const std::uint16_t crc = crc16Ccitt(raw.data(), kHeaderSize + packet.payload_size);
    raw[kHeaderSize + packet.payload_size] = static_cast<std::uint8_t>(crc & 0xFFu);
    raw[kHeaderSize + packet.payload_size + 1] = static_cast<std::uint8_t>(crc >> 8u);

    const std::size_t encoded = cobsEncode(raw.data(), raw_size, output, output_size - 1);
    if (encoded == 0) {
        return 0;
    }
    output[encoded] = 0;
    return encoded + 1;
}

bool decodePacket(const std::uint8_t* data, std::size_t size, Packet* packet) {
    if (packet == nullptr) {
        return false;
    }
    std::array<std::uint8_t, kMaxFrameSize> raw{};
    const std::size_t raw_size = cobsDecode(data, size, raw.data(), raw.size());
    if (raw_size < kHeaderSize + kCrcSize || raw[0] != kProtocolVersion) {
        return false;
    }

    const std::size_t payload_size = raw[6];
    if (payload_size > kMaxPayloadSize || raw_size != kHeaderSize + payload_size + kCrcSize) {
        return false;
    }
    const std::uint16_t expected = crc16Ccitt(raw.data(), kHeaderSize + payload_size);
    const std::uint16_t actual = static_cast<std::uint16_t>(raw[kHeaderSize + payload_size]) |
                                 (static_cast<std::uint16_t>(raw[kHeaderSize + payload_size + 1]) << 8u);
    if (actual != expected) {
        return false;
    }

    packet->type = static_cast<MessageType>(raw[1]);
    packet->sequence = readU32Le(&raw[2]);
    packet->payload_size = payload_size;
    for (std::size_t i = 0; i < payload_size; ++i) {
        packet->payload[i] = raw[kHeaderSize + i];
    }
    return true;
}

bool makeVelocityCommandPacket(const MotionCommand& command, Packet* packet) {
    if (packet == nullptr) {
        return false;
    }
    packet->type = MessageType::CmdVelocity;
    packet->sequence = command.sequence;
    packet->payload_size = 12;
    writeFloat(&packet->payload[0], command.vx_mps);
    writeFloat(&packet->payload[4], command.vy_mps);
    writeFloat(&packet->payload[8], command.wz_radps);
    return true;
}

bool motionCommandFromPacket(const Packet& packet, MotionCommand* command) {
    if (command == nullptr) {
        return false;
    }
    if (packet.type != MessageType::CmdVelocity || packet.payload_size != 12) {
        return false;
    }
    command->vx_mps = readFloat(&packet.payload[0]);
    command->vy_mps = readFloat(&packet.payload[4]);
    command->wz_radps = readFloat(&packet.payload[8]);
    command->sequence = packet.sequence;
    command->valid = true;
    return true;
}

}  // namespace hexapod::protocol
