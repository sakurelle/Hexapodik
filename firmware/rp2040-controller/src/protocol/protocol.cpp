#include "protocol/protocol.h"

#include "protocol/cobs.h"
#include "protocol/crc.h"

namespace hexapod::protocol {
namespace {

constexpr std::size_t kHeaderSize = 7;
constexpr std::size_t kCrcSize = 2;

void writeU16Le(std::uint8_t* out, std::uint16_t value) {
    out[0] = static_cast<std::uint8_t>(value & 0xFFu);
    out[1] = static_cast<std::uint8_t>(value >> 8u);
}

std::uint16_t readU16Le(const std::uint8_t* in) {
    return static_cast<std::uint16_t>(in[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8u);
}

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

}  // namespace

void Parser::reset() {
    frame_size_ = 0;
    discarding_ = false;
}

bool Parser::push(std::uint8_t byte, Packet* out_packet) {
    if (byte == 0) {
        if (discarding_) {
            discarding_ = false;
            frame_size_ = 0;
            return false;
        }
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

    if (discarding_) {
        return false;
    }
    if (frame_size_ >= frame_.size()) {
        ++dropped_frames_;
        frame_size_ = 0;
        discarding_ = true;
        return false;
    }
    frame_[frame_size_++] = byte;
    return false;
}

std::size_t encodePacket(const Packet& packet, std::uint8_t* output, std::size_t output_size) {
    if (output == nullptr || packet.payload_size > kMaxPayloadSize || output_size < 2) {
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
    writeU16Le(&raw[kHeaderSize + packet.payload_size], crc);

    const std::size_t encoded = cobsEncode(raw.data(), raw_size, output, output_size - 1);
    if (encoded == 0) {
        return 0;
    }
    output[encoded] = 0;
    return encoded + 1;
}

bool decodePacket(const std::uint8_t* data, std::size_t size, Packet* packet) {
    if (data == nullptr || packet == nullptr) {
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
    const std::uint16_t actual = readU16Le(&raw[kHeaderSize + payload_size]);
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

bool makePwmCommandPacket(MessageType type,
                          std::uint32_t sequence,
                          const std::array<std::uint16_t, kServoCount>& pulses_us,
                          Packet* packet) {
    if (packet == nullptr || (type != MessageType::Arm && type != MessageType::SetPwm)) {
        return false;
    }
    packet->type = type;
    packet->sequence = sequence;
    packet->payload_size = kPwmPayloadSize;
    for (std::size_t i = 0; i < kServoCount; ++i) {
        writeU16Le(&packet->payload[i * 2], pulses_us[i]);
    }
    return true;
}

bool pwmCommandFromPacket(const Packet& packet, std::array<std::uint16_t, kServoCount>* pulses_us) {
    if (pulses_us == nullptr ||
        (packet.type != MessageType::Arm && packet.type != MessageType::SetPwm) ||
        packet.payload_size != kPwmPayloadSize) {
        return false;
    }
    for (std::size_t i = 0; i < kServoCount; ++i) {
        (*pulses_us)[i] = readU16Le(&packet.payload[i * 2]);
    }
    return true;
}

bool makeStatusPacket(std::uint32_t sequence, const StatusPayload& status, Packet* packet) {
    if (packet == nullptr) {
        return false;
    }
    packet->type = MessageType::Status;
    packet->sequence = sequence;
    packet->payload_size = kStatusPayloadSize;
    packet->payload[0] = static_cast<std::uint8_t>(status.state);
    packet->payload[1] = static_cast<std::uint8_t>(status.error);
    packet->payload[2] = status.outputs_enabled ? 1u : 0u;
    packet->payload[3] = status.input_mask;
    writeU32Le(&packet->payload[4], status.last_command_sequence);
    writeU32Le(&packet->payload[8], status.dropped_frames);
    return true;
}

bool statusFromPacket(const Packet& packet, StatusPayload* status) {
    if (status == nullptr || packet.type != MessageType::Status || packet.payload_size != kStatusPayloadSize) {
        return false;
    }
    status->state = static_cast<ControllerState>(packet.payload[0]);
    status->error = static_cast<ErrorCode>(packet.payload[1]);
    status->outputs_enabled = (packet.payload[2] & 1u) != 0;
    status->input_mask = packet.payload[3];
    status->last_command_sequence = readU32Le(&packet.payload[4]);
    status->dropped_frames = readU32Le(&packet.payload[8]);
    return true;
}

}  // namespace hexapod::protocol
