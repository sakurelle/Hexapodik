#include "command/uart_command_source.h"

namespace hexapod {

void UartCommandSource::init() {
    transport_.init();
}

MotionCommand UartCommandSource::readMotionCommand(std::uint64_t now_us) {
    for (;;) {
        const int byte = transport_.readByte();
        if (byte < 0) {
            break;
        }

        protocol::Packet packet;
        if (parser_.push(static_cast<std::uint8_t>(byte), &packet)) {
            MotionCommand decoded;
            if (protocol::motionCommandFromPacket(packet, &decoded)) {
                last_command_ = decoded;
                last_command_us_ = now_us;
            }
        }
    }

    MotionCommand result = last_command_;
    result.valid = last_command_us_ != 0 &&
                   now_us >= last_command_us_ &&
                   now_us - last_command_us_ <= kCommandWatchdogTimeoutUs;
    return result;
}

}  // namespace hexapod
