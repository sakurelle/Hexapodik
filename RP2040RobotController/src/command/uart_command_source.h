#pragma once

#include <cstdint>

#include "command/command_source.h"
#include "drivers/uart_transport.h"
#include "protocol/protocol.h"

namespace hexapod {

class UartCommandSource : public ICommandSource {
public:
    explicit UartCommandSource(UartTransport& transport) : transport_(transport) {}

    void init();
    MotionCommand readMotionCommand(std::uint64_t now_us) override;
    std::uint32_t lastSequence() const { return last_command_.sequence; }
    std::uint32_t droppedFrames() const { return parser_.droppedFrames(); }

private:
    UartTransport& transport_;
    protocol::Parser parser_{};
    MotionCommand last_command_{};
    std::uint64_t last_command_us_ = 0;
};

}  // namespace hexapod
