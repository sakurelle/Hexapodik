#include <array>
#include <cstdint>

#include "config/robot_config.h"
#include "controller/low_level_controller.h"
#include "drivers/contact_sensors.h"
#include "drivers/servo_controller.h"
#include "drivers/uart_transport.h"
#include "protocol/protocol.h"

#if defined(PICO_ON_DEVICE)
#include "pico/stdlib.h"
#endif

namespace hexapod {
namespace {

void sendStatus(UartTransport& uart,
                const LowLevelController& controller,
                const ContactSensors& contacts,
                const protocol::Parser& parser,
                std::uint32_t response_sequence) {
    protocol::Packet packet;
    if (!protocol::makeStatusPacket(response_sequence,
                                    controller.status(contacts.inputMask(), parser.droppedFrames()),
                                    &packet)) {
        return;
    }
    std::array<std::uint8_t, protocol::kMaxFrameSize + 2> frame{};
    const std::size_t size = protocol::encodePacket(packet, frame.data(), frame.size());
    if (size != 0) {
        uart.write(frame.data(), size);
    }
}

}  // namespace
}  // namespace hexapod

int main() {
    using namespace hexapod;

    UartTransport uart;
    ServoController servos;
    ContactSensors contacts;
    protocol::Parser parser;

    uart.init(kUartBaud);
    servos.init();
    LowLevelController controller(servos);
    controller.reset();

#if defined(PICO_ON_DEVICE)
    contacts.init(time_us_64());
#else
    contacts.init(0);
#endif

    while (true) {
#if defined(PICO_ON_DEVICE)
        const std::uint64_t now_us = time_us_64();
#else
        const std::uint64_t now_us = 0;
#endif
        contacts.update(now_us);
        controller.tick(now_us);

        for (;;) {
            const int byte = uart.readByte();
            if (byte < 0) {
                break;
            }
            protocol::Packet packet;
            if (!parser.push(static_cast<std::uint8_t>(byte), &packet)) {
                continue;
            }
            const ProcessResult result = controller.processPacket(packet, now_us);
            if (result.send_status) {
                sendStatus(uart, controller, contacts, parser, result.status_sequence);
            }
        }

#if defined(PICO_ON_DEVICE)
        tight_loop_contents();
#else
        break;
#endif
    }
    return 0;
}
