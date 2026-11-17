#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "config/robot_config.h"
#include "controller/low_level_controller.h"
#include "drivers/contact_sensors.h"
#include "drivers/servo_controller.h"
#include "protocol/protocol.h"

namespace {

class FakePwm final : public hexapod::IPwmOutput {
public:
    bool enable(const std::array<std::uint16_t, hexapod::kServoCount>& pulses_us) override {
        if (fail_next) {
            fail_next = false;
            return false;
        }
        pulses = pulses_us;
        is_enabled = true;
        ++enable_count;
        return true;
    }

    bool update(const std::array<std::uint16_t, hexapod::kServoCount>& pulses_us) override {
        if (!is_enabled || fail_next) {
            fail_next = false;
            return false;
        }
        pulses = pulses_us;
        ++update_count;
        return true;
    }

    void disable() override {
        is_enabled = false;
        ++disable_count;
    }

    bool enabled() const override { return is_enabled; }

    std::array<std::uint16_t, hexapod::kServoCount> pulses{};
    bool is_enabled = false;
    bool fail_next = false;
    int enable_count = 0;
    int update_count = 0;
    int disable_count = 0;
};

std::array<std::uint16_t, hexapod::kServoCount> samplePulses(std::uint16_t first = 1000) {
    std::array<std::uint16_t, hexapod::kServoCount> pulses{};
    for (std::size_t i = 0; i < pulses.size(); ++i) {
        pulses[i] = static_cast<std::uint16_t>(first + i * 25u);
    }
    return pulses;
}

hexapod::protocol::Packet pwmPacket(hexapod::protocol::MessageType type,
                                    std::uint32_t sequence,
                                    const std::array<std::uint16_t, hexapod::kServoCount>& pulses) {
    hexapod::protocol::Packet packet;
    assert(hexapod::protocol::makePwmCommandPacket(type, sequence, pulses, &packet));
    return packet;
}

void testPinAndChannelOrder() {
    using namespace hexapod;
    assert(kServoGpios[channelIndex(LegId::FL, JointId::Tibia)] == 2);
    assert(kServoGpios[channelIndex(LegId::FL, JointId::Femur)] == 3);
    assert(kServoGpios[channelIndex(LegId::FL, JointId::Coxa)] == 4);
    assert(kServoGpios[channelIndex(LegId::FR, JointId::Tibia)] == 17);
    assert(kServoGpios[channelIndex(LegId::FR, JointId::Femur)] == 18);
    assert(kServoGpios[channelIndex(LegId::FR, JointId::Coxa)] == 19);
    assert((kContactGpios == std::array<std::uint8_t, kLegCount>{20, 21, 22, 26, 27, 28}));

    const auto pulses = samplePulses();
    protocol::Packet packet = pwmPacket(protocol::MessageType::Arm, 7, pulses);
    std::array<std::uint16_t, kServoCount> decoded{};
    assert(protocol::pwmCommandFromPacket(packet, &decoded));
    assert(decoded == pulses);
}

void testProtocolRoundTripAndRecovery() {
    using namespace hexapod;
    const auto pulses = samplePulses();
    const protocol::Packet source = pwmPacket(protocol::MessageType::Arm, 0x12345678u, pulses);
    std::array<std::uint8_t, protocol::kMaxFrameSize + 2> encoded{};
    const std::size_t encoded_size = protocol::encodePacket(source, encoded.data(), encoded.size());
    assert(encoded_size > 0);
    assert(encoded[encoded_size - 1] == 0);

    protocol::Parser parser;
    protocol::Packet decoded;
    bool got_packet = false;
    for (std::size_t i = 0; i < encoded_size; ++i) {
        got_packet = parser.push(encoded[i], &decoded) || got_packet;
    }
    assert(got_packet);
    assert(decoded.type == protocol::MessageType::Arm);
    assert(decoded.sequence == 0x12345678u);
    std::array<std::uint16_t, kServoCount> decoded_pulses{};
    assert(protocol::pwmCommandFromPacket(decoded, &decoded_pulses));
    assert(decoded_pulses == pulses);

    auto corrupted = encoded;
    corrupted[encoded_size / 2] ^= 0x40u;
    got_packet = false;
    for (std::size_t i = 0; i < encoded_size; ++i) {
        got_packet = parser.push(corrupted[i], &decoded) || got_packet;
    }
    assert(!got_packet);

    for (std::size_t i = 0; i < encoded_size / 2; ++i) {
        parser.push(encoded[i], &decoded);
    }
    assert(!parser.push(0, &decoded));

    got_packet = false;
    for (std::size_t i = 0; i < encoded_size; ++i) {
        got_packet = parser.push(encoded[i], &decoded) || got_packet;
    }
    assert(got_packet);
    assert(parser.droppedFrames() == 2);
}

void testKnownWireExamples() {
    using namespace hexapod;
    protocol::Packet disarm;
    disarm.type = protocol::MessageType::Disarm;
    disarm.sequence = 1;
    const std::array<std::uint8_t, 11> expected_disarm = {
        0x04, 0x01, 0x03, 0x01, 0x01, 0x01, 0x01, 0x03, 0x1E, 0x2D, 0x00,
    };
    std::array<std::uint8_t, protocol::kMaxFrameSize + 2> frame{};
    const std::size_t size = protocol::encodePacket(disarm, frame.data(), frame.size());
    assert(size == expected_disarm.size());
    for (std::size_t i = 0; i < size; ++i) {
        assert(frame[i] == expected_disarm[i]);
    }

    protocol::Packet status;
    status.type = protocol::MessageType::GetStatus;
    status.sequence = 2;
    const std::array<std::uint8_t, 11> expected_status = {
        0x04, 0x01, 0x04, 0x02, 0x01, 0x01, 0x01, 0x03, 0x8D, 0x0B, 0x00,
    };
    const std::size_t status_size = protocol::encodePacket(status, frame.data(), frame.size());
    assert(status_size == expected_status.size());
    for (std::size_t i = 0; i < status_size; ++i) {
        assert(frame[i] == expected_status[i]);
    }

    std::array<std::uint16_t, kServoCount> neutral{};
    neutral.fill(1500);
    protocol::Packet arm;
    assert(protocol::makePwmCommandPacket(protocol::MessageType::Arm, 3, neutral, &arm));
    const std::array<std::uint8_t, 47> expected_arm = {
        0x04, 0x01, 0x01, 0x03, 0x01, 0x01, 0x28, 0x24,
        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
        0x83, 0xAE, 0x00,
    };
    const std::size_t arm_size = protocol::encodePacket(arm, frame.data(), frame.size());
    assert(arm_size == expected_arm.size());
    for (std::size_t i = 0; i < arm_size; ++i) {
        assert(frame[i] == expected_arm[i]);
    }
}

void testArmSetAndAtomicRejection() {
    using namespace hexapod;
    FakePwm pwm;
    LowLevelController controller(pwm);
    controller.reset();
    assert(controller.state() == protocol::ControllerState::Disarmed);
    assert(!pwm.enabled());

    const auto initial = samplePulses();
    controller.processPacket(pwmPacket(protocol::MessageType::Arm, 10, initial), 1000);
    assert(controller.state() == protocol::ControllerState::Armed);
    assert(pwm.enabled());
    assert(pwm.pulses == initial);

    auto invalid = samplePulses(1100);
    invalid[9] = 999;
    controller.processPacket(pwmPacket(protocol::MessageType::SetPwm, 11, invalid), 2000);
    assert(controller.error() == protocol::ErrorCode::InvalidPwm);
    assert(pwm.pulses == initial);
    assert(pwm.update_count == 0);
    assert(controller.lastCommandSequence() == 10);

    auto next = samplePulses(1200);
    controller.processPacket(pwmPacket(protocol::MessageType::SetPwm, 11, next), 3000);
    assert(controller.error() == protocol::ErrorCode::None);
    assert(pwm.pulses == next);
    assert(pwm.update_count == 1);
    assert(controller.lastCommandSequence() == 11);
}

void testStaleDoesNotUpdateOrRefreshTimeout() {
    using namespace hexapod;
    FakePwm pwm;
    LowLevelController controller(pwm);
    controller.reset();
    const auto initial = samplePulses();
    controller.processPacket(pwmPacket(protocol::MessageType::Arm, 100, initial), 1000);

    auto stale = samplePulses(1200);
    controller.processPacket(pwmPacket(protocol::MessageType::SetPwm, 100, stale), 200000);
    assert(controller.error() == protocol::ErrorCode::StaleSequence);
    assert(pwm.pulses == initial);
    controller.tick(251000);
    assert(controller.state() == protocol::ControllerState::CommunicationFault);
    assert(controller.error() == protocol::ErrorCode::CommandTimeout);
    assert(pwm.enabled());
    assert(pwm.pulses == initial);
}

void testTimeoutRequiresDisarmThenArm() {
    using namespace hexapod;
    FakePwm pwm;
    LowLevelController controller(pwm);
    controller.reset();
    const auto initial = samplePulses();
    controller.processPacket(pwmPacket(protocol::MessageType::Arm, 1, initial), 1000);
    controller.tick(251000);
    assert(controller.state() == protocol::ControllerState::CommunicationFault);
    assert(pwm.enabled());

    auto next = samplePulses(1200);
    controller.processPacket(pwmPacket(protocol::MessageType::SetPwm, 2, next), 252000);
    assert(controller.state() == protocol::ControllerState::CommunicationFault);
    assert(controller.error() == protocol::ErrorCode::RequiresDisarm);
    assert(pwm.pulses == initial);

    protocol::Packet disarm;
    disarm.type = protocol::MessageType::Disarm;
    disarm.sequence = 3;
    controller.processPacket(disarm, 253000);
    assert(controller.state() == protocol::ControllerState::Disarmed);
    assert(!pwm.enabled());

    controller.processPacket(pwmPacket(protocol::MessageType::Arm, 4, next), 254000);
    assert(controller.state() == protocol::ControllerState::Armed);
    assert(pwm.enabled());
    assert(pwm.pulses == next);
}

void testStatusWhileDisarmed() {
    using namespace hexapod;
    FakePwm pwm;
    LowLevelController controller(pwm);
    controller.reset();

    protocol::Packet request;
    request.type = protocol::MessageType::GetStatus;
    request.sequence = 77;
    const ProcessResult result = controller.processPacket(request, 0);
    assert(result.send_status);
    assert(result.status_sequence == 77);

    const protocol::StatusPayload status = controller.status(0x25, 9);
    assert(status.state == protocol::ControllerState::Disarmed);
    assert(status.error == protocol::ErrorCode::None);
    assert(!status.outputs_enabled);
    assert(status.input_mask == 0x25);
    assert(status.dropped_frames == 9);

    protocol::Packet response;
    assert(protocol::makeStatusPacket(77, status, &response));
    protocol::StatusPayload decoded;
    assert(protocol::statusFromPacket(response, &decoded));
    assert(decoded.state == protocol::ControllerState::Disarmed);
    assert(decoded.input_mask == 0x25);
}

void testInputDebounce() {
    using namespace hexapod;
    InputDebouncer inputs;
    inputs.reset(0x00, 0);
    inputs.update(0x01, 2000);
    inputs.update(0x00, 6000);
    inputs.update(0x01, 9000);
    assert(inputs.stableMask() == 0x00);
    inputs.update(0x01, 18999);
    assert(inputs.stableMask() == 0x00);
    inputs.update(0x01, 19000);
    assert(inputs.stableMask() == 0x01);
}

std::uint32_t eventDurationUs(const hexapod::ServoController::Event& event) {
    return event.delay_ticks + hexapod::ServoController::kPioSegmentOverheadTicks;
}

void testIndependentPwmEventBuffer() {
    using namespace hexapod;
    auto pulses = samplePulses();
    ServoController::EventBuffer events{};
    const std::size_t count = ServoController::buildEventBuffer(pulses, &events);
    assert(count == kServoCount + 1);
    assert(events[0].mask == ServoController::kAllServoPinsMask);
    assert(eventDurationUs(events[0]) == pulses[0]);
    for (std::size_t i = 1; i < kServoCount; ++i) {
        assert(eventDurationUs(events[i]) ==
               static_cast<std::uint32_t>(pulses[i] - pulses[i - 1]));
        assert((events[i].mask & (1u << (i - 1))) == 0);
        assert((events[i].mask & (1u << i)) != 0);
    }
    std::uint32_t total_us = 0;
    for (std::size_t i = 0; i < count; ++i) {
        total_us += eventDurationUs(events[i]);
    }
    assert(total_us == kServoFrameUs);
}

}  // namespace

int main() {
    testPinAndChannelOrder();
    testProtocolRoundTripAndRecovery();
    testKnownWireExamples();
    testArmSetAndAtomicRejection();
    testStaleDoesNotUpdateOrRefreshTimeout();
    testTimeoutRequiresDisarmThenArm();
    testStatusWhileDisarmed();
    testInputDebounce();
    testIndependentPwmEventBuffer();
    std::puts("All RP2040 low-level controller tests passed.");
    return 0;
}
