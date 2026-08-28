#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>

#include "config/servo_config.h"
#include "drivers/rc_input.h"
#include "drivers/servo_controller.h"
#include "locomotion/contact_adaptation.h"
#include "locomotion/gait_generator.h"
#include "locomotion/locomotion_controller.h"
#include "locomotion/support_polygon.h"
#include "robot/kinematics.h"
#include "robot/robot_geometry.h"
#include "robot/workspace.h"

namespace {

bool near(float a, float b, float eps) {
    return std::fabs(a - b) <= eps;
}

float dist(hexapod::Vec3 a, hexapod::Vec3 b) {
    const auto d = a - b;
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

std::array<hexapod::ContactState, hexapod::kLegCount> contactStatesFromRaw(bool raw) {
    using namespace hexapod;
    std::array<ContactState, kLegCount> states{};
    for (ContactState& state : states) {
        state.raw = raw;
        state.stable = raw;
        state.stable_ms = std::max(kContactPressDebounceMs, kContactReleaseDebounceMs);
        state.healthy = true;
        state.health = SensorHealth::Ok;
    }
    return states;
}

void updateSyntheticGroundContacts(const hexapod::GaitOutput& out,
                                   const std::array<hexapod::Vec3, hexapod::kLegCount>& neutral,
                                   std::array<bool, hexapod::kLegCount>* raw_contacts,
                                   hexapod::ContactManager* contacts,
                                   float dt_s) {
    using namespace hexapod;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (!out.swing[i]) {
            (*raw_contacts)[i] = true;
        } else if (out.leg_phase[i] == LegPhase::Lift) {
            (*raw_contacts)[i] = out.feet_body_mm[i].z <= neutral[i].z + 2.0f;
        } else if (out.leg_phase[i] == LegPhase::Descend || out.leg_phase[i] == LegPhase::GroundSearch) {
            (*raw_contacts)[i] = out.feet_body_mm[i].z <= neutral[i].z + 0.5f;
        } else {
            (*raw_contacts)[i] = false;
        }
    }
    contacts->update(*raw_contacts, dt_s);
}

void assertFeetUnchanged(const hexapod::GaitOutput& before, const hexapod::GaitOutput& after) {
    using namespace hexapod;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        assert(dist(before.feet_body_mm[i], after.feet_body_mm[i]) < 0.001f);
    }
}

void testFkIkRoundTrip() {
    using namespace hexapod;
    for (float coxa_deg = -35.0f; coxa_deg <= 35.0f; coxa_deg += 17.5f) {
        for (float femur_deg = -25.0f; femur_deg <= 35.0f; femur_deg += 15.0f) {
            for (float tibia_deg = -30.0f; tibia_deg <= 60.0f; tibia_deg += 15.0f) {
                const JointAngles q{degToRad(coxa_deg), degToRad(femur_deg), degToRad(tibia_deg)};
                if (!jointsWithinLimits(q) || isSingular(q)) {
                    continue;
                }
                const Vec3 p = forwardKinematics(q);
                const IkResult ik = inverseKinematics(p, q);
                assert(ik.valid);
                assert(dist(forwardKinematics(ik.angles), p) < 0.75f);
                assert(near(ik.angles.coxa_rad, q.coxa_rad, degToRad(0.75f)));
                assert(near(ik.angles.femur_rad, q.femur_rad, degToRad(1.0f)));
                assert(near(ik.angles.tibia_rad, q.tibia_rad, degToRad(1.0f)));
            }
        }
    }
}

void testNeutralPose() {
    using namespace hexapod;
    const auto neutral = neutralFeetBody();
    std::array<JointAngles, kLegCount> previous{};
    std::array<JointAngles, kLegCount> joints{};
    assert(validateAllFootTargets(neutral, previous, &joints));
    for (std::size_t i = 0; i < kLegCount; ++i) {
        assert(isFinite(neutral[i]));
        assert(jointsWithinLimits(joints[i]));
    }
}

void testInvalidTargets() {
    using namespace hexapod;
    assert(!inverseKinematics({1000.0f, 0.0f, 0.0f}, {}).valid);
    assert(!inverseKinematics({1.0f, 0.0f, 0.0f}, {}).valid);
    assert(!inverseKinematics({std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}, {}).valid);
    const JointAngles singular{0.0f, 0.0f, degToRad(104.65f)};
    assert(isSingular(singular));
}

void testContactManagerBounceAndEvents() {
    using namespace hexapod;
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(false);
    contacts.reset(raw);

    raw[indexOf(LegId::FL)] = true;
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).pressed_event);
    raw[indexOf(LegId::FL)] = false;
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).pressed_event);
    raw[indexOf(LegId::FL)] = true;
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).pressed_event);
    raw[indexOf(LegId::FL)] = false;
    contacts.update(raw, 0.020f);
    assert(!contacts.state(LegId::FL).pressed_event);
    assert(!contacts.state(LegId::FL).stable);

    raw[indexOf(LegId::FL)] = true;
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).pressed_event);
    contacts.update(raw, 0.005f);
    assert(contacts.state(LegId::FL).pressed_event);
    assert(contacts.state(LegId::FL).stable);
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).pressed_event);

    raw[indexOf(LegId::FL)] = false;
    contacts.update(raw, 0.010f);
    assert(!contacts.state(LegId::FL).released_event);
    contacts.update(raw, 0.005f);
    assert(contacts.state(LegId::FL).released_event);
    contacts.update(raw, 0.005f);
    assert(!contacts.state(LegId::FL).released_event);
}

void testGait() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    const GaitOutput first = gait.update({60.0f, 0.0f, 0.0f}, contacts.states(), 0.01f);
    assert(first.mode == LocomotionMode::Running);

    bool saw_a = false;
    bool saw_b = false;
    Vec3 prev = first.feet_body_mm[indexOf(LegId::FR)];
    GaitOutput out = first;
    for (int i = 0; i < 420; ++i) {
        updateSyntheticGroundContacts(out, neutral, &raw, &contacts, 0.005f);
        out = gait.update({60.0f, 0.0f, 0.25f}, contacts.states(), 0.005f);
        assert(out.fault == FaultCode::None);
        assert(out.stop_reason == StopReason::None);
        saw_a = saw_a || out.swing[indexOf(LegId::FR)];
        saw_b = saw_b || out.swing[indexOf(LegId::FL)];
        const Vec3 now = out.feet_body_mm[indexOf(LegId::FR)];
        assert(dist(now, prev) < 20.0f);
        prev = now;
    }
    assert(saw_a);
    assert(saw_b);
}

void testHighAtStartOfLiftIsNotTouchdownOrFault() {
    using namespace hexapod;
    GaitGenerator gait;
    gait.reset(neutralFeetBody());
    auto contacts = contactStatesFromRaw(true);

    GaitOutput out{};
    for (int i = 0; i < 160; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, contacts, 0.005f);
        if (out.swing[indexOf(LegId::FR)]) {
            break;
        }
    }
    assert(out.swing[indexOf(LegId::FR)]);
    assert(out.leg_phase[indexOf(LegId::FR)] == LegPhase::Lift);
    assert(!out.released_this_swing[indexOf(LegId::FR)]);
    assert(!gait.legStates()[indexOf(LegId::FR)].touchdown_locked);
    assert(out.fault == FaultCode::None);
    assert(out.stop_reason == StopReason::None);
}

void testNormalSwingContactLifecycle() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    bool saw_lift = false;
    bool saw_release = false;
    bool saw_descend = false;
    bool saw_touchdown = false;
    GaitOutput out{};

    for (int i = 0; i < 520; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, contacts.states(), 0.005f);
        assert(out.fault == FaultCode::None);
        assert(out.stop_reason == StopReason::None);
        saw_lift = saw_lift || out.leg_phase[indexOf(LegId::FR)] == LegPhase::Lift;
        saw_release = saw_release || out.released_this_swing[indexOf(LegId::FR)];
        saw_descend = saw_descend || out.leg_phase[indexOf(LegId::FR)] == LegPhase::Descend;
        saw_touchdown = saw_touchdown || (!out.swing[indexOf(LegId::FR)] && saw_descend);
        updateSyntheticGroundContacts(out, neutral, &raw, &contacts, 0.005f);
        if (saw_lift && saw_release && saw_descend && saw_touchdown) {
            break;
        }
    }

    assert(saw_lift);
    assert(saw_release);
    assert(saw_descend);
    assert(saw_touchdown);
}

void testContactStuckHighRequestsControlledStop() {
    using namespace hexapod;
    GaitGenerator gait;
    gait.reset(neutralFeetBody());
    auto contacts = contactStatesFromRaw(true);

    GaitOutput out{};
    for (int i = 0; i < 260; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, contacts, 0.005f);
        assert(out.fault == FaultCode::None);
        if (out.stop_reason == StopReason::SensorStuckHigh) {
            break;
        }
    }

    assert(out.fault == FaultCode::None);
    assert(out.mode == LocomotionMode::Stopping);
    assert(out.stop_reason == StopReason::SensorStuckHigh);
    assert(out.stop_leg != LegId::Count);
    assert(out.sensor_health[indexOf(out.stop_leg)] == SensorHealth::SuspectStuckHigh);
}

void testNoGroundRequestsControlledStopAfterGroundSearch() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    bool saw_ground_search = false;
    GaitOutput out{};
    for (int i = 0; i < 900; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, contacts.states(), 0.005f);
        assert(out.fault == FaultCode::None);
        saw_ground_search = saw_ground_search ||
                            out.leg_phase[indexOf(LegId::FR)] == LegPhase::GroundSearch ||
                            out.leg_phase[indexOf(LegId::ML)] == LegPhase::GroundSearch ||
                            out.leg_phase[indexOf(LegId::RR)] == LegPhase::GroundSearch;
        for (std::size_t i_leg = 0; i_leg < kLegCount; ++i_leg) {
            if (!out.swing[i_leg]) {
                raw[i_leg] = true;
            } else if (out.leg_phase[i_leg] == LegPhase::Lift) {
                raw[i_leg] = out.feet_body_mm[i_leg].z <= neutral[i_leg].z + 2.0f;
            } else {
                raw[i_leg] = false;
            }
        }
        contacts.update(raw, 0.005f);
        if (out.stop_reason == StopReason::NoGround) {
            break;
        }
    }

    assert(saw_ground_search);
    assert(out.fault == FaultCode::None);
    assert(out.mode == LocomotionMode::Stopping);
    assert(out.stop_reason == StopReason::NoGround);
    assert(out.stop_leg != LegId::Count);
}

void testTouchdownOnlyIgnoresInstantSupportLoss() {
    using namespace hexapod;
    GaitGenerator gait;
    gait.reset(neutralFeetBody());
    auto contacts = contactStatesFromRaw(false);

    GaitOutput out{};
    for (int i = 0; i < 180; ++i) {
        out = gait.update({60.0f, 0.0f, 0.0f}, contacts, 0.005f);
        assert(out.support_ok);
        assert(!out.waiting_for_support);
        assert(out.fault == FaultCode::None);
        if (out.swing[indexOf(LegId::FR)]) {
            break;
        }
    }
    assert(out.swing[indexOf(LegId::FR)] || out.swing[indexOf(LegId::ML)] || out.swing[indexOf(LegId::RR)]);
}

void testChatterDuringStanceDoesNotStopGait() {
    using namespace hexapod;
    GaitGenerator gait;
    gait.reset(neutralFeetBody());
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    GaitOutput out{};
    for (int i = 0; i < 60; ++i) {
        raw[indexOf(LegId::FL)] = (i % 2) == 0;
        contacts.update(raw, 0.003f);
        out = gait.update({40.0f, 0.0f, 0.0f}, contacts.states(), 0.003f);
        assert(out.fault == FaultCode::None);
        assert(out.stop_reason == StopReason::None);
    }
}

void testTouchdownBeforeDescendRequestsControlledStop() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    GaitOutput out{};
    bool injected_early_press = false;
    for (int i = 0; i < 520; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, contacts.states(), 0.005f);
        for (std::size_t leg_i = 0; leg_i < kLegCount; ++leg_i) {
            if (!out.swing[leg_i]) {
                raw[leg_i] = true;
            } else if (out.leg_phase[leg_i] == LegPhase::Lift) {
                raw[leg_i] = out.feet_body_mm[leg_i].z <= neutral[leg_i].z + 2.0f;
            } else if (out.leg_phase[leg_i] == LegPhase::Transfer && out.released_this_swing[leg_i]) {
                raw[leg_i] = true;
                injected_early_press = true;
            } else {
                raw[leg_i] = false;
            }
        }
        contacts.update(raw, 0.010f);
        if (out.stop_reason == StopReason::EarlyCollision) {
            break;
        }
    }

    assert(injected_early_press);
    assert(out.fault == FaultCode::None);
    assert(out.stop_reason == StopReason::EarlyCollision);
    assert(out.stop_leg != LegId::Count);
}

void testSyntheticTripodCycles() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    ContactManager contacts;
    std::array<bool, kLegCount> raw{};
    raw.fill(true);
    contacts.reset(raw);

    bool saw_tripod_a = false;
    bool saw_tripod_b = false;
    GaitOutput out{};
    for (int i = 0; i < 900; ++i) {
        updateSyntheticGroundContacts(out, neutral, &raw, &contacts, 0.005f);
        out = gait.update({65.0f, 0.0f, 0.2f}, contacts.states(), 0.005f);
        assert(out.fault == FaultCode::None);
        assert(out.support_ok);
        assert(out.stop_reason == StopReason::None);
        saw_tripod_a = saw_tripod_a || out.swing[indexOf(LegId::FR)];
        saw_tripod_b = saw_tripod_b || out.swing[indexOf(LegId::FL)];
    }

    assert(saw_tripod_a);
    assert(saw_tripod_b);
    assert(out.mode == LocomotionMode::Running);
    assert(out.fault == FaultCode::None);
}

void testSupportPolygon() {
    using namespace hexapod;
    const auto neutral = neutralFeetBody();
    std::array<bool, kLegCount> tripod_a{};
    tripod_a[indexOf(LegId::FR)] = true;
    tripod_a[indexOf(LegId::ML)] = true;
    tripod_a[indexOf(LegId::RR)] = true;
    assert(hasStableSupport(neutral, tripod_a));
}

void testServoMapping() {
    using namespace hexapod;
    const auto center = angleToPulse(LegId::FL, JointId::Coxa, 0.0f);
    assert(center.valid && center.pulse_us == 1500);
    const auto plus_left = angleToPulse(LegId::FL, JointId::Coxa, degToRad(15.0f));
    const auto plus_right = angleToPulse(LegId::FR, JointId::Coxa, degToRad(15.0f));
    assert(plus_left.valid && plus_left.pulse_us > 1500);
    assert(plus_right.valid && plus_right.pulse_us < 1500);
    assert(!angleToPulse(LegId::FL, JointId::Coxa, degToRad(100.0f)).valid);
}

std::uint32_t eventDurationUs(const hexapod::ServoController::Event& event) {
    return event.delay_ticks + hexapod::ServoController::kPioSegmentOverheadTicks;
}

void testServoEventBufferAllCenter() {
    using namespace hexapod;
    std::array<std::uint16_t, kServoCount> pulses{};
    pulses.fill(1500);

    ServoController::EventBuffer events{};
    const std::size_t count = ServoController::buildEventBuffer(pulses, &events);

    assert(count == 2);
    assert(events[0].mask == ServoController::kAllServoPinsMask);
    assert(eventDurationUs(events[0]) == 1500);
    assert(events[1].mask == 0);
    assert(eventDurationUs(events[1]) == kServoFrameUs - 1500);
    assert(eventDurationUs(events[0]) + eventDurationUs(events[1]) == kServoFrameUs);
}

void testServoEventBufferDifferentPulseWidths() {
    using namespace hexapod;
    std::array<std::uint16_t, kServoCount> pulses{};
    pulses.fill(2000);
    pulses[0] = 1000;
    pulses[1] = 1500;
    pulses[17] = 2000;

    ServoController::EventBuffer events{};
    const std::size_t count = ServoController::buildEventBuffer(pulses, &events);

    assert(count == 4);
    assert(events[0].mask == ServoController::kAllServoPinsMask);
    assert(eventDurationUs(events[0]) == 1000);
    assert((events[1].mask & (1u << 0)) == 0);
    assert((events[1].mask & (1u << 1)) != 0);
    assert((events[1].mask & (1u << 17)) != 0);
    assert(eventDurationUs(events[1]) == 500);
    assert((events[2].mask & (1u << 1)) == 0);
    assert((events[2].mask & (1u << 17)) != 0);
    assert(eventDurationUs(events[2]) == 500);
    assert(events[3].mask == 0);
    assert(eventDurationUs(events[3]) == kServoFrameUs - 2000);

    std::uint32_t total_us = 0;
    for (std::size_t i = 0; i < count; ++i) {
        total_us += eventDurationUs(events[i]);
    }
    assert(total_us == kServoFrameUs);
    assert(kServoGpios[0] == 2);
    assert(kServoGpios[17] == 19);
}

void testRcInputFreshFailsafeAndClamp() {
    using namespace hexapod;
    RcInput rc;
    assert(rc.failsafe(1000));
    assert(!rc.forwardFresh(1000));
    assert(!rc.yawFresh(1000));

    rc.onEdge(kRcForwardGpio, true, 10000);
    rc.onEdge(kRcForwardGpio, false, 11700);
    rc.onEdge(kRcYawGpio, true, 20000);
    rc.onEdge(kRcYawGpio, false, 21500);

    const RcInputSnapshot moving = rc.snapshot(22000);
    assert(moving.forward_pulse_us == 1700);
    assert(moving.yaw_pulse_us == 1500);
    assert(moving.forward_fresh);
    assert(moving.yaw_fresh);
    assert(!moving.failsafe);

    const BodyCommand forward = rc.readCommand(22000);
    assert(forward.vx_mm_s > 0.0f);
    assert(std::fabs(forward.yaw_rad_s) < 0.001f);

    rc.onEdge(kRcForwardGpio, true, 30000);
    rc.onEdge(kRcForwardGpio, false, 32400);
    const BodyCommand clamped = rc.readCommand(33000);
    assert(near(clamped.vx_mm_s, kMaxVxMmS, 0.01f));

    rc.onEdge(kRcYawGpio, true, 40000);
    rc.onEdge(kRcYawGpio, false, 41900);
    const BodyCommand yaw = rc.readCommand(42000);
    assert(yaw.yaw_rad_s > 0.0f);

    const RcInputSnapshot stale = rc.snapshot(200000);
    assert(!stale.forward_fresh);
    assert(!stale.yaw_fresh);
    assert(stale.failsafe);
    const BodyCommand stopped = rc.readCommand(200000);
    assert(near(stopped.vx_mm_s, 0.0f, 0.001f));
    assert(near(stopped.yaw_rad_s, 0.0f, 0.001f));
}

}  // namespace

int main() {
    testFkIkRoundTrip();
    testNeutralPose();
    testInvalidTargets();
    testContactManagerBounceAndEvents();
    testGait();
    testHighAtStartOfLiftIsNotTouchdownOrFault();
    testNormalSwingContactLifecycle();
    testContactStuckHighRequestsControlledStop();
    testNoGroundRequestsControlledStopAfterGroundSearch();
    testTouchdownOnlyIgnoresInstantSupportLoss();
    testChatterDuringStanceDoesNotStopGait();
    testTouchdownBeforeDescendRequestsControlledStop();
    testSyntheticTripodCycles();
    testSupportPolygon();
    testServoMapping();
    testServoEventBufferAllCenter();
    testServoEventBufferDifferentPulseWidths();
    testRcInputFreshFailsafeAndClamp();
    std::puts("All host tests passed.");
    return 0;
}
