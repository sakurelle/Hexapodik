#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "config/servo_config.h"
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

void testGait() {
    using namespace hexapod;
    GaitGenerator gait;
    const auto neutral = neutralFeetBody();
    gait.reset(neutral);
    std::array<bool, kLegCount> contacts{};
    contacts.fill(true);

    const GaitOutput first = gait.update({60.0f, 0.0f, 0.0f}, contacts, 0.01f);
    assert(first.mode == LocomotionMode::Running);

    bool saw_a = false;
    bool saw_b = false;
    Vec3 prev = first.feet_body_mm[indexOf(LegId::FR)];
    GaitOutput out = first;
    for (int i = 0; i < 320; ++i) {
        for (std::size_t leg_i = 0; leg_i < kLegCount; ++leg_i) {
            contacts[leg_i] = !out.swing[leg_i] ||
                              out.feet_body_mm[leg_i].z <= neutral[leg_i].z + 1.0f;
        }
        out = gait.update({60.0f, 0.0f, 0.25f}, contacts, 0.005f);
        assert(out.fault == FaultCode::None);
        saw_a = saw_a || out.swing[indexOf(LegId::FR)];
        saw_b = saw_b || out.swing[indexOf(LegId::FL)];
        const Vec3 now = out.feet_body_mm[indexOf(LegId::FR)];
        assert(dist(now, prev) < 20.0f);
        prev = now;
    }
    assert(saw_a);
    assert(saw_b);

    const Vec3 fr = gait.legStates()[indexOf(LegId::FR)].target_body_mm;
    const Vec3 ml = gait.legStates()[indexOf(LegId::ML)].target_body_mm;
    const Vec3 rr = gait.legStates()[indexOf(LegId::RR)].target_body_mm;
    assert(std::fabs(fr.y - ml.y) > 1.0f || std::fabs(fr.y - rr.y) > 1.0f);
}

void testContact() {
    using namespace hexapod;
    ContactDebouncer debounce;
    std::array<bool, kLegCount> raw{};
    raw.fill(false);
    debounce.reset(raw);
    raw[indexOf(LegId::FL)] = true;
    debounce.update(raw, 0.005f);
    assert(!debounce.contact(LegId::FL));
    debounce.update(raw, 0.008f);
    assert(debounce.contact(LegId::FL));

    GaitGenerator gait;
    gait.reset(neutralFeetBody());
    raw.fill(true);
    GaitOutput out{};
    for (int i = 0; i < 140; ++i) {
        out = gait.update({70.0f, 0.0f, 0.0f}, raw, 0.005f);
        if (out.fault == FaultCode::ContactStuck) {
            break;
        }
    }
    assert(out.fault == FaultCode::ContactStuck);
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

}  // namespace

int main() {
    testFkIkRoundTrip();
    testNeutralPose();
    testInvalidTargets();
    testGait();
    testContact();
    testSupportPolygon();
    testServoMapping();
    std::puts("All host tests passed.");
    return 0;
}
