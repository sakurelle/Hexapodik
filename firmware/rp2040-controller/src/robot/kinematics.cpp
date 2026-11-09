#include "robot/kinematics.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace hexapod {
namespace {

float clamp(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
}

float sqr(float value) {
    return value * value;
}

bool finite(JointAngles a) {
    return std::isfinite(a.coxa_rad) && std::isfinite(a.femur_rad) && std::isfinite(a.tibia_rad);
}

float distance2(JointAngles a, JointAngles b) {
    return sqr(a.coxa_rad - b.coxa_rad) + sqr(a.femur_rad - b.femur_rad) + sqr(a.tibia_rad - b.tibia_rad);
}

}  // namespace

JointAngles logicalToPhysical(JointAngles logical) {
    return {
        logical.coxa_rad + kCoxaModelZeroRad,
        logical.femur_rad + kFemurModelZeroRad,
        logical.tibia_rad + kTibiaModelZeroRad,
    };
}

JointAngles physicalToLogical(JointAngles physical) {
    return {
        physical.coxa_rad - kCoxaModelZeroRad,
        physical.femur_rad - kFemurModelZeroRad,
        physical.tibia_rad - kTibiaModelZeroRad,
    };
}

Vec3 forwardKinematics(JointAngles logical_angles) {
    const JointAngles q = logicalToPhysical(logical_angles);
    const float radial_mm = kLcoxaMm + kLfemurMm * std::cos(q.femur_rad) +
                            kLtibiaMm * std::cos(q.femur_rad + q.tibia_rad);
    const float z_mm = kLfemurMm * std::sin(q.femur_rad) +
                       kLtibiaMm * std::sin(q.femur_rad + q.tibia_rad);
    return {
        radial_mm * std::cos(q.coxa_rad),
        radial_mm * std::sin(q.coxa_rad),
        z_mm,
    };
}

bool jointsWithinLimits(JointAngles q, float margin_rad) {
    if (!finite(q)) {
        return false;
    }
    return q.coxa_rad >= kCoxaLimitMinRad + margin_rad && q.coxa_rad <= kCoxaLimitMaxRad - margin_rad &&
           q.femur_rad >= kFemurLimitMinRad + margin_rad && q.femur_rad <= kFemurLimitMaxRad - margin_rad &&
           q.tibia_rad >= kTibiaLimitMinRad + margin_rad && q.tibia_rad <= kTibiaLimitMaxRad - margin_rad;
}

bool isSingular(JointAngles logical_angles) {
    const JointAngles q = logicalToPhysical(logical_angles);
    return !finite(q) || std::fabs(std::sin(q.tibia_rad)) < kSingularitySinMin;
}

IkResult inverseKinematics(Vec3 p, JointAngles previous_logical) {
    if (!isFinite(p)) {
        return {};
    }

    const float q_coxa_physical = std::atan2(p.y, p.x);
    const float radial_mm = std::sqrt(p.x * p.x + p.y * p.y);
    const float r_mm = radial_mm - kLcoxaMm;
    const float d2 = r_mm * r_mm + p.z * p.z;
    const float d = std::sqrt(d2);
    constexpr float kEpsilon = 1.0e-4f;

    if (!std::isfinite(d) || d > kLfemurMm + kLtibiaMm + kEpsilon ||
        d < std::fabs(kLfemurMm - kLtibiaMm) - kEpsilon) {
        return {};
    }

    const float cos_tibia = clamp((d2 - kLfemurMm * kLfemurMm - kLtibiaMm * kLtibiaMm) /
                                      (2.0f * kLfemurMm * kLtibiaMm),
                                  -1.0f, 1.0f);
    const float tibia_abs = std::acos(cos_tibia);
    const std::array<float, 2> tibia_branches = {tibia_abs, -tibia_abs};

    IkResult best;
    float best_distance = 1.0e30f;
    for (float q_tibia_physical : tibia_branches) {
        const float q_femur_physical =
            std::atan2(p.z, r_mm) -
            std::atan2(kLtibiaMm * std::sin(q_tibia_physical),
                       kLfemurMm + kLtibiaMm * std::cos(q_tibia_physical));
        const JointAngles candidate =
            physicalToLogical({q_coxa_physical, q_femur_physical, q_tibia_physical});

        if (!jointsWithinLimits(candidate) || isSingular(candidate)) {
            continue;
        }
        const float score = distance2(candidate, previous_logical);
        if (!best.valid || score < best_distance) {
            best.valid = true;
            best.angles = candidate;
            best_distance = score;
        }
    }

    return best;
}

}  // namespace hexapod
