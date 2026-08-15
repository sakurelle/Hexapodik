#include "robot/kinematics.hpp"

#include <cmath>

namespace robot {
namespace {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

float clamp_unit(float value) {
    return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}

Vector3 to_leg_frame(const LegGeometry &leg, const FootTarget &target) {
    const float dx = target.x_mm - leg.body_mount.x_mm;
    const float dy = target.y_mm - leg.body_mount.y_mm;
    const float c = std::cos(leg.mount_yaw_rad);
    const float s = std::sin(leg.mount_yaw_rad);
    return Vector3{c * dx + s * dy, -s * dx + c * dy,
                   target.z_mm - leg.body_mount.z_mm};
}

FootTarget to_body_frame(const LegGeometry &leg, const Vector3 &local) {
    const float c = std::cos(leg.mount_yaw_rad);
    const float s = std::sin(leg.mount_yaw_rad);
    return FootTarget{leg.body_mount.x_mm + c * local.x_mm - s * local.y_mm,
                      leg.body_mount.y_mm + s * local.x_mm + c * local.y_mm,
                      leg.body_mount.z_mm + local.z_mm};
}

} // namespace

Vector3 body_to_leg_frame(const RobotGeometry &geometry, servo::Leg leg,
                          const FootTarget &target) {
    return to_leg_frame(geometry.legs[servo::leg_index(leg)], target);
}

FootTarget leg_to_body_frame(const RobotGeometry &geometry, servo::Leg leg,
                             const Vector3 &local) {
    return to_body_frame(geometry.legs[servo::leg_index(leg)], local);
}

IkResult solve_leg_ik(const RobotGeometry &geometry, servo::Leg leg,
                      const FootTarget &target) {
    IkResult result{};
    if (!geometry_is_valid(geometry)) {
        return result;
    }

    const Vector3 local = body_to_leg_frame(geometry, leg, target);
    const float coxa_rad = std::atan2(local.y_mm, local.x_mm);
    const float radial = std::sqrt(local.x_mm * local.x_mm + local.y_mm * local.y_mm) -
                         geometry.coxa_length_mm;
    const float z = local.z_mm;
    const float distance_sq = radial * radial + z * z;
    const float distance = std::sqrt(distance_sq);
    const float femur = geometry.femur_length_mm;
    const float tibia = geometry.tibia_length_mm;
    if (distance <= 0.0f || distance > femur + tibia ||
        distance < std::fabs(femur - tibia)) {
        return result;
    }

    const float cosine_knee = clamp_unit((distance_sq - femur * femur - tibia * tibia) /
                                         (2.0f * femur * tibia));
    // The measured logical Tibia convention is opposite the physical relative
    // angle.  The negative elbow branch matches tibia_zero_offset_deg.
    const float tibia_relative_rad = -std::acos(cosine_knee);
    const float femur_rad = std::atan2(z, radial) -
                            std::atan2(tibia * std::sin(tibia_relative_rad),
                                       femur + tibia * std::cos(tibia_relative_rad));
    result.reachable = true;
    result.pose = LegPose{
        coxa_rad * RAD_TO_DEG,
        femur_rad * RAD_TO_DEG - geometry.femur_zero_offset_deg,
        geometry.tibia_zero_offset_deg - tibia_relative_rad * RAD_TO_DEG,
    };
    return result;
}

FootTarget forward_kinematics(const RobotGeometry &geometry, servo::Leg leg,
                              const LegPose &pose) {
    if (!geometry_is_valid(geometry)) {
        return FootTarget{};
    }
    const float coxa = pose.coxa_deg * DEG_TO_RAD;
    const float femur = (geometry.femur_zero_offset_deg + pose.femur_deg) * DEG_TO_RAD;
    const float tibia_relative =
        (geometry.tibia_zero_offset_deg - pose.tibia_deg) * DEG_TO_RAD;
    const float tibia_absolute = femur + tibia_relative;
    const float radial = geometry.coxa_length_mm +
                         geometry.femur_length_mm * std::cos(femur) +
                         geometry.tibia_length_mm * std::cos(tibia_absolute);
    const Vector3 local{radial * std::cos(coxa), radial * std::sin(coxa),
                        geometry.femur_length_mm * std::sin(femur) +
                        geometry.tibia_length_mm * std::sin(tibia_absolute)};
    return leg_to_body_frame(geometry, leg, local);
}

} // namespace robot
