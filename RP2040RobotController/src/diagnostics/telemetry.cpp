#include "diagnostics/telemetry.h"

#include <cstdio>

namespace hexapod {

std::uint32_t contactRawMask(const std::array<ContactState, kLegCount>& contacts) {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i].raw) {
            mask |= 1u << i;
        }
    }
    return mask;
}

std::uint32_t contactStableMask(const std::array<ContactState, kLegCount>& contacts) {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i].stable) {
            mask |= 1u << i;
        }
    }
    return mask;
}

std::uint32_t contactHealthyMask(const std::array<ContactState, kLegCount>& contacts) {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i].healthy) {
            mask |= 1u << i;
        }
    }
    return mask;
}

std::uint32_t supportMask(const LocomotionStepResult& step) {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (step.leg_phase[i] == LegPhase::Stance || step.leg_phase[i] == LegPhase::LandedHold) {
            mask |= 1u << i;
        }
    }
    return mask;
}

int formatTelemetryLine(const Telemetry& telemetry, char* buffer, std::size_t buffer_size) {
    if (buffer == nullptr || buffer_size == 0) {
        return 0;
    }
    return std::snprintf(buffer,
                         buffer_size,
                         "state=%s src=%s control=%s seq=%lu age_ms=%lu cmd=%.3f,%.3f,%.3f filtered=%.1f,%.1f,%.3f cycle=%.3f level=%.2f stride_x=%.1f stride_y=%.1f phase=%.3f contacts=0x%02lx support=0x%02lx health=0x%02lx block=%s recovery=%s workspace=%.2f joint_vmax=%.1f ik_errors=%lu faults=0x%08lx",
                         robotStateName(telemetry.robot_state),
                         commandSourceModeName(telemetry.command_source),
                         controlModeName(telemetry.control_mode),
                         static_cast<unsigned long>(telemetry.last_command_sequence),
                         static_cast<unsigned long>(telemetry.command_age_ms),
                         telemetry.requested_command.vx_mps,
                         telemetry.requested_command.vy_mps,
                         telemetry.requested_command.wz_radps,
                         telemetry.filtered_command.vx_mm_s,
                         telemetry.filtered_command.vy_mm_s,
                         telemetry.filtered_command.yaw_rad_s,
                         telemetry.cycle_hz,
                         telemetry.command_level,
                         telemetry.stride_x_mm,
                         telemetry.stride_y_mm,
                         telemetry.phase,
                         static_cast<unsigned long>(telemetry.stable_contact_mask),
                         static_cast<unsigned long>(telemetry.support_mask),
                         static_cast<unsigned long>(telemetry.healthy_sensor_mask),
                         blockReasonName(telemetry.block_reason),
                         recoveryReasonName(telemetry.recovery_reason),
                         telemetry.workspace_scale,
                         telemetry.max_requested_joint_velocity_deg_s,
                         static_cast<unsigned long>(telemetry.ik_error_counter),
                         static_cast<unsigned long>(telemetry.fault_flags));
}

}  // namespace hexapod
