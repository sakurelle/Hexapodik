from __future__ import annotations

from math import isfinite

from .config import Config, JOINTS_MATH, LEGS, ServoCalibration, WIRE_CHANNELS
from .kinematics import Angles


class CalibrationError(ValueError):
    pass


def angle_to_pwm(calibration: ServoCalibration, geometry_angle: float) -> int:
    if not isfinite(geometry_angle):
        raise CalibrationError(f"non-finite angle for {calibration.name}")
    relative = calibration.direction * (geometry_angle - calibration.center_geometry)
    low_limit, high_limit = calibration.limits
    if relative < low_limit or relative > high_limit:
        raise CalibrationError(
            f"{calibration.name}: angle {relative:.6f} rad is outside mechanical limits "
            f"[{low_limit:.6f}, {high_limit:.6f}]"
        )
    angles, pulses = calibration.angles, calibration.pulses
    if relative < angles[0] or relative > angles[-1]:
        raise CalibrationError(f"{calibration.name}: angle lies outside calibrated range")
    segment = 0 if relative <= angles[1] else 1
    fraction = (relative - angles[segment]) / (angles[segment + 1] - angles[segment])
    pulse = round(pulses[segment] + fraction * (pulses[segment + 1] - pulses[segment]))
    if pulse < 1000 or pulse > 2000:
        raise CalibrationError(f"{calibration.name}: interpolated PWM {pulse} us is invalid")
    return pulse


class PwmMapper:
    def __init__(self, config: Config):
        self.config = config

    def map_all(self, leg_angles: dict[str, Angles]) -> tuple[int, ...]:
        if set(leg_angles) != set(LEGS):
            raise CalibrationError("angles must be supplied for all six legs")
        by_name: dict[str, float] = {}
        for leg, angles in leg_angles.items():
            by_name[f"{leg}.Coxa"] = angles.coxa
            by_name[f"{leg}.Femur"] = angles.femur
            by_name[f"{leg}.Tibia"] = angles.tibia
        return tuple(angle_to_pwm(self.config.servos[name], by_name[name]) for name in WIRE_CHANNELS)

    def centers(self) -> tuple[int, ...]:
        return tuple(angle_to_pwm(item, item.center_geometry) for item in (self.config.servos[name] for name in WIRE_CHANNELS))

    def unverified(self) -> tuple[str, ...]:
        return tuple(name for name in WIRE_CHANNELS if not self.config.servos[name].hardware_verified)
