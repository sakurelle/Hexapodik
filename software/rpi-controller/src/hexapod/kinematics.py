from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin, sqrt

from .config import Config, Mount


class KinematicsError(ValueError):
    pass


@dataclass(frozen=True)
class Angles:
    coxa: float
    femur: float
    tibia: float

    def finite(self) -> bool:
        return all(isfinite(value) for value in (self.coxa, self.femur, self.tibia))


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float

    def finite(self) -> bool:
        return all(isfinite(value) for value in (self.x, self.y, self.z))


class LegKinematics:
    def __init__(self, lengths: tuple[float, float, float], numerical_tolerance: float = 1e-9):
        self.lc, self.lf, self.lt = lengths
        self.tolerance = numerical_tolerance
        if min(lengths) <= 0 or not all(isfinite(value) for value in lengths):
            raise KinematicsError("link lengths must be finite and positive")

    def forward(self, angles: Angles) -> Point:
        if not angles.finite():
            raise KinematicsError("joint angles are not finite")
        rho = self.lc + self.lf * cos(angles.femur) + self.lt * cos(angles.femur + angles.tibia)
        return Point(
            rho * cos(angles.coxa),
            rho * sin(angles.coxa),
            self.lf * sin(angles.femur) + self.lt * sin(angles.femur + angles.tibia),
        )

    def inverse(self, point: Point, singularity_margin: float = 1e-5) -> Angles:
        if not point.finite():
            raise KinematicsError("target point is not finite")
        radial = hypot(point.x, point.y)
        r = radial - self.lc
        d = (r * r + point.z * point.z - self.lf * self.lf - self.lt * self.lt) / (2 * self.lf * self.lt)
        if d < -1.0 - self.tolerance or d > 1.0 + self.tolerance:
            raise KinematicsError(f"target is unreachable (D={d:.9f})")
        d = min(1.0, max(-1.0, d))
        root_sq = max(0.0, 1.0 - d * d)
        if sqrt(root_sq) < singularity_margin:
            raise KinematicsError("target is too close to a knee singularity")
        tibia = atan2(-sqrt(root_sq), d)
        femur = atan2(point.z, r) - atan2(self.lt * sin(tibia), self.lf + self.lt * cos(tibia))
        result = Angles(atan2(point.y, point.x), femur, tibia)
        if not result.finite() or result.tibia >= 0:
            raise KinematicsError("invalid IK result")
        return result


def leg_to_body(point: Point, mount: Mount) -> Point:
    c, s = cos(mount.yaw), sin(mount.yaw)
    return Point(
        mount.x + c * point.x - s * point.y,
        mount.y + s * point.x + c * point.y,
        mount.z + point.z,
    )


def body_to_leg(point: Point, mount: Mount) -> Point:
    dx, dy = point.x - mount.x, point.y - mount.y
    c, s = cos(mount.yaw), sin(mount.yaw)
    return Point(c * dx + s * dy, -s * dx + c * dy, point.z - mount.z)


def neutral_body_points(config: Config) -> dict[str, Point]:
    local = Point(*config.neutral_local)
    return {leg: leg_to_body(local, config.mounts[leg]) for leg in config.mounts}
