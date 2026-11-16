from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from .config import Config, LEGS
from .kinematics import Point, neutral_body_points


def _cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    lower: list[tuple[float, float]] = []
    for point in points:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(points):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def support_margin(points: list[tuple[float, float]], center: tuple[float, float]) -> float:
    hull = convex_hull(points)
    if len(hull) < 3:
        return float("-inf")
    margin = float("inf")
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        edge_x, edge_y = end[0] - start[0], end[1] - start[1]
        length = hypot(edge_x, edge_y)
        signed_distance = (edge_x * (center[1] - start[1]) - edge_y * (center[0] - start[0])) / length
        margin = min(margin, signed_distance)
    return margin


def _smoothstep5(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)


def _lift_bump(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 16.0 * value * value * (1.0 - value) * (1.0 - value)


@dataclass
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def near_zero(self) -> bool:
        return abs(self.vx) < 1e-5 and abs(self.vy) < 1e-5 and abs(self.wz) < 1e-4


@dataclass
class Transfer:
    leg: str
    elapsed: float
    start: Point
    target: Point
    contact_z: float | None = None
    search_depth: float = 0.0


class GaitPlanner:
    """Slow statically-stable gait with at most one foot in transfer."""

    def __init__(self, config: Config):
        self.config = config
        self.neutral = neutral_body_points(config)
        self.feet = dict(self.neutral)
        self.filtered = Twist()
        self.transfer: Transfer | None = None
        self.next_index = 0
        self.mode = "STANDING"
        self.hold_reason: str | None = None
        self.fault_reason: str | None = None

    def reset(self) -> None:
        self.feet = dict(self.neutral)
        self.filtered = Twist()
        self.transfer = None
        self.next_index = 0
        self.mode = "STANDING"
        self.hold_reason = None
        self.fault_reason = None

    def _filter(self, requested: Twist, dt: float) -> None:
        alpha = min(1.0, dt / max(self.config.speed_filter_tau_s, dt))
        self.filtered.vx += alpha * (requested.vx - self.filtered.vx)
        self.filtered.vy += alpha * (requested.vy - self.filtered.vy)
        self.filtered.wz += alpha * (requested.wz - self.filtered.wz)

    def _stance_integrate(self, leg: str, dt: float) -> None:
        point = self.feet[leg]
        vx = -self.filtered.vx + self.filtered.wz * point.y
        vy = -self.filtered.vy - self.filtered.wz * point.x
        self.feet[leg] = Point(point.x + vx * dt, point.y + vy * dt, point.z)

    def _can_transfer(self, leg: str) -> tuple[bool, float]:
        supports = [(point.x, point.y) for name, point in self.feet.items() if name != leg]
        margin = support_margin(supports, self.config.com_xy)
        return margin >= self.config.stability_margin, margin

    def _landing_target(self, leg: str) -> Point:
        neutral = self.neutral[leg]
        # Five other legs transfer before this leg moves again. Landing half of
        # that stance interval ahead keeps the step centered around neutral.
        lead = self.config.transfer_s * 2.5
        rotational_x = -self.filtered.wz * neutral.y
        rotational_y = self.filtered.wz * neutral.x
        return Point(
            neutral.x + lead * (self.filtered.vx + rotational_x),
            neutral.y + lead * (self.filtered.vy + rotational_y),
            neutral.z,
        )

    def _start_transfer(self) -> None:
        for offset in range(len(self.config.transfer_order)):
            index = (self.next_index + offset) % len(self.config.transfer_order)
            leg = self.config.transfer_order[index]
            allowed, margin = self._can_transfer(leg)
            if allowed:
                self.transfer = Transfer(leg, 0.0, self.feet[leg], self._landing_target(leg))
                self.next_index = (index + 1) % len(self.config.transfer_order)
                self.hold_reason = None
                return
        self.hold_reason = (
            f"no leg can be transferred with required support margin "
            f"{self.config.stability_margin:.3f} m"
        )

    def _return_to_neutral(self, dt: float) -> None:
        complete = True
        max_step = self.config.return_speed * dt
        for leg in LEGS:
            point, target = self.feet[leg], self.neutral[leg]
            dx, dy, dz = target.x - point.x, target.y - point.y, target.z - point.z
            distance = (dx * dx + dy * dy + dz * dz) ** 0.5
            if distance > max_step and distance > 0:
                scale = max_step / distance
                self.feet[leg] = Point(point.x + dx * scale, point.y + dy * scale, point.z + dz * scale)
                complete = False
            else:
                self.feet[leg] = target
        self.mode = "STANDING" if complete else "STOPPING"

    def update(self, requested: Twist, dt: float, contact_mask: int | None = None) -> dict[str, Point]:
        if dt <= 0 or dt > 0.25:
            raise ValueError(f"planner dt {dt:.6f} s is invalid")
        requested = Twist(
            max(-self.config.vx_max, min(self.config.vx_max, requested.vx)),
            max(-self.config.vy_max, min(self.config.vy_max, requested.vy)),
            max(-self.config.wz_max, min(self.config.wz_max, requested.wz)),
        )
        if self.fault_reason is not None:
            self.hold_reason = self.fault_reason
            self.mode = "STANDING"
            return dict(self.feet)
        self._filter(requested, dt)
        stopping = requested.near_zero()

        if self.transfer is None and stopping:
            self._return_to_neutral(dt)
            return dict(self.feet)

        if self.transfer is None:
            self._start_transfer()
            if self.transfer is None:
                self.mode = "STANDING"
                return dict(self.feet)

        active = self.transfer
        assert active is not None
        for leg in LEGS:
            if leg != active.leg:
                self._stance_integrate(leg, dt)

        active.elapsed = min(self.config.transfer_s, active.elapsed + dt)
        phase = active.elapsed / self.config.transfer_s
        progress = _smoothstep5(phase)
        z = active.start.z + (active.target.z - active.start.z) * progress + self.config.foot_lift * _lift_bump(phase)
        if self.config.contact_levels_verified and contact_mask is not None and phase >= 0.5:
            bit_set = bool(contact_mask & (1 << LEGS.index(active.leg)))
            contact = bit_set if self.config.contact_active_high else not bit_set
            if contact and active.contact_z is None:
                active.contact_z = self.feet[active.leg].z
        if active.contact_z is not None:
            z = max(z, active.contact_z)
        self.feet[active.leg] = Point(
            active.start.x + (active.target.x - active.start.x) * progress,
            active.start.y + (active.target.y - active.start.y) * progress,
            z,
        )
        self.mode = "STOPPING" if stopping else "WALKING"
        if phase >= 1.0:
            if self.config.contact_levels_verified and active.contact_z is None:
                step = min(self.config.return_speed * dt, self.config.contact_search_limit - active.search_depth)
                if step > 0:
                    active.search_depth += step
                    self.feet[active.leg] = Point(active.target.x, active.target.y, active.target.z - active.search_depth)
                    return dict(self.feet)
                self.hold_reason = (
                    f"{active.leg}: no contact within {self.config.contact_search_limit:.3f} m search limit"
                )
                self.fault_reason = self.hold_reason
                self.transfer = None
                self.mode = "STANDING"
                return dict(self.feet)
            self.feet[active.leg] = Point(
                active.target.x,
                active.target.y,
                active.contact_z if active.contact_z is not None else active.target.z,
            )
            self.transfer = None
        return dict(self.feet)
