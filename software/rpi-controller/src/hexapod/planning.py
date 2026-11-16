from __future__ import annotations

from .calibration import PwmMapper
from .config import Config, LEGS
from .gait import GaitPlanner, Twist
from .kinematics import Angles, LegKinematics, Point, body_to_leg


def solve_feet(config: Config, feet_body: dict[str, Point]) -> dict[str, Angles]:
    solver = LegKinematics(config.lengths)
    result: dict[str, Angles] = {}
    for leg in LEGS:
        result[leg] = solver.inverse(body_to_leg(feet_body[leg], config.mounts[leg]))
    return result


def feet_to_pwm(config: Config, feet_body: dict[str, Point]) -> tuple[int, ...]:
    return PwmMapper(config).map_all(solve_feet(config, feet_body))


def simulate_gait(config: Config, duration_s: float = 12.0) -> dict[str, float | int]:
    planner = GaitPlanner(config)
    dt = 1.0 / config.control_hz
    command = Twist(config.vx_max, config.vy_max * 0.5, config.wz_max * 0.5)
    minimum_pwm, maximum_pwm = 2000, 1000
    samples = 0
    transfers: set[str] = set()
    previous_transfer = None
    for _ in range(round(duration_s / dt)):
        feet = planner.update(command, dt)
        pwm = feet_to_pwm(config, feet)
        minimum_pwm = min(minimum_pwm, *pwm)
        maximum_pwm = max(maximum_pwm, *pwm)
        if planner.transfer is not None:
            transfers.add(planner.transfer.leg)
            previous_transfer = planner.transfer.leg
        samples += 1
    for _ in range(round(8.0 / dt)):
        feet = planner.update(Twist(), dt)
        pwm = feet_to_pwm(config, feet)
        minimum_pwm = min(minimum_pwm, *pwm)
        maximum_pwm = max(maximum_pwm, *pwm)
        samples += 1
    max_neutral_error = max(
        ((planner.feet[leg].x - planner.neutral[leg].x) ** 2
         + (planner.feet[leg].y - planner.neutral[leg].y) ** 2
         + (planner.feet[leg].z - planner.neutral[leg].z) ** 2) ** 0.5
        for leg in LEGS
    )
    return {
        "samples": samples,
        "legs_transferred": len(transfers),
        "minimum_pwm_us": minimum_pwm,
        "maximum_pwm_us": maximum_pwm,
        "return_error_m": max_neutral_error,
    }
