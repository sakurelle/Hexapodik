from __future__ import annotations

import argparse
import json
import logging
from math import pi
import sys
import time

from .calibration import PwmMapper, angle_to_pwm
from .config import LEGS, WIRE_CHANNELS, default_config_path, load_config
from .kinematics import Angles, KinematicsError, LegKinematics, Point
from .planning import feet_to_pwm, simulate_gait, solve_feet
from .protocol import PicoState
from .rc import RcReceiver
from .runtime import HexapodRuntime
from .serial_link import PicoLink


def _status_dict(status) -> dict[str, object]:
    return {
        "state": status.state.name,
        "error": status.error.name,
        "outputs_enabled": status.outputs_enabled,
        "contact_input_mask": f"0b{status.input_mask:06b}",
        "contact_inputs_high": [leg for index, leg in enumerate(LEGS) if status.input_mask & (1 << index)],
        "last_command_sequence": status.last_command_sequence,
        "dropped_protocol_frames": status.dropped_frames,
    }


def command_validate(config) -> int:
    solver = LegKinematics(config.lengths)
    center_deg = (0.0, 35.2666666667, -108.0833333333)
    center = Angles(*(value * pi / 180.0 for value in center_deg))
    control_fk = solver.forward(center)
    target = Point(0.150, 0.0, -0.100)
    target_ik = solver.inverse(target)
    round_trip = solver.forward(target_ik)
    unreachable_ok = False
    try:
        solver.inverse(Point(1.0, 0.0, 0.0))
    except KinematicsError:
        unreachable_ok = True
    gait = simulate_gait(config)
    result = {
        "configuration": "valid",
        "wire_channels": list(WIRE_CHANNELS),
        "calibrations_verified": config.all_calibrations_verified,
        "unverified_servos": list(PwmMapper(config).unverified()),
        "control_fk_m": [control_fk.x, control_fk.y, control_fk.z],
        "control_fk_error_m": ((control_fk.x - config.control_local[0]) ** 2 + (control_fk.z - config.control_local[2]) ** 2) ** 0.5,
        "target_ik_deg": [target_ik.coxa * 180 / pi, target_ik.femur * 180 / pi, target_ik.tibia * 180 / pi],
        "target_round_trip_error_m": ((round_trip.x - target.x) ** 2 + (round_trip.y - target.y) ** 2 + (round_trip.z - target.z) ** 2) ** 0.5,
        "unreachable_target_rejected": unreachable_ok,
        "gait_simulation": gait,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if result["control_fk_error_m"] > 2e-6 or result["target_round_trip_error_m"] > 1e-9:
        return 1
    if gait["legs_transferred"] != 6 or gait["return_error_m"] > 1e-9:
        return 1
    return 0


def command_pico_status(config) -> int:
    with PicoLink(config) as link:
        print(json.dumps(_status_dict(link.get_status()), indent=2, ensure_ascii=False))
    return 0


def command_disarm(config) -> int:
    with PicoLink(config) as link:
        link.disarm()
        status = link.get_status()
        print(json.dumps(_status_dict(status), indent=2, ensure_ascii=False))
        return 0 if status.state == PicoState.DISARMED and not status.outputs_enabled else 1


def command_rc_monitor(config, duration: float) -> int:
    deadline = None if duration <= 0 else time.monotonic() + duration
    with RcReceiver(config) as receiver:
        print(f"gpiochip={receiver.chip_path} label={config.rc_chip_label}; Ctrl-C to stop")
        while deadline is None or time.monotonic() < deadline:
            receiver.poll(0.1)
            now_ns = time.monotonic_ns()
            columns = []
            for name, sample in receiver.samples.items():
                width = "---" if sample.width_us is None else f"{sample.width_us:7.1f}us"
                age = sample.age_s(now_ns)
                age_text = "inf" if age == float("inf") else f"{age * 1000:6.1f}ms"
                columns.append(f"{name}: {width} age={age_text} valid={receiver.valid(name, now_ns)}")
            print(" | ".join(columns), flush=True)
    return 0


def _hold_frames(
    link: PicoLink,
    start: tuple[int, ...],
    target: tuple[int, ...],
    transition_s: float,
    hz: float,
    hold_s: float = 0.5,
) -> None:
    link.arm(start)
    status = link.get_status()
    if status.state != PicoState.ARMED:
        raise RuntimeError(f"Pico rejected ARM: {status}")
    period = 1.0 / hz
    steps = max(1, round(transition_s * hz))
    try:
        for index in range(steps):
            fraction = (index + 1) / steps
            fraction = fraction * fraction * (3.0 - 2.0 * fraction)
            frame = tuple(round(a + (b - a) * fraction) for a, b in zip(start, target))
            link.set_pwm(frame)
            time.sleep(period)
        for _ in range(max(1, round(hold_s * hz))):
            link.set_pwm(target)
            time.sleep(period)
    finally:
        link.disarm()


def command_calibrate_servo(config, name: str, delta_deg: float, confirmed: bool) -> int:
    if not confirmed:
        raise RuntimeError("refusing motion: isolate one unloaded servo and pass --confirm-isolated-servo")
    if name not in config.servos:
        raise RuntimeError(f"unknown servo {name!r}")
    if abs(delta_deg) > 5.0 or abs(delta_deg) < 0.1:
        raise RuntimeError("calibration delta must be between 0.1 and 5 degrees")
    mapper = PwmMapper(config)
    center = mapper.centers()
    calibration = config.servos[name]
    geometry_target = calibration.center_geometry + calibration.direction * delta_deg * pi / 180.0
    target = list(center)
    index = WIRE_CHANNELS.index(name)
    target[index] = angle_to_pwm(calibration, geometry_target)
    print(f"testing {name}: channel={index}, {center[index]} -> {target[index]} us")
    with PicoLink(config) as link:
        _hold_frames(link, center, tuple(target), 1.0, config.control_hz)
    return 0


def command_move_leg(config, leg: str, xyz: tuple[float, float, float], confirmed: bool) -> int:
    if not confirmed:
        raise RuntimeError("refusing motion: suspend the selected leg and pass --confirm-suspended-leg")
    solver = LegKinematics(config.lengths)
    control = Point(*config.control_local)
    angles = {name: solver.inverse(control) for name in LEGS}
    angles[leg] = solver.inverse(Point(*xyz))
    target = PwmMapper(config).map_all(angles)
    centers = PwmMapper(config).centers()
    with PicoLink(config) as link:
        _hold_frames(link, centers, target, 2.0, config.control_hz)
    return 0


def command_stand(config, hold_seconds: float, confirmed: bool) -> int:
    if not confirmed:
        raise RuntimeError("refusing motion: support the robot and pass --confirm-robot-supported")
    mapper = PwmMapper(config)
    if mapper.unverified():
        raise RuntimeError("refusing stance until all 18 servo calibrations are hardware-verified")
    solver = LegKinematics(config.lengths)
    control_angles = {leg: solver.inverse(Point(*config.control_local)) for leg in LEGS}
    neutral_angles = {leg: solver.inverse(Point(*config.neutral_local)) for leg in LEGS}
    start = mapper.map_all(control_angles)
    target = mapper.map_all(neutral_angles)
    with PicoLink(config) as link:
        _hold_frames(link, start, target, 3.0, config.control_hz, max(0.0, hold_seconds))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="hexapodctl")
    parser.add_argument("--config", default=str(default_config_path()))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("validate", help="validate config, FK/IK and the full gait offline")
    sub.add_parser("pico-status", help="read Pico state and six contact inputs")
    sub.add_parser("disarm", help="explicitly disable all Pico PWM outputs")
    monitor = sub.add_parser("rc-monitor", help="show RC pulse width, age and validity")
    monitor.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl-C")
    calibration = sub.add_parser("calibrate-servo", help="small movement of one isolated servo")
    calibration.add_argument("servo", choices=WIRE_CHANNELS)
    calibration.add_argument("--delta-deg", type=float, default=2.0)
    calibration.add_argument("--confirm-isolated-servo", action="store_true")
    move = sub.add_parser("move-leg", help="move one suspended leg to a local XYZ target")
    move.add_argument("leg", choices=LEGS)
    move.add_argument("x", type=float)
    move.add_argument("y", type=float)
    move.add_argument("z", type=float)
    move.add_argument("--confirm-suspended-leg", action="store_true")
    stand = sub.add_parser("stand", help="smoothly enter neutral stance, hold, then DISARM")
    stand.add_argument("--hold-seconds", type=float, default=5.0)
    stand.add_argument("--confirm-robot-supported", action="store_true")
    sub.add_parser("run", help="run the 50 Hz RC controller")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        config = load_config(args.config)
        if args.command == "validate":
            return command_validate(config)
        if args.command == "pico-status":
            return command_pico_status(config)
        if args.command == "disarm":
            return command_disarm(config)
        if args.command == "rc-monitor":
            return command_rc_monitor(config, args.duration)
        if args.command == "calibrate-servo":
            return command_calibrate_servo(config, args.servo, args.delta_deg, args.confirm_isolated_servo)
        if args.command == "move-leg":
            return command_move_leg(config, args.leg, (args.x, args.y, args.z), args.confirm_suspended_leg)
        if args.command == "stand":
            return command_stand(config, args.hold_seconds, args.confirm_robot_supported)
        if args.command == "run":
            return HexapodRuntime(config).run()
        raise AssertionError(args.command)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        logging.getLogger(__name__).error("%s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
