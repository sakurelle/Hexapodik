from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi
from pathlib import Path
import tomllib

LEGS = ("FL", "ML", "RL", "RR", "MR", "FR")
JOINTS_MATH = ("Coxa", "Femur", "Tibia")
JOINTS_WIRE = ("Tibia", "Femur", "Coxa")
WIRE_CHANNELS = tuple(f"{leg}.{joint}" for leg in LEGS for joint in JOINTS_WIRE)


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class Mount:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ServoCalibration:
    name: str
    center_geometry: float
    angles: tuple[float, float, float]
    pulses: tuple[int, int, int]
    direction: int
    limits: tuple[float, float]
    hardware_verified: bool


@dataclass(frozen=True)
class RcChannel:
    gpio: int
    center_us: int
    minimum_us: int
    maximum_us: int
    deadband_us: int
    inverted: bool


@dataclass(frozen=True)
class Config:
    path: Path
    uart_port: str
    uart_baud: int
    uart_timeout_s: float
    uart_lock: str
    lengths: tuple[float, float, float]
    neutral_local: tuple[float, float, float]
    control_local: tuple[float, float, float]
    mounts: dict[str, Mount]
    servos: dict[str, ServoCalibration]
    rc_chip_label: str
    rc_freshness_s: float
    rc_neutral_tolerance_us: int
    rc_enable_threshold_us: int
    rc_channels: dict[str, RcChannel]
    control_hz: float
    vx_max: float
    vy_max: float
    wz_max: float
    foot_lift: float
    transfer_s: float
    stability_margin: float
    com_xy: tuple[float, float]
    speed_filter_tau_s: float
    return_speed: float
    contact_levels_verified: bool
    contact_active_high: bool
    contact_search_limit: float
    transfer_order: tuple[str, ...]

    @property
    def all_calibrations_verified(self) -> bool:
        return all(item.hardware_verified for item in self.servos.values())


def _rad(value: float) -> float:
    return float(value) * pi / 180.0


def _triple(values: object, label: str) -> tuple[float, float, float]:
    if not isinstance(values, list) or len(values) != 3:
        raise ConfigError(f"{label} must contain exactly three values")
    result = tuple(float(value) for value in values)
    if not all(isfinite(value) for value in result):
        raise ConfigError(f"{label} contains a non-finite value")
    return result  # type: ignore[return-value]


def load_config(path: str | Path) -> Config:
    config_path = Path(path).resolve()
    with config_path.open("rb") as stream:
        raw = tomllib.load(stream)

    serial = raw["serial"]
    geometry = raw["geometry"]
    rc = raw["rc"]
    planner = raw["planner"]
    safety = raw["safety"]

    mounts: dict[str, Mount] = {}
    for leg in LEGS:
        item = raw["mounts"].get(leg)
        if item is None:
            raise ConfigError(f"missing mount for {leg}")
        mounts[leg] = Mount(float(item["x"]), float(item["y"]), float(item["z"]), _rad(item["yaw_deg"]))

    centers_deg = geometry["center_angles_deg"]
    servos: dict[str, ServoCalibration] = {}
    for name in WIRE_CHANNELS:
        item = raw["servos"].get(name)
        if item is None:
            raise ConfigError(f"missing servo calibration for {name}")
        joint = name.split(".")[1]
        angles_deg = _triple(item["angles_deg"], f"servos.{name}.angles_deg")
        pulses_raw = item["pulses_us"]
        limits_deg = item["limits_deg"]
        if len(pulses_raw) != 3 or len(limits_deg) != 2:
            raise ConfigError(f"invalid calibration point count for {name}")
        direction = int(item["direction"])
        if direction not in (-1, 1):
            raise ConfigError(f"direction for {name} must be -1 or 1")
        servos[name] = ServoCalibration(
            name=name,
            center_geometry=_rad(float(centers_deg[joint])),
            angles=tuple(_rad(value) for value in angles_deg),  # type: ignore[arg-type]
            pulses=tuple(int(value) for value in pulses_raw),  # type: ignore[arg-type]
            direction=direction,
            limits=(_rad(float(limits_deg[0])), _rad(float(limits_deg[1]))),
            hardware_verified=bool(item["hardware_verified"]),
        )

    rc_channels: dict[str, RcChannel] = {}
    for name in ("vx", "vy", "wz", "enable"):
        item = raw["rc_channels"].get(name)
        if item is None:
            raise ConfigError(f"missing RC channel {name}")
        rc_channels[name] = RcChannel(
            gpio=int(item["gpio"]),
            center_us=int(item["center_us"]),
            minimum_us=int(item["minimum_us"]),
            maximum_us=int(item["maximum_us"]),
            deadband_us=int(item["deadband_us"]),
            inverted=bool(item["inverted"]),
        )

    result = Config(
        path=config_path,
        uart_port=str(serial["port"]),
        uart_baud=int(serial["baud"]),
        uart_timeout_s=float(serial["response_timeout_s"]),
        uart_lock=str(serial["lock_file"]),
        lengths=_triple(geometry["lengths_m"], "geometry.lengths_m"),
        neutral_local=_triple(geometry["neutral_local_m"], "geometry.neutral_local_m"),
        control_local=_triple(geometry["control_local_m"], "geometry.control_local_m"),
        mounts=mounts,
        servos=servos,
        rc_chip_label=str(rc["gpiochip_label"]),
        rc_freshness_s=float(rc["freshness_s"]),
        rc_neutral_tolerance_us=int(rc["neutral_tolerance_us"]),
        rc_enable_threshold_us=int(rc["enable_threshold_us"]),
        rc_channels=rc_channels,
        control_hz=float(planner["control_hz"]),
        vx_max=float(planner["vx_max_m_s"]),
        vy_max=float(planner["vy_max_m_s"]),
        wz_max=float(planner["wz_max_rad_s"]),
        foot_lift=float(planner["foot_lift_m"]),
        transfer_s=float(planner["transfer_s"]),
        stability_margin=float(planner["stability_margin_m"]),
        com_xy=(float(planner["com_x_m"]), float(planner["com_y_m"])),
        speed_filter_tau_s=float(planner["speed_filter_tau_s"]),
        return_speed=float(planner["return_speed_m_s"]),
        contact_levels_verified=bool(safety["contact_levels_verified"]),
        contact_active_high=bool(safety["contact_active_high"]),
        contact_search_limit=float(safety["contact_search_limit_m"]),
        transfer_order=tuple(str(item) for item in planner["transfer_order"]),
    )
    validate_config(result)
    return result


def validate_config(config: Config) -> None:
    if config.uart_baud <= 0 or config.control_hz <= 0:
        raise ConfigError("UART baud and control frequency must be positive")
    if set(config.transfer_order) != set(LEGS) or len(config.transfer_order) != len(LEGS):
        raise ConfigError("planner.transfer_order must contain every leg exactly once")
    if set(config.servos) != set(WIRE_CHANNELS):
        raise ConfigError("servo table must exactly match the 18 wire channels")
    gpios = [channel.gpio for channel in config.rc_channels.values()]
    if len(set(gpios)) != 4:
        raise ConfigError("RC GPIO offsets must be unique")
    for name, servo in config.servos.items():
        if not (servo.angles[0] < servo.angles[1] < servo.angles[2]):
            raise ConfigError(f"calibration angles are not increasing for {name}")
        if not (servo.pulses[0] < servo.pulses[1] < servo.pulses[2]):
            raise ConfigError(f"calibration PWM values are not increasing for {name}")
        if servo.limits[0] >= servo.limits[1]:
            raise ConfigError(f"mechanical limits are invalid for {name}")
        if min(servo.pulses) < 1000 or max(servo.pulses) > 2000:
            raise ConfigError(f"Pico PWM range exceeded by {name}")
    for name, channel in config.rc_channels.items():
        if not (channel.minimum_us < channel.center_us < channel.maximum_us):
            raise ConfigError(f"invalid RC range for {name}")


def default_config_path() -> Path:
    installed = Path("/home/pi/hexapod/config/hexapod.toml")
    if installed.exists():
        return installed
    return Path(__file__).resolve().parents[2] / "config" / "hexapod.toml"
