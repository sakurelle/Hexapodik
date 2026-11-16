from __future__ import annotations

from dataclasses import dataclass
import glob
import select
import time

from .config import Config, RcChannel


class RcError(RuntimeError):
    pass


@dataclass
class PulseSample:
    width_us: float | None = None
    timestamp_ns: int | None = None
    rising_ns: int | None = None

    def age_s(self, now_ns: int | None = None) -> float:
        if self.timestamp_ns is None:
            return float("inf")
        return ((now_ns or time.monotonic_ns()) - self.timestamp_ns) / 1e9


def _discover_chip(gpiod, expected_label: str, offsets: list[int]):
    errors: list[str] = []
    for path in sorted(glob.glob("/dev/gpiochip*")):
        try:
            chip = gpiod.Chip(path, gpiod.Chip.OPEN_BY_PATH)
            label = chip.label()
            if label == expected_label and max(offsets) < chip.num_lines():
                return chip, path
            chip.close()
        except OSError as exc:
            errors.append(f"{path}: {exc}")
    detail = "; ".join(errors) if errors else "no GPIO character devices found"
    raise RcError(f"gpiochip with label {expected_label!r} not available: {detail}")


class RcReceiver:
    """Four-channel PWM reader using libgpiod 1.x kernel edge timestamps."""

    def __init__(self, config: Config):
        self.config = config
        self.chip = None
        self.chip_path = ""
        self.lines = {}
        self.fd_names: dict[int, str] = {}
        self.samples = {name: PulseSample() for name in config.rc_channels}

    def open(self) -> "RcReceiver":
        try:
            import gpiod
        except ImportError as exc:
            raise RcError("python3 gpiod module is not installed") from exc
        if hasattr(gpiod, "request_lines"):
            raise RcError("libgpiod API 2.x detected; this build expects Ubuntu 24.04 API 1.x")
        offsets = [channel.gpio for channel in self.config.rc_channels.values()]
        self.chip, self.chip_path = _discover_chip(gpiod, self.config.rc_chip_label, offsets)
        try:
            for name, channel in self.config.rc_channels.items():
                line = self.chip.get_line(channel.gpio)
                line.request(consumer="hexapod-rc", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
                self.lines[name] = line
                self.fd_names[line.event_get_fd()] = name
        except Exception:
            self.close()
            raise
        return self

    def close(self) -> None:
        for line in self.lines.values():
            line.release()
        self.lines.clear()
        self.fd_names.clear()
        if self.chip is not None:
            self.chip.close()
            self.chip = None

    def __enter__(self) -> "RcReceiver":
        return self.open()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def poll(self, timeout_s: float = 0.0) -> None:
        if not self.fd_names:
            raise RcError("RC receiver is not open")
        ready, _, _ = select.select(list(self.fd_names), [], [], timeout_s)
        for fd in ready:
            name = self.fd_names[fd]
            line = self.lines[name]
            while line.event_wait(sec=0):
                event = line.event_read()
                sample = self.samples[name]
                if event.type == event.RISING_EDGE:
                    sample.rising_ns = event.timestamp
                elif event.type == event.FALLING_EDGE and sample.rising_ns is not None:
                    width_us = (event.timestamp - sample.rising_ns) / 1000.0
                    if 500.0 <= width_us <= 2500.0:
                        sample.width_us = width_us
                        sample.timestamp_ns = event.timestamp
                    sample.rising_ns = None

    def valid(self, name: str, now_ns: int | None = None) -> bool:
        sample = self.samples[name]
        channel = self.config.rc_channels[name]
        return (
            sample.width_us is not None
            and channel.minimum_us - 100 <= sample.width_us <= channel.maximum_us + 100
            and sample.age_s(now_ns) <= self.config.rc_freshness_s
        )

    def all_fresh(self, now_ns: int | None = None) -> bool:
        return all(self.valid(name, now_ns) for name in self.samples)

    def normalized(self, name: str) -> float:
        sample = self.samples[name]
        if sample.width_us is None:
            raise RcError(f"no RC sample for {name}")
        channel = self.config.rc_channels[name]
        delta = sample.width_us - channel.center_us
        if abs(delta) <= channel.deadband_us:
            value = 0.0
        elif delta > 0:
            value = (delta - channel.deadband_us) / (channel.maximum_us - channel.center_us - channel.deadband_us)
        else:
            value = (delta + channel.deadband_us) / (channel.center_us - channel.minimum_us - channel.deadband_us)
        value = max(-1.0, min(1.0, value))
        return -value if channel.inverted else value

    def sticks_neutral(self) -> bool:
        return all(
            self.samples[name].width_us is not None
            and abs(self.samples[name].width_us - self.config.rc_channels[name].center_us)
            <= self.config.rc_neutral_tolerance_us
            for name in ("vx", "vy", "wz")
        )

    def enabled(self) -> bool:
        sample = self.samples["enable"].width_us
        if sample is None:
            return False
        channel = self.config.rc_channels["enable"]
        raw = sample >= self.config.rc_enable_threshold_us
        return not raw if channel.inverted else raw

    def command(self) -> tuple[float, float, float]:
        return (
            self.normalized("vx") * self.config.vx_max,
            self.normalized("vy") * self.config.vy_max,
            self.normalized("wz") * self.config.wz_max,
        )
