from __future__ import annotations

import logging
import signal
import time

from .calibration import PwmMapper
from .config import Config, LEGS
from .gait import GaitPlanner, Twist
from .kinematics import Point, leg_to_body
from .planning import feet_to_pwm
from .protocol import PicoState
from .rc import RcReceiver
from .serial_link import PicoLink, SerialLinkError

LOG = logging.getLogger(__name__)


class RuntimeFault(RuntimeError):
    pass


def _interpolate(start: Point, end: Point, value: float) -> Point:
    value = value * value * (3.0 - 2.0 * value)
    return Point(
        start.x + (end.x - start.x) * value,
        start.y + (end.y - start.y) * value,
        start.z + (end.z - start.z) * value,
    )


def smooth_stand(link: PicoLink, rc: RcReceiver, config: Config, duration_s: float = 3.0) -> None:
    mapper = PwmMapper(config)
    unverified = mapper.unverified()
    if unverified:
        raise RuntimeFault("calibration is not hardware-verified: " + ", ".join(unverified))
    centers = mapper.centers()
    link.arm(centers)
    status = link.get_status()
    if status.state != PicoState.ARMED or not status.outputs_enabled:
        raise RuntimeFault(f"Pico rejected ARM: state={status.state.name}, error={status.error.name}")

    control_local = Point(*config.control_local)
    neutral_local = Point(*config.neutral_local)
    control = {leg: leg_to_body(control_local, config.mounts[leg]) for leg in LEGS}
    neutral = {leg: leg_to_body(neutral_local, config.mounts[leg]) for leg in LEGS}
    period = 1.0 / config.control_hz
    started = time.monotonic()
    deadline = started
    while True:
        rc.poll(0.0)
        if not rc.all_fresh() or not rc.enabled():
            raise RuntimeFault("RC became stale or enable switched OFF during stand transition")
        progress = min(1.0, (time.monotonic() - started) / duration_s)
        feet = {leg: _interpolate(control[leg], neutral[leg], progress) for leg in LEGS}
        link.set_pwm(feet_to_pwm(config, feet))
        if progress >= 1.0:
            return
        deadline += period
        time.sleep(max(0.0, deadline - time.monotonic()))


class HexapodRuntime:
    def __init__(self, config: Config):
        self.config = config
        self.stop_requested = False
        self.state = "DISARMED"
        self.missed_deadlines = 0

    def request_stop(self, _signum=None, _frame=None) -> None:
        self.stop_requested = True

    def run(self) -> int:
        signal.signal(signal.SIGTERM, self.request_stop)
        signal.signal(signal.SIGINT, self.request_stop)
        planner = GaitPlanner(self.config)
        mapper = PwmMapper(self.config)
        period = 1.0 / self.config.control_hz
        enable_off_seen = False
        last_status_time = 0.0
        contact_mask = 0
        previous_contact_mask: int | None = None
        calibration_warning_logged = False

        with PicoLink(self.config) as link, RcReceiver(self.config) as rc:
            try:
                link.disarm()
                status = link.get_status()
                if status.state != PicoState.DISARMED or status.outputs_enabled:
                    raise RuntimeFault(f"Pico is not safely disarmed: {status}")
                LOG.info("controller started DISARMED; Pico link and RC GPIO are open")
                deadline = time.monotonic()
                previous = deadline
                while not self.stop_requested:
                    now = time.monotonic()
                    dt = min(0.1, max(1e-6, now - previous))
                    previous = now
                    rc.poll(0.0)
                    fresh = rc.all_fresh()
                    enabled = fresh and rc.enabled()

                    if self.state == "DISARMED":
                        if fresh and not enabled:
                            enable_off_seen = True
                        if not self.config.all_calibrations_verified and not calibration_warning_logged:
                            LOG.warning("ARM inhibited: 18 servo directions/limits are not hardware-verified")
                            calibration_warning_logged = True
                        can_arm = (
                            self.config.all_calibrations_verified
                            and fresh
                            and rc.sticks_neutral()
                            and enable_off_seen
                            and enabled
                        )
                        if can_arm:
                            latest = link.get_status()
                            if latest.state != PicoState.DISARMED or latest.outputs_enabled:
                                raise RuntimeFault(f"Pico pre-ARM state is unsafe: {latest}")
                            LOG.info("valid OFF->ON edge received; entering neutral stance")
                            try:
                                smooth_stand(link, rc, self.config)
                            except Exception:
                                link.disarm()
                                raise
                            planner.reset()
                            self.state = "STANDING"
                            enable_off_seen = False
                    else:
                        if not fresh:
                            requested = Twist()
                            self.state = "STOPPING"
                        elif not enabled:
                            requested = Twist()
                            if planner.mode != "STANDING":
                                self.state = "STOPPING"
                        else:
                            requested = Twist(*rc.command())
                        feet = planner.update(requested, dt, contact_mask)
                        link.set_pwm(feet_to_pwm(self.config, feet))
                        self.state = planner.mode
                        if planner.hold_reason:
                            LOG.error("planner holding stance: %s", planner.hold_reason)

                    if now - last_status_time >= 0.2:
                        status = link.get_status()
                        last_status_time = now
                        contact_mask = status.input_mask
                        if previous_contact_mask != contact_mask:
                            LOG.info("Pico contact input mask changed: 0b%s", format(contact_mask, "06b"))
                            previous_contact_mask = contact_mask
                        if status.state in (PicoState.COMMUNICATION_FAULT, PicoState.OUTPUT_FAULT):
                            raise RuntimeFault(f"Pico fault: state={status.state.name}, error={status.error.name}")
                        if self.state != "DISARMED" and (
                            status.state != PicoState.ARMED or not status.outputs_enabled
                        ):
                            raise RuntimeFault(f"Pico unexpectedly left ARMED: {status}")

                    deadline += period
                    now = time.monotonic()
                    if now > deadline:
                        skipped = int((now - deadline) / period) + 1
                        self.missed_deadlines += skipped
                        deadline = now + period
                        if self.missed_deadlines == skipped or self.missed_deadlines % 50 < skipped:
                            LOG.warning("control-loop deadline misses: %d", self.missed_deadlines)
                    time.sleep(max(0.0, deadline - time.monotonic()))
            except Exception:
                self.state = "FAULT"
                LOG.exception("controller entered FAULT")
                raise
            finally:
                try:
                    link.disarm()
                    LOG.info("DISARM sent during shutdown")
                except Exception as exc:
                    LOG.error("could not send DISARM during shutdown: %s", exc)
        return 0
