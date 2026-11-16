from __future__ import annotations

from contextlib import AbstractContextManager
import os
import time
from typing import BinaryIO

from .config import Config
from .protocol import MessageType, Packet, Status, StreamParser, encode_packet, parse_status, pwm_packet

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows is used only for offline development/tests.
    fcntl = None  # type: ignore[assignment]


class SerialLinkError(RuntimeError):
    pass


class UartLock(AbstractContextManager["UartLock"]):
    def __init__(self, path: str):
        self.path = path
        self.stream: BinaryIO | None = None

    def __enter__(self) -> "UartLock":
        if fcntl is None:
            raise SerialLinkError("UART locking requires a POSIX host")
        try:
            self.stream = open(self.path, "a+b")
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, BlockingIOError) as exc:
            if self.stream is not None:
                self.stream.close()
                self.stream = None
            raise SerialLinkError(f"UART is already owned (lock {self.path}): {exc}") from exc
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.stream is not None:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
            self.stream.close()
            self.stream = None


class PicoLink(AbstractContextManager["PicoLink"]):
    def __init__(self, config: Config):
        self.config = config
        self.serial = None
        self.lock = UartLock(config.uart_lock)
        self.parser = StreamParser()
        self.sequence = int(time.monotonic_ns()) & 0xFFFFFFFF

    def __enter__(self) -> "PicoLink":
        self.lock.__enter__()
        try:
            import serial
            self.serial = serial.Serial(
                self.config.uart_port,
                self.config.uart_baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0,
                write_timeout=self.config.uart_timeout_s,
            )
            self.serial.reset_input_buffer()
        except Exception as exc:
            self.lock.__exit__(None, None, None)
            raise SerialLinkError(f"cannot open {self.config.uart_port}: {exc}") from exc
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        self.lock.__exit__(exc_type, exc, traceback)

    def next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return self.sequence

    def send(self, packet: Packet) -> None:
        if self.serial is None:
            raise SerialLinkError("serial link is not open")
        frame = encode_packet(packet)
        written = self.serial.write(frame)
        if written != len(frame):
            raise SerialLinkError(f"short UART write: {written}/{len(frame)} bytes")

    def arm(self, pulses_us: tuple[int, ...]) -> int:
        sequence = self.next_sequence()
        self.send(pwm_packet(MessageType.ARM, sequence, pulses_us))
        return sequence

    def set_pwm(self, pulses_us: tuple[int, ...]) -> int:
        sequence = self.next_sequence()
        self.send(pwm_packet(MessageType.SET_PWM, sequence, pulses_us))
        return sequence

    def disarm(self) -> int:
        sequence = self.next_sequence()
        self.send(Packet(MessageType.DISARM, sequence))
        return sequence

    def read_packets(self) -> list[Packet]:
        if self.serial is None:
            raise SerialLinkError("serial link is not open")
        waiting = self.serial.in_waiting
        return self.parser.push(self.serial.read(max(1, waiting)))

    def get_status(self, timeout_s: float | None = None) -> Status:
        sequence = self.next_sequence()
        self.send(Packet(MessageType.GET_STATUS, sequence))
        deadline = time.monotonic() + (timeout_s or self.config.uart_timeout_s)
        while time.monotonic() < deadline:
            for packet in self.read_packets():
                if packet.message_type == MessageType.STATUS and packet.sequence == sequence:
                    return parse_status(packet)
            time.sleep(0.001)
        raise SerialLinkError(f"Pico STATUS timeout after {timeout_s or self.config.uart_timeout_s:.3f} s")
