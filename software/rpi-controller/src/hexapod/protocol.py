from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

PROTOCOL_VERSION = 1
MAX_PAYLOAD = 64


class ProtocolError(ValueError):
    pass


class MessageType(IntEnum):
    ARM = 0x01
    SET_PWM = 0x02
    DISARM = 0x03
    GET_STATUS = 0x04
    STATUS = 0x80


class PicoState(IntEnum):
    DISARMED = 0
    ARMED = 1
    COMMUNICATION_FAULT = 2
    OUTPUT_FAULT = 3


class PicoError(IntEnum):
    NONE = 0
    INVALID_LENGTH = 1
    INVALID_PWM = 2
    STALE_SEQUENCE = 3
    NOT_ARMED = 4
    REQUIRES_DISARM = 5
    COMMAND_TIMEOUT = 6
    OUTPUT_FAILURE = 7
    UNSUPPORTED_MESSAGE = 8


@dataclass(frozen=True)
class Packet:
    message_type: MessageType
    sequence: int
    payload: bytes = b""


@dataclass(frozen=True)
class Status:
    state: PicoState
    error: PicoError
    outputs_enabled: bool
    input_mask: int
    last_command_sequence: int
    dropped_frames: int


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0 or index + code - 1 > len(data):
            raise ProtocolError("malformed COBS frame")
        output.extend(data[index:index + code - 1])
        index += code - 1
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_packet(packet: Packet) -> bytes:
    if len(packet.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload is too large")
    if not 0 <= packet.sequence <= 0xFFFFFFFF:
        raise ProtocolError("sequence is outside uint32 range")
    header = struct.pack("<BBIB", PROTOCOL_VERSION, int(packet.message_type), packet.sequence, len(packet.payload))
    decoded = header + packet.payload
    decoded += struct.pack("<H", crc16_ccitt(decoded))
    return cobs_encode(decoded) + b"\x00"


def decode_packet(frame: bytes) -> Packet:
    if frame.endswith(b"\x00"):
        frame = frame[:-1]
    decoded = cobs_decode(frame)
    if len(decoded) < 9:
        raise ProtocolError("packet is truncated")
    version, raw_type, sequence, payload_size = struct.unpack_from("<BBIB", decoded)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if payload_size > MAX_PAYLOAD or len(decoded) != 7 + payload_size + 2:
        raise ProtocolError("payload length does not match packet")
    expected = crc16_ccitt(decoded[:-2])
    actual = struct.unpack_from("<H", decoded, len(decoded) - 2)[0]
    if actual != expected:
        raise ProtocolError("CRC mismatch")
    try:
        message_type = MessageType(raw_type)
    except ValueError as exc:
        raise ProtocolError(f"unknown message type 0x{raw_type:02x}") from exc
    return Packet(message_type, sequence, decoded[7:-2])


def pwm_packet(message_type: MessageType, sequence: int, pulses_us: tuple[int, ...]) -> Packet:
    if message_type not in (MessageType.ARM, MessageType.SET_PWM):
        raise ProtocolError("PWM payload is valid only for ARM or SET_PWM")
    if len(pulses_us) != 18:
        raise ProtocolError("exactly 18 PWM channels are required")
    if any(pulse < 1000 or pulse > 2000 for pulse in pulses_us):
        raise ProtocolError("PWM outside Pico range 1000..2000 us")
    return Packet(message_type, sequence, struct.pack("<18H", *pulses_us))


def parse_status(packet: Packet) -> Status:
    if packet.message_type != MessageType.STATUS or len(packet.payload) != 12:
        raise ProtocolError("not a valid STATUS packet")
    state, error, flags, input_mask, last_sequence, dropped = struct.unpack("<BBBBII", packet.payload)
    try:
        return Status(PicoState(state), PicoError(error), bool(flags & 1), input_mask & 0x3F, last_sequence, dropped)
    except ValueError as exc:
        raise ProtocolError("STATUS contains an unknown enum value") from exc


class StreamParser:
    def __init__(self, max_frame_size: int = 80):
        self.buffer = bytearray()
        self.discarding = False
        self.max_frame_size = max_frame_size
        self.dropped_frames = 0

    def push(self, data: bytes) -> list[Packet]:
        packets: list[Packet] = []
        for byte in data:
            if byte == 0:
                if self.discarding:
                    self.discarding = False
                    self.buffer.clear()
                    continue
                if not self.buffer:
                    continue
                try:
                    packets.append(decode_packet(bytes(self.buffer)))
                except ProtocolError:
                    self.dropped_frames += 1
                self.buffer.clear()
            elif not self.discarding:
                if len(self.buffer) >= self.max_frame_size:
                    self.dropped_frames += 1
                    self.buffer.clear()
                    self.discarding = True
                else:
                    self.buffer.append(byte)
        return packets
