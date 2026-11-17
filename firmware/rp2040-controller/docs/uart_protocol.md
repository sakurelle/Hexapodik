# UART protocol v1

## Transport and framing

UART0 uses 115200 baud, 8 data bits, no parity, one stop bit. Every decoded packet has this layout:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | version, currently `0x01` |
| 1 | 1 | message type |
| 2 | 4 | sequence, `uint32_le` |
| 6 | 1 | payload length |
| 7 | N | payload |
| 7+N | 2 | CRC16-CCITT, little-endian |

CRC uses polynomial `0x1021`, initial value `0xFFFF`, and covers bytes `0..6+N`. The complete decoded packet is COBS-encoded and terminated by `0x00`. There are no packed C structs on the wire.

A malformed, truncated, overlong, wrong-version or bad-CRC frame is discarded. The parser resynchronizes at the next zero delimiter. Such a frame does not update PWM and does not refresh the 250 ms motion timeout. The cumulative discarded-frame count is returned in status.

## Commands

### ARM (`0x01`)

Payload length must be exactly 36 bytes: 18 little-endian unsigned pulse widths. Channel order is:

```text
FL_Tibia, FL_Femur, FL_Coxa,
ML_Tibia, ML_Femur, ML_Coxa,
RL_Tibia, RL_Femur, RL_Coxa,
RR_Tibia, RR_Femur, RR_Coxa,
MR_Tibia, MR_Femur, MR_Coxa,
FR_Tibia, FR_Femur, FR_Coxa
```

Every value must be within `1000..2000` microseconds. ARM is accepted only in `DISARMED`. It applies the complete set and starts PWM; firmware never supplies a default pose.

### SET_PWM (`0x02`)

Payload is identical to ARM. It is accepted only in `ARMED`, and its sequence must be newer than the last accepted motion sequence using wrapping 32-bit comparison. Duplicate and stale frames are rejected without changing outputs or timeout.

### DISARM (`0x03`)

Payload must be empty. A valid DISARM is always accepted, including from communication/output fault, and immediately disables PWM. It starts a new command session, so the next valid ARM establishes a new sequence baseline.

### GET_STATUS (`0x04`)

Payload is empty. It never refreshes the motion timeout. The response uses the request sequence so the Raspberry Pi can match request and response. Status is available while PWM is disabled.

## STATUS (`0x80`)

Payload length is 12 bytes:

| Offset | Size | Meaning |
|---:|---:|---|
| 0 | 1 | controller state |
| 1 | 1 | last error |
| 2 | 1 | flags; bit 0 = PWM outputs enabled |
| 3 | 1 | debounced input mask; bits 0..5 = FL..FR, `1` = GPIO HIGH/open |
| 4 | 4 | last accepted ARM/SET_PWM sequence, `uint32_le` |
| 8 | 4 | discarded protocol frames, `uint32_le` |

Controller states:

| Value | State |
|---:|---|
| 0 | `DISARMED` |
| 1 | `ARMED` |
| 2 | `COMMUNICATION_FAULT` |
| 3 | `OUTPUT_FAULT` |

Error codes:

| Value | Error |
|---:|---|
| 0 | `NONE` |
| 1 | `INVALID_LENGTH` |
| 2 | `INVALID_PWM` |
| 3 | `STALE_SEQUENCE` |
| 4 | `NOT_ARMED` |
| 5 | `REQUIRES_DISARM` |
| 6 | `COMMAND_TIMEOUT` |
| 7 | `OUTPUT_FAILURE` |
| 8 | `UNSUPPORTED_MESSAGE` |

## State transitions

```text
boot -> DISARMED (PWM LOW)
DISARMED -- valid ARM[18] --> ARMED
ARMED -- valid SET_PWM[18] --> ARMED
ARMED -- 250 ms without valid ARM/SET_PWM --> COMMUNICATION_FAULT (hold PWM)
COMMUNICATION_FAULT -- SET_PWM/ARM --> unchanged, REQUIRES_DISARM
any state -- valid DISARM --> DISARMED (PWM LOW)
DISARMED -- valid ARM[18] --> ARMED
```

Invalid PWM, bad CRC, stale sequence and GET_STATUS do not refresh the timeout.

## Verified frame examples

The examples below include the final `00` COBS delimiter.

`DISARM`, sequence 1:

```text
decoded: 01 03 01 00 00 00 00 1E 2D
wire:    04 01 03 01 01 01 01 03 1E 2D 00
```

`GET_STATUS`, sequence 2:

```text
decoded: 01 04 02 00 00 00 00 8D 0B
wire:    04 01 04 02 01 01 01 03 8D 0B 00
```

`ARM`, sequence 3, all 18 channels set to 1500 us (`DC 05` little-endian):

```text
decoded:
01 01 03 00 00 00 24
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
83 AE

wire:
04 01 01 03 01 01 28 24
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
83 AE 00
```
