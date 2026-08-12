#!/usr/bin/env python3
import argparse
import pathlib
import re
import sys
import tempfile


SCHEMA = {
    "CONTROL_MODE": ("mode", None, None),
    "RC_MIN_US": ("uint16", 500, 3000),
    "RC_CENTER_US": ("uint16", 500, 3000),
    "RC_MAX_US": ("uint16", 500, 3000),
    "RC_VALID_MIN_US": ("uint16", 500, 3000),
    "RC_VALID_MAX_US": ("uint16", 500, 3000),
    "RC_DEADBAND_US": ("uint16", 0, 500),
    "RC_FORWARD_REVERSED": ("bool", None, None),
    "RC_STEER_REVERSED": ("bool", None, None),
    "RC_MIN_SPEED": ("float", 0.0, 1.0),
    "RC_MAX_SPEED": ("float", 0.0, 1.0),
    "RC_SMOOTHING_MS": ("uint32", 0, 1000),
    "RC_SIGNAL_TIMEOUT_MS": ("uint32", 1, 5000),
    "RC_ARM_NEUTRAL_MS": ("uint32", 0, 10000),
    "RC_DEBUG": ("bool", None, None),
    "RC_DEBUG_RATE_HZ": ("uint32", 1, 100),
    "SERIAL_DASHBOARD": ("bool", None, None),
    "SERIAL_DASHBOARD_RATE_HZ": ("uint32", 1, 100),
    "SERIAL_DASHBOARD_ANSI": ("bool", None, None),
    "GAIT_LIFT_FEMUR_DELTA_DEG": ("float", -45.0, 45.0),
    "GAIT_LIFT_TIBIA_DELTA_DEG": ("float", -45.0, 45.0),
    "GAIT_COXA_SWING_MIN_DEG": ("float", 0.0, 45.0),
    "GAIT_COXA_SWING_MAX_DEG": ("float", 0.0, 45.0),
    "GAIT_CYCLE_SLOW_MS": ("uint32", 100, 5000),
    "GAIT_CYCLE_FAST_MS": ("uint32", 100, 5000),
    "GAIT_TURN_GAIN": ("float", 0.0, 3.0),
}


def fail(path, line_no, message):
    where = f"{path}:{line_no}: " if line_no else f"{path}: "
    raise ValueError(where + message)


def parse_bool(value, path, line_no):
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    fail(path, line_no, f"expected true or false, got {value!r}")


def parse_value(key, value, path, line_no):
    kind, min_value, max_value = SCHEMA[key]
    if kind == "mode":
        if value not in ("UART", "RC_PWM"):
            fail(path, line_no, "CONTROL_MODE must be UART or RC_PWM")
        return value
    if kind == "bool":
        return parse_bool(value, path, line_no)
    if kind in ("uint16", "uint32"):
        if not re.fullmatch(r"[0-9]+", value):
            fail(path, line_no, f"{key} must be an integer")
        parsed = int(value)
    else:
        try:
            parsed = float(value)
        except ValueError:
            fail(path, line_no, f"{key} must be a number")
    if parsed < min_value or parsed > max_value:
        fail(path, line_no, f"{key}={parsed} is outside {min_value}..{max_value}")
    return parsed


def parse_params(path):
    values = {}
    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if "=" not in line:
            fail(path, line_no, "expected KEY=VALUE")
        key, value = [part.strip() for part in line.split("=", 1)]
        if key not in SCHEMA:
            fail(path, line_no, f"unknown key {key!r}")
        if key in values:
            fail(path, line_no, f"duplicate key {key!r}")
        values[key] = parse_value(key, value, path, line_no)

    missing = [key for key in SCHEMA if key not in values]
    if missing:
        fail(path, 0, "missing required keys: " + ", ".join(missing))

    validate_cross_fields(values, path)
    return values


def validate_cross_fields(values, path):
    if not values["RC_MIN_US"] < values["RC_CENTER_US"] < values["RC_MAX_US"]:
        fail(path, 0, "expected RC_MIN_US < RC_CENTER_US < RC_MAX_US")
    if values["RC_VALID_MIN_US"] >= values["RC_VALID_MAX_US"]:
        fail(path, 0, "expected RC_VALID_MIN_US < RC_VALID_MAX_US")
    if values["RC_VALID_MIN_US"] > values["RC_MIN_US"]:
        fail(path, 0, "expected RC_VALID_MIN_US <= RC_MIN_US")
    if values["RC_VALID_MAX_US"] < values["RC_MAX_US"]:
        fail(path, 0, "expected RC_VALID_MAX_US >= RC_MAX_US")
    if values["RC_DEADBAND_US"] >= min(
        values["RC_CENTER_US"] - values["RC_MIN_US"],
        values["RC_MAX_US"] - values["RC_CENTER_US"],
    ):
        fail(path, 0, "RC_DEADBAND_US is too large for RC range")
    if values["RC_MIN_SPEED"] > values["RC_MAX_SPEED"]:
        fail(path, 0, "expected RC_MIN_SPEED <= RC_MAX_SPEED")
    if values["GAIT_COXA_SWING_MIN_DEG"] > values["GAIT_COXA_SWING_MAX_DEG"]:
        fail(path, 0, "expected GAIT_COXA_SWING_MIN_DEG <= GAIT_COXA_SWING_MAX_DEG")
    if values["GAIT_CYCLE_FAST_MS"] > values["GAIT_CYCLE_SLOW_MS"]:
        fail(path, 0, "expected GAIT_CYCLE_FAST_MS <= GAIT_CYCLE_SLOW_MS")


def cpp_float(value):
    text = f"{float(value):.6g}"
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return f"{text}f"


def generate_header(values, input_path):
    lines = [
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        "namespace config {",
        "",
        "enum class ControlInputMode {",
        "    UART,",
        "    RC_PWM",
        "};",
        "",
        f"constexpr ControlInputMode CONTROL_MODE = ControlInputMode::{values['CONTROL_MODE']};",
    ]
    for key, (kind, _, _) in SCHEMA.items():
        if key == "CONTROL_MODE":
            continue
        value = values[key]
        if kind == "bool":
            literal = "true" if value else "false"
            ctype = "bool"
        elif kind == "float":
            literal = cpp_float(value)
            ctype = "float"
        elif kind == "uint16":
            literal = str(value)
            ctype = "uint16_t"
        else:
            literal = str(value)
            ctype = "uint32_t"
        lines.append(f"constexpr {ctype} {key} = {literal};")
    lines.extend([
        "",
        "} // namespace config",
        "",
    ])
    return "\n".join(lines)


def write_if_changed(output_path, text):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() and output_path.read_text(encoding="utf-8") == text:
        return
    output_path.write_text(text, encoding="utf-8")


def run_self_test():
    base = "\n".join(f"{key}=1000" for key in SCHEMA)
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = pathlib.Path(tmp)
        good = tmp_path / "good.txt"
        good.write_text(
            """# RC
CONTROL_MODE=RC_PWM
RC_MIN_US=900
RC_CENTER_US=1500
RC_MAX_US=2100
RC_VALID_MIN_US=800
RC_VALID_MAX_US=2200
RC_DEADBAND_US=50
RC_FORWARD_REVERSED=false
RC_STEER_REVERSED=true
RC_MIN_SPEED=0.30
RC_MAX_SPEED=1.00
RC_SMOOTHING_MS=80
RC_SIGNAL_TIMEOUT_MS=120
RC_ARM_NEUTRAL_MS=500
RC_DEBUG=true
RC_DEBUG_RATE_HZ=10
SERIAL_DASHBOARD=true
SERIAL_DASHBOARD_RATE_HZ=10
SERIAL_DASHBOARD_ANSI=true
GAIT_LIFT_FEMUR_DELTA_DEG=12
GAIT_LIFT_TIBIA_DELTA_DEG=-15
GAIT_COXA_SWING_MIN_DEG=6
GAIT_COXA_SWING_MAX_DEG=12
GAIT_CYCLE_SLOW_MS=1200
GAIT_CYCLE_FAST_MS=450
GAIT_TURN_GAIN=1.0
""",
            encoding="utf-8",
        )
        assert parse_params(good)["RC_STEER_REVERSED"] is True
        for name, text in {
            "duplicate": good.read_text(encoding="utf-8") + "RC_DEBUG=false\n",
            "unknown": good.read_text(encoding="utf-8") + "BOGUS=1\n",
            "bad_range": good.read_text(encoding="utf-8").replace("RC_DEADBAND_US=50", "RC_DEADBAND_US=900"),
        }.items():
            bad = tmp_path / f"{name}.txt"
            bad.write_text(text, encoding="utf-8")
            try:
                parse_params(bad)
            except ValueError:
                pass
            else:
                raise AssertionError(f"{name} case should fail")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if not args.input or not args.output:
        parser.error("--input and --output are required unless --self-test is used")

    try:
        values = parse_params(args.input)
        write_if_changed(args.output, generate_header(values, args.input))
    except ValueError as exc:
        print(f"robot params error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
