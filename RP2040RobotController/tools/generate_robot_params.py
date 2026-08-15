#!/usr/bin/env python3
import argparse
import pathlib
import re
import sys
import tempfile


SCHEMA = {
    "CONTROL_MODE": ("mode", None, None),
    "RC_MIN_US": ("uint16", 500, 3000), "RC_CENTER_US": ("uint16", 500, 3000),
    "RC_MAX_US": ("uint16", 500, 3000), "RC_VALID_MIN_US": ("uint16", 500, 3000),
    "RC_VALID_MAX_US": ("uint16", 500, 3000), "RC_DEADBAND_US": ("uint16", 0, 500),
    "RC_ACTIVE_THRESHOLD": ("float", 0.0, 1.0),
    "RC_FORWARD_REVERSED": ("bool", None, None), "RC_STEER_REVERSED": ("bool", None, None),
    "RC_MIN_SPEED": ("float", 0.0, 1.0), "RC_MAX_SPEED": ("float", 0.0, 1.0),
    "RC_SMOOTHING_MS": ("uint32", 0, 1000), "RC_SIGNAL_TIMEOUT_MS": ("uint32", 1, 5000),
    "RC_ARM_NEUTRAL_MS": ("uint32", 0, 10000), "RC_DEBUG": ("bool", None, None),
    "RC_DEBUG_RATE_HZ": ("uint32", 1, 100), "SERIAL_DASHBOARD": ("bool", None, None),
    "SERIAL_DASHBOARD_RATE_HZ": ("uint32", 1, 100), "SERIAL_DASHBOARD_ANSI": ("bool", None, None),
    "STAND_COXA_DEG": ("float", -45.0, 45.0), "STAND_FEMUR_DEG": ("float", -45.0, 45.0),
    "STAND_TIBIA_DEG": ("float", -45.0, 45.0),
    "GAIT_CYCLE_MS": ("uint32", 100, 5000), "GAIT_DUTY_FACTOR": ("float", 0.5, 0.95),
    "GAIT_STEP_HEIGHT_MM": ("float", 0.0, 200.0), "GAIT_STRIDE_MM": ("float", 0.0, 500.0),
    "MAX_FORWARD_SPEED_MM_S": ("float", 0.0, 2000.0), "MAX_YAW_RATE_DEG_S": ("float", 0.0, 720.0),
}


def fail(path, line, message):
    raise ValueError(f"{path}:{line}: {message}" if line else f"{path}: {message}")


def parse_value(key, value, path, line):
    kind, minimum, maximum = SCHEMA[key]
    if kind == "mode":
        if value not in ("UART", "RC_PWM"):
            fail(path, line, "CONTROL_MODE must be UART or RC_PWM")
        return value
    if kind == "bool":
        if value.lower() not in ("true", "false"):
            fail(path, line, f"{key} must be true or false")
        return value.lower() == "true"
    if kind.startswith("uint"):
        if not re.fullmatch(r"[0-9]+", value):
            fail(path, line, f"{key} must be an integer")
        parsed = int(value)
    else:
        try:
            parsed = float(value)
        except ValueError:
            fail(path, line, f"{key} must be a number")
    if parsed < minimum or parsed > maximum:
        fail(path, line, f"{key}={parsed} is outside {minimum}..{maximum}")
    return parsed


def parse_params(path):
    values = {}
    for line, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = raw.split("#", 1)[0].strip()
        if not text:
            continue
        if "=" not in text:
            fail(path, line, "expected KEY=VALUE")
        key, value = (part.strip() for part in text.split("=", 1))
        if key not in SCHEMA:
            fail(path, line, f"unknown key {key!r}")
        if key in values:
            fail(path, line, f"duplicate key {key!r}")
        values[key] = parse_value(key, value, path, line)
    missing = [key for key in SCHEMA if key not in values]
    if missing:
        fail(path, 0, "missing required keys: " + ", ".join(missing))
    if not values["RC_MIN_US"] < values["RC_CENTER_US"] < values["RC_MAX_US"]:
        fail(path, 0, "expected RC_MIN_US < RC_CENTER_US < RC_MAX_US")
    if values["RC_VALID_MIN_US"] > values["RC_MIN_US"] or values["RC_VALID_MAX_US"] < values["RC_MAX_US"]:
        fail(path, 0, "valid RC range must contain the configured range")
    if values["RC_MIN_SPEED"] > values["RC_MAX_SPEED"]:
        fail(path, 0, "expected RC_MIN_SPEED <= RC_MAX_SPEED")
    return values


def cpp_float(value):
    text = f"{float(value):.6g}"
    return (text if any(ch in text for ch in ".eE") else text + ".0") + "f"


def generate_header(values):
    lines = ["#pragma once", "", "#include <cstdint>", "", "namespace config {", "",
             "enum class ControlInputMode { UART, RC_PWM };", "",
             f"constexpr ControlInputMode CONTROL_MODE = ControlInputMode::{values['CONTROL_MODE']};"]
    for key, (kind, _, _) in SCHEMA.items():
        if key == "CONTROL_MODE":
            continue
        value = values[key]
        if kind == "bool": ctype, literal = "bool", "true" if value else "false"
        elif kind == "float": ctype, literal = "float", cpp_float(value)
        elif kind == "uint16": ctype, literal = "uint16_t", str(value)
        else: ctype, literal = "uint32_t", str(value)
        lines.append(f"constexpr {ctype} {key} = {literal};")
    return "\n".join(lines + ["", "} // namespace config", ""])


def write_if_changed(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text(encoding="utf-8") != text:
        path.write_text(text, encoding="utf-8")


def run_self_test():
    content = "\n".join([
        "CONTROL_MODE=RC_PWM", "RC_MIN_US=900", "RC_CENTER_US=1500", "RC_MAX_US=2100",
        "RC_VALID_MIN_US=800", "RC_VALID_MAX_US=2200", "RC_DEADBAND_US=80", "RC_ACTIVE_THRESHOLD=0.12",
        "RC_FORWARD_REVERSED=false", "RC_STEER_REVERSED=false", "RC_MIN_SPEED=0.45", "RC_MAX_SPEED=1",
        "RC_SMOOTHING_MS=80", "RC_SIGNAL_TIMEOUT_MS=120", "RC_ARM_NEUTRAL_MS=500", "RC_DEBUG=true",
        "RC_DEBUG_RATE_HZ=10", "SERIAL_DASHBOARD=true", "SERIAL_DASHBOARD_RATE_HZ=10", "SERIAL_DASHBOARD_ANSI=true",
        "STAND_COXA_DEG=0", "STAND_FEMUR_DEG=9", "STAND_TIBIA_DEG=18", "GAIT_CYCLE_MS=650",
        "GAIT_DUTY_FACTOR=0.58", "GAIT_STEP_HEIGHT_MM=20", "GAIT_STRIDE_MM=30",
        "MAX_FORWARD_SPEED_MM_S=60", "MAX_YAW_RATE_DEG_S=30", "",
    ])
    with tempfile.TemporaryDirectory() as directory:
        good = pathlib.Path(directory) / "good.txt"
        good.write_text(content, encoding="utf-8")
        assert parse_params(good)["GAIT_DUTY_FACTOR"] == 0.58
        bad = pathlib.Path(directory) / "bad.txt"
        bad.write_text(content + "REMOVED_OPTION=true\n", encoding="utf-8")
        try:
            parse_params(bad)
        except ValueError:
            return 0
        raise AssertionError("legacy key must be rejected")


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
        write_if_changed(args.output, generate_header(parse_params(args.input)))
    except ValueError as error:
        print(f"robot params error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
