# ServoTester

ServoTester is now a PlatformIO ESP-IDF controller for one hexapod leg with three servos:

- Coxa: rotates the leg relative to the body.
- Femur: raises and lowers the thigh.
- Tibia: bends the lower leg.

The project targets a normal ESP32 Dev Module / ESP32-WROOM-32 (`esp32dev`) and uses C++, ESP-IDF, LEDC, FreeRTOS, and the ESP-IDF console/stdin path. It does not use Arduino Framework, `Arduino.h`, `setup()`, `loop()`, the Arduino Servo library, or external libraries.

## PlatformIO Configuration

`platformio.ini`:

```ini
[platformio]
default_envs = esp32dev
src_dir = main

[env:esp32dev]
platform = platformio/espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
upload_speed = 115200
```

Open exactly the `ServoTester` folder in Visual Studio Code, not the parent `Hexapodik` folder.

## Wiring

| Joint | ESP32 GPIO | LEDC channel |
|---|---:|---:|
| Tibia | GPIO25 | 2 |
| Femur | GPIO26 | 1 |
| Coxa | GPIO27 | 0 |

PWM frequency is 50 Hz, with a 20 ms period and a 16-bit LEDC timer in low-speed mode.

## Servo Power

Do not power three servos from the ESP32.

- Use a separate 5-6 V power supply for the servos.
- Connect servo power-supply GND and ESP32 GND together.
- Put an electrolytic capacitor of at least 1000-2200 uF near the servo connectors.
- Prefer adding one 100 nF capacitor near each servo connector.
- Make sure the servo power supply can handle the startup current of all three servos at once.

## Calibration

The firmware uses individual three-point calibration for each physical servo. These values apply only to these three calibrated servos. If you move servos between joints, move the calibration values with the physical servos.

| Joint | GPIO | -45 deg | 0 deg | +45 deg |
|---|---:|---:|---:|---:|
| Tibia | 25 | 2017 | 1498 | 969 |
| Femur | 26 | 1976 | 1462 | 934 |
| Coxa | 27 | 1995 | 1491 | 961 |

All three servos are inverted: as the command angle increases, the pulse width decreases. The firmware does not replace these measured values with a generic 1000-2000 us range.

Commanded angles are clamped to `-45..+45` degrees. Before PWM is written, the final pulse is also clamped to the individual calibration range of the selected servo.

## Serial Monitor

Open the Serial Monitor at 115200 baud. Commands are ended by Enter; CR, LF, and CRLF are accepted. Maximum command length is 128 characters.

### Commands

```text
leg <coxa> <femur> <tibia>
coxa <angle>
femur <angle>
tibia <angle>
speed <degrees_per_second>
center
status
stop
test coxa
test femur
test tibia
demo
disable
enable
help
```

Examples:

```text
help
status
center
leg 10 -20 30
coxa 15
femur -10
tibia 25
speed 30
stop
test coxa
demo
disable
enable
```

`speed` is clamped to `5..180` deg/s. The default is `45` deg/s. Movement is smoothed by a FreeRTOS task that runs every 20 ms and limits each update to:

```text
speed_deg_per_sec * 0.020
```

`stop` sets all target angles to the current angles and keeps PWM active. `disable` stops PWM output for all three servos so they stop holding position. `enable` resumes PWM using the saved current angles, without jumping through an arbitrary pose.

`demo` runs this safe sequence without inverse kinematics:

| Pose | Coxa | Femur | Tibia |
|---|---:|---:|---:|
| 1 | 0 | 0 | 0 |
| 2 | -15 | -15 | +20 |
| 3 | +15 | -15 | +20 |
| 4 | 0 | 0 | 0 |

Do not run large angles automatically on the first boot. Start with `help`, `status`, `center`, and `leg 0 0 0`.

## Build, Upload, Monitor

Check the project configuration:

```powershell
pio project config
```

Build:

```powershell
pio run -e esp32dev
```

List connected devices:

```powershell
pio device list
```

Upload:

```powershell
pio run -e esp32dev -t upload
```

Open the monitor:

```powershell
pio device monitor --baud 115200
```

The helper scripts also use PlatformIO:

```powershell
.\build.ps1
.\flash.ps1
.\flash.ps1 COM5
.\monitor.ps1
.\monitor.ps1 COM5
```

If several serial ports are present and PlatformIO cannot choose automatically, use the port shown by `pio device list`. If flashing hangs at `Connecting...`, hold the BOOT button when upload starts and release it after the connection message appears. Close Serial Monitor before uploading again if the COM port is busy.

Physical servo motion must be verified visually on the connected leg; the firmware build alone cannot prove that the mechanical motion is safe.
