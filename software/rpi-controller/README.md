# Hexapodik Raspberry Pi controller

Python 3 controller for Raspberry Pi 4. It reads four RC PWM channels through
libgpiod, plans one statically stable foot transfer at a time, solves IK, maps
the 18 geometric joint angles through independent calibrations, and sends one
complete PWM frame to the RP2040 at 50 Hz.

The Pico wire format is not duplicated or guessed: `hexapod.protocol` implements
protocol v1 from `firmware/rp2040-controller/docs/uart_protocol.md` (115200 8N1,
COBS, CRC16-CCITT, ARM/SET_PWM/DISARM/GET_STATUS/STATUS).

## Safety state

- Every process start sends `DISARM` and confirms the Pico is `DISARMED`.
- The service cannot ARM until all 18 `hardware_verified` flags are true, every
  RC channel is fresh, the three motion sticks are neutral, and the enable
  channel has made a fresh OFF -> ON transition.
- A normal enable OFF completes the current transfer and returns smoothly to the
  neutral stance. An explicit `disarm` disables PWM.
- Stale RC commands zero body motion and enter `STOPPING`. A UART/Pico fault
  enters `FAULT`; restart never resumes motion automatically.
- Contact bits are diagnostic only while `contact_levels_verified = false`.
  After physical polarity verification, early contact during descent freezes
  further downward motion. Missing contact searches at most
  `contact_search_limit_m`, then holds and reports the reason.
- Calibration and single-leg commands require explicit physical-setup flags.

Pico firmware intentionally holds its last PWM frame after its 250 ms UART
timeout. Therefore loss of UART is a fault, not a guarantee that a loaded robot
becomes limp. Use the independent power stop appropriate for the stand.

## Layout

- `src/hexapod/` — protocol, kinematics, calibration, gait, RC and service code.
- `config/hexapod.toml` — geometry, mounts, RC mapping, limits and all 18 servo
  records.
- `systemd/hexapod.service` — system service running as user `pi`.
- `tests/` — protocol vectors, FK/IK, rejection and full-gait tests.

Angles are radians and lengths are metres internally. UART channel order is
`FL, ML, RL, RR, MR, FR`, with `Tibia, Femur, Coxa` inside each leg.

## Offline validation

From this directory:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 -m hexapod.cli --config config/hexapod.toml validate
```

`validate` checks the supplied control pose, the documented IK target,
unreachable-target rejection and a complete gait plus smooth stop. A software
simulation is not a physical walking test.

## Installation on Raspberry Pi

The deployment directory is `/home/pi/hexapod`. After copying this directory:

```bash
cd /home/pi/hexapod
PYTHONPATH=src python3 -m hexapod.cli --config config/hexapod.toml validate
sudo install -m 0644 systemd/hexapod.service /etc/systemd/system/hexapod.service
sudo systemctl daemon-reload
sudo systemctl start hexapod.service
```

Do not enable autostart until `systemctl status hexapod.service` shows that the
process remains `DISARMED`. When that is confirmed:

```bash
sudo systemctl enable hexapod.service
```

Logs:

```bash
journalctl -u hexapod.service -f
```

## Commands

Use the installed executable and config prefix below:

```bash
cd /home/pi/hexapod
CTL='env PYTHONPATH=src python3 -m hexapod.cli --config config/hexapod.toml'
```

Read-only/offline checks:

```bash
$CTL validate
$CTL pico-status
$CTL rc-monitor
$CTL rc-monitor --duration 10
```

Stop the service before a direct UART command; the nonblocking lock deliberately
allows only one UART owner:

```bash
sudo systemctl stop hexapod.service
$CTL disarm
```

Hardware motion commands (do not run before the stated setup is physically
true):

```bash
# Exactly one unloaded servo connected; all other servo signal/power paths isolated.
$CTL calibrate-servo FL.Coxa --delta-deg 2 --confirm-isolated-servo

# Selected leg suspended and the robot securely supported.
$CTL move-leg FL 0.150 0 -0.100 --confirm-suspended-leg

# Only after all 18 direction/limit records are verified.
$CTL stand --hold-seconds 5 --confirm-robot-supported
```

RC operation is the service itself:

```bash
sudo systemctl start hexapod.service
```

It waits in `DISARMED`; with fresh neutral channels, move enable OFF and then ON.
There is one gait only. Enable OFF stops walking and holds stance. To disable PWM:

```bash
sudo systemctl stop hexapod.service
$CTL disarm
```

## Required hardware measurements

Before stance or walking:

1. With servo power disconnected, verify Pico status, every contact input bit,
   20 ms PWM period, channel order and DISARM-low behavior.
2. Record RC endpoints/centres, channel inversions, transmitter-off behavior and
   configure receiver failsafe so enable becomes OFF.
3. Isolate one unloaded servo at a time. Determine `direction`, safe
   `limits_deg`, calibration points, then set `hardware_verified = true` for that
   servo only.
4. Verify whether physical foot contact is GPIO HIGH or LOW before setting
   `contact_levels_verified = true` and `contact_active_high` accordingly.
5. Measure the real COM; `com_x_m = 0`, `com_y_m = 0` is only an initial
   approximation.
