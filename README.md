<div align="center">

# CyberDog2-app

**A Python SDK, a browser control app and a protocol emulator for the Xiaomi CyberDog 2.**

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Third-party: Apache-2.0](https://img.shields.io/badge/third--party-Apache--2.0-lightgrey.svg)](THIRD_PARTY_NOTICES.md)
[![Python](https://img.shields.io/badge/python-3.8%20%7C%203.12-3776AB.svg?logo=python&logoColor=white)](requirements.txt)
[![Tests](https://img.shields.io/badge/tests-821%20passed-brightgreen.svg)](#tests)
[![Status](https://img.shields.io/badge/status-tested%20on%20the%20real%20dog-green.svg)](#project-status)

[Quick start](#quick-start) · [SDK](#the-sdk) · [App](#the-app) · [Emulator](#the-emulator) · [Probe](#the-probe-on-the-real-dog) · [Docs](#documentation) · [License](#license)

🇮🇹 [Guida dell'app in italiano](app2/README.md)

</div>

---

The CyberDog 2 app on a phone talks to the dog over gRPC with mutual TLS on port 50052. This
project speaks the same protocol from a PC. It contains these parts:

- **`cyberdog_sdk/`** is a Python client for the dog's `GrpcApp` service: motions, walking,
  posture, LEDs, sensors, camera, maps and navigation.
- **`app2/`** is a control page in the browser, served by a local Python server on
  `127.0.0.1:8794`. It has keyboard driving, motions, posture, lights, camera and maps.
- **`emulator/`** is a fake dog with the same gRPC interface. All tests run against it, so no
  test needs the robot.
- **`tools/probe_dog.py`** is a guided session on the real dog. It answers the questions that
  the source code cannot answer.

> [!WARNING]
> **The CyberDog 2 can walk at up to 1.6 m/s.** Run motion commands only when the dog
> stands on a flat floor with at least 1.5 m of free space around it. Never run them on a table.
> Keep the emergency stop (Space in the app, Ctrl+C in the probe) ready. After an emergency
> stop the motors switch off, and the dog drops to the floor.

## Features

| Area | What the app and the SDK do |
|---|---|
| Driving | W/A/S/D/Q/E keys, four gaits, a speed slider. The dog stops when no command arrives for 0.4 s. |
| Motions | Stand and sit on key 1, eight tricks on keys 2 to 9. Motions that move far ask for a "floor clear" confirmation. Jumps and flips are not available. |
| Posture | Body height, roll, pitch and yaw. |
| Emergency stop | Sent three times on a separate channel, with a retry on the main channel. It works in every state of the page. |
| Movement switch | Off at every page load. While it is off, the page refuses drive, motion, posture and navigation commands. |
| Lights and sensors | Head, tail and mini LEDs, the electronic skin, ToF, ultrasonic, odometry and battery data. |
| Camera | Still frames from the dog's camera, pulled on four parallel channels. |
| Maps and navigation | Map display, go to a point, return to the charging dock, stop navigation. |

## Project status

Version `v2.0.0` passes 821 tests against the emulator. A Gazebo run passed the main sequence:
stand, drive, trick, emergency stop while walking, and lie down. The probe then ran a full
session on a real CyberDog 2 with the Chinese firmware, and every step passed.

On the real dog the servo watchdog stood the dog up again 0.61 to 0.72 s after the last walk
frame, later than Xiaomi's source code says. The SDK therefore sends a third ESTOP 1.0 s after
the last frame. With that fix, an emergency stop while the dog walked kept the dog down.

| | Emulator | Gazebo simulator | Real dog |
|---|:-:|:-:|:-:|
| Connection, status, battery | ✅ | ✅ | ✅ |
| Driving, stand, sit, tricks, lie down | ✅ | ✅ | ✅ |
| Emergency stop | ✅ | ✅ | ✅ while walking |
| Posture | ✅ | head up only | ✅ |
| LEDs | ✅ | n/a | ✅ |
| Electronic skin | ✅ | n/a | n/a: the test dog has none |
| Sensors | ✅ | n/a | ✅ in the probe, ⚠️ one read in the app timed out |
| Camera, map display | ✅ | n/a | ✅ |
| Navigation | ✅ | n/a | ❌ not tested |

The v1 controller in `app/` also ran on the real dog. The open issues are in
[`docs/design/reports/build_sdk.md`](docs/design/reports/build_sdk.md).

## Quick start

**Requirements:** Windows 10 or 11, Python 3.12 (3.8 works on Linux), and a CyberDog 2 on the
same network as the PC. The scripts assume the Windows mobile hotspot, which gives the dog the
address `192.168.137.115`. [`docs/connection.md`](docs/connection.md) explains the network and
how to put the dog on the hotspot with a QR code.

**1. Clone the repository and install the dependencies.**

```powershell
git clone https://github.com/Giorgio-Cottini/CyberDog2-app.git
cd CyberDog2-app
python -m pip install -r requirements.txt
```

**2. Download the certificates.** The dog accepts only clients with a certificate signed by its
CA. Xiaomi publishes that CA and a matching client certificate in
[`MiRoboticsLab/bridges`](https://github.com/MiRoboticsLab/bridges). This repository does not
include them (see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)).

```powershell
$base = "https://raw.githubusercontent.com/MiRoboticsLab/bridges/047a0a48b9f2411ed535555eaa6c2276699378ac/params/toml_config/cyberdog_grpc"
New-Item -ItemType Directory -Force certs | Out-Null
curl.exe -fsSL "$base/ca-cert.pem"     -o certs/ca-cert.pem
curl.exe -fsSL "$base/server-cert.pem" -o certs/client-cert.pem
curl.exe -fsSL "$base/server-key.pem"  -o certs/client-key.pem
```

**3. Start the app.**

```powershell
.\app2\run.ps1                         # the real dog at 192.168.137.115:50052
.\app2\run.ps1 -Dog 127.0.0.1:50052    # the emulator on the same PC
```

The browser opens `http://127.0.0.1:8794/`. Turn on the movement switch, then press **1** to
stand.

## The SDK

```python
import time
from cyberdog_sdk import CyberDog

dog = CyberDog("192.168.137.115")   # reads the certificates from certs/
dog.connect()
try:
    print(dog.status())
    dog.stand()
    for _ in range(20):             # walk forward at 0.2 m/s for about 2 s
        dog.drive.move(0.2, 0.0, 0.0)
        time.sleep(0.1)
    dog.drive.stop()
    dog.sit()
finally:
    dog.close()
```

`dog.drive` stops by itself when `move()` is not called again within 0.5 s. `dog.estop()`
sends the emergency stop. `dog.trick()` refuses risky motions unless the caller passes
`allow_risky=True`. The transport refuses the commands on its forbidden list, such as reboot,
power off and firmware update, unless the caller passes `force=True`. The API contract is in
[`docs/design/contracts.md`](docs/design/contracts.md).

## The app

The app is a single page, [`app2/ui.html`](app2/ui.html), and a local server,
[`app2/server.py`](app2/server.py). It listens only on `127.0.0.1`, so no other computer on the
network can use it. Only one browser tab controls the dog at a time. A second tab can only stop
the dog. The full guide, with all keys, is in Italian in
[`app2/README.md`](app2/README.md).

## The emulator

```powershell
python -m emulator --port 50052
```

The emulator models the services that the SDK and the app use: motions with their real
preconditions, the walking stream and its watchdog, LEDs, the visual programming engine, the
camera, maps and navigation. `--controller lcm` connects it to Xiaomi's Gazebo simulator through
LCM. That mode runs only inside the WSL simulator image (see
[`docs/design/reports/gazebo_lcm-link.md`](docs/design/reports/gazebo_lcm-link.md)).

## The probe on the real dog

```powershell
python tools\probe_dog.py              # part 1 (the dog stays still), then part 2 (motions)
python tools\probe_dog.py --no-motion  # part 1 only
```

Part 2 starts only after the person at the dog types `libero` ("clear" in Italian). Ctrl+C sends
the emergency stop. The results go to `tools/probe_results-<date>-<time>.json`. That file holds
the dog's serial number and Wi-Fi details, so `.gitignore` excludes it.

## Tests

```powershell
python -m pytest tests/sdk tests/app tests/integration tests/emulator tests/tools -q
```

| Suite | What it covers |
|---|---|
| `tests/sdk` | SDK units, emergency stop races, the walking dead-man |
| `tests/emulator` | the emulator against the protocol notes |
| `tests/integration` | the SDK against a running emulator |
| `tests/app` | the app server and its HTTP API |
| `tests/tools` | the probe, against the emulator |
| `tests/gazebo` | end to end in Gazebo, only inside the WSL simulator image |

The five suites in the command give 821 passed on Windows (Python 3.12) and on WSL (Python 3.8).

## Repository layout

```text
cyberdog_sdk/        Python SDK (transport, motions, walking, posture, camera, LEDs, navigation)
app2/                browser control app v2: server.py, ui.html, run.ps1, Italian guide
emulator/            fake dog with the same gRPC interface; controllers/ holds the Gazebo link
tools/probe_dog.py   guided probe session on the real dog
tests/               test suites, see above
docs/connection.md   network, Wi-Fi QR code and control channel
docs/protocol/       how the dog behaves, from Xiaomi's sources
docs/design/         contracts, reports, simulator baseline, UI drafts
proto/, ref/         Xiaomi's gRPC definition and server source (Apache-2.0)
app/                 v1 controller, kept unchanged
dog_cli.py, scan.py, make_qr.py, keepalive.ps1, watch_dog.sh, run_all.ps1
                     v1 helpers: command line client, port scan, Wi-Fi QR code, hotspot keepalive
```

## Documentation

| Document | Content |
|---|---|
| [`docs/protocol/grpc_fidelity.md`](docs/protocol/grpc_fidelity.md) | the gRPC server: commands, replies, errors, timings |
| [`docs/protocol/motion.md`](docs/protocol/motion.md) | motion IDs, preconditions, the walking stream |
| [`docs/protocol/nav_map.md`](docs/protocol/nav_map.md) | maps and navigation |
| [`docs/protocol/vp_led_sensors.md`](docs/protocol/vp_led_sensors.md) | the visual programming engine, LEDs, sensors |
| [`docs/protocol/sim_bridge.md`](docs/protocol/sim_bridge.md) | the link to the Gazebo simulator |
| [`docs/design/contracts.md`](docs/design/contracts.md) | the binding API of the SDK, the emulator and the app |
| [`docs/connection.md`](docs/connection.md) | the network, the Wi-Fi QR code, the control channel, `dog_cli.py` |
| [`CHANGELOG.md`](CHANGELOG.md) | the releases |

## License

The original code and documentation are under the **MIT License**, see [`LICENSE`](LICENSE).

A few files come from Xiaomi's CyberDog 2 open source code and stay under the **Apache License
2.0** ([`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt)). These are the gRPC definition in
`proto/`, the server source in `ref/` and the motion table in `emulator/data/`.
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) lists each file, its source and its changes,
together with the licenses of the Python dependencies.

## Disclaimer

This project is independent. Xiaomi does not make, endorse or support it. Xiaomi and CyberDog
are trademarks of Xiaomi Corporation. The software comes without warranty, as the license
states. You use it on your own robot at your own risk.
