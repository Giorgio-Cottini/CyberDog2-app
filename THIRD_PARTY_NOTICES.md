# Third-party notices

The original code and documentation in this repository are under the MIT License
([`LICENSE`](LICENSE)). This file lists everything that is not original work, its source and its
license. The files in section 1 stay under their own license, the Apache License 2.0. Its full
text is in [`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt).

## 1. Files derived from Xiaomi's CyberDog 2 open source code

Xiaomi publishes the CyberDog 2 software under the Apache License 2.0 in the
[MiRoboticsLab](https://github.com/MiRoboticsLab) organization on GitHub.

| File in this repository | Source | Changes |
|---|---|---|
| [`ref/cyberdog_app.cpp`](ref/cyberdog_app.cpp) | [`MiRoboticsLab/bridges`](https://github.com/MiRoboticsLab/bridges) `cyberdog_grpc/src/cyberdog_app.cpp`, commit `047a0a4` | None. Only the line endings differ. Copyright (c) 2023 Xiaomi Corporation. Kept as the reference for the protocol notes. |
| [`proto/cyberdog_app.proto`](proto/cyberdog_app.proto) | [`MiRoboticsLab/bridges`](https://github.com/MiRoboticsLab/bridges) `cyberdog_grpc/protos/cyberdog_app.proto`, commit `047a0a4` | Modified. The file header lists the changes. The wire format is unchanged. |
| [`proto/cyberdog_app_pb2.py`](proto/cyberdog_app_pb2.py), [`proto/cyberdog_app_pb2_grpc.py`](proto/cyberdog_app_pb2_grpc.py) | Generated from `proto/cyberdog_app.proto` by `grpc_tools.protoc` | Generated code, not edited. |
| [`emulator/data/motion_id_map.json`](emulator/data/motion_id_map.json) | [`MiRoboticsLab/motion`](https://github.com/MiRoboticsLab/motion) `motion_action/preset/motion_id_map.toml` | Converted from TOML to JSON (134 entries). The values are unchanged. |

Other uses of the same sources:

- **Protocol notes** ([`docs/protocol/`](docs/protocol)). They describe how the dog behaves, with
  file and line references into the Xiaomi sources and short excerpts of them.
- **The QR helper** ([`make_qr.py`](make_qr.py)). It uses the fixed AES key from Xiaomi's
  `connector` source, so that the dog can read the Wi-Fi QR code.

## 2. Material that is not in this repository

**The mTLS certificates.** The dog accepts only clients that present a certificate signed by its
CA. Xiaomi publishes the CA certificate and a certificate with its private key in
[`MiRoboticsLab/bridges`](https://github.com/MiRoboticsLab/bridges/tree/047a0a48b9f2411ed535555eaa6c2276699378ac/params/toml_config/cyberdog_grpc)
under `params/toml_config/cyberdog_grpc/`. This repository does not redistribute them, because
a private key does not belong in version control even when it is public. The README explains how
to download them into `certs/`.

**The simulator.** The Gazebo tests (`tests/gazebo/`) and `emulator/controllers/lcm_link.py`
run inside a separate WSL distribution with Xiaomi's CyberDog simulator, ROS 2, Gazebo and
LCM. None of those are in this repository.

## 3. Python dependencies (installed separately, not redistributed)

| Package | Used by | License (from the package metadata) |
|---|---|---|
| [grpcio](https://pypi.org/project/grpcio/) | SDK, app, emulator, probe | Apache-2.0 |
| [grpcio-tools](https://pypi.org/project/grpcio-tools/) | regenerating `proto/*_pb2*.py` | Apache-2.0 |
| [protobuf](https://pypi.org/project/protobuf/) | v1 app (`app/`), `dog_cli.py` | BSD-3-Clause |
| [pytest](https://pypi.org/project/pytest/) | tests | MIT |
| [pycryptodome](https://pypi.org/project/pycryptodome/) | `make_qr.py` | BSD-2-Clause AND Unlicense AND MIT |
| [qrcode](https://pypi.org/project/qrcode/) | `make_qr.py` | BSD |
| [lcm](https://github.com/lcm-proj/lcm) | `emulator/controllers/lcm_link.py`, simulator only | LGPL-2.1 |

## 4. Trademarks

Xiaomi and CyberDog are trademarks of Xiaomi Corporation. This project is independent: Xiaomi
does not make, endorse or support it. The names appear here only to say which robot the software
works with.
