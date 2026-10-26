# Changelog

This file follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the versions
follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- The emergency stop passed on the real dog: stopped while walking, the dog stayed down. The
  README shows the result.

## [2.0.0] - 2026-10-26

The first public release. It contains the v1 controller and the v2 rewrite. The probe ran a
full session on a real CyberDog 2, and every step passed. The emergency stop was not tested on
the real dog.

### Added

- `cyberdog_sdk/`: a Python SDK for the dog's gRPC service. It covers motions with
  precondition checks, walking with a dead-man stop, posture, LEDs, the electronic skin,
  sensors, the camera, maps and navigation.
- `app2/`: a control page in the browser with keyboard driving, motions on keys 1 to 9, gaits,
  speed, posture, lights, camera and maps. It has a movement switch that is off at every load,
  and a "floor clear" confirmation for motions that move far.
- `emulator/`: a fake dog with the same gRPC interface, and a link to Xiaomi's Gazebo
  simulator through LCM.
- `tools/probe_dog.py`: a guided session that measures how the real dog behaves.
- `docs/protocol/`: protocol notes taken from Xiaomi's sources.
- `docs/design/`: the API contracts, the build reports and the simulator baseline.
- `app/`: the v1 controller, kept unchanged, for W/A/S/D/Q/E driving, stand, lie down,
  emergency stop, volume, telemetry and the camera. `dog_cli.py`, `scan.py`, `make_qr.py`,
  `keepalive.ps1`, `watch_dog.sh` and `run_all.ps1` help with the connection, the Wi-Fi QR
  code and the Windows hotspot.
- 821 tests on Windows (Python 3.12) and on WSL (Python 3.8).
- `LICENSE` (MIT), `THIRD_PARTY_NOTICES.md` and `LICENSES/Apache-2.0.txt`.
- `docs/connection.md`: the network, the Wi-Fi QR code and the control channel.

### Known limits

- The emergency stop was not tested on the real dog. It goes three times: the third ESTOP goes
  1.0 s after the end of the walk stream, because the real dog's servo watchdog fired 0.61 to
  0.72 s after the last frame. The SDK, the app and the probe all send it.
- Navigation was not tested on the real dog.
- The repository does not include the mTLS certificates. The README explains how to download
  them from Xiaomi's repository.

[Unreleased]: https://github.com/Giorgio-Cottini/CyberDog2-app/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/Giorgio-Cottini/CyberDog2-app/releases/tag/v2.0.0
