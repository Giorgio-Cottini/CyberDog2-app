# CyberDog v2: build contracts

This file fixes the interfaces between the parts of v2, so that each part can be built and tested on its own.
The protocol facts live in `docs/protocol/*.md`. Those specs are the arbiter: when code and spec disagree, the spec wins unless the real dog proves it wrong.

## 0. Ground rules for every part

- **v1 stays untouched:** do not edit `app/`, `dog_cli.py`, `proto/` or `ref/`.
- **Python:** every module in `cyberdog_sdk/` and `emulator/` must run on **Python 3.8** (the simulator VM) and **3.12** (Windows). Use `from __future__ import annotations`. No `match`, no `tomllib`, no `str.removeprefix`, no `X | Y` outside annotations, no 3.9+ stdlib APIs.
- **Dependencies:** grpcio only, plus the standard library. Do not import protobuf or the generated stubs in `proto/`. Use `cyberdog_sdk/wire.py` (hand-written codec, already cross-checked byte-for-byte against the generated stubs).
- **Windows run:** `python` is 3.12. Packages come from `pip install -r requirements.txt`, or from a local `pylibs/` folder (`python -m pip install --target pylibs ...`). The root `conftest.py` puts `pylibs/`, when it exists, and the repo root on `sys.path`. Run tests with `python -m pytest` from the repo root.
- **Certificates:** `certs/ca-cert.pem` (CA), `certs/client-cert.pem` + `certs/client-key.pem` (CN `cyberdog2.server`). The SDK presents this pair as its client identity. The emulator presents the same pair as its server identity, exactly like the dog. TLS target name override: `cyberdog2.server`. Verified: a grpcio server with this pair and `require_client_auth=True` accepts the SDK and refuses a client without a certificate.
- **Style:** match the comment density and naming of `cyberdog_sdk/wire.py` and `app/server.py`. No new frameworks.

## 1. Safety rules (binding)

The SDK, the emulator and the app enforce these. Tests must cover each one.

1. **Name codes never sent** by the SDK unless the caller passes `force=True` to `Transport.call`: 9030 (unlock), 9031 (reboot), 10006 (power off), 12001 (leg calibration), 55001 (test disconnect), and 5000-5999 (OTA). The app never passes `force`. A name code outside 0..2**32-1 raises ValueError, also with `force`: the wire keeps the low 32 bits, so 9031 + 2**32 would reach the dog as 9031.
2. **6004 `type`** must be in {1, 5, 7, 9, 11, 13}; never 0. Always send `type`, `outdoor` and `keep_distance`. Type 1 needs both `goalX` and `goalY`.
3. **6002 delete-all guard:** `only_delete: true` with an empty or invalid label list wipes every map. Refuse it unless the caller passes `confirm_delete_all=True`. Label coordinates are always JSON floats.
4. **Motion `value` bit 0x04** (factory calibration) is always masked off.
5. **4004 `file_name`** containing `/`, `\` or `..` is refused.
6. **Never empty `params`:** send `"{}"` at least.
7. **JSON typing:** float arrays are JSON doubles (`0.0`, never `0`); int fields are JSON ints (`143`, never `143.0`). Arrays have exact sizes: `vel_des`, `rpy_des`, `pos_des`, `ctrl_point` 3; `acc_des`, `foot_pose` 6; `step_height` 2.
8. **Motion risk classes** (motion.md §6): R0 static, R1 in place and dynamic, R2 travels or long, R3 jumps, R4 flips/RL/special. The SDK refuses R3 and R4 unless `allow_risky=True`. The app never offers R3 or R4. R2 needs a "floor clear" confirmation in the app.
9. **ESTOP** goes on its own channel, never behind a lock or a queue.
10. **Movement gate in the app:** off at server start, forced off on every page load, required for every motion, posture, servo and navigation command. ESTOP is always allowed.
11. **Default speeds** do not rise above v1: 0.2 m/s forward, 0.15 m/s strafe, 0.4 rad/s turn at the default speed setting.
12. `qr.html` is never read into code, committed, or served.

## 2. Folder layout

```
cyberdog_sdk/            the SDK (part A)
  wire.py                done: protobuf codec
  transport.py           mTLS channels, call / call_json / get_file, guards 1, 5, 6
  codes.py               NameCode, MotionId, result-code meanings (it + en)
  motions.py             motion catalog with risk classes
  client.py              CyberDog high-level client
  servo.py               Driver (walk, 1002) and Posture (201 stream)
  camera.py              still-frame grabber (4002 / getFile), ported from app/server.py
  vp.py                  VP 2002 task builder: LED, sensors, file drop + 4004 fetch
  nav.py                 maps and navigation (6002/6003/6004/6009/6010 + 10001 polling)
emulator/                the fake dog (part B)
  __main__.py            python -m emulator --port 50052 [--controller fake|lcm] [--time-scale 1.0] [--control-port 50080]
  server.py              gRPC GrpcApp over mTLS, rapidjson-faithful parsing, handlers
  manager.py             motion_manager logic (sim_bridge.md §2.6), over a Controller
  controllers/fake.py    FakeController: response-level locomotion FSM + kinematic pose
  controllers/lcm_link.py  LcmController: real LCM to Gazebo (part C)
  vp.py, camera.py, nav.py, device.py   the other dog services
  control.py             HTTP control/inspection endpoint for tests
  data/motion_id_map.json  done: export of motion_id_map.toml (134 entries)
app2/                    the new app (part D): server.py, ui.html, run.ps1, README.md
tools/probe_dog.py       mid-work probe on the real dog (part E)
tests/                   pytest: tests/sdk, tests/emulator, tests/integration, tests/app, tests/gazebo
conftest.py, pytest.ini  markers: gazebo (needs the WSL sim), slow
```

## 3. SDK public API (part A)

Names and signatures below are binding; internals are free. All times are seconds, all distances metres, all angles radians.

```python
from cyberdog_sdk import (CyberDog, Transport, Driver, Posture, Gait, MotionResult, Status,
                          DogError, DogUnreachable, ForbiddenCommand, RiskyMotion, catalog)

dog = CyberDog(host="192.168.137.115", port=50052, cert_dir=None)   # cert_dir None -> <repo>/certs
dog.connect(timeout=5.0)          # waits for both channels to be READY, else DogUnreachable
dog.close()                       # waits for an estop() in progress (its second ESTOP), then stops streamers
                                  # (END if one was walking) and the camera, closes channels;
                                  # a move()/set() waiting to start, or called later, sends nothing

# raw access
dog.transport.call(name_code, params=None, timeout=10.0, force=False) -> list[str]   # data of every RecResponse
dog.transport.call_json(name_code, params=None, timeout=10.0) -> list[dict]          # non-JSON items skipped
dog.transport.get_file(name_code, params, timeout=30.0) -> FileResult(error_code, file_name, data: bytes)

# status and info
dog.status(timeout=3.0) -> Status      # 10001: motion_id, task_status, task_sub_status, state, state_code,
                                       # wired_charging, wireless_charging, self_check_code, raw (dict)
dog.device_info(*flags, timeout=15.0) -> dict   # 1001; flags like "bat_info", "stand", "motor_temper", "wifi",
                                                # "volume"; "version" only when asked explicitly (slow, up to 13 s)
dog.set_volume(level: int) -> bool     # 1012

# result motions (1004); blocking
dog.motion(motion_id, *, duration=0, timeout=25.0, allow_risky=False, **arrays) -> MotionResult
    # MotionResult(motion_id, result: bool, code: int, message: str, still_running: bool)
    # still_running is True when the bridge returned error 321 for a motion whose catalog length > 20 s
    # 202 with duration 0 raises ValueError: the dog never ends it and adds rpy_des/pos_des again every 2 s
    # (fsm_state_balancestand.cpp:338-396). While estop() runs, a motion other than 0 is not sent: it returns
    # MotionResult(code NOT_SENT_ESTOP = -2, "not sent: estop() was called"), since a 111 then would clear the
    # latch. result false with code 0 (the manager left the reply unfilled after an ESTOP during the 111 it
    # inserts) has the message "no result (ESTOP during the stand first)", never "OK"
dog.stand()     # 111
dog.sit()       # 143
dog.lie_down()  # 101
dog.estop()     # 0, on the priority channel, timeout 3 s; halts Driver and Posture first (no END), then
                # always sends ESTOP three times. The second goes out from its own thread (not a daemon: an
                # interpreter exit waits for it) at max(halts done, end of the halted Driver stream) + WATCHDOG_S
                # 0.3 s (the halts end just before the first send; a busy process can delay that send by the
                # repeat thread's start, about 1 ms), so a watchdog WalkStand 112 from any frame that reached the
                # dog lands before it, also
                # when the first ESTOP is late or lost. The third goes from the same thread at the same point
                # + ESTOP_LATE_S 1.0 s: on the real dog the 112 came 0.61-0.72 s after the last frame. Neither
                # waits for the first reply (ESTOP's 1004 can
                # take up to 1 s). While estop() runs no stream starts or sends: a move()/set() meanwhile is
                # dropped (no velocity, target unchanged). Returns after all replies (at least 1.0 s): the
                # first reply with result true, else the last; raises when the last one raised, and a
                # KeyboardInterrupt or other BaseException met on the way is raised after all went out
                # (a send it interrupts is repeated, up to SHIELD_TRIES 3 times per ESTOP; a failed thread
                # start sends the repeats from the calling thread). A send the priority channel refuses at once
                # (UNAVAILABLE: that channel is in reconnect backoff) goes again at once on the main lane; a
                # deadline does not. Channels retry a lost connection every 0.5-1 s.
                # Never returns None. last_motion is 0 from the moment the first is sent.
                # Why three times: the watchdog's WalkStand bypasses motion_manager's ESTOP latch
                # (motion_handler.cpp:281-300); the open-source controller refuses mode 11 out of ESTOP
                # (fsm_state_passive.cpp:65-93), the dog's firmware is unverified, so the SDK does not rely on it
dog.trick(motion_id, allow_risky=False) -> MotionResult
    # catalog-checked; after a sit (motion_id 143/214/148-150 in status) sends 111 first unless the target is
    # 148-150, 111, 101 or 0; 148-150 send 143 first when not sitting, and also at status 214 when last_motion
    # is known and not 140/143/148-150 (the dog reports 214 after every mode-62 trick). Steps: client.trick_steps()
    # No step goes out once an ESTOP was sent, or estop() began, after trick() was called: code NOT_SENT_ESTOP.
    # A 1004 already on the link when estop() begins is not covered: if it reaches the dog after both ESTOPs,
    # a 111 clears the latch (build_sdk.md open issues)
dog.last_motion -> int | None   # last motion id this client completed with result true; 0 from the moment an
                                # ESTOP is sent; None on a new client (unknown: trick() keeps the status-only rule)

# servo streaming (1002)
dog.drive: Driver
    Driver.set_gait(gait: Gait | int)       # Gait.SLOW 303, MEDIUM 308, FAST 305, AUTO 304
    Driver.move(vx, vy, wz)                 # starts or updates the stream; clamped to the gait limits
    Driver.deadman_s = 0.5                  # dead-man, on by default: no move() for this long -> one END, stream stops
                                            # (counted from the later of the last move() and the moment the stream thread
                                            # began; checked at frame boundaries, so on a slow link up to FRAME_DEADLINE 0.25 s later);
                                            # None turns it off for scripts, which then must call stop(); also Driver(..., deadman_s=)
    # every stream: one END only if it sent at least one frame (END alone runs WalkStand 112).
    # halt()/ESTOP: no frame starts once it is seen, and it cancels a move()/set() that is restarting the stream
    # at that moment (a cancelled move() leaves no velocity, a cancelled set() leaves the target unchanged);
    # a frame already past the last check can still reach the dog, which the second ESTOP covers.
    # A move()/set() that begins after estop() returned starts a new stream (the dog answers 3040 until 111).
    # halt_thread() is halt() returning the stream thread it stopped (or None); estop() joins exactly that one.
    # A crash inside the stream thread (any BaseException) sends one END if the stream was walking and records
    # "stream stopped: ..." in stats["last_error"], also when that END fails; a move() during the crash END
    # starts a new stream. Stream threads are daemons: at interpreter exit no END goes
    # out and the dog's watchdog stops it (WalkStand 112) about 250 ms later.
    # A set() that arrives while move() takes over from the Posture stream applies on top of the walking reset.
    Driver.stop()                           # one END frame, then the stream stops
    Driver.step_height = 0.05               # checked when set, as are move()'s values: ValueError for bool, str,
                                            # bytes, NaN, inf and ints too large for a float, TypeError for other non-numbers;
                                            # clamped to the doc range when sent as [h, h]
    Driver.active -> bool; Driver.stats -> dict (frames sent, errors, max gap ms between frames that went through)
    # rate 25 Hz; each frame is its own RPC with a 0.25 s deadline; a gait change while active sends END,
    # waits 0.15 s, then restarts with the new gait
dog.posture: Posture
    Posture.set(z=None, roll=None, pitch=None, yaw=None)   # absolute, clamped to the conservative limits below;
                                                          # streams 201 at 15 Hz for 0.6 s after the last change, then END (pose held)
    Posture.reset()                                        # z 0.235, rpy 0, same streaming
    # conservative limits: z 0.18..0.30, roll +/-0.35, pitch -0.2..0.25, yaw +/-0.5
    # Driver and Posture never stream at the same time; starting one stops the other (END, 0.15 s)

# LED, skin, sensors
dog.led(target: str, color=(r, g, b) | None, effect: str = "on", seconds: float = 3.0) -> str   # VP 2002, returns task id
dog.led_off(target: str) -> str
    # the dog keeps one VP debug task and a new one kills the running one, so one slot per client (vp.Slot)
    # serialises every 2002 and each task carries the client's whole LED state: its own work, freed() for targets
    # released in the last 60 s, a replay of every running hold, then each hold's freed() at its end time
dog.skin(mode: int, wave_ms: int = 3000) -> bool        # 11001 {"data":true} then 11002
dog.read_sensors(kinds=("bms", "tof", "ultrasonic", "odom"), timeout=20.0) -> dict
    # VP debug task writes /home/mi/Camera/<id>.json; the SDK polls getFile 4004 until found or timeout
    # holds the VP slot for the whole read (send and polling): an LED call made meanwhile waits for it
    # task bodies must be valid Python 3.6 (the dog's VP runtime); VP call names come from the VP sources

# camera
dog.camera.start(); dog.camera.latest() -> (seq, jpeg_bytes) | None; dog.camera.stop()
dog.camera.fps -> float                  # over the last 12 frames; 0 when no frame came for 2 s (STALE_S)
    # a START (start() or a re-arm after error 5) is dropped once stop() ran; stop() never waits behind a START
    # (up to 10 s on the dog), and a START that answers after stop() is followed by another STOP, so STOP is the
    # last 4002 command; start() counts fps from zero again

# maps and navigation
dog.nav.get_map(name="") -> MapData      # 6003: resolution, width, height, origin (x, y), data (list[int]), labels
dog.nav.set_labels(map_name, labels, *, only_delete=False, confirm_delete_all=False) -> bool   # 6002
dog.nav.start_mapping() / save_map(name) / relocalize() / go_to(x, y) / dock() -> NavTask
dog.nav.stop(task_type) -> int           # 6009 result code
dog.nav.state() -> NavState              # from 10001 task_status / task_sub_status, decoded per nav_map.md §4
    # stop_type for AB (status 1/21) is 1 only after the goal went out (sub_status 300 or 307), else 0: before
    # feedback 300 a type 1 stop answers success, does nothing, and the goal still goes out (nav_map.md §5 d);
    # type 0 (ResetNav) stops it in every phase and leaves the dog not localized (101)
```

`catalog` is `cyberdog_sdk.motions`: `catalog.get(id) -> Motion(id, key, name_it, name_en, risk, lr_safe, min_ms, typical_s, needs)`, `catalog.all()`, `catalog.by_risk(*classes)`.

## 4. Emulator contracts (part B)

### 4.1 Controller interface (between `manager.py` and a controller)

```python
class Controller:
    def publish(self, cmd: dict) -> None
        # cmd = robot_control_cmd_lcmt fields: mode, gait_id, contact, life_count, vel_des[3], rpy_des[3],
        #       pos_des[3], acc_des[6], ctrl_point[3], foot_pose[6], step_height[2], value, duration
    def on_response(self, cb) -> None
        # cb(resp: dict) at about 50 Hz; resp = mode, gait_id, contact, order_process_bar, switch_status,
        #       ori_error, footpos_error, motor_error[12]
    def ground_truth(self) -> dict
        # {"x","y","z","roll","pitch","yaw","vx","vy","wz","t"}: Gazebo simulator_state for lcm, the
        # kinematic integrator for fake
    def close(self) -> None
```

- `manager.py` implements sim_bridge.md §2.2-2.4 and §2.6: ESTOP latch, busy (3011), duration rule (3010), unknown id (3008), CheckPostMotion with automatic 111, 20 Hz heartbeat with an unchanged life_count, the servo watchdog (4 misses of 50 ms), WalkStand 112 on END, and the reverse map. Result codes use the numbers in motion.md §2.8.
- The heartbeat, the manager timeouts and the 20 s gRPC cap run on **wall-clock time**. With the fake controller, `--time-scale s` multiplies every motion length and every manager timeout by `s`, so tests run fast; the gRPC cap becomes `20 * s`.
- `FakeController` reproduces at the response level: the per-state transition tables and BAN_TRANS (sim_bridge.md §4), progress bars over motion lengths, the idle fillers, the 0.5 s command timeout to PureDamper (7), and a 2D kinematic pose for mode 11 (velocity integration) and posture modes (z, rpy).
- `LcmController` speaks LCM exactly as sim_bridge.md §1. It never publishes `motor_ctrl`.

### 4.2 Control endpoint (HTTP on 127.0.0.1:<control-port>, default 50080; 0.0.0.0 inside WSL)

- `GET /state` returns JSON: `motion` (motion_id, switch_status, bar, busy, estop_latched), `ground_truth`, `leds` ({head, tail, mini}: {r, g, b, effect, owner} or null), `skin`, `servo` (frames received, last frame, gaps), `nav` (task, status, pose), `camera` (state, files), `calls` (count per nameCode), `refused` (list of refused requests with reason).
- `POST /reset` restores the boot state (lying down, motion_id 101).
- `POST /fault {"dep": "<ros service name>", "mode": "ok|absent|timeout|hang"}` injects faults (grpc_fidelity.md §9 item 11).
- `POST /battery {"soc": int, "charging": bool}`.

### 4.3 Boot state

Lying down (motion_id 101, mode 7), battery 80 %, not charging, task_status 101 (idle), one saved map "casa" with two labels, camera idle, no LED owned.

## 5. Gazebo path (part C)

- Inside the `CyberDogSim` WSL distro: `/root/sim_up.sh` starts Gazebo and the controller; `/root/sim_down.sh` stops them. Use `wsl.exe -d CyberDogSim -e bash -lc '<cmd>'` (always `-e`, never `--`).
- `/root/emu_up.sh` (new) starts `python3 -m emulator --controller lcm --host 0.0.0.0 --port 50052 --control-port 50080` from `/mnt/c/.../CyberDog`, detached, logs in `/root/sim_logs/emu.log`. Windows reaches it at `127.0.0.1:50052` and `127.0.0.1:50080` through WSL localhost forwarding.
- The real-time factor is about 0.35 (world solver) or 0.6 (`FAST=1`). Tests use wall-clock tolerances divided by the measured RTF.
- grpcio for Python 3.8 is installed in the VM with `pip3 install grpcio` (the newest version that supports 3.8).

## 6. App (part D)

- `app2/server.py`: stdlib `ThreadingHTTPServer` on 127.0.0.1:8794 (v1 keeps 8793), built only on `cyberdog_sdk`. `--dog host:port` selects the real dog (default `192.168.137.115:50052`) or the emulator.
- `app2/ui.html`: one self-contained page, Italian labels, no CDN.
- Keys: W/S forward and back, A/D turn, Q/E strafe (hold to move), Space ESTOP, G cycles the gait, `+`/`-` speed, 1 stand/sit toggle (111 when sitting or lying, 143 when standing), 2-9 tricks 141, 123, 146, 144, 145, 174, 173, 140.
- 140 (dance, 60 s, R2) asks "pavimento libero?" before it runs.

### 6.1 HTTP API (binding between `ui.html` and `server.py`)

All bodies are JSON. Errors are `{"ok": false, "error": "<Italian message>", "code": <int or null>}` with HTTP 400 (bad input), 403 (movement gate off for a gated command), 409 (`{"need_confirm": true, ...}` for an R2 motion sent without `confirm`; `{"busy": true}` while a motion runs; `{"navigating": true}` during autonomous navigation), 502 (the dog refused or did not answer). Success is `{"ok": true, ...}`.

Autonomous navigation means goto, dock or tracking running (task_status 1, 21, 9, 11, 13, 3), or a goto or dock accepted that the status poll does not show yet. During it, `/api/drive` (non-zero), `/api/posture`, `/api/posture/reset`, `/api/motion` and `/api/action` answer 409 `{"navigating": true}`. For motion and action the message tells the user to press «Ferma navigazione» first; the navigation check comes before the R2 confirm. Reason: no 1002 stream during autonomous navigation (nav_map.md 4), and what the dog does with a 1004 then is unverified, so the app never mixes the two. ESTOP and `/api/nav/stop` stay ungated.

| Method and path | Body | Gated | Notes |
|---|---|---|---|
| GET `/` | | | ui.html |
| GET `/api/state` | | | the state object below |
| GET `/api/events` | | | SSE. `event: state` (the state object, at most 5 per second) and `event: log` (`{"t", "level": "info"\|"warn"\|"error", "msg"}`) |
| GET `/api/catalog` | | | `{"motions": [{"id", "name", "risk", "needs_confirm", "typical_s"}], "keymap": {"1": "stand_sit", "2": 141, ..., "9": 140}, "gaits": [{"key": "slow", "name", "motion_id", "max": [vx, vy, wz]}]}`. R3/R4 never listed |
| GET `/api/map` | | | `{"ok", "name", "resolution", "width", "height", "origin": [x, y], "data": [int], "labels": [{"name", "x", "y"}]}` (data: -1 unknown, 0 free, 100 occupied), plus `"yaw"`, `"success"` (0 = map sent) and, when success != 0, `"message"` with width 0 and empty data. One 6003 at a time, shared by every request. A request waits at most quick_s (1 s), then gets `{"ok": true, "pending": true}` while the 6003 runs; the UI asks again 1 s later. A finished outcome no request collected is served for MAP_KEEP_S (5 s); a 6003 still running after MAP_WAIT_S (12 s) gives 502; code 321 says the dog's map service did not answer in 10 s. The dog sends only the saved map, so the UI does not poll while mapping. After an error the UI keeps the map it shows and asks again after 5 s, doubling up to 60 s; when the dog connects again and the page has no map (or a retry is waiting), it asks at once and the backoff starts again at 5 s |
| GET `/stream.mjpg` | | | camera MJPEG (multipart/x-mixed-replace) |
| POST `/api/arm` | `{"armed": bool}` | | disarming also stops every stream |
| POST `/api/estop` | `{}` | never | always allowed; the body is ignored, so a malformed or oversize body still runs the ESTOP (the JSON Content-Type stays required) |
| POST `/api/drive` | `{"x", "y", "z"}`, each -1..1 | yes | stick values: x forward, y left, z turn left. The server scales them by speed and gait. The UI resends at least every 150 ms while a key is held; after 400 ms of silence the server stops (END). All zeros stops. The server calls `Driver.move` on every accepted `/api/drive`, also when the values did not change, so the SDK dead-man (0.5 s) never cuts a held key |
| POST `/api/gait` | `{"gait": "slow"\|"medium"\|"fast"\|"auto"}` | no | |
| POST `/api/speed` | `{"level": 0.1..1.0}` | no | fraction of the gait limit; the default level gives 0.2 m/s forward with "slow" |
| POST `/api/action` | `{"name": "stand"\|"sit"\|"lie"\|"stand_sit_toggle"}` | yes | 409 `{"navigating": true}` during autonomous navigation |
| POST `/api/motion` | `{"id": int, "confirm": bool}` | yes | catalog ids only; 409 `{"navigating": true}` during autonomous navigation |
| POST `/api/posture` | any of `{"z", "roll", "pitch", "yaw"}` | yes | absolute; clamped by the SDK |
| POST `/api/posture/reset` | `{}` | yes | |
| POST `/api/led` | `{"target": "head"\|"tail"\|"mini", "color": "#rrggbb"\|null, "effect": str, "seconds": number}` | no | |
| POST `/api/led/off` | `{"target"}` | no | |
| POST `/api/skin` | `{"mode": 0..6, "wave_ms": int}` | no | |
| POST `/api/sensors/refresh` | `{}` | no | starts a VP read; the result lands in `state.sensors` |
| POST `/api/camera` | `{"on": bool}` | no | |
| POST `/api/volume` | `{"level": 0..100}` | no | |
| POST `/api/nav/start` | `{"task": "mapping"\|"relocalize"\|"goto"\|"dock", "x"?, "y"?}` | yes | goto needs x and y in map metres; 409 `{"busy": true}` while a motion runs; while a goto/dock waits for the dog's accept, motions and drive get 409 `{"navigating": true}`. A goto/dock that the dog accepts after an ESTOP or a `/api/nav/stop` (from any tab) is stopped at once and answers 409 `{"cancelled": true}`. The 6009 waits for any stop still running (at most `NAV_STOP_WAIT_S` 10 s) and is always sent, since 10001 reads a cache; its type is the one the state shows for that task, else 0 for a goto and 9 for a dock. A FAILED answer with no task left in the state means the stop before already ended it. The error text says which of these happened: stopped, stop sent and pending, already stopped, or not stopped. The gate and both stop counters are read in one lock block, so an ESTOP right after the gate check either disarms first (403) or counts. With the stop wait the answer can take about 23 s; the page waits 25 s |
| POST `/api/nav/save_map` | `{"name"}` | yes | `{"ok": true, "accepted": 1, "pending": true}` when the save has not finished after 9 s; a later failure is an error log event |
| POST `/api/nav/stop` | `{}` | never | the 6009 type comes from `NavState.stop_type`: a goto stopped before feedback 300 gets type 0 and the dog is no longer localized. `{"ok": true, "stopped": true, "pending": true}` when the 6009 has not answered after quick_s (type 0 runs ResetNav, 2 s or more); its late result is a log event. A second stop while one runs sends no second 6009 and gets the same pending answer |

One active tab: the page takes the Web Lock `cyberdog-app2-tab`. Only the tab that holds it opens `/api/events`, `/stream.mjpg` and `/api/map`; any other tab shows a cover with STOP and «Usa questa scheda» (which takes the lock and forces a disarm). Behind the cover the page is inert (no focus, no keys except Space = STOP), and its API layer refuses every POST except `/api/estop`, `/api/nav/stop` and `/api/arm` with `armed: false`. A goto or dock started in the tab that lost the lock keeps running; the new active tab shows it and can stop it. A browser that refuses the lock (site data blocked) runs the tab as active, as without Web Locks. Reason: Chromium shares 6 connections per host across tabs, and two tabs' long-lived streams could leave ESTOP waiting.

Timing (binding for both sides):
- **POST `/api/motion` and `/api/action`** return as soon as the server has handed the command to a worker thread: `{"ok": true, "queued": true}`. They never wait for the 1004 result, which can take 25 s. The result arrives in `state.motion.last_result` and as a log event. While a motion runs, `state.motion.busy` is true, and a second motion or action gets 409 `{"busy": true}`. ESTOP and `/api/nav/stop` are always accepted.
- **Every new result has a new `last_result.seq`**: an integer that rises by one on every result, ESTOP's id 0 included (the ESTOP reply's `result` carries it too). Refusals (4xx) are not results. The UI detects a new outcome by `seq`; the same trick twice gives two results with equal fields and different `seq`.
- **After a 321 "still running"** (`MotionResult.still_running`, a motion of 20 s or more): the motion goes on, and later 1004s get 3011 until it ends (motion.md, verifier additions). `state.motion.busy` stays true, with the status poll's motion id and name, and no `last_result` is written yet. Busy ends when two consecutive 10001 polls show another motion id; `last_result` is then `{"ok": false, "code": 321, "still_running": false, "message": "terminato dopo circa N s, senza conferma dal cane"}`. Cap: 1.5 times the catalog length (`typical_s` or `min_exec_time`, at least 20 s) from the 1004 send, for example 90 s for 140, 45 s for 151 and 180 s for the 120 s dances; at the cap busy ends with a warning log and `still_running: true`. ESTOP ends busy at once; disarming does not, because the dog keeps moving.
- **POST `/api/nav/start` and `/api/nav/save_map`** return after the dog's accept reply (up to about 7 s): `{"ok": true, "accepted": 1}`, or 502 with the reason. Progress then arrives through `state.nav` and `state.task`.
- **POST `/api/sensors/refresh`** returns at once; `state.sensors.pending` stays true until the VP read-back lands (3-20 s).
- **Every other POST** returns within about 1 s.

Cross-site protection (any web page can POST to 127.0.0.1): every POST carries `Content-Type: application/json`, or the server answers 415. The `Host` header must be `127.0.0.1:<port>` or `localhost:<port>`, and an `Origin` header, when present, must be `http://127.0.0.1:<port>` or `http://localhost:<port>`; otherwise 403. The server sends no CORS headers.

Pitch: the API takes `pitch` in radians, exactly as the SDK `Posture` (201, absolute). In Gazebo a positive 201 pitch lowers the nose. The UI labels the slider "testa giù" and "testa su", never "+" or "-", and keeps the sign in one constant (`PITCH_UP_SIGN = -1`), so that one edit flips it if the real dog disagrees.

State object:

```
{"connected": bool, "target": "192.168.137.115:50052", "emulator": bool, "latency_ms": int|null,
 "armed": bool, "battery": {"soc": int|null, "charging": bool},
 "motion": {"id": int|null, "name": str, "busy": bool,
            "last_result": {"id", "ok", "code", "message", "still_running", "seq"}|null},
 "drive": {"active": bool, "gait": "slow", "speed": 0.3, "vx", "vy", "wz"},
 "posture": {"z", "roll", "pitch", "yaw"},
 "task": {"status": int|null, "sub": int|null, "name": str},
 "leds": {"head": {...}|null, "tail": ..., "mini": ...}, "skin": {"mode": int|null},
 "sensors": {"t": float|null, "pending": bool, "bms": {...}, "tof": {...}, "ultrasonic": {...}, "odom": {...}},
 "camera": {"on": bool, "fps": float},
 "nav": {"state": str, "task": str|null, "goal": [x, y]|null}}
```

## 7. Tests

- `tests/sdk`: unit tests with a fake transport (no network).
- `tests/emulator`: raw gRPC against the emulator, checking fidelity to the specs.
- `tests/integration`: the SDK against the emulator over real mTLS, fake controller, `--time-scale 0.05`.
- `tests/app`: the app's HTTP API against the emulator.
- `tests/gazebo` (marker `gazebo`): the SDK against the emulator with the LCM controller in WSL; skipped when 127.0.0.1:50080 does not answer.
