# Protocol spec: vp_led_sensors

Extracted from MiRoboticsLab sources and verified against the code by a second reader.

# vp_led_sensors: corrected protocol spec (CyberDog 2, CN firmware, ROS 2 Galactic)

Paths are relative to `S`, a folder with the [MiRoboticsLab](https://github.com/MiRoboticsLab) repositories cloned side by side under their GitHub names.

Tags:
- **[CODE]** verified in source.
- **[DOC]** documentation only.
- **[INF]** inference, still to be verified.
- **[REAL]** observed on the dog.
- **[CLOSED]** closed or cloud component.

The PC talks only to `grpcapi.GrpcApp` on :50052 with mTLS (`bridges/cyberdog_grpc/src/cyberdog_app.cpp:121-125`). The reverse channel PC:8981 is unusable because we lack the `cyberdog2.client` cert.

---

## 0. Transport facts that shape everything

- **RPCs [CODE]** (`protos/cyberdog_app.proto:25-27`):
  - `sendMsg(SendRequest) -> stream RecResponse`
  - `getFile(SendRequest) -> stream FileChunk{error_code, file_name, file_size, buffer}` (`:162-167`)
- **2002 VISUAL_FRONTEND_MSG [CODE].**
  - `params` is published verbatim as `std_msgs/String` on ROS `frontend_message` (`cyberdog_app.cpp:161-162`, `:3683-3687`).
  - No `RecResponse` is written, so the stream closes empty. It is fire-and-forget.
- **2001 VISUAL_BACKEND_MSG [CODE].**
  - Engine replies go to ROS `robotend_message` (`cyberdog_app.cpp:157-159`, `:783-787`).
  - They are then pushed to the phone through the reverse client: `send_grpc_msg` `:942-975`, gated by `can_process_messages_ && app_stub_ && connect_mark_` (`:471-481`).
  - **Consequence [INF]: the PC cannot see any engine feedback** (parse errors, FSM refusals, task state, inquiry lists). It needs an out-of-band return path (§4).
- **Also reverse-channel-only [CODE]:**
  - heartbeat, including `batt_soc` (`:483-567`);
  - LASER_SCAN 6011 (`:1859-1904`);
  - 4003 FILES_NOT_DOWNLOAD_COMPLETE (`:3980-3998`).
- **Bad `params` JSON [CODE].** The app replies `{"error_code":323}` with nameCode 0, because the nameCode is set only after parsing (`:3664-3670`). The error format is built by `returnErrorGrpc` (`:978-993`).
- **Dispatcher [CODE].** Each nameCode has a queue of 10 that drops the oldest message (`include/msg_dispatcher.hpp:37`, `:59-61`).

## 1. VP engine (`interaction/cyberdog_vp/cyberdog_vp_engine`)

### 1.1 Request JSON (`bridges/protocol/ros/msg/VisualProgrammingOperate.msg:68-79`)

Fields: `type, id, target_id[], describe, style, operate, mode, condition, body`. (`fsm`, `dependent[]` and `be_depended[]` are internal.)

**Enums [CODE]:**

| Field | Values | Source |
|---|---|---|
| type | `task \| module \| AI \| SLAM` | `:4-7` |
| mode | `single \| cycle` for tasks; others in `:9-29` | `:9-29` |
| operate | `inquiry, save, delete, debug, run, suspend, recover, shutdown, start, stop` | `:31-40` |
| state | `null, error, wait_run, run_wait, run, suspend, shutdown, normal` | `:42-49` |

**Validation [CODE]** (`frontend_message.cpp:446-476`):
- `type`, `id` and `operate` must be non-empty strings.
- `target_id` **key must always exist** as an array of non-empty strings (`:63-113`, `:98-101`). It may be `[]` only for `inquiry` (`:449-451`).
- `describe`, `style`, `mode`, `condition` and `body` must be non-empty strings **only for `save` and `debug`**.
- The type must be one of the four above, otherwise the code is 8021 (`:481-494`).

**Task rules [CODE]** (`frontend_message.cpp:223-353`):

| operate | Rule |
|---|---|
| `save` | `describe` must be unique among the other tasks, else 8025 (`:232-297`). `mode` single or cycle, else 8028. `condition` validated, else 8029. |
| `run` | If `mode` is given, `condition` is re-validated. |
| `debug` | `target_id[0]=="debug"`, `mode=="single"`, `condition=="now"` (`:321-343`). |
| `inquiry`, `delete`, `shutdown`, `suspend`, `recover` | Only `id` and `target_id` are needed. |
| `start`, `stop`, or anything else from the frontend | 8027. start and stop are internal: the task process calls them through the `option_vp_engine` service (`abilityset.toml:33`). |

**Parse codes [CODE]** (`include/cyberdog_vp_engine/frontend_message.hpp:40-52`):

| Code | Meaning |
|---|---|
| 0 | efficient |
| 8021 | invalid |
| 8022 | type |
| 8023 | id |
| 8024 | target_id |
| 8025 | describe |
| 8026 | style |
| 8027 | operate |
| 8028 | mode |
| 8029 | condition |
| 8030 | body |

Parse feedback, on robotend only (`frontend_message.cpp:528-548`):

`{"feedback":{"type":"parse","id":..,"operate":..,"state":<code>,"describe":..}}`

### 1.2 Conditions [CODE]

- **single** (`frontend_message.cpp:140-159`):
  - `now`, or `now + <N><unit>` with **no space between N and the unit**, e.g. `now + 5minutes` (`:155`).
  - Other at-style forms must match the regex (doc: `blogs/docs/cn/cyberdog_vp_protocol_document_cn.md:180-275`).
- **cycle** (`:161-206`):
  - Exactly 5 cron fields separated by a single whitespace.
  - **Day of week 1-7 only** (`:197-202`). The doc's 0-6 is wrong for 0, and names are rejected.

### 1.3 FSM gate (`include/cyberdog_vp_engine/fsm.hpp:82-148`) [CODE]

| FSM state | Allowed task operations |
|---|---|
| Uninitialized, SetUp, TearDown | none |
| SelfCheck, OTA, Error | inquiry, suspend, shutdown, stop |
| Active, Protected (= low battery, `fsm.cpp:111-181`) | all |
| DeActive, LowPower | inquiry, save, delete, suspend, shutdown, stop |

- The default is `Uninitialized` (`bridges/params/toml_config/manager/state_machine_config.toml:5`) until the manager drives `vp_engine`, which is listed as an actuator (`:2`).
- A refusal produces feedback on robotend (`fsm.cpp:259-282`).

### 1.4 Task state flow (`include/cyberdog_vp_engine/task.hpp:50-145`) [CODE]

**Single mode:**

| From | Operation → resulting state |
|---|---|
| null | save→wait_run, debug→run_wait, delete→null |
| error | save, debug, delete, inquiry |
| wait_run | save, debug, delete, inquiry, shutdown→shutdown, run→run_wait |
| run_wait | debug, inquiry, shutdown→shutdown, (start→run, internal) |
| run | run, (start), recover, debug, inquiry, suspend→suspend, shutdown/stop→shutdown |
| suspend | suspend, debug, inquiry, recover→run, shutdown/stop→shutdown |
| shutdown | shutdown, debug, save, inquiry, delete, run→run_wait |

**Cycle mode:** identical, except that shutdown or stop from run or suspend goes to **run_wait**, so the task stays in cron. A second `shutdown` from run_wait reaches shutdown (`:116-136`).

Consequences:
- You cannot save or delete in run_wait, run or suspend. Shut down first.
- run is impossible from error and run_wait.
- A non-existent target fails (`task.cpp:464-474`).

### 1.5 Dispatch (`src/interact.cpp:531-674`) [CODE]

1. JSON validation runs first. If it fails, the engine replies with parse feedback (`:663-666`).
2. Next comes the FSM gate (`:649-662`).
3. Then the request runs:
   - **Only `delete` iterates over all target_ids.** Every other operation uses `target_id.front()` (`:629-645`).
   - `inquiry` filters by the full target_id list and by mode, skipping `id`, `terminal_default`, `visual_default`, `debug` and null-state entries (`base.cpp:488-527`).
   - The inquiry reply is `{"response":{...,"list":[...]}}` (`base.cpp:521-573`).
4. The local reply is published (`:600`) **before** the backend sync (`:601-625`).

**Backend [CLOSED].**
- `bes_http_srv` is enabled because `backend=true` (`cyberdog_vp/config/abilityset.toml:42`, `:73`).
- Calls are a POST to "programs" with 3000 ms (`interact.cpp:324-332`), a 3 s service wait (`:344`) and a 5 s reply wait (`:350`).
- For non-inquiry operations this delays the **next** queued message, not the current one.
- For `inquiry` with empty target_id, the backend inquiry runs **before** the local reply (`:560-586`). Each returned item is re-applied locally as a backend message (`:579-581`). The cloud can therefore inject or overwrite tasks.
- No server for `BesHttp.srv` exists in the open repos.

### 1.6 Build and schedule [CODE]

- **Startup prerequisites:** `at -V`, `service cron status` (the cron service must be **running**) and `pyflakes --version` (`include/cyberdog_vp_engine/interact.hpp:64-68`). If any fails, the engine calls `exit(-1)` (`interact.cpp:43-46`).
- **Paths:**
  - Workspace: `/home/mi/.cyberdog/cyberdog_vp/workspace` (`abilityset.toml:76` plus `/workspace`, `common.cpp:149-172`).
  - Sources: `workspace/task/src/<target_id>.py` and `.sh` (`task.cpp:71`, `:118`).
  - Log: `workspace/log/<target_id>.log`, **overwritten** on every run (`task.cpp:190-192`, `:549-563`).
- **Generated .py**, in this order:
  1. LICENSES and the header from `utils.py:171-198`: imports, `get_argv`, `cyberdog = Cyberdog(task_id, get_namespace(), True, task_parameters)`, `cyberdog.set_log(False)`.
  2. Auto-imports of `StateCode`, `LedConstraint` and `MotionSequence*`, only if the body text contains them (`utils.py:153-165`).
  3. `cyberdog.task.start()`.
  4. **The body verbatim at top level.**
  5. `cyberdog.task.stop()` and `cyberdog.shutdown()` (`task.cpp:164-170`).
- **Runtime:** Python 3.6 with `ROS_DOMAIN_ID=42` (`include/cyberdog_vp_engine/base.hpp:55`, `:57`). The .sh runs `python3 <id>.py cyberdog_vp_task:=<id> > <log>` (`task.cpp:183-195`).
- **pyflakes failure** runs `rm <file>.py*` only. The `.sh` stays and the state becomes error (`task.cpp:194-203`).
- **Scheduling:** single uses `at` and cycle uses `crontab` (`linux_at.cpp:70-86`, `:135-200`; `linux_cron.cpp:127-209`).
- **Cancel quirks:**
  - Cron removal matches by **substring** (`linux_cron.cpp:195`).
  - The at-cancel chain is joined with `&&`, so the process is not killed if `atrm` fails (`linux_at.cpp:189-193`).
- **Security:** target_id is used unsanitised in paths and shell commands (`task.cpp:71`, `:118`, `:190`). Use only `[A-Za-z0-9_]`.

### 1.7 Task runtime [CODE]

- `cyberdog.task.start()` waits for the engine service, 60 s by default (`abilityset.toml:62-63`; `sdk/task.cpp:129-178`).
- **The FSM re-check and suspend take effect only inside `cyberdog.task.block(id)`** (`sdk/task.cpp:362-400`, `:453-473`). Bodies without `block()` cannot be paused, so use shutdown.
- `cyberdog.ready(timeout=30)` exists (`src/api/cyberdog.cpp:26-64`).
- With `voice_operation=true`, the engine accepts ASR voice control on the dog (`interact.cpp:118-127`; `abilityset.toml:74`). `decorate_body=false` (`:75`).
- Other types (`frontend_message.cpp:355-434`): `module` supports save, delete and inquiry. `AI` and `SLAM` support inquiry only.
- `cyberdog_vp_terminal` is an on-dog tool only (`cyberdog_vp_terminal/src/sdk/debug_engine.cpp:52-104`).
- **[DOC] errors:**
  - The doc's debug example omits `describe` and `style`, so it fails with 8025 (`cyberdog_vp_protocol_document_cn.md:390-409`).
  - `:480` writes `cyberdog.stand_up()` without `.motion`.

## 2. LED

### 2.1 `led_execute` (`bridges/protocol/ros/srv/LedExecute.srv`) [CODE]

**Request** (`:77-84`): `occupation` (bool; false releases), `client`, `target`, `mode`, `effect`, `r_value`, `g_value`, `b_value` (0-255, used only in USER_DEFINED mode).

**Constants:**

| Field | Values | Source |
|---|---|---|
| client | `lowpower, bms, connector, vp, tracking, system` | `:3-8` |
| target | HEAD 1, TAIL 2, MINI 3 | `:11-13` |
| mode | SYSTEM_PREDEFINED 0x01, USER_DEFINED 0x02 | `:17`, `:20` |
| head/tail USER effects | 0x01-0x09 | `:27-35` |
| head/tail SYSTEM effects | 0xA0 off, 0xA1-0xA7 red, 0xA8-0xAE blue, 0xAF-0xB5 yellow | `:39-63` |
| MINI USER effects | 0x30, 0x31 | `:66-67` |
| MINI SYSTEM effects | 0x32 off, 0x33-0x36 | `:70-74` |

**Response codes** (`:89-95`):

| Code | Meaning |
|---|---|
| 0 | succeed |
| 1107 | timeout |
| 1121 | target |
| 1122 | client/priority |
| 1123 | mode |
| 1124 | effect |
| 1125 | LOW_PRIORITY |

### 2.2 Priority arbitration [CODE]

- Priority ids: lowpower 0, bms 1, connector 2, **vp 3**, tracking 4, system 5. A lower id wins (`bridges/params/toml_config/device/led_priority.toml:1-26`).
- A lower-priority request is stored and answered with 1125.
- When a client releases (`occupation=false`), `find_cmd` plays the next occupied lower-priority slot, stored requests included (`devices/cyberdog_led/src/led_plugin.cpp:449-535`, `:537-575`).

### 2.3 Reaching LEDs from the PC

- There is **no LED nameCode** in the proto and no `led_execute` client in the app (`protos/cyberdog_app.proto:71-150`). LEDs are reachable only from a VP task [CODE].
- **VP API** (`src/api/led.cpp:26-44`; `src/sdk/led.cpp:52-64`, `:138-240`), all with client `"vp"`:
  - `cyberdog.led.play(target, effect)` sends SYSTEM_PREDEFINED;
  - `cyberdog.led.play_rgb(target, effect, r, g, b)` sends USER_DEFINED;
  - `cyberdog.led.freed(target)` sends `occupation=false`.
- Enum: `LedConstraint.target_head|target_tail|target_mini`, `effect_line_*` (USER) and `system_effect_line_*` (SYSTEM) (`src/api/common.cpp:399+`).
- **Check `ret.response.code`, not `ret.state.code`.** The state is success whenever the service answered (`sdk/led.cpp:116-130`).
- `cyberdog.shutdown()`, which the generated footer always calls, frees all three LEDs (`sdk/cyberdog.cpp:229-260`).

## 3. Electronic skin

**gRPC 11001 ENABLE_ELEC_SKIN [CODE].**
- Request: `{"data":bool}`. Codes: 322 = no service after 3 s, 323 = missing `data`, 321 = no reply within 5 s.
- Reply: `{"success":bool,"message":str}` (`cyberdog_app.cpp:3495-3532`).
- `data=false` restores the default skin (`motion/skin_manager/src/skin_manager.cpp:89-105`).

**gRPC 11002 SET_ELEC_SKIN [CODE].**
- Request: `{"mode":int,"wave_cycle_time":int}`. Both keys are required.
- Modes (`bridges/protocol/ros/srv/ElecSkin.srv:1-6`):

  | Mode | Effect |
  |---|---|
  | 0 | black |
  | 1 | white |
  | 2 | front→back |
  | 3 | back→front |
  | 4 | blink |
  | 5 | random |
  | 6 | dynamic (by stance leg) |

- `wave_cycle_time` is passed for **all** modes, 0 and 1 included (`skin_manager.cpp:114-135`). Its unit is ms by analogy with VP `duration_ms` [INF].
- Order of checks: 322 (service) **before** 323 (params), then 321 (5 s reply timeout).
- Reply: `{"success":bool}`. It is false when not enabled, because the server returns early (`skin_manager.cpp:111-113`; `cyberdog_app.cpp:3534-3572`).

**VP [CODE].**
- `cyberdog.skin.discolored(model=0, duration_ms=1000)` calls disable, then enable, then `set_elec_skin(mode=model, wave_cycle_time=duration_ms)` (`src/sdk/skin.cpp:279-329`).
- `cyberdog.skin.electrochromic(...)` writes **raw CAN** and bypasses skin_manager (`sdk/skin.cpp:26-36`, `:80-181`).

## 4. Sensors

### 4.1 gRPC-native readback (works without the reverse channel)

**1001 GET_DEVICE_INFO** (`cyberdog_app.cpp:2677-2749`).
- Flags: `is_sn, is_version, is_uid, is_nick_name, is_volume, is_mic_state, is_voice_control, is_wifi, is_bat_info, is_motor_temper, is_audio_state, is_device_model, is_stand, is_lowpower_control` map to enables[0..13] (`:2721-2734`). The `uid` string is passed too.
- Bugs [CODE]:
  - `is_uid` is read into `is_version` (`:2705`).
  - After a 3 s `wait_for`, `future.get()` blocks until a reply arrives (`:2737-2747`).
- The reply is the manager's JSON (`manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp:157-513`):
  - The first `is_version` call may return `{}` because the version query is asynchronous (`:515-550`).
  - `is_audio_state` uses the `uid` param plus a dog-side uid (`:453-479`, `:645`; doc `blogs/docs/cn/grpc_protocol.md:469-530`).
- [REAL]: only `is_stand` and `is_bat_info` were observed on the dog. All other fields are [CODE] or [DOC].

**10001 STATUS_REQUEST** (`cyberdog_app.cpp:2502-2548`) [CODE]. It takes no params and replies:

```
{"motion_status":{"motion_id"},
 "task_status":{"task_status","task_sub_status"},
 "self_check_status":{"code","description"},
 "state_switch_status":{"state","code"},
 "charging_status":{"wired_charging","wireless_charging"}}
```

### 4.2 Via a VP task

**Topics** (`cyberdog_vp/config/abilityset.toml:10-21`):

| Topic | Notes |
|---|---|
| `imu_out` | motion_bridge publishes **`imu`** (`motion/motion_bridge/src/imu_bridge.cpp:26`), so it may never update [INF] |
| `odom_out` | published by `motion/motion_bridge/src/odom_out_publisher.cpp:34-36` at about 50 Hz (`:71`); pose from LCM leg odometry, twist in body frame (`:87-101`) |
| `ultrasonic_payload` | |
| `touch_status` | |
| `bms_status` | |
| `scan` | |
| `gps_payload` | |
| `head_tof_payload` | |
| `rear_tof_payload` | |

QoS is SensorDataQoS (`include/cyberdog_vp_abilityset/common.hpp:236-237`).

**`get_data(timeout=5)` semantics [CODE].**
- It sets the state to 7944 (no_data_update), waits up to `timeout` for the next message, and **returns the cached message even on timeout**. That can be stale data or default zeros, with no exception (`sdk/bms.cpp:69-86`; `api/bms.cpp:30-32`).
- StateCodes: 0, 7921, 7922, 7923, 7927, 7930, 7944, 7946, 7949 (`common.hpp:443-465`).

**BMS** (`bridges/protocol/ros/msg/BmsStatus.msg:1-39`; `src/api/ros_protocol.cpp:65-110`):
- Fields: `batt_volt` (**mV**), `batt_curr` (**mA**), `batt_soc` (%) [INF], `batt_temp` (**°C**).
- The units come from binding docstrings. The simulator comments say V/MA/C (`devices/cyberdog_bms/src/bms_plugin.cpp:370-392`). [REAL] check pending.
- Also exposed: `power_adapter_temp`, `wireless_charging_temp`, `batt_st`, `bms_state_one`, `bms_state_two`, `batt_health`, `batt_loop_number`, and `power_normal / wired_charging / finished_charging / motor_shutdown / soft_shutdown / wp_place / wp_charging`.
- Real data path: `bms_plugin.cpp:198-250`.

**TOF** (`bridges/protocol/ros/msg/SingleTofPayload.msg:3-5`):
- About 10 Hz, range 0.15-0.66 m.
- Python exposes only `header, data_available, tof_position (0 LF, 1 RF, 2 LR, 3 RR), data[] (m)`. **intensity is not exposed** (`ros_protocol.cpp:470-483`).
- Head: `left_head` and `right_head`. Rear: `left_rear` and `right_rear` (`:516-517`, `:565-566`).
- `data = (raw*2 + 50) * 0.001` m (`sensors/cyberdog_tof/src/tof_plugin.cpp:28`, `:363-372`).
- Obstacle detection: 0.06 m, 80 %, 1.0 s (`abilityset.toml:67-69`; `src/sdk/tof.cpp:124-170`).

**Ultrasonic:**
- range = raw×0.001 m and `field_of_view = 15.0` (float) (`sensors/cyberdog_ultrasonic/src/ultrasonic_plugin.cpp:378-381`).
- The 5 Hz rate exists only in the simulator (`:394-412`). The real rate is unknown [INF].

**GPS:**
- `lat` and `lon` are scaled ×1e-7. Other fields: `sec, nanosec, itow, fix_type, num_sv` (`ros_protocol.cpp:435-443`; `sensors/cyberdog_gps/src/bcmgps_plugin.cpp:165-182`).
- Zeros at about 10 Hz exist **only in the simulator** (`:183-205`). The real rate and no-fix behaviour are unknown [INF].

**Touch:**
- Codes: `devices/cyberdog_touch/include/cyberdog_touch/touch_sensor_handler.hpp:31-42`.
- An event stays valid for 30000 ms (`abilityset.toml:57`; `src/sdk/touch.cpp:56-84`).

**Lidar:** the `scan` topic. gRPC 6011 is reverse-channel only (`cyberdog_app.cpp:1859-1904`).

### 4.3 Getting results back to the PC [INF, built on CODE pieces]

- Engine feedback (2001) and task logs are not reachable without the reverse channel.
- **Workaround:**
  1. The task body writes a file under `/home/mi/Camera/`.
  2. The PC calls `getFile(SendRequest{namecode:4004, params:'{"file_name":"<name>"}'})` (`cyberdog_app.cpp:3938-3942`).
  3. It receives `FileChunk`s of up to 4 MB. A missing file gives `error_code` 9.
  4. **The file is deleted after sending** (`bridges/cyberdog_grpc/include/transmit_files.hpp:30`, `:111-206`, `:114`, `:147-151`, `:195`).
- Unverified [INF]: whether the at or cron task user may write in `/home/mi/Camera`.
- **Hazard [CODE]:** `file_name` is not sanitised, so `../` can read and then `remove()` arbitrary files. Never send a name containing `../`.

## 5. Minimal examples

All examples are sent with `sendMsg(nameCode=2002, params=<JSON string>)`. No reply comes back to the PC, so use §4.3 to read results.

### A. Debug LED blink: blue head light for 3 s, then release

```json
{"type":"task","id":"pc-0001","target_id":["debug"],"operate":"debug",
 "describe":"pc led test","style":"pc","mode":"single","condition":"now",
 "body":"cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 0, 0, 255)\ntime.sleep(3)\ncyberdog.led.freed(LedConstraint.target_head)"}
```

- `describe` and `style` must be non-empty.
- `LedConstraint` is auto-imported.
- A system effect works the same way: `cyberdog.led.play(LedConstraint.target_tail, LedConstraint.system_effect_line_red_blink)`.

### B. Save a single task, then cancel it

1. Save:

   ```json
   {"type":"task","id":"pc-0002","target_id":["pc_led_1"],"operate":"save",
    "describe":"pc led 1","style":"pc","mode":"single","condition":"now + 1minutes",
    "body":"cyberdog.led.play(LedConstraint.target_head, LedConstraint.system_effect_line_red_on)\ntime.sleep(5)"}
   ```

2. Cancel with `{"type":"task","id":"pc-0003","target_id":["pc_led_1"],"operate":"shutdown"}`.
3. Then delete with `operate: "delete"`.

`describe` must be unique among saved tasks.

### C. Sensor readback with the file return path [INF]

1. Debug body:

   ```
   import json
   b = cyberdog.bms.get_data(5)
   open('/home/mi/Camera/vp_bms.json','w').write(json.dumps({'soc': b.batt_soc, 'mv': b.batt_volt, 'ma': b.batt_curr, 'c': b.batt_temp}))
   ```

2. Then call `getFile` with nameCode 4004 and `{"file_name":"vp_bms.json"}`.

### D. Skin and status without VP

- 11001 `{"data":true}`.
- Then 11002 `{"mode":2,"wave_cycle_time":3000}`.
- 10001 with params `{}`.
- 1001 `{"is_bat_info":true,"is_stand":true}`.


## Unknowns that need the real dog

- Whether cyberdog_vp_engine is running and initialized on this firmware: it needs at, cron and pyflakes, and fails silently otherwise (interact.cpp:73-80). Test: VP debug LED example A.
- The engine FSM state during normal use (Active or Protected is needed for debug/run), and whether the FSM rejection TTS (is_online=true) is audible offline.
- Whether at-jobs run as a user that can write /home/mi/Camera, whether that folder exists, and whether the gRPC process can read and delete the dropped file (file-drop example C).
- The real per-message latency of 2002 operations offline, given the backend waits on a serial queue (estimate 3-8 s).
- Whether shutdown actually stops a running debug task (task_option_request vs. atrm on a running job followed by kill -9), and whether the LED stays occupied after a kill -9.
- Whether VP LED requests get 1125 LOW_PRIORITY in normal states (bms or connector holding the head LED), and whether a stored low-priority request resumes automatically.
- Whether electrochromic skin hardware is fitted, and whether 11001/11002 work end to end.
- Units of BmsStatus batt_volt, batt_curr and batt_temp, and therefore of 1001 bat_info voltage and temperature.
- Actual publish rates of bms_status, scan, ultrasonic_payload, gps_payload, imu_out and odom_out, and whether sensors publish at all when idle or in low power.
- Whether imu_out and odom_out are published on this firmware (no publisher in the open repo).
- 1001 is_version latency on the first call (OTA action), and behaviour when query_divice_info is slow (future.get() blocks).

## Closed or missing in the public sources

- bes_http_srv node and the cloud VP 'programs' API (backend=true in abilityset.toml:73 but the source is not in the repo)
- cyberdog2.client certificate and trust material for the dog-to-PC reverse channel (needed for 2001, 6011 and heartbeat)
- connector provider_ip logic that decides which PC IP the dog dials on :8981
- State machine config / MachineActuator that drives the VP engine FSM (Active/Protected/etc.)
- Publishers of imu_out and odom_out (not in the open repo)
- LED and electrochromic skin MCU firmware behind led_plugin and the skin_manager ElecSkin driver
- OTA action server used for 1001 version
- Online TTS backend behind audio.play / OnlinePlay and the FSM rejection speech

## Verifier corrections (17)

- **Claim:** The VP equivalent of gRPC 11001/11002 is cyberdog.skin.electrochromic(...), which calls the enable_elec_skin and set_elec_skin services.
  - **Correct:** Use cyberdog.skin.discolored(model=0, duration_ms=1000). It calls enable_elec_skin(false), then enable_elec_skin(true), then set_elec_skin(mode=model, wave_cycle_time=duration_ms). electrochromic(model, position, rendering, outset, duration_ms) is a separate raw-CAN path. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/skin.cpp:26-36, :80-181, :279-329; src/api/skin.cpp:29, :37-41
- **Claim:** No publisher for odom_out exists in the open repos, so VP cyberdog.odometer has no data source.
  - **Correct:** odom_out is published by motion/motion_bridge/src/odom_out_publisher.cpp at about 50 Hz. [CODE]
  - Source: motion/motion_bridge/src/odom_out_publisher.cpp:30-36, :71, :87-101
- **Claim:** TOF data[] is the raw value in mm multiplied by 0.001 (metres).
  - **Correct:** data[i] = (raw*2 + kTofOffset) * 0.001 m, with kTofOffset = 50. [CODE]
  - Source: sensors/cyberdog_tof/src/tof_plugin.cpp:28, :363-372
- **Claim:** GPS publishes zeros when there is no fix (bcmgps_plugin.cpp:199-202) at about 10 Hz (:192).
  - **Correct:** The real GPS rate and the no-fix behaviour are UNKNOWN [INF]. Only the simulator publishes zeros at about 10 Hz. lat and lon are scaled by 1e-7. [CODE]
  - Source: sensors/cyberdog_gps/src/bcmgps_plugin.cpp:165-182 (real), :183-205 (simulation)
- **Claim:** Ultrasonic publishes at 5 Hz and its field of view is the integer 15.
  - **Correct:** The real-path rate is not stated in code [INF]. range = raw*0.001 m and field_of_view = 15.0 (float). [CODE]
  - Source: sensors/cyberdog_ultrasonic/src/ultrasonic_plugin.cpp:378-381 (real), :394-412 (simulation)
- **Claim:** TOF intensity can be read from a VP task (SingleTofPayload.intensity).
  - **Correct:** intensity is not readable from Python. Use data_available, tof_position (0 = left-front, 1 = right-front, 2 = left-rear, 3 = right-rear) and data[] in metres. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_abilityset/src/api/ros_protocol.cpp:470-483
- **Claim:** BMS units are V / A / °C, or unknown.
  - **Correct:** Treat batt_volt as mV, batt_curr as mA and batt_temp as °C. This rests on [DOC-in-code] and needs a [REAL] check. batt_soc is a percentage. [INF]
  - Source: interaction/cyberdog_vp/cyberdog_vp_abilityset/src/api/ros_protocol.cpp:68-71; devices/cyberdog_bms/src/bms_plugin.cpp:370-392
- **Claim:** When a higher-priority client frees an LED, a stored lower-priority request resumes [INF].
  - **Correct:** [CODE] When a client releases a LED (occupation=false), the plugin plays the next occupied lower-priority slot.
  - Source: devices/cyberdog_led/src/led_plugin.cpp:449-535, :537-575
- **Claim:** If pyflakes fails, the engine deletes the generated task files.
  - **Correct:** The .py file is removed and the .sh file remains on disk. The task state becomes error. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_engine/src/task.cpp:194-203
- **Claim:** Tasks live in /home/mi/.cyberdog/cyberdog_vp/task/<id>.py and logs in .../log.
  - **Correct:** Workspace: /home/mi/.cyberdog/cyberdog_vp/workspace. Sources: .../workspace/task/src/<target_id>.py and .sh. Log: .../workspace/log/<target_id>.log, overwritten on each run with `>`. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp/config/abilityset.toml:76; interaction/cyberdog_vp/cyberdog_vp_engine/src/common.cpp:149-172; task.cpp:71, :118, :190-192, :549-563
- **Claim:** The single-mode condition accepts the `now + 5 minutes` form, with a space before the unit.
  - **Correct:** Valid forms are `now` and `now + <N><unit>` with no space between N and the unit, e.g. `now + 5minutes`. Other at-style times must match the regex. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_engine/src/frontend_message.cpp:140-159 (:155); blogs/docs/cn/cyberdog_vp_protocol_document_cn.md:180-275
- **Claim:** The cycle condition is a standard crontab expression with day of week 0-6, as documented.
  - **Correct:** The condition takes 5 single-space-separated cron fields, with day of week 1-7. The doc's 0-6 is wrong for 0. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_engine/src/frontend_message.cpp:161-206 (:197-202); blogs/docs/cn/cyberdog_vp_protocol_document_cn.md:180-275
- **Claim:** All 1001 GET_DEVICE_INFO fields were observed on the dog [REAL].
  - **Correct:** Tag is_stand and is_bat_info [REAL]. Tag every other field [CODE] or [DOC].
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:2677-2749; manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp:157-513
- **Claim:** The engine only needs the at and crontab binaries present.
  - **Correct:** The cron service must be running, not merely installed. at and pyflakes must be installed. Otherwise the engine exits(-1) and no 2002 message is ever handled. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_engine/include/cyberdog_vp_engine/interact.hpp:64-68; src/interact.cpp:43-46
- **Claim:** While a task runs, the engine re-checks the FSM, and suspend pauses any task.
  - **Correct:** A body that never calls cyberdog.task.block(<id>) is never re-gated by the FSM and cannot actually be paused. suspend only flips the registry state. Use shutdown (kill) instead. [CODE]
  - Source: interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/task.cpp:362-400, :453-473
- **Claim:** Backend sync with bes_http_srv delays task start or the reply by up to several seconds.
  - **Correct:** Expect at most about 3 s extra (service wait) when bes_http_srv is absent, and at most about 5 s (reply wait) when it is present. The delay falls on the next message for non-inquiry operations, and on the reply itself for an inquiry with empty target_id. The closed cloud backend can inject or overwrite local tasks through that recursion. [CODE] [CLOSED]
  - Source: interaction/cyberdog_vp/cyberdog_vp_engine/src/interact.cpp:324-381 (:344, :350), :543-558, :560-586 (:579-581), :600-625; cyberdog_vp/config/abilityset.toml:42, :73
- **Claim:** In 11002 SET_ELEC_SKIN, modes 0 and 1 ignore wave_cycle_time, and a missing parameter is reported before service availability.
  - **Correct:** Both mode and wave_cycle_time are required (323 if either is missing). 322 (no service after 3 s) is checked before 323. 321 means no reply within 5 s. The reply is {"success":bool}, and success is false when 11001 data=true was not sent first. [CODE]
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3534-3572; motion/skin_manager/src/skin_manager.cpp:107-135

## Verifier additions

- The engine never accepts start or stop from the frontend. For type task, only inquiry, save, delete, debug, run, suspend, recover and shutdown pass judge_task. start and stop return 8027; they are internal and come from the task process through the option_vp_engine service. interaction/cyberdog_vp/cyberdog_vp_engine/src/frontend_message.cpp:223-230, :288-353; cyberdog_vp/config/abilityset.toml:33
- The target_id key must always be present as a JSON array. get_strs flags a missing key even when must=false. For inquiry the array may be empty ([]), and every element must be a non-empty string. frontend_message.cpp:63-113 (:98-101), :449-451
- describe, style, mode, condition and body are mandatory, non-empty strings only for save and debug. For save, describe must be unique among the other tasks (8025). Debug requires target_id[0]=="debug", mode=="single" and condition=="now". frontend_message.cpp:232-297, :321-343, :452-476
- Parse result codes: 0 efficient, 8021 invalid json/type, 8022 type, 8023 id, 8024 target_id, 8025 describe, 8026 style, 8027 operate, 8028 mode, 8029 condition, 8030 body. interaction/cyberdog_vp/cyberdog_vp_engine/include/cyberdog_vp_engine/frontend_message.hpp:40-52
- Feedback JSON shape for a parse or FSM failure: {"feedback":{"type":"parse","id","operate","state":<code>,"describe"}}. A task result uses the same feedback shape, and inquiry returns {"response":{...,"list":[...]}}. frontend_message.cpp:528-548; fsm.cpp:259-282; base.cpp:104-127, :521-573
- Task STATE_FLOW: save and delete are impossible in run_wait, run and suspend, so shutdown first. run is impossible from error and run_wait. Inquiry is not defined from null. For a cycle task, shutdown or stop from run or suspend goes to run_wait and the task stays scheduled; a second shutdown from run_wait reaches shutdown. A single task ends in shutdown. interaction/cyberdog_vp/cyberdog_vp_engine/include/cyberdog_vp_engine/task.hpp:50-145; task.cpp:434-547 (:464-474 non-existent target fails)
- FSM gate per state. Uninitialized, SetUp and TearDown allow nothing. SelfCheck, OTA and Error allow inquiry, suspend, shutdown and stop. Active and Protected allow everything. DeActive and LowPower allow inquiry, save, delete, suspend, shutdown and stop. The default state is Uninitialized until the manager state machine drives vp_engine, which is listed as an actuator. include/cyberdog_vp_engine/fsm.hpp:82-148; bridges/params/toml_config/manager/state_machine_config.toml:2, :5; fsm.cpp:111-181
- Only delete iterates over all target_ids. Inquiry filters by the whole target_id list plus mode, skipping id, terminal_default, visual_default, debug and null-state entries. Every other operation uses target_id.front(), so the doc's multi-target suspend example only acts on the first id. interact.cpp:629-645; base.cpp:488-527
- The generated .py file is LICENSES, then a header (imports, get_argv, `cyberdog = Cyberdog(task_id, get_namespace(), True, task_parameters)`, `cyberdog.set_log(False)`), then `cyberdog.task.start()`, the body verbatim at top level, `cyberdog.task.stop()` and `cyberdog.shutdown()`. StateCode, LedConstraint and MotionSequence* are auto-imported only if the body references them. cyberdog_vp/script/utils.py:153-198; cyberdog_vp_engine/src/python_interpreter.cpp:94-124; task.cpp:164-170
- Task environment: python3.6 and ROS_DOMAIN_ID=42 are set in the .sh header, and the .sh runs `python3 <id>.py cyberdog_vp_task:=<id> > <log>`. cyberdog_vp_engine/include/cyberdog_vp_engine/base.hpp:49-69 (:55, :57); task.cpp:183-195
- VP StateCode values: 0 success, 7921 fail, 7922 uninitialized, 7923 fsm_does_not_allow, 7927 timeout, 7930 parameter_is_invalid, 7944 no_data_update, 7946 service_appear_timeout, 7949 service_request_timeout. interaction/cyberdog_vp/cyberdog_vp_abilityset/include/cyberdog_vp_abilityset/common.hpp:443-465
- VP LED API: cyberdog.led.play(target, effect) sends SYSTEM_PREDEFINED; play_rgb(target, effect, r, g, b) sends USER_DEFINED; freed(target) sends occupation=false. All three use client "vp" (priority 3). The returned .state.code is success whenever the service answered, even if response.code is not 0, so check .response.code (1125 = LOW_PRIORITY). cyberdog_vp_abilityset/src/api/led.cpp:26-44; src/sdk/led.cpp:52-64, :116-130, :138-240; src/api/ros_protocol.cpp:826-828
- cyberdog.shutdown() frees the head, tail and mini LEDs, and the generated footer always calls it. cyberdog_vp_abilityset/src/sdk/cyberdog.cpp:229-260 (:236-238); task.cpp:166-170
- The gRPC app has no LED nameCode and no led_execute client, so the PC can drive LEDs only through a VP task. bridges/cyberdog_grpc/protos/cyberdog_app.proto:71-150; bridges/cyberdog_grpc/src/cyberdog_app.cpp (no LedExecute reference)
- get_data(timeout=5) sets the state to 7944, waits up to timeout for the next message, and returns the cached message even on timeout: stale data or default zeros, with no error raised. Subscriptions use SensorDataQoS. cyberdog_vp_abilityset/src/sdk/bms.cpp:69-86; src/api/bms.cpp:30-32; include/cyberdog_vp_abilityset/common.hpp:236-237
- VP subscribes to imu_out, but motion_bridge publishes imu, so cyberdog.imu.get_data() may never update [INF]. cyberdog_vp/config/abilityset.toml:10; motion/motion_bridge/src/imu_bridge.cpp:26
- Fields exposed for BMS: batt_volt, batt_curr, batt_soc, batt_temp, power_adapter_temp, wireless_charging_temp, batt_st, bms_state_one, bms_state_two, batt_health, batt_loop_number and the power_* flags (normal, wired_charging, finished_charging, motor_shutdown, soft_shutdown, wp_place, wp_charging). cyberdog_vp_abilityset/src/api/ros_protocol.cpp:65-110; bridges/protocol/ros/msg/BmsStatus.msg:1-39
- TOF obstacle-detection parameters: 0.06 m, 80 %, 1.0 s. The touch event stays valid for 30000 ms. Service waits default to 60 s and response waits to 10 s. cyberdog_vp/config/abilityset.toml:57, :62-63, :67-69
- Engine voice control: with voice_operation=true, ASR text can operate tasks on the dog. cyberdog_vp_engine/src/interact.cpp:118-127; abilityset.toml:74
- 1001 quirks. is_uid is read into is_version, so enables[2] is never set. After the 3 s wait_for, future.get() blocks until a reply arrives. The first is_version call may return {} because GetVersion is asynchronous. Audio state uses the uid param plus a dog-side uid. bridges/cyberdog_grpc/src/cyberdog_app.cpp:2705, :2736-2747; manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp:453-479, :515-550, :645
- Cron cancel removes every crontab line that contains the id as a substring, so id "1" also hits "12". The at-cancel chain uses &&, so the process is not killed if atrm fails. cyberdog_vp_engine/src/linux_cron.cpp:168-209 (:195); linux_at.cpp:135-200 (:189-193)
- target_id is used unsanitised in file paths and shell commands, which allows path or shell injection [CODE]. Use only [A-Za-z0-9_]. cyberdog_vp_engine/src/task.cpp:71, :118, :190
- Other types: module supports save, delete and inquiry. AI and SLAM support inquiry only. frontend_message.cpp:355-434
- 4003 FILES_NOT_DOWNLOAD_COMPLETE is pushed only on the reverse channel. Heartbeat, including batt_soc, and LASER_SCAN 6011 also go only through app_stub_ (reverse channel). bridges/cyberdog_grpc/src/cyberdog_app.cpp:483-567 (:499-521), :1859-1904, :3980-3998
- Per-nameCode dispatcher queue: 10 messages, oldest dropped. bridges/cyberdog_grpc/include/msg_dispatcher.hpp:37, :59-61
- Extra VP API: audio.offline_play(audio_id) sends is_online=false, and cyberdog.ready(timeout=30) exists. cyberdog_vp_abilityset/src/api/audio.cpp:28-54; src/api/cyberdog.cpp:26-64
- cyberdog_vp_terminal (debug_engine) is an on-dog tool only and is not reachable from the PC. interaction/cyberdog_vp/cyberdog_vp_terminal/src/sdk/debug_engine.cpp:52-104
- Doc errors: the debug example omits describe and style, which are required, so it fails with 8025. The example at :480 uses cyberdog.stand_up() without .motion. blogs/docs/cn/cyberdog_vp_protocol_document_cn.md:390-409, :480
- getFile 4004 resolves /home/mi/Camera/ + file_name with no sanitising and calls remove() after sending, so a ../ name can read and then DELETE arbitrary files [CODE]. A missing file returns error_code 9. Chunks are up to 4 MB. bridges/cyberdog_grpc/include/transmit_files.hpp:30, :111-206 (:114, :147-151, :195); cyberdog_app.cpp:3938-3942