# Protocol spec: motion

Extracted from MiRoboticsLab sources and verified against the code by a second reader.

# Motion spec (corrected and verified): CyberDog 2, China firmware, control through `grpcapi.GrpcApp` :50052 (mTLS)

## 0. Conventions

**Tags**
- [CODE]: read in the NX source.
- [FW-PUB]: public locomotion firmware, commit ad79bfd (2023-09-07). The shipped MR813 build is closed and may differ.
- [DOC]: official docs only.
- [INF]: inference.
- [CLOSED]: depends on closed code.
- [CLOUD]: needs the internet.

**Path roots**
- `S` = a folder with the [MiRoboticsLab](https://github.com/MiRoboticsLab) repositories cloned side by side under their GitHub names. The bridge file is identical to `ref/cyberdog_app.cpp` in this repository apart from CRLF.
- `B` = `S/bridges/cyberdog_grpc/src/cyberdog_app.cpp`
- `MM` = `S/motion/motion_manager/src/motion_manager.cpp`
- `MMH` = `.../include/motion_manager/motion_manager.hpp`
- `MD` = `S/motion/motion_manager/src/motion_decision.cpp`
- `MDH` = `.../motion_decision.hpp`
- `MH` = `S/motion/motion_manager/src/motion_handler.cpp`
- `MHH` = `.../motion_handler.hpp`
- `MA` = `S/motion/motion_action/src/motion_action.cpp`
- `MAC` = `S/motion/motion_action/include/motion_action/motion_macros.hpp`
- `MAP` = `S/motion/motion_action/preset/motion_id_map.toml`
- `FW` = `S/cyberdog_locomotion`
- `CI` = `FW/common/src/command_interface/command_interface.cpp`
- `LG` = `FW/control/src/convex_mpc/convex_mpc_loco_gaits.cpp`
- `YAML` = `FW/common/config/cyberdog2-ctrl-user-parameters.yaml`
- `CF` = `FW/common/include/control_flags_release.hpp`
- `VP` = `S/interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/motion.cpp`
- `VPDOC` = `S/blogs/docs/cn/cyberdog_vp_protocol_document_cn.md`
- `LOCO` = `S/blogs/docs/en/cyberdog_loco_en.md`
- `IMG` = `S/blogs/docs/en/image/cyberdog_loco/motion_list_en.png`
- `GDOC` = `S/blogs/docs/cn/grpc_protocol.md`
- `JSON` = `S/utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp`
- `APP` = `app/server.py` (the v1 app in this repository)

## 1. Transport

**The RPC.** `GrpcApp.sendMsg(SendRequest{nameCode, params JSON string})` returns a stream of `RecResponse{namecode, data}`. The server is the synchronous ServerBuilder, with keepalive at 1000 ms (B:576-599). sendMsg calls ProcessMsg synchronously (`S/bridges/cyberdog_grpc/src/cyberdog_app_server.cpp:35-44`). The sync server serves concurrent RPCs on separate threads [INF: gRPC default]. [CODE]

**Motion nameCodes** (`S/bridges/cyberdog_grpc/protos/cyberdog_app.proto:73-76`). 1001 is GET_DEVICE_INFO, not motion.
- 1002 MOTION_SERVO_REQUEST
- 1003 MOTION_SERVO_RESPONSE: robot-to-app only, via `send_msgs_`/`app_stub_->sendRequest` (B:471-481, 741-751); the outbound queue size is 2 (B:147).
- 1004 MOTION_CMD_REQUEST

Related nameCodes:
- 8011 / 8012 / 8013: BLE-remote gear set / push / get (proto:135-137).
- 10001 STATUS_REQUEST (proto:140).
- 12001 (proto:149).

**Dispatch** (B:3653-3870): 1002 at 3677, 1004 at 3680, 8011 at 3835, 8013 at 3840, 10001 at 3843.
- A JSON parse error returns 323, written before `set_namecode`, so that reply has nameCode 0 (B:3665-3670).
- Bridge error replies are `{"error_code":N}` (B:978-993): 321 = service call failed or timed out, 322 = service unavailable (other handlers), 323 = JSON error.

**Field typing** (critical, [CODE]). See §8.

## 2. The 1004 path end to end (result command)

### 2.1 Request
`params` fields read by the bridge (B:2933-2943):
- int: `motion_id`, `contact`, `duration`, `value`
- float arrays: `vel_des`, `rpy_des`, `pos_des`, `acc_des`, `ctrl_point`, `foot_pose`, `step_height`

The bridge does **not** read `cmd_source`, `gait_toml` or `toml_data`, so cmd_source is always 0 (APP).

Array sizes must be exact, otherwise the whole array is zero-filled (MA:97-103; MAC:111-121):

| Array | Size |
|---|---|
| step_height | 2 |
| vel_des | 3 |
| rpy_des | 3 |
| pos_des | 3 |
| ctrl_point | 3 |
| acc_des | 6 |
| foot_pose | 6 |

GDOC:64-79 (step_height[3], foot_pose[3]) is wrong. Omitted arrays become zeros, which is fine for one-shot tricks. [CODE]

The service definition is `S/bridges/protocol/ros/srv/MotionResultCmd.srv:1-30`. Its source constants are FSM=-2, DEBUG=-1, APP=0, AUDIO=1, VIS=2, BLUTELE=3, ALGO=4. The comment "pos_des m/s" is a typo for m.

### 2.2 Bridge
`motionCMDRequestHandle` (B:2924-2966) calls `callMotionServoCmd` (B:753-780):
1. `wait_for_service` up to 3 s.
2. Waits for the future up to 20 s.
3. Either failure returns **321** (B:2945-2947).

On success it replies `{"motion_id", "result", "code"}` (B:2950-2952); a JSON encode failure returns 323 (B:2956).
- **Client deadline: 25 s or more.** Any motion that runs longer than about 20 s returns 321 while it is still executing. [CODE]

### 2.3 MotionManager gate (MM:265-299)
`IsStateValid` (MM:126-141):
- Active: OK.
- Protected (low battery): OK only for non-protected commands {111, 101, 102, 0, 212} (MM:270-275). Anything else, **including walking gaits**, returns **3013**. The voice prompt '电量低，请充电后尝试' ("battery low, please charge and try again") is played through online TTS and can delay the reply by up to 2 s (MM:278-280; MMH:93-113) [CLOUD].
- Any other state: **3003**.

ESTOP also raises the internal report code 3088 (MM:293-297), which is not returned to the client.

FSM-driven get-downs use cmd_source FSM and bypass the busy check:
- TearDown: 102, retried forever (MM:150-165).
- LowPower: 101, 3 retries (MM:201-224).
- OTA: 101 (MM:226-241).

They are skipped if the current motion is already ESTOP or GETDOWN (MMH:114-125). [CODE]

### 2.4 MotionDecision (MD:139-156)
- **Estop latch** (MDH:161-176): while latched, only 0 and 111 pass; any other id returns **3040**.
- Result commands have no priority check (`IsModeValid()` always returns true, MDH:193-196).
- `estop_ = (motion_id == 0)` runs on every result command that passes the gate (MD:153). The latch therefore clears as soon as a 111 is accepted, even if the stand later fails.
- The result service is in a **Reentrant** callback group on a MultiThreadedExecutor (MM:42-43, 53-58), so ESTOP can preempt a running 1004.

[CODE]

### 2.5 MotionHandler::HandleResultCmd (MH:634-776)
1. **Dance interruption**: needs status ExecutingResult, a current motion in dance_map {140, 176, 177, 178} (MH:986-992), and request 101 (MH:637-639). ESTOP or an interruption notifies `execute_cv_` (MH:640-644).
2. **Busy**: returns **3011** unless the request is a dance interruption, ESTOP, or from FSM (MH:645-655).
3. **IsCommandValid** (MH:93-119):
   - 400 bypasses the check.
   - An id not in the map returns **3008**.
   - `min_exec_time > 0` requires duration == 0; `< 0` requires duration > 0. A violation returns **3010**. `== 0` is not checked.
4. **CheckMotors**, except for ESTOP (MH:948-984): fatal bits {0, 2, 8} return 3032; over-heat bits {1, 24} return 3033; any other bit returns 3034.
5. **CheckPostMotion** (MH:871-889):
   - 111 and 0 always pass.
   - Current motion -1 fails.
   - Otherwise the requested **mode** (map.front()) must be in the current motion's `post_motion` list.
   - On failure the handler runs **111 automatically** and synchronously (MH:686-710). If ESTOP arrives during that 111, the handler returns without filling the response (MH:701-709).
6. **400**: uses `request->gait_toml`, which the bridge never sets, then SequenceDefImpl, failing with 3031 (MH:711-722). **400 cannot be used through 1004.**
7. **123** shows the black electronic skin, then the white one (MH:723-725, 770-772). Dance ids call `Sing(play_id)` (MH:754-758). 101 while `execute_dance_` is true calls Sing(9999); that flag is cleared after execution (MH:760-769), so the stop is racy.

[CODE]

### 2.6 ExecuteResultCmd (MH:401-622)
1. Execute (MH:428). 400 is executed twice (MH:429-433).
2. No feedback within 200 ms returns **3035** (MH:394-399, 435-441).
3. switch_status checks (MH:446-536). TRANSITIONING waits up to 3000 ms, then returns **3027** (MH:538-551).
4. **Wait budget** (MH:589-598):
   - min_exec_time > 0: min_exec_time.
   - min_exec_time < 0: duration + 500.
   - min_exec_time == 0: 500 if duration is 0, otherwise 2 x duration.
5. **Completion**: motion_id == wait_id and order_process_bar > 95 (MH:857-863). A timeout returns **3028** (MH:600-609).
6. CheckMotionResult (MH:302-384, 610-617).
7. Success: result true, code 0, motion_id = current motion (MH:618-620).

[CODE]

### 2.7 Action, LCM and firmware
`MotionAction::Execute` (MA:81-116):
- mode and gait are taken from the map;
- contact 0 becomes 15; duration and value are passed through;
- life_count is incremented on each Execute;
- the command is published on LCM `robot_control_cmd`.

`WriteLcm` republishes the last command every 50 ms (MA:474-483; MAC:82).

Firmware `Lcm2Cmd` (CI:981-1070) [FW-PUB]:
- A command with the **same life_count is ignored**, as is mode -1 (CI:983). The republish is only a keepalive or loss recovery.
- The LCM-silence fallback is commented out in FW-PUB (CI:985-997). The doc says the shipped firmware does a protective get-down after 500 ms without LCM (LOCO:117) [CLOSED].
- `duration /= 2` (500 Hz ticks, CI:1008); `cmd_source = 10` means absolute velocities (CI:1009; `command_interface.hpp:59-65`).
- Duration 0 persists until replaced (LOCO:155-159).
- A new mode-62 command is dropped while a motion trigger is active (CI:1032-1033).
- Recovery-stand gaits 17/18 insert a transition (CI:1045-1051).

Feedback: `ReadActionResponseLcm` (MA:407-459):
- mode 0 gives motion_id 0;
- otherwise the first map id (ascending) whose [mode, gait] matches;
- no match gives -1.

The status topic is published at 10 Hz (MH:70-77).

### 2.8 Result codes

| code | meaning | source |
|---|---|---|
| 0 | OK | MH:618-620 |
| 3001 | servo priority lost (servo path only) | MD:77-80 |
| 3003 | FSM not Active or Protected | MM:126-141 |
| 3008 | unknown motion_id | MH:82-119 |
| 3010 | duration rule violated | MH:93-119 |
| 3011 | busy | MH:168-177, 645-655 |
| 3013 | protected command in Protected state | MM:126-141 |
| 3021 / 3022 / 3023 / 3024 / 3025 / 3026 | switch status Estop / BanTrans / Edamp / Lifted / OverHeat / LowBat | MH:302-384 |
| 3027 | transition timeout | MH:538-551 |
| 3028 | execute timeout | MH:600-609 |
| 3031 | sequence definition error | MH:711-722 |
| 3032 / 3033 / 3034 | motor offline / over-heat / overload | MH:948-984 |
| 3035 | LCM feedback timeout | MH:435-441; MD:111-118 |
| 3040 | estop latched | MDH:161-184 |
| 3050-3056 | OriErr / FootPosErr / StandStuck / MotorOverHeat / MotorOverCurr / MotorErr / Charging | MH:302-384 |
| 321 / 322 / 323 | bridge errors | B:978-993 |

- 3029 and 3041 are never produced (MAC:40-68).
- Legacy codes 300-341 are in `S/bridges/protocol/ros/msg/MotionCode.msg:1-16`.
- switch_status values are in `MotionStatus.msg`: NORMAL 0, TRANSITIONING 1, ESTOP 2, EDAMP 3, LIFTED 4, BAN_TRANS 5, OVER_HEAT 6, LOW_BAT 7, ORI_ERR 8, FOOTPOS_ERR 9, STAND_STUCK 10, MOTOR_OVER_HEAT 11, MOTOR_OVER_CURR 12, MOTOR_ERR 13, CHARGING 14.
- `ReportErrorCode` rate-limits the same (code, motion) pair to once every 180 s (MD:200-230).

## 3. The 1002 servo stream

**Bridge** (B:2900-2922):
- Reads `motion_id`, `cmd_type`, `vel_des`, `rpy_des`, `pos_des`, `acc_des`, `ctrl_point`, `foot_pose`, `step_height` and `value`. It does not read cmd_source, contact or duration.
- Publishes on `motion_servo_cmd` (SystemDefaultsQoS, B:149-151) and replies with **empty data immediately**. There is no result on the forward channel.
- `cmd_type`: SERVO_START=0, SERVO_DATA=1, SERVO_END=2 (`MotionServoCmd.msg:1-21`). START and DATA are handled identically.

**Manager**: `MotionServoCmdCallback` (MM:250-263) applies the FSM gate with protected_cmd=false, so walking by servo works in the Protected state. The subscription is in the default mutually exclusive group (MM:50-52).

**Decision** (MD:67-84):
- Estop latched: **3040**.
- Then `IsModeValid(cmd_source)` returns **3001** if `priority[src] > priority[current]`. APP (0) loses only to keyboard-debug -1, which pose_teleop/motion_teleop use (MDH:197-200; `S/motion/motion_action/scripts/pose_teleop.py:222`; `motion_teleop.py:208-217`).
- Then SetMode.

**Handler** (MH:164-232):
1. A result command executing returns **3011** (MH:168-177).
2. SetWorkStatus(ExecutingServo).
3. For non-END frames, `if (!AllowServoCmd)`: AllowServoCmd returns true only if a pre-motion already ran and the id is not 309; otherwise it calls CheckPostMotion **on every frame** (MH:891-896). On failure the handler disables the watchdog and runs **111** synchronously, or **118** for 309 (MH:193-216).
   - Frames arriving meanwhile wait in the DDS queue (middleware default depth) [CODE/INF].
   - If the pre-motion fails, the status goes Idle and the response carries its code.
4. Execute the frame with duration 0 and contact 0 (MA:31-63), Tick, and arm the watchdog (MH:217-226).

**Watchdog** (MH:234-249; MAC:83): Tock every 50 ms. 4 consecutive misses call StopServoCmd **200-250 ms after the last frame**. Send at **20-50 Hz**:
- the VP SDK uses 50 Hz, with the comment "要求不小于20hz" ("must not be below 20 Hz") (VP:902-922);
- the docs say within 200 ms (`S/blogs/docs/en/motion_manager_en.md:35`).

**END / stop** (MH:227-229, 251-300): StopServoCmd sends WalkStand 112 with the last frame's value and step_height, without waiting.
- It is skipped if the last motion was 201, if the FSM is in TearDown or OTA, or if the pre-motion failed.
- It then sets Idle synchronously and resets the decision mode.

**Response 1003**: one per frame from ServoResponseThread, `{motion_id, order_process_bar, status, code}`. A feedback timeout gives motion_id -1 and 3035 (MD:91-121). 1003 is reverse-channel only, so it is not used here; poll 10001 instead.

**ESTOP during a stream**: later frames get 3040 and do not Tick the watchdog. About 200-250 ms later WalkStand 112 overwrites the ESTOP command [CODE]. Whether the firmware accepts it is [INF]. For a controlled stop send END, then 101.

## 4. Posture: 201 / 202 / 211 / 212

| id | map | meaning (MAP comment) | duration rule | post list |
|---|---|---|---|---|
| 201 | [3,0] | absolute force-control posture; firmware limits speed | min 0, duration 0 or more (MAP:744) | [0,7,12,11,21,3] |
| 202 | [3,3] | relative force-control posture; duration sets speed | min 0 (MAP:752) | same |
| 203 | [3,1] | registration only, do not send | | |
| 211 | [21,5] | absolute position-control posture; duration sets speed | **must be > 0** (MAP:768) | [0,12,7,21,5,64,62] |
| 212 | [21,0] | relative position-control posture | **must be > 0** (MAP:776) | same |
| 213 | [21,2] | handshake (lifted leg at low stiffness) | > 0 | same |
| 214 | [21,3] | interpolation when no task is running (VP); the dog reports it after sit tricks [INF] | > 0 | same |

**Units**: rad, m, ms. The VP SDK converts degrees to radians and seconds to milliseconds (VP:596-620). [CODE]

**Ranges**
- Doc image (IMG):
  - 201: rpy min [-0.52, -0.25, -0.65], max [0.52, 0.3, 0.65]; z default 0.235, range 0.15-0.32.
  - 211: z 0.15-0.27, duration > 0.
  - 212: "clamped by designed limitation".
  [DOC]
- pose_teleop (201 servo at about 10 Hz, cmd_source -1): z 0.15-0.32, default 0.235; roll +/-0.52; pitch -0.25..0.3; yaw +/-0.65; step 0.02 per frame (`pose_teleop.py:68, 95, 120-138, 222-226`). [CODE]
- yaml `rpy_min [-0.47, -0.25, -0.65]`, `rpy_max [0.47, 0.3, 0.65]` (YAML:127-128). [FW-PUB]
- VP constraint table (VPDOC:3044 onward):
  - centroid x relative +/-0.4 (absolute not open); y relative +/-0.3; z absolute 0.1-0.3, relative +/-0.2;
  - fulcrum (ctrl_point) +/-0.5 m;
  - roll, pitch and yaw: absolute +/-0.45 rad, relative +/-0.9;
  - duration 0-6 s.
  [DOC]
- **Pitch sign conflict**: VPDOC:3233 says "up negative", but the LOCO:472-495 example uses `rpy_des [0, 0.3, 0]` for "Head up" (duration 500) and `[0, -0.3, 0]` for "Head down". Verify on the robot.

**VP calls**: 201 at VP:622-671 (z, rpy); 202 at VP:673-726; 211 at VP:728-770 (z only); 212 at VP:772-832 (xyz, rpy, fulcrum).

**Preset**: `preset/211.toml` holds pos_des [0, 0, 0.2] and duration 1000. `PoseControlDefinitively` (211, z 0.225, duration 200) is never called (MH:270-279).

**Timing**: a one-shot 201 through 1004 has a 500 ms budget (or 2 x duration) and may return 3028 even though the pose is reached [INF]. Prefer streaming 201 by servo; END keeps the pose because WalkStand is skipped after 201. For 211/212 the budget is duration + 500.

Tricks (62/64) may follow 211-214 directly. Walking (11) or 201/202 (3) after 211-214 gets an auto-111. 212 is allowed in the Protected state.

## 5. Locomotion: walking mode, speed level, gait height

### 5.1 Gaits

| id | map | name (MAP / doc) | velocity limits [vx, vy, wz] | source |
|---|---|---|---|---|
| 303 | [11,27] | slow walk (TrotSlow) | +/-[0.6, 0.2, 1.3] | YAML:157-158; IMG |
| 308 | [11,3] | medium walk; doc "Fast Trot" | +/-[1.2, 0.4, 2.25] | YAML:155-156; IMG |
| 305 | [11,10] | fast walk; doc "Flying Trot" | [-1.2, -0.55, -2.5]..[1.6, 0.55, 2.5] | YAML:153-154 |
| 304 | [11,26] | auto-frequency trot | auto-switch ladder; envelope trot_8_3 [-1.4, -0.6, -2.5]..[1.6, 0.6, 2.5]; doc [-1.2, -0.6, -2.5]..[1.6, 0.6, 2.5] | LG:481-495, 553-562; YAML:167-168 |
| 309 | [11,28] | follow gait | auto-switch; pre-motion 118 | MH:193-216 |
| 125 | [11,4] | dog-walking (passive trot) | +/-[1.2, 0.4, 0.8]; doc +/-[0.65, 0.4, 0.8]; follows estimated velocity | LG:2515-2532 |
| 301 | [11,7] | bound (fore-aft hop) | +/-[0.3, 0.3, 1.5] | YAML:159-160 |
| 302 | [11,2] | pronk (small four-leg hops) | yaml [-0.22, -0.4, -1.5]..[0.19, 0.4, 1.5]; doc [-0.10, -0.15, -0.50]..[0.20, 0.15, 0.50] | YAML:161-162 |

Other locomotion ids (avoid):
- 306 [11,6] tripod (min -1);
- 307 [81,0] RL spin;
- 310 [11,68] ballet gait;
- 311 [82,0] RL bounce;
- 340-343 stairs, slope, TOF stairs; 344 [6,0] visual stairs; 345 [25,0] adaptive walk;
- 390-392 registration only.

Default limits are +/-[2, 1, 2.5] (YAML:151-152). Velocities are absolute because cmd_source is 10 (LG:2245-2249). They are clamped per gait with an acceleration limit (LG:2534-2535) and cross-coupled (vel_scale_limit_type 0, LG:2403-2445). [FW-PUB]

The MAP comment says a locomotion id sent through the **service** needs duration > 0 (MAP:802). Locomotion has min_exec_time 0, so this is not enforced. Walk by servo stream.

### 5.2 Speed level
There is no speed-level field for gRPC motion. A "speed level" is the client's choice of gait plus a velocity scale.

The app's 8011/8013 gear changes only the **BLE remote** (B:2412-2447; `S/devices/cyberdog_bluetooth/cyberdog_bluetooth/bluetooth_node.py:138, 648-650`). Gears are (303, 0.6, 1.0), (308, 1.0, 1.5), (305, 1.6, 2.0), read as (motion_id, vx_max, wz_max). The BLE loop sends at 20 Hz or less with cmd_source 3 and step_height 0.05, never uses vy, and sends step_height 0 on END (bluetooth_node.py:985-1018).

- 8011 replies with empty data.
- 8013 returns `{"data": n}`, or 322/321 after 3 s waits.
- 8012 is a push.

A sensible client ladder mirrors these gears [INF].

### 5.3 Step (gait) height
Only `step_height[0]` is used: `step = ratio * step_height[0]`, clamped (LG:2289-2293). [FW-PUB]

| gaits | ratio | clamp |
|---|---|---|
| 303, 308 | 1.0 | [0, 0.06] (LG:1121-1124) |
| 305 | 1.2 | [0.03, 0.072] (LG:1235-1237) |
| 302 | 1.2 | first step 0.005, then [0.015, 0.072] (LG:1322-1355) |
| 301 | 0.8 | first step 0.005, then [0.005, 0.048] (LG:1281-1320) |

- On leaving locomotion the step height becomes 0.035.
- Recommended value 0.05; doc range 0-0.06 (GDOC:24-36; LOCO:148).
- **A missing step_height means 0 lift.**

### 5.4 Body height, pitch, walking mode
**Body height**: 0.25 (YAML:102) + |vx| * delta, where delta is 0 except for 305 (0.02). It is clamped to [0.23, des + 0.07] (LG:1126, 1217, 2316-2327). pos_des[2] is ignored while walking in FW-PUB; the doc's 0.2-0.28 for 112 is [CLOSED/unverified].

**Pitch**: `rpy_des[1]` is clamped to +/-0.1 x scale (LG:1116-1117, 2299-2300). It is 0 for 112 and 125. Roll is not commandable (LG:2298). Yaw is driven only by `vel_des[2]`.

**value bits** [FW-PUB]:

| value | effect | source |
|---|---|---|
| 0 | "内八" (pigeon-toed) default stance | GDOC:24-36 |
| 2 (0x02) | "vertical": energy saving off, landing offset y = 0, body pinned to 0.23 | `fsm_state_locomotion.cpp:374-378`; LG:1189-1194 |
| 0x04 | **factory speed-offset calibration (remounts /mnt/misc); never send** | CI:1058-1064 |
| 0x08 | quick brake | LG:2331-2337 |
| bit0 | user gait only | `convex_mpc_motion_gaits.cpp:1272, 1381` |

## 6. Preset motion list

**Column meanings**
- `min` is MAP min_exec_time in ms. It is the manager's wait budget, not the real length.
- "Doc" gives the IMG valid-duration lower bounds, which approximate the motion length.
- "Stand" gives the CheckPostMotion behaviour. Where the requested mode is not in the current motion's post list, the manager inserts 111 itself.
- Post lists:
  - 111: [0,12,7,21,3,6,11,62,5,64,16,17,23,9,80,81,82]
  - 112: [0,12,7,21,3,6,11,23,80,81,82] (no 62/64/16/17)
  - 101/102: [0,12,7,80]
  - 214: includes 62/64
- **Risk classes [INF]**: R0 static; R1 in place, dynamic or two-leg; R2 travels or runs long; R3 ballistic jumps; R4 flips, RL or special.
- "LR-safe" means suitable for a living room [INF]: a flat, non-slip floor and about 1.5 m x 1.5 m of clear space.
- FW-PUB-absent ids run only on the closed firmware.

**SIT DOWN = 143. LIE DOWN = 101 (doc "Active Lie-down"); 102 is the backup (doc "Passive Lie-down", high joint damping).**

### Basic / state

| id | map | name | min | risk / notes |
|---|---|---|---|---|
| 0 | [0,0] | ESTOP (OFF) | 1000 | latches; then only 0 or 111 |
| 101 | [7,1] | lie down | 5000 | doc 0 or >3000; R0, LR-safe |
| 102 | [7,0] | lie down, backup | 5000 | R0 |
| 111 | [12,0] | recovery stand | 15000 | doc 0 or >6000; R0, LR-safe |
| 112 | [11,1] | walk-stand | 5000 | R0 |
| 113, 114, 115 | | registration only | | never send |
| 117 / 118 / 119 | [80,2/1/0] | RL stand-up fast / slow / medium | 5000 | R2 |
| 124 / 138 | [12,17] / [12,18] | lie on left / right side and recover | 20000 | R2 |
| 125 | [11,4] | dog-walking | 1000 | R2 |

### Tricks, mode 62
Stand behaviour: direct from 111, 211-214 and 214-sit; **auto-111 from 112 or lying**. After a sit (214), the client must send 111 before the next trick.

| id | name | min | doc | risk |
|---|---|---|---|---|
| 141 | shake left hand | 15000 | >8600 | R0, LR-safe |
| 142 | shake right hand | 15000 | >8600 | R0, LR-safe |
| **143** | **sit down** | 15000 | >1300 | R0, LR-safe |
| 148 | sit + swing head left | 15000 | | R0, LR-safe; send 143 first |
| 149 | sit + swing head right | 15000 | | R0, LR-safe; send 143 first |
| 150 | sit + swing head left-right | 15000 | | R0, LR-safe; send 143 first |
| 144 | hip circles | 15000 | >3900 | R0, LR-safe |
| 145 | head circles | 15000 | >3900 | R0, LR-safe |
| 146 | stretch | 15000 | >3300 | R0, LR-safe |
| 173 | squats | 15000 | | R0, LR-safe |
| 155 / 156 | lift left / right leg and nod | 15000 | | R1, LR-safe |
| 161 | march in place nodding | 15000 | | R1, LR-safe |
| 164 / 165 | alternate front / rear leg lifts | 15000 | | R1, LR-safe |
| 174 | human-like push-ups | 15000 | >14000 | R1, LR-safe |
| 157 / 158 | leg splits | 15000 | | R1 |
| 152 | moonwalk | 15000 | >9000 | R2 |
| 159 | walk and nod | 15000 | | R2 |
| 160 | splay/adduct walk | 15000 | | R2 |
| 167 / 169 | side-stretch / side-swing stepping | 15000 | | R2 |
| 170 / 171 | right / left leg fore-aft stepping | 15000 | | R2 |
| 151 | ballet | 30000 | >27000 | R2 |
| 140 | dance collection | 60000 | | R2; music 60003; 101 interrupts |
| 176 / 177 / 178 | dance 1 / 2 / 3 | 120000 | | R2; 101 interrupts; 178 absent from FW-PUB |
| 501-505 | dance 4-8 | 120000 | | R2; not interruptible by 101; absent from FW-PUB |
| 153 / 154 | jumping jacks | 15000 | | R3 |
| 162 / 163 | front / rear legs fore-aft jumps | 15000 | | R3 |
| 166 | jump collection | 15000 | | R3 |
| 168 | fore-aft swing jumps | 15000 | | R3 |
| 139 | running flip | 30000 | | R4; absent from FW-PUB |
| 172 | consecutive backflips | 15000 | | R4; absent from FW-PUB |
| 179 | front flip with clap | 15000 | | R4; absent from FW-PUB |
| 400 | user sequence | | | not usable through the bridge |

Long motions (dances 120 s, 140 60 s, 151 30 s) make the 1004 call return 321 after about 23 s while the motion continues.

### Mode 64 (two-leg)
Direct from 111 and 211-214.

| id | map | name | min | doc | risk |
|---|---|---|---|---|---|
| 123 | [64,0] | bow / greeting | 10000 | >8700 | R1, LR-safe with 1 m of clearance |
| 175 | [64,1] | bow + heart | 15000 | | R1, LR-safe with 1 m of clearance |

### Jumps (mode 16 or 23)
All R3, not LR-safe.

| id | map | name | min | doc |
|---|---|---|---|---|
| 130 | [16,0] | 3D jump, turn left 90° | 1500 | >790 |
| 131 | [16,3] | 3D jump, turn right 90° | 1500 | >920 |
| 132 | [16,1] | 3D jump, forward 60 cm | 2500 | >790 |
| 133 | [16,4] | 3D jump, forward 30 cm | 1500 | |
| 134 | [16,2] | 3D jump, left 20 cm | 1500 | |
| 135 | [16,5] | 3D jump, right 20 cm | 1500 | |
| 136 | [16,6] | 3D jump, up 30 cm | 1500 | >760 |
| 137 | [16,9] | jump down a step | 5000 | |
| 147 | [16,13] | jump down from a platform | 15000 | |
| 500 | [16,16] | jump with raised paw | 1000 | |
| 126 | [23,0] | jump up a step | 5000 | |

147, 500 and 126 are absent from FW-PUB.

### Flips and special (R4, never indoors)
- Mode 17 (absent from FW-PUB):
  - 120 flip-stand trigger;
  - 121 backflip; 122 front flip;
  - 127 / 128 right / left side flip;
  - 129 running front flip;
  - 506 / 507 platform back / front flip (1800-1900);
  - 508 / 509 skateboard back / front flip.
- The rest, all 10000 and all absent from FW-PUB: 510 / 511 [5,x] skateboard; 512 [86,0] RL upright walk; 513 [65,1] handstand split.
- The flips (mode 17) are 8000 and need battery of at least 50 % for a front flip and at least 20 % for a backflip (LOCO:330-337).

### Custom
180-199 [11,80..99] are preset custom 0-19 (15000). They likely need uploaded user gaits [INF].

VP enum names differ (148-150 are 'shake_ass_*'; 308 is fast_walking; 305 is run_fast_walking). The VP enum lacks 139, 147, 172 and 179 (`S/interaction/cyberdog_vp/cyberdog_vp_abilityset/include/cyberdog_vp_abilityset/common.hpp:347-426`).

## 7. Recommended client sequences [CODE-derived, INF where marked]

**Stand**: `1004 {"motion_id":111,"duration":0}`. Wait up to about 18 s; the client deadline is 25 s.

**Sit**: `143`. Sit with head swing: `143`, then `148`/`149`/`150`. **Leave the sit with `111`** before any other trick.

**Lie down**: `101`. **Get up**: `111`.

**Trick**: send the id with duration 0. The manager inserts 111 automatically if needed, except after a sit.

**Walk** (servo, 20-50 Hz, one dedicated RPC per frame or a non-blocking sender):
```json
{"motion_id":303,"cmd_type":1,"vel_des":[0.3,0.0,0.0],"rpy_des":[0.0,0.0,0.0],"pos_des":[0.0,0.0,0.0],"acc_des":[0.0,0.0,0.0,0.0,0.0,0.0],"ctrl_point":[0.0,0.0,0.0],"foot_pose":[0.0,0.0,0.0,0.0,0.0,0.0],"step_height":[0.05,0.05],"value":0}
```
Stop with a `cmd_type:2` frame; the manager then sends 112. Wait about 100 ms before the next 1004 [INF].

**ESTOP**: `1004 {"motion_id":0}` on its own RPC, never queued behind another call. Afterwards only 0 and 111 are accepted (anything else returns 3040). Recover with 111.

**Busy (3011)**: poll 10001 until the motion changes, or interrupt (ESTOP; 101 only for dances 140 and 176-178).

## 8. JSON typing rules [CODE]
- Float arrays: every element must be a JSON double (`0.0`, not `0`). rapidjson `IsFloat` requires kDoubleFlag (`S/cyberdog_ws/third_party/rapidjson/include/rapidjson/document.h:972-977`). The loop stops at the first int (JSON:492-511), and the size mismatch then zero-fills the whole array (MAC:111-121).
- Int fields (`motion_id`, `cmd_type`, `duration`, `value`, `contact`) must be JSON ints (JSON:513-528). `143.0` leaves the field at 0. For motion_id, 0 is ESTOP/OFF, and a servo frame with motion_id 0 is executed (CheckPostMotion(0) always passes).
- Scalar floats accept ints (JSON:583-600).

## 9. Feedback
`10001 STATUS_REQUEST` (B:~2502-2548) returns:
- `motion_status{motion_id}`
- `task_status{task_status, task_sub_status}`
- `self_check_status{code, description}`
- `state_switch_status{state, code}`
- `charging_status{wired_charging, wireless_charging}`

State switch values: -2 error, -1 unknown, 0 active, 1 low battery, 2 low power, 3 OTA, 4 shutdown (`StateSwitchStatus.msg:1-2`).

`low_power_exit` takes up to 13 s and returns `{success}` or 321/322 (B:2550-2577). Auto-low-power on/off returns `{success}` (B:2579-2609).

## 10. Known defects in the current client (APP)
- `Forward.send` uses **one shared lock** with a default timeout of 8 s (APP:135-141). A running 1004 blocks servo frames, which trips the 200-250 ms watchdog, and blocks ESTOP (APP:355). The timeout is also shorter than the bridge's 23 s.
- The servo loop (APP:251-278) makes blocking calls with timeout=2 and `cmd_source:0`, which is ignored. It sends no `value` field (0 is fine). Verify that `step_height` is present and that all floats are serialized as doubles (APP:373-375 casts with float(), and json.dumps writes `0.0`) [INF].
- The velocity limit claim (APP:29: x,y <= 1.5, yaw <= 2.0) does not match the default gait 303 (APP:66), which is limited to +/-[0.6, 0.2, 1.3].
- The reverse server expects 1003 (APP:300-301). Do not depend on it.

## 11. Closed and cloud dependencies, open questions
- [CLOSED] The shipped firmware implements modes 5/6/17/23/25/65/82/86, jump gaits 13/16, and mode-62 ids 35/36/40/113-118. Its 500 ms LCM-silence get-down and its body-height handling may differ from FW-PUB.
- [CLOUD] The 3013 voice prompt uses online TTS (MMH:93-113).
- [INF] The 214 report after sit tricks; the firmware's reaction to WalkStand after ESTOP; the DDS queue depth during an auto-stand; the pitch sign.
- Verify on the robot: run 111, 143, 111, a 303 stream, END, then 101, while polling 10001.


## Unknowns that need the real dog

- Whether the shipped motion-control firmware matches public cyberdog_locomotion ad79bfd. In particular: are the low-battery and overheat flip bans active (the public code hard-codes them false), and do mode-62 gaits 35, 36 and 40 (ids 172, 179, 139) and ids 500-513 exist? If they are missing, what code comes back?
- Whether pos_des[2] in 1002 frames or in 1004 112 changes the walking or standing body height. The doc says 0.2-0.28; the public firmware ignores it.
- The effective velocity envelope of 304 (trot 24-16 with auto-switch): about ±0.2 m/s per the yaml, or up to 1.6 m/s per the doc.
- The motion_id reported after 143 and the other mode-62 tricks: 214 (inferred from the VP SDK) or the trick id. Does the dog stay seated after 143 until the next command?
- The actual 1004 return time for 140, 151, 176-178 and 501-505. Do they reliably return 321 at 20 s while the dance continues? Can 101 interrupt them from a second concurrent RPC?
- Whether the gRPC server processes concurrent sendMsg calls: servo frames and ESTOP while a 1004 is blocked for up to 20 s.
- Real gaps on Wi-Fi: how often does the servo watchdog (~200 ms) fire with 50 Hz frames from the PC, and how quickly does walking resume after an automatic WalkStand?
- Behaviour during the automatic stand-up on the first servo frame (111 runs synchronously): are queued frames dropped or replayed, and how long until the gait motion_id shows in 10001?
- The 1004 result for locomotion ids sent with duration > 0: code 0, or 3028 because order_process_bar never passes 95.
- Whether 201 streamed at 10-20 Hz is held stably after END or a stream gap, and whether 111 restores normal standing cleanly from mode 3 and mode 21.
- The safe working ranges of 211, 212 and 202 on this unit (roll limit 0.47 or 0.52, pitch, yaw, height 0.15-0.27).
- Whether value bit1 (vertical legs) visibly changes the gait and lowers the body to 0.23 m, and whether quick-brake bit3 is honoured.
- At what battery percentage the system FSM enters Protected (state 1) and LowPower (state 2), and whether 10002 LowPower exit works from the app channel.
- What 148, 149 and 150 actually do: sit and swing the head (toml) or shake the butt (VP).
- Whether 10001 STATUS_REQUEST answers reliably at 2-5 Hz without the reverse channel, and whether its motion_id updates in time to confirm each command.
- Whether 123 and 175 (hind-leg stand) and 124 and 138 (side roll) are safe on a given floor surface.

## Closed or missing in the public sources

- The firmware actually running on the dog's motion-control board (MR813). The public cyberdog_locomotion is dated 2023-09, is stripped (the battery and overheat checks are hard-coded false) and lacks newer gait ids.
- The official Xiaomi Android app source. How it exposes posture (height, tilt), speed levels and the trick list is unknown.
- "铁蛋能力集参数约束表" (CyberDog capability-set parameter constraint table), the parameter-limit table the VP protocol doc refers to. It is not in any repo.
- The content of preset user gaits 180-199 and of the user gait or sequence files for 400 and 401. SEQUENCE_CUSTOM toml_data cannot be passed through the 1004 bridge.
- The 1003 servo response, 8012 tread updates and the heartbeat Ticks with motion_id. They travel only over the reverse gRPC channel, which needs the missing cyberdog2 client cert.
- The dance music assets (audio ids 60003-60007) played by the audio module during dances. They belong to another module or the cloud and are not verifiable here.
- The cyberdog_manager battery thresholds and state_machine_config that decide Protected and LowPower. They are not in the motion repos read.
- Error-report uploading through LcmEventUploader and the cloud. It is closed and not needed for control.
- A simulator path is not covered by this area. Every behaviour above comes from source reading, not from execution.

## Verifier corrections (21)

- **Claim:** step_height for walking: max 0.06 m for every gait (servo or 1004).
  - **Correct:** 303 and 308 (slow and medium trot): ratio 1.0, clamp [0, 0.06]. 305 (TrotFast): x1.2, clamp [0.03, 0.072]. 302 (pronk): x1.2, first step 0.005, then [0.015, 0.072]. 301 (bound): x0.8, first step 0.005, then [0.005, 0.048]. On leaving locomotion it is forced to 0.035. [FW-PUB]
  - Source: FW/control/src/convex_mpc/convex_mpc_loco_gaits.cpp:1121-1124, 1235-1237, 1281-1320, 1322-1355, 2280-2293; FW/common/config/cyberdog2-ctrl-user-parameters.yaml:213
- **Claim:** Walking pitch (rpy_des[1]) range is +/-0.2 rad.
  - **Correct:** +/-0.1 rad multiplied by rpy_cmd_scale (speed dependent). kStand (112) and passive trot (125) clamp pitch to 0. Roll is not commandable while walking; it comes from the user params. [FW-PUB]
  - Source: FW/control/src/convex_mpc/convex_mpc_loco_gaits.cpp:205-216, 1004-1010, 1116-1117, 1443-1444, 2203-2204, 2218-2219, 2298-2300
- **Claim:** Walking body height = 0.25 m plus a delta that grows with |vx|; pos_des[2] sets height.
  - **Correct:** Height = 0.25 (yaml) + clamp(|vx| * delta / vmax, 0, delta). delta = 0 for 303 and 308, 0.02 for 305. The result is clamped to [0.23, des+0.07]. When value bit1 (0x02, 'vertical') turns energy saving off, height is pinned to 0.23. pos_des[2] in a walking frame has no effect in FW-PUB. The doc's 0.2-0.28 range for 112 is unverified against closed firmware. [FW-PUB]
  - Source: FW/control/src/convex_mpc/convex_mpc_loco_gaits.cpp:1126, 1217, 2316-2327; FW/control/src/fsm_states/fsm_state_locomotion.cpp:374-378; yaml:102
- **Claim:** A pose or gait is held because motion_action republishes the LCM command at 20 Hz.
  - **Correct:** The 20 Hz republish (WriteLcm) only covers lost packets. Duration 0 means 'until replaced'. FW-PUB has no LCM-silence fallback (the code is commented out), but the doc says the shipped firmware does a protective get-down after 500 ms without LCM. Treat the republish as a required keepalive for the shipped build. [CODE + FW-PUB; shipped behaviour CLOSED]
  - Source: FW/common/src/command_interface/command_interface.cpp:983-999; S/motion/motion_action/src/motion_action.cpp:474-483; S/blogs/docs/en/cyberdog_loco_en.md:117, 142, 155-159
- **Claim:** The VP capability parameter constraint table does not exist or was not found.
  - **Correct:** Section '4.1 铁蛋能力集参数约束表' is at line 3044. centroid x relative +/-0.4 m (absolute not open); y relative +/-0.3; z absolute 0.1-0.3, relative +/-0.2; fulcrum xyz +/-0.5 m. roll, pitch and yaw: absolute +/-0.45 rad, relative +/-0.9 rad. vx +/-1.6, vy +/-1.2 m/s, wz +/-114.6 deg/s. Leg lift 0.005-0.1 (default 0.03). Duration 0-6 s (default 1). [DOC]
  - Source: S/blogs/docs/cn/cyberdog_vp_protocol_document_cn.md:3044, 3068, 3098, 3128, 3203, 3233, 3263, 3308, 3323, 3383, 3428
- **Claim:** Motion 400 (SEQUENCE_CUSTOM) is driven by the toml_data field.
  - **Correct:** 400 cannot be used through 1004. The sequence would be empty and most likely fail with 3031 (SequenceDefError). [CODE; the failure code is INF]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:711-722; S/bridges/cyberdog_grpc/src/cyberdog_app.cpp:2933-2943; S/bridges/protocol/ros/srv/MotionResultCmd.srv:1-30
- **Claim:** FW-PUB is missing only mode-62 motion ids 35, 36 and 40.
  - **Correct:** Also absent from FW-PUB:
- mode 17 (all flips: 120-122, 127-129, 506-509), with no flip FSM state;
- mode 23 (126); mode 5 (510, 511); mode 6 (344); mode 25 (345); mode 65 (513); mode 82 (311); mode 86 (512);
- jump gaits 13 and 16 (147, 500);
- mode-62 ids 113-118 (178, 501-505), as well as 35, 36 and 40 (172, 179, 139).
These live only in the closed shipped firmware. [FW-PUB + CLOSED]
  - Source: FW/common/include/control_flags_release.hpp:18-33, 157-165, 181-242; S/motion/motion_action/preset/motion_id_map.toml:100-180, 252, 316, 516, 564, 572, 980-1084
- **Claim:** The manager always stands the dog up from a sit before the next trick, so the client never needs to send 111 after 143.
  - **Correct:** After 143 or 148-150, the client must send 111 itself before any other mode-62/64 trick, as the VP SDK does. Before 148-150 it must send 143 unless the dog is already sitting. Walking (mode 11) from 214 does get the auto-111. [CODE + INF on the 214 report]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:686-710, 871-889; S/interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/motion.cpp:565-583; motion_id_map.toml:792-799
- **Claim:** Only the first non-END servo frame is checked for the pre-motion (auto 111).
  - **Correct:** The check runs on every frame until a pre-motion has actually executed. For 309 it runs on every frame, and 309 uses 118 as its pre-motion instead of 111. [CODE]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:193-226, 891-896
- **Claim:** Servo frames arriving during the automatic stand-up are silently dropped.
  - **Correct:** Frames wait in the DDS reader queue (SystemDefaultsQoS, middleware default depth; likely KEEP_LAST 1 with CycloneDDS [INF]) and are processed after the stand. The watchdog is disabled during the stand. If the stand fails, the frame's response carries the failure code and exec_servo_pre_motion_failed_ is set. [CODE]
  - Source: S/motion/motion_manager/src/motion_manager.cpp:42-52; S/motion/motion_manager/src/motion_handler.cpp:193-216
- **Claim:** The servo watchdog stops the stream after gaps of 150-250 ms.
  - **Correct:** The stop (followed by WalkStand 112) fires 200-250 ms after the last frame. Send frames every 50 ms or less (20 Hz or more; the VP SDK uses 50 Hz and its comment requires at least 20 Hz). [CODE]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:234-249; S/motion/motion_action/include/motion_action/motion_macros.hpp:83; motion.cpp:902-922
- **Claim:** Sending 101 after a dance stops the music and the dance.
  - **Correct:** 101 interrupts only 140 and 176-178 while they are running. Dances 501-505, 151 and the other long tricks cannot be interrupted except by ESTOP 0; a 101 sent during them gets 3011. [CODE]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:637-655, 754-772, 986-992
- **Claim:** The client deadline for a 1004 call should be at least 22 s.
  - **Correct:** Use a client deadline of 25 s or more. Any motion longer than about 20 s (140 60 s, 151 30 s, 139 30 s, dances 120 s) returns 321 while it is still running. [CODE]
  - Source: S/bridges/cyberdog_grpc/src/cyberdog_app.cpp:753-780, 2945-2947
- **Claim:** After a servo END frame you must wait for 112 to finish or the next 1004 gets 3011.
  - **Correct:** 3011 happens only if the 1004 is processed before the END frame (a race between the topic and the service). A short gap (about 100 ms) after END is enough. [CODE; race is INF]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:227-229, 251-300
- **Claim:** value bit0 selects the dance or MPC-trajectory variant of mode-62 tricks.
  - **Correct:** bit0 = wbc_user_mpc_traj_gait_select_ for user gaits (gait 110, ids 400/401) only. Send value 0 for tricks. [FW-PUB]
  - Source: FW/control/src/convex_mpc/convex_mpc_motion_gaits.cpp:1272, 1381; S/blogs/docs/en/cyberdog_loco_en.md:121-136
- **Claim:** The motion nameCodes are 1001-1004.
  - **Correct:** 1002 MOTION_SERVO_REQUEST, 1003 MOTION_SERVO_RESPONSE (robot to app, reverse channel only), 1004 MOTION_CMD_REQUEST. [CODE]
  - Source: S/bridges/cyberdog_grpc/protos/cyberdog_app.proto:73-76
- **Claim:** Our servo frames (cmd_source 0, APP) always win the priority check.
  - **Correct:** While pose_teleop.py or motion_teleop.py (cmd_source -1) owns the stream, APP frames get 3001. APP beats Audio, Vis, BluTele and Algo. Result commands (1004) have no priority check at all. [CODE]
  - Source: S/motion/motion_manager/include/motion_manager/motion_decision.hpp:103-112, 193-200; motion_decision.cpp:77-82; cyberdog_app.cpp:2905-2918; S/motion/motion_action/scripts/pose_teleop.py:222
- **Claim:** The doc range for 304 conflicts with yaml trot_24_16 (+/-[0.2, 0.15, 1.3]).
  - **Correct:** The effective envelope is the top of the ladder, trot_8_3 [-1.4,-0.6,-2.5]..[1.6,0.6,2.5]. This matches the doc image for 304 ([-1.2,-0.6,-2.5]..[1.6,0.6,2.5]) except vx_min. [FW-PUB + INF]
  - Source: FW/control/src/convex_mpc/convex_mpc_loco_gaits.cpp:481-495, 553-562; yaml:147, 167-168, 185-186; S/blogs/docs/en/image/cyberdog_loco/motion_list_en.png
- **Claim:** A one-shot 201 sent through 1004 reliably returns after about 500 ms.
  - **Correct:** It returns within about 0.5 s (+0.2 s feedback, plus up to 3 s if TRANSITIONING). It may return 3028 ExecuteTimeout even though the pose is reached. Prefer 201 as a servo stream (pose_teleop does this) or with a duration > 0. [CODE + INF]
  - Source: S/motion/motion_manager/src/motion_handler.cpp:538-609, 857-863; motion_id_map.toml:744-751
- **Claim:** The FSM state handlers all use 101 (citation missing for TearDown).
  - **Correct:** TearDown sends 102 (passive) and retries forever. LowPower sends 101 with 3 retries. OTA sends 101. All of them skip if the current motion is ESTOP or GETDOWN. They are sent with cmd_source FSM, which bypasses the busy check. [CODE]
  - Source: S/motion/motion_manager/src/motion_manager.cpp:150-165, 201-224, 226-241; motion_manager.hpp:114-125; motion_handler.cpp:645-655
- **Claim:** The gRPC doc's 1004 field shapes (step_height[3], foot_pose[3]) can be used as written.
  - **Correct:** Sizes must be step_height 2, vel_des 3, rpy_des 3, pos_des 3, ctrl_point 3, acc_des 6, foot_pose 6. A 3-element step_height becomes [0, 0]. [CODE; doc is wrong]
  - Source: S/motion/motion_action/src/motion_action.cpp:97-103; S/motion/motion_action/include/motion_action/motion_macros.hpp:111-121; S/blogs/docs/cn/grpc_protocol.md:64-79

## Verifier additions

- JSON typing trap. CyberdogJson::Get(vector<float>) stops at the first element that is not IsFloat(); rapidjson IsFloat requires kDoubleFlag, so integer literals such as 0 fail. The vector keeps a partial result, and GET_VALUE then zero-fills the whole array because the size is wrong. Always write 0.0. The int32 Get fails on 143.0, so motion_id stays at its default of 0 (ESTOP). Scalar float Get accepts ints. Sources: S/utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp:492-528, 583-600; S/cyberdog_ws/third_party/rapidjson/include/rapidjson/document.h:972-977; motion_macros.hpp:111-121
- A servo frame whose motion_id is not a JSON int becomes motion_id 0. The id is in the map ([0,0], mode kOff), and CheckPostMotion(0) always passes, so the frame is executed as an OFF command [CODE chain; physical effect INF]. Sources: cyberdog_app.cpp:2905; motion_handler.cpp:82-91, 871-889; motion_id_map.toml:6-11
- ESTOP during a servo stream. Later frames get 3040 and do not Tick the watchdog. About 200-250 ms later StopServoCmd sends WalkStand 112, which overwrites the ESTOP in lcm_cmd_. Whether the firmware accepts 11/1 from OFF is INF (the doc's legal transitions from 0 exclude 11). Prefer an END frame followed by 101 for a controlled stop, and keep ESTOP for real emergencies. Sources: motion_decision.cpp:67-76; motion_handler.cpp:234-300; motion_action.cpp:474-483
- Result commands can preempt each other. The motion_result_cmd service sits in a Reentrant group on a MultiThreadedExecutor, and ESTOP and the dance-interruption 101 notify execute_cv_. The client must send ESTOP on its own concurrent RPC. Sources: motion_manager.cpp:42-43, 53-58; motion_handler.cpp:640-655
- Client defect, shared lock. Forward.send in app/server.py:135-141 holds one lock for every forward call (servo, 1004, ESTOP), with a default timeout of 8 s. A 1004 such as 111 (budget up to 15 s, plus the bridge's 23 s) therefore blocks servo frames, which trips the 200-250 ms watchdog, and blocks ESTOP. The 8 s timeout is also shorter than the bridge's 23 s. Sources: app/server.py:135-141, 251-278, 355
- Client defect, velocity limits. server.py:29 limits x,y to 1.5 m/s and yaw to 2.0 rad/s for all gaits, but the default gait 303 (server.py:66) is limited to +/-[0.6, 0.2, 1.3]. The firmware clamps silently. Use per-gait limits. Sources: yaml:151-168; server.py:29, 66, 373-375
- Protected FSM state (low battery). 1004 accepts only 111, 101, 102, 0 and 212; everything else, including walking gaits, returns 3013 together with an online TTS prompt that waits up to 2 s. The servo path passes protected_cmd=false, so walking by servo still works. Sources: motion_manager.cpp:126-141, 250-263, 265-280; motion_manager.hpp:93-113
- 3013 depends on the cloud. OnlineAudioPlay sets is_online=true, so the voice prompt needs online TTS, and the 3013 reply is delayed by up to 2 s. Source: motion_manager.hpp:93-113
- The estop latch is set by `estop_ = (motion_id == ESTOP)` on every result command that passes the gate. After ESTOP only 0 and 111 pass; anything else returns 3040. The latch clears as soon as a 111 is accepted by the gate, even if the stand then fails. ESTOP's post list [0,12,7,15] means a 101 sent after ESTOP returns 3040: send 111 first. Sources: motion_decision.cpp:139-156; motion_decision.hpp:161-184; motion_id_map.toml:6-11
- After a 321 timeout (20 s future), the motion keeps running on the robot. Later 1004 calls get 3011 until it ends; poll 10001 motion_status. Sources: cyberdog_app.cpp:753-780; motion_handler.cpp:645-655
- When a dance is interrupted with 101, the dance's own 1004 returns early (execute_cv_ is notified), probably with code 0 [INF]. Source: motion_handler.cpp:637-644
- Codes 3029 and 3041 are never produced; they appear only in commented code. 3001 occurs only on the servo path. 3088 is an internal report code raised for ESTOP and is not returned in the response. Sources: motion_macros.hpp:40-68; motion_decision.cpp:77-80; motion_manager.cpp:293-297
- When a request fails to parse, the 323 reply is sent before set_namecode, so it carries nameCode 0. Source: cyberdog_app.cpp:3665-3670
- 1004 always maps both service-unavailable and future timeout to 321. Other handlers use 322 for an unavailable service. Sources: cyberdog_app.cpp:753-780, 2945-2947, 2421-2447
- The bridge does not read cmd_source, gait_toml or toml_data for 1004, and does not read cmd_source, contact or duration for 1002. 1004 contact 0 becomes 15 (all feet). Servo frames are always sent with duration 0 and contact 0. Sources: cyberdog_app.cpp:2905-2918, 2933-2943; motion_action.cpp:31-63, 94-96
- Firmware Lcm2Cmd halves the duration (500 Hz ticks), forces cmd_source 10 (absolute velocities), and drops a new mode-62 command while a motion trigger is active [FW-PUB]. Sources: FW/common/src/command_interface/command_interface.cpp:1004-1009, 1023-1043
- value bits [FW-PUB]: 0x02 turns energy saving off, which sets the landing offset y to 0 ('vertical') and pins body height to 0.23. 0x04 remounts /mnt/misc and starts the factory speed-offset calibration, so never set it. 0x08 selects quick-brake mode. Sources: fsm_state_locomotion.cpp:374-378; convex_mpc_loco_gaits.cpp:1189-1194, 2325-2337; command_interface.cpp:1058-1064; grpc_protocol.md:24-36
- Velocity cross-coupling (vel_scale_limit_type 0) reduces vy and wz authority at high vx, and acceleration is limited [FW-PUB]. Sources: convex_mpc_loco_gaits.cpp:2403-2445, 2534-2535; yaml:144
- Passive trot 125 follows the estimated velocity (dog-walking); its commanded velocity is not authoritative [FW-PUB]. Source: convex_mpc_loco_gaits.cpp:2515-2532
- The VP SDK sit compensation: 'sitting' means the old motion is one of {140,143,148,149,150} and the current motion is 214. Requests for 148-150 send 143 first; any other request except 111 sends 111 first. Before a servo command the SDK stands with 111 unless the current motion is the same gait, 111 or 112. Sources: S/interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/motion.cpp:565-583, 955-960
- Mode-21 post lists ([0,12,7,21,5,64,62]) allow tricks directly after 211-214, but not walking (11) or force posture (3); those get an auto-111. Source: motion_id_map.toml:768-799
- The NX post lists are what gate us, and they are narrower than the firmware's own legal transitions in the doc image (for example, 112 cannot go directly to 62). Sources: motion_id_map.toml:43-51; S/blogs/docs/en/image/cyberdog_loco/motion_list_en.png
- Doc naming: 102 is 'Passive Lie-down' (high joint damping) and 101 is 'Active Lie-down'. 308 is 'Fast Trot' and 305 is 'Flying Trot', while the toml calls them medium and fast walk. Source: motion_list_en.png; motion_id_map.toml:19-31, 836, 860
- Pitch sign conflict: the VP doc says pitch is 'up negative, down positive', but the loco doc example uses rpy_des [0, 0.3, 0] for 'Head up'. Verify on the robot. Sources: cyberdog_vp_protocol_document_cn.md:3233; cyberdog_loco_en.md:472-495
- The status topic motion_id is resolved as the first map id (ascending) whose [mode, gait] matches. Mode 0 gives 0, an unknown pair gives -1, and with current motion -1 CheckPostMotion fails, which forces an auto-111. Sources: motion_action.cpp:407-459; motion_handler.cpp:871-889
- Servo response 1003 is per frame (ServoResponseThread). An LCM feedback timeout produces motion_id -1 and code 3035. 1003 is pushed only robot-to-app through send_msgs_ (bridge outbound queue size 2). Sources: motion_decision.cpp:91-121; cyberdog_app.cpp:142-147, 471-481, 741-751
- The BLE remote (not our path) streams 1002-equivalent frames at 20 Hz or less with cmd_source 3 and step_height 0.05; it never uses vy; its END frame sets step_height 0. Gears are (303, 0.6, 1.0), (308, 1.0, 1.5), (305, 1.6, 2.0). 8011/8013 only change the BLE remote's gear. Sources: S/devices/cyberdog_bluetooth/cyberdog_bluetooth/bluetooth_node.py:138, 648-650, 985-1018
- Dance music is played by Sing(play_id) through AudioTextPlay (140 to 60003, 176 to 60005, 177 to 60006, 178 to 60007). 123 switches the electronic skin black and then white. Sources: motion_handler.cpp:723-772, 986-992; motion_handler.hpp:183-189
- PoseControlDefinitively (211 with z 0.225 and duration 200) is declared but never called. The 211 preset file holds pos_des [0, 0, 0.2] and duration 1000. Sources: motion_handler.cpp:270-279; S/motion/motion_action/preset/211.toml