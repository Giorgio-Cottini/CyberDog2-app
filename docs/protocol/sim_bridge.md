# Protocol spec: sim_bridge

Extracted from MiRoboticsLab sources and verified against the code by a second reader.

# Simulator bridge: app commands (1002/1004) to the cyberdog_locomotion LCM interface

Source root for every citation: a folder with the [MiRoboticsLab](https://github.com/MiRoboticsLab) repositories cloned side by side under their GitHub names. Citations are `path:line` relative to that root.

Tags:
- **[code]**: read directly in the source.
- **[doc]**: taken from `blogs/docs/cn/cyberdog_loco_cn.md`.
- **[inferred]**: my reasoning, not stated anywhere.
- **[closed]**: depends on a component that is not in the public repos, or was not read.

## 0. Where the bridge sits

On the real dog, the app talks gRPC to `cyberdog_app`:

**1002 `MOTION_SERVO_REQUEST`** [code]
- It becomes the ROS2 topic `motion_servo_cmd` (`protocol/msg/MotionServoCmd`). See `bridges/cyberdog_grpc/protos/cyberdog_app.proto:73-76`, `bridges/cyberdog_grpc/src/cyberdog_app.cpp:142-151`, `:2900-2922` and `:3677-3682`.
- The handler reads: motion_id, cmd_type, vel_des, rpy_des, pos_des, acc_des, ctrl_point, foot_pose, step_height, value.
- It does **not** set cmd_source, and answers the gRPC call immediately with empty data (`cyberdog_app.cpp:2900-2922`).
- Status comes back separately as 1003, built from the `motion_servo_response` subscription (`cyberdog_app.cpp:142-151,741-751`).

**1004 `MOTION_CMD_REQUEST`** [code]
- It becomes the ROS2 service `motion_result_cmd` (`MotionResultCmd`) (`cyberdog_app.cpp:153-154`, `:753-780`, `:2924-2966`).
- Timing: `wait_for_service` 3 s, then a **20 s** future wait.
- Errors: a failure or timeout gives gRPC error 321; a JSON encode failure gives 323.
- The handler forwards only: motion_id, vel_des, rpy_des, pos_des, acc_des, ctrl_point, foot_pose, step_height, contact, duration, value.
- It does **not** forward `gait_toml`, `toml_data` or `cmd_source`.

**Translation layer**
- `motion_manager` (MotionManager → MotionDecision → MotionHandler) and `motion_action` (MotionAction) turn these into LCM `robot_control_cmd` for the locomotion controller.

**The bridge** replaces motion_manager + motion_action with a small Python process.
- It takes the same JSON fields the gRPC handler reads and speaks LCM to the open-source controller.
- The controller runs in Gazebo as `./cyberdog_control m s` (`cyberdog_simulator/cyberdog_gazebo/launch/cyberdog_control_launch.py:34`; `cyberdog_locomotion/control/user/sub_main.cpp:19-25,43-63,82-99`) [code].
- Build and launch [doc/code]:
  - Ubuntu 20.04 + ROS 2 Galactic (`cyberdog_sim/README.md:7`).
  - `BUILD_ROS` must be ON in the cyberdog_locomotion CMakeLists (`:68`).
  - Build with `colcon build` (`:72`).
  - Launch with `launchsim.py` and the launch commands at `:78,87,100`.
  - `cyberdog_sim.repos` pins locomotion `master` and simulator `main`.

Why not run the real motion_manager [inferred]:
- It would be the most faithful option.
- But it depends on `cyberdog_system`, `manager_base`, `cyberdog_machine`, `cyberdog_debug`, `bes_transmit` and `skin_manager` (`motion/motion_manager/package.xml:12-20`, `motion/motion_action/package.xml:13-19`), which pulls in most of cyberdog_ws.
- The translator below re-implements the parts of its logic that matter.
- The numeric values of `KeyCode`/`MotionCode` (kTargetBusy, kEstop, kComLcmTimeout, ...) live in cyberdog_system and were not read [closed/not read]. The pseudo-code uses their names.

## 1. LCM interface (exact)

### 1.1 URLs and channels

| Channel | Type | URL | Direction | Rate | Source |
|---|---|---|---|---|---|
| `robot_control_cmd` | robot_control_cmd_lcmt | udpm://239.255.76.67:7671?ttl=255 | bridge -> controller | 2-500 Hz [doc]; controller timeout 0.5 s | `motion/motion_action/include/motion_action/motion_macros.hpp:87,90`; `cyberdog_locomotion/simbridge/include/simulation_bridge.hpp:29-42`; doc `cyberdog_loco_cn.md:95-97` |
| `user_gait_file` | file_send_lcmt | 7671 | bridge -> controller | on demand | `motion_macros.hpp:87,92`; `simulation_bridge.hpp:29-42` |
| `user_gait_result` | file_recv_lcmt | 7671 (the controller publishes it on the cmd URL) | controller -> bridge | one reply per file | `simulation_bridge.cpp:244-250`; `motion_action.cpp:251-333`; doc `:359` |
| `robot_control_response` | robot_control_response_lcmt | udpm://239.255.76.67:7670?ttl=255 | controller -> bridge | 50 Hz, plus extra frames (see 1.3) | `motion_macros.hpp:88`; `cyberdog_locomotion/control/src/robot_runner.cpp:22,367-393`; doc `:265-336` |
| `motor_temperature` | danger_states_lcmt | 7670 | controller -> bridge | 1 Hz (`lcm_iterations_ % 500 == 3`) | `robot_runner.cpp:395-404`; doc `:342` |
| `state_estimator` | state_estimator_lcmt | udpm://239.255.76.67:7669?ttl=255 | controller -> anyone | 50 Hz (every 10th control tick; `lcm_debug_switch: 0`) | `robot_runner.cpp:20,217-237`; `robot-defaults.yaml:40`; `motion_action.cpp:306-314` |
| `global_to_robot` | localization_lcmt | udpm://239.255.76.67:7667?ttl=255 | controller -> anyone | 50 Hz | `robot_runner.cpp:19-21,217-237`; `cyberdog_locomotion/common/include/utilities/toolkit.hpp:16-19`; doc `:372` |
| `leg_control_data` | leg_control_data_lcmt | 7667 | controller -> anyone | 50 Hz | `robot_runner.cpp:217-237` |
| `simulator_state` | simulator_lcmt | LCM default URL (**[inferred]** udpm://239.255.76.67:7667?ttl=0) | Gazebo plugin -> anyone | every 2nd Gazebo step, about 500 Hz at RTF 1 | `cyberdog_simulator/cyberdog_gazebo/src/lcmhandler.cpp:19-26,77-80`; `legged_plugin.cpp:184-219`; `world/simple.world:6-8` (step 0.001, update rate 1000) |
| `motor_ctrl` | motor_ctrl_lcmt | 7667 | **never publish** (see below) | | `simulation_bridge.hpp:29-42`; `command_interface.cpp:84-88` |
| `motion-list` | trajectory_command_lcmt | 7671 | unused by the bridge | | `simulation_bridge.hpp:29-42` |
| `exec_request` | CyberDog 1 legacy request | 7671 | do not use | | `cyberdog_locomotion/common/include/control_flags_release.hpp:10`; `simulation_bridge.hpp:29-42` |

More facts about the URLs and the sim:
- `GetLcmUrl(ttl)` always uses port 7667. `GetLcmUrlWithPort` builds the others (`toolkit.hpp:16-19,28-31`). [code]
- The Gazebo `LCMHandler` builds `lcm_` with the default constructor (`lcmhandler.cpp:19-26`). The URL is LCM_DEFAULT_URL, or LCM's built-in default. [inferred]
- Multicast route:
  - The official scripts enable multicast on an interface and add the route `224.0.0.0/4 dev <if>`: `loco_hl_example/scripts/auto_lcm_init.sh:11-12`, `cyberdog_locomotion/scripts/config_network_lcm.sh:21-35` (`-I <iface>`), doc `:85`. [code/doc]
  - On a single Linux/WSL2 host, applying the same commands to `lo` is [inferred].
- Simulated time:
  - The controller step is 2 ms (`robot-defaults.yaml:5` controller_dt 0.002).
  - In the sim it runs once every 2 Gazebo steps of 1 ms. Each control step calls ProcessGamepadCommand, Run and LCMPublishByThread (`simulation_bridge.cpp:16-77,231-234`). [code]
- Wall-clock versus sim time:
  - Controller timers use `CLOCK_MONOTONIC` (`cyberdog_locomotion/common/include/utilities/timer.hpp:27,46`), so `CMD_TIMEOUT` (0.5 s) is **wall-clock**.
  - `duration` counts **control ticks**, which are sim time.
  - With RTF < 1, motions take longer in wall time, while the 0.5 s timeout and every bridge timeout below do not scale. [code + inferred consequence]
- Gamepad and command sources:
  - Gazebo's gamepad input only refreshes `gamepad_timer_` (`command_interface.cpp:53-61`).
  - With `use_rc: 1` (`robot-defaults.yaml:15`), PrepareCmd chooses, in order: rc/udp timers (under 0.5 s), then `MotorCtrl_mode_flag_`, then `robot_control_cmd` (under 0.5 s), then `exec_request`, then the timeout fallback (`command_interface.cpp:133-171`). [code]
- **Never publish `motor_ctrl`.** One message sets `MotorCtrl_mode_flag_ = true`. Nothing in the open code clears it (the flag appears only at `:45,86,153`), so LCM commands are ignored afterwards. [code]
- **Do not run** `cyberdog_example/keybroad_commander` **or** `cyberdog_example/cyberdogmsg_sender`.
  - Both set `use_rc=0` through the `yaml_parameter` topic: `keybroad_commander.cpp:34-40`, `cyberdogmsg_sender.cpp:36`. cyberdogmsg_sender also sets `control_mode` (`:46,57,95`).
  - With `use_rc=0` only the gamepad path is used and LCM is ignored (`command_interface.cpp:139-148`). [code]
- The Gazebo sim loads `robot-defaults.yaml` and the user parameter file through `legged_simparam.cpp:46,54`, and listens on `yaml_parameter` (`:37`). [code]

### 1.2 robot_control_cmd_lcmt

Defined at `cyberdog_locomotion/common/lcm_type/lcm/robot_control_cmd_lcmt.lcm:1-15`. `bridges/protocol/lcm/robot_control_cmd_lcmt.lcm` is identical (diffed). [code]

| field | type | meaning (doc `cyberdog_loco_cn.md:101-141`, plus code) |
|---|---|---|
| mode | int8 | FSM mode (MotionMode, `control_flags_release.hpp:18-33`): 0 Off, 3 QpStand, 7 PureDamper, 9 Lifted, 11 Locomotion, 12 RecoveryStand, 15 MotorCtrl, 16 Jump3d, 21 PoseCtrl, 22 ForceJump, 62 Motion (pre-recorded list), 64 TwoLegStand, 80 RlReset, 81 RlRapid. -1 means ignore the frame (`command_interface.cpp:983`). |
| gait_id | int8 | Sub-action. In 11: GaitId (`control_flags_release.hpp:52-141`); <= 49 goes to the locomotion MPC, larger values to the motion-gait MPC (`fsm_state_locomotion.cpp:349-361`). In 16: JumpId (`:157-165`). In 62: MotionId (`:181-242`). In 21: 5 is the absolute height transform, 2 is incremental with a damped swing leg, any other value is incremental (`fsm_state_posectrl.cpp:129-181`). The label "task-space interpolation" for 21/3 is only a map comment (`motion_id_map.toml:790`). |
| contact | int8 | In 21: the low 4 bits are the support legs; 0x0F means all four, and clearing a bit lifts that leg (doc `:119-141`). In 21, 0 is treated as 0x0F (`fsm_state_posectrl.cpp:123-124`). In user-gait steps: landing_gain*10 (`interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/motion.cpp:296`). |
| life_count | int8 | Edge trigger. A frame is processed only if life_count differs from the last processed one (`command_interface.cpp:983,1003`). The controller starts at -1 (`:11`). |
| vel_des[3] | float | x, y, yaw rate. Doc ranges: y up to 1.6, yaw up to 2.5 (`:101-116`). |
| rpy_des[3] | float | In 21 (gait != 5): an offset; rpy_des/duration is added every tick (`fsm_state_posectrl.cpp:136-141`). Ignored in 21/5. |
| pos_des[3] | float | In 21 (gait != 5): an incremental CoM offset. In 21/5: pos_des[2] is the absolute body height, clamped to 0.10-0.30 (`fsm_state_posectrl.cpp:153-163`). |
| acc_des[6] | float | Used by 22 (force jump), and as user-gait weights (doc `:119-141`; `motion.cpp:310-315`). |
| ctrl_point[3] | float | Pitch control point in 21. In user gaits: [LH foot x, LH foot y, friction] (`motion.cpp:317-319`). |
| foot_pose[6] | float | Lifted-foot offset in 21. In user gaits: RF/LF/RH x,y landing offsets (`motion.cpp:321-326`). |
| step_height[2] | float | Swing height (doc text: 0-0.06 m; table: up to 0.08). User gaits pack it per leg pair: `[0] = int(RF*1e3) + int(LF*1e3)*1e3` (front), `[1] = int(RH*1e3) + int(LH*1e3)*1e3` (hind) (`motion.cpp:328-335`); the doc uses ceil() (`:678-679`). |
| value | int32 | bit0: use_mpc_traj. bit1: vertical (1) or inner-eight (0) gait. **bit2: factory speed calibration.** In mode 11, bit2 runs `system("mount -o remount,rw /mnt/misc/")` and sets a speed-offset flag (`command_interface.cpp:1058-1064`). The bridge must mask it. |
| duration | int32 | In ms. The controller divides by 2, so the unit is one 2 ms tick (`command_interface.cpp:1008`). **0 means continuous**: it clears the queue, except for mode 62 (`:1023-1030`). **>0 means queued** (`:1031-1043,1070`). |

Python `lcm-gen` packs int8 fields with a signed-byte format. life_count must therefore be wrapped to [-128, 127]; mode/gait values up to 123 fit. [inferred]

Queue and timeout semantics [code]:

1. **Expiry and fillers.** A timed command is popped from the queue when it expires. If the queue is then empty, the controller inserts a filler (`command_interface.cpp:17-43,206-250`). Every filler has duration 200 ticks (400 ms):
   - `qp_empty` 3/1 after mode 3;
   - `loco_empty` 11/1 after mode 11 with gait <= 30;
   - `loco_stand` 11/31 after mode 11 with gait > 30;
   - `pose_empty` 21/3 (contact 0x0F) otherwise.

   The fillers reverse-map to 203, 112, 113 and 214. `sit_down_trans` (21/5, pos z 0.225, 400 ticks) is also defined there.
2. **RecoveryStand 12/17 and 12/18** sent with duration 0 get a 12/0 step of 4 ticks inserted before them (`:1044-1051`).
3. **Mode 62 while a list runs.** A mode-62 command is ignored while a motion list is already running (`motion_trigger_ > 0`). Leading duration-0 entries are popped (`:1031-1043`).
4. **11/110** increments `motion_list_step` (`:1052-1056`).
5. **Process bar.** While the controller reports switch status 1, the queue iterator does not advance (`:440-441`). The process bar is computed at `:448-457`.
6. **Timeout.**
   - `CMD_TIMEOUT` is 0.5 s (`common/include/command_interface/command_interface.hpp:29`).
   - Every received frame restarts the LCM timer, **including duplicates with the same life_count** (`command_interface.cpp:80-83`).
   - When every source has timed out, the mode becomes PureDamper (7). It becomes Off instead if `command_.mode` is Off or the FSM is already Off (`:162-170`).

### 1.3 robot_control_response_lcmt

Defined at `cyberdog_locomotion/common/lcm_type/lcm/robot_control_responce_lcmt.lcm:1-10`. The filename is misspelled; the struct is `robot_control_response_lcmt`. [code]

| field | type | meaning |
|---|---|---|
| mode | int8 | `GetFsmMode()`. Returns 62 while a motion list runs (motion_trigger > 0 and the state is not Off or PureDamper). Otherwise the FSM state (`cyberdog_locomotion/control/include/control/cyberdog_controller.hpp:57-67`). |
| gait_id | int8 | `GetFsmGaitId()`. In 62: the motion id. In 11: `gait_cmd_used`, the commanded gait. Otherwise `robot_current_state->gait_id` (`cyberdog_controller.hpp:68-78`; `fsm_state_locomotion.cpp:519-520`). |
| contact | int8 | Bit i is set when the force on foot i exceeds 5 N (`robot_runner.cpp:367-393`). |
| order_process_bar | int8 | 0-100 progress (`cyberdog_controller.hpp:96-105`; `command_interface.cpp:448-457`). It is 100 for duration-0 locomotion (`fsm_state_locomotion.cpp:298-313`). In 21 it is trans_iter/duration*100 (`fsm_state_posectrl.cpp:127`). RecoveryStand reports 99, then 100 (`fsm_state_recoverystand.cpp:633-635`). |
| switch_status | int8 | `GetSwitchStatus()` (`cyberdog_controller.hpp:79-87`), in three steps. (1) If `ban_trans_flag > 0`, return it: 5 BAN_TRANS when the next state differs from the commanded mode (`control_fsm.cpp:139-146`); 6 OVER_HEAT and 7 LOW_BAT are possible in the enum but never occur in open code, because `CheckBatteryAndMotor` forces both false (`control_fsm.cpp:599-618`). (2) Otherwise return 1 TRANSITIONING if the operating mode is kNormal, the reported gait differs from the commanded gait, and the mode is not 62. (3) Otherwise return the FSM operating mode: 0 NORMAL, 1 TRANSITIONING, 2 ESTOP, 3 EDAMP, 4 LIFTED (`control_fsm.hpp:37`). The ROS-side enum extends this to 8-14 (ORI_ERR .. CHARGING) (`bridges/protocol/ros/msg/MotionStatus.msg:9-24`). The .lcm comment "4:BAN_TRANS" (`robot_control_responce_lcmt.lcm:6`) is stale. |
| ori_error | int8 | Doc `:265-336`. |
| footpos_error | int16 | Compressed from int32 flags (`robot_runner.cpp:386-387`). Bit layout at doc `:303-307`. |
| motor_error[12] | int32 | From the spine flags (`robot_runner.cpp:388-389`). Bit layout at doc `:311-336`, where bits 30-31 are the mode. motion_manager treats any of bits 0-29 as an error, bits {0,2,8} as fatal (kHwMotorOffline), {1,24} as over-heat, and anything else as overload (`motion_handler.cpp:948-984`). Normal hardware value: `kMotorNormal = INT32_MIN` (`motion_macros.hpp:86`). In the sim the simulated spine flags are presumably 0 [inferred]. |

The response is published [code] (`robot_runner.cpp:390-391`):
- when switch_status changes;
- every 10 iterations (50 Hz);
- on **every tick** while `order_process_bar >= 95` and the previous bar was not 100.

### 1.4 Other types

**User-gait files** [code]
- `file_send_lcmt { string data; }` and `file_recv_lcmt { int8 result; }`, where 0 is OK and 1 is an error (`common/lcm_type/lcm/user_gait_file_t.lcm:1-6`).
- The controller accepts data only if it starts with `# Gait Def`, stored as `user_gait_file_`, or `# Gait Params`, stored as `user_gait_list_file_`. Anything else returns 1 (`command_interface.cpp:90-101`). Both strings start out empty (`:48-49`).
- **Crash hazard** [code + inferred consequence]. Two calls run `substr(length()-N)` on these strings with no guard. On an empty string, `std::out_of_range` is thrown, and `SimulationBridge::Run` rethrows it (`simulation_bridge.cpp:71-76`), so the controller process is expected to die:
  - `MotionListFullCppTomlRead` (`:1186`, outside its try block at `:1201`), on `[62,110]`;
  - `NonPeriodicGait::InitGait` (`control/src/convex_mpc/gait.cpp:379`), on the first `[11,110]` (`fsm_state_locomotion.cpp:459-463`; `convex_mpc_motion_gaits.cpp:255-262`).

**State types** [code]
- `state_estimator_lcmt`: p, vWorld, vBody, p_abs, vWorld_abs, vBody_abs, vRemoter, rpy, omegaBody, omegaWorld, quat[4], aBody, aWorld, contactEstimate[4] (float), and int64 timestamp (`state_estimator_lcmt.lcm:1-17`).
- `localization_lcmt`: xyz, vxyz, rpy, omegaBody, vBody, timestamp (`localization_lcmt.lcm:1-8`).
- `leg_control_data_lcmt`: q, qd, p, v, tau_est, force_est and force_desired, each [12], plus joint limit arrays (`leg_control_data_lcmt.lcm:1-12`).
- `simulator_lcmt` (all double except timesteps): vb[3], rpy[3], int64 timesteps, time, quat[4], R[9], omegab[3], omega[3], p[3], v[3], vbd[3], and q/qd/qdd/tau/tauAct/f_foot/p_foot[12] (`common/lcm_type/lcm/simulator_lcmt.lcm:1-22`). The Gazebo header has the same order (`cyberdog_simulator/cyberdog_gazebo/include/simulator_lcmt.hpp:17-51`).

Generate the Python types with `lcm-gen -p` from these exact .lcm files, so the fingerprints match. [inferred]

## 2. Translation the real dog performs (motion_manager + motion_action)

### 2.1 The motion_id map

`share/motion_action/preset/motion_id_map.toml` is parsed at startup (`motion_action.cpp:174-202`). Each `[[motion_ids]]` entry has:
- `motion_id`;
- `map = [mode, gait_id]`, read as mode = front and gait = back;
- `pre_motion`;
- `post_motion` (the modes allowed to follow it);
- `min_exec_time` (ms).

The public copy is `motion/motion_action/preset/motion_id_map.toml` (1087 lines, 134 entries). No two entries share a [mode, gait] pair (checked).

Selected entries [code]:

| id | entry |
|---|---|
| 0 | [0,0], min 1000, post [0,12,7,15] |
| 101 | [7,1], min 5000, post [0,12,7,80] (`:19`) |
| 102 | [7,0], min 5000 |
| 111 | [12,0], min 15000, post [0,12,7,21,3,6,11,62,5,64,16,17,23,9,80,81,82] (`:35-39`) |
| 112 | [11,1], min 5000 (`:43`) |
| 113 / 114 | [11,31] min 5000 / [9,0] min 0. Registration only, not sent (`:49-50`, `:58`) |
| 117-119 | [80,2]/[80,1]/[80,0], min 5000 (RlReset) |
| 123 | [64,0], min 10000 (`:124`) |
| 124 / 138 | [12,17] / [12,18], min 20000 |
| 139 | [62,40], min 30000 |
| 140 | [62,0], min 60000 (`:260`) |
| 142 | [62,2], min 15000 (`:276`) |
| 151 | [62,11], min 30000 |
| 176-178 | [62,111/112/113], min 120000 |
| 201 / 202 / 203 | [3,0] / [3,3] min 0 / [3,1] min -1 (203: "not open, registration only") |
| 211 | [21,5], min -1, post [0,12,7,21,5,64,62] (**no 11**) (`:768`) |
| 212 | [21,0], min -1, same post (`:776`) |
| 213 / 214 | [21,2] / [21,3], min -1 |
| 303 | [11,27], min 0 (`:820`) |
| 304 | [11,26], min 0 |
| 307 | [81,0], min 0 (RlRapid) |
| 309 | [11,28], min 0 (`:868`) |
| 400 | [62,110], min 15000 (`:964`) |
| 401 | [11,110], min 0 |
| 501-505 | [62,114-118], min 120000 |

The `preset/<id>.toml` files are used only by tests (`motion/motion_action/test/publisher_test.cpp:42`, `client_test.cpp:61-62`, `manager_test.cpp:139,176`). [code]

### 2.2 1004 (MotionResultCmd), shown in code

0. **Decision layer** (`motion_decision.cpp:140-155`).
   - ESTOP latch: while the latch is set, only 0 and 111 pass; anything else gets kEstop (`motion_decision.hpp:161-176`).
   - Then the cmd_source priority check (`motion_decision.hpp:197-200`).
   - Then `estop_ = (motion_id == 0)`, so any non-0 command that passes, such as 111, clears the latch.
   - Manager-state check (`motion_manager.cpp:126-141,265-286`): commands are accepted in kActive. In kProtected only 111, 101, 102, 0 and 212 pass. [closed: the manager state comes from cyberdog_machine]
1. **Interrupt and busy** (`motion_handler.cpp:637-655`).
   - An ESTOP, or a 101 GETDOWN while a dance (140/176/177/178) runs, calls `execute_cv_.notify_all()` (`:640-644`).
   - Otherwise, if the handler is not idle, the command is rejected with kTargetBusy. This includes the case where a servo stream is active. Exceptions: ESTOP, the dance interruption, and cmd_source FSM (-2).
2. **Validity** (`motion_handler.cpp:93-119`).
   - 400 (SEQUENCE_CUSTOM) bypasses all checks.
   - Otherwise the id must be in the map, or the code is kUnSupport.
   - `min_exec_time > 0` requires `duration == 0`.
   - `min_exec_time < 0` requires `duration > 0`.
   - `min_exec_time == 0` accepts any duration.
   - A violation gives kParametersInvalid.
   - So 111, 101 and 123 need duration 0, and 211/212 need duration > 0.
3. **Motor check** on response motor_error bits 0-29, skipped for ESTOP (`motion_handler.cpp:666-674,948-984`).
4. **Pre-motion** (`motion_handler.cpp:686-710`).
   - If `CheckPostMotion(id)` fails, a full 111 RecoveryStand runs first. If that fails, or an ESTOP arrives meanwhile, the request fails.
   - `CheckPostMotion` (`:871-889`): true for 111 and 0; false if the current motion_id is -1; otherwise checks whether the target's **mode** is in the current motion's `post_motion`.
5. **Publish** (`motion_action.cpp:81-116`).
   - Mode and gait come from the map.
   - `contact = req.contact == 0 ? 15 : req.contact` (`:94`).
   - `duration = req.duration` (`:96`).
   - value and arrays are copied. GET_VALUE zero-fills an array of the wrong length (`motion_macros.hpp:111-121`).
   - `life_count = life_count_++` (`:106`); it is an `int8_t` and wraps (`motion_action.hpp:193`).
   - Publish (`:107`).
6. **Second frame for 400.** For 400 a second request carrying only motion_id 400 is executed, so [62,110] goes out with contact 15 and duration 0 (`motion_handler.cpp:429-433`). On the MotionSequenceShow path, the first Execute only uploads pace_toml and publishes nothing (`motion_action.cpp:119-132`).
7. **Wait for any response**, up to 200 ms, else kComLcmTimeout (`motion_handler.cpp:394-399`; `motion_macros.hpp:85`). Then sleep until 15 ms after the publish.
8. **switch_status check** (`motion_handler.cpp:446-555`).
   - Statuses 2-14 fail immediately (the same set as CheckMotionResult, `:302-384`).
   - 1 TRANSITIONING waits up to 3000 ms for (motion_id == target and NORMAL), else kMotionTransitionTimeout (`motion_macros.hpp:84`).
9. **Completion wait** (`motion_handler.cpp:589-609`).
   - timeout = min_exec_time if > 0;
   - duration + 500 if < 0;
   - otherwise (duration == 0 ? 500 : 2*duration);
   - on timeout: kMotionExecuteTimeout.
   - The condition variable is woken when the reverse-mapped response motion_id == target **and** order_process_bar > 95 (`:832-864`).
   - It is also woken by the ESTOP/dance notify in step 1. The wait has **no predicate**, so an interrupted motion proceeds straight to step 10.
   - Reverse map (`motion_action.cpp:407-459`): mode 0 maps to ESTOP (0). Otherwise the first map entry matching (mode, gait_id), in std::map ascending order (`motion_action.hpp:182`). Otherwise -1.
10. **Final check** of switch_status and motors (`motion_handler.cpp:610-617`).
    - Returns {motion_id, result, code}.
    - The motion_id returned is the **current** reverse-mapped id, not the requested one (`:618-620`).
11. **App-visible cap.** If steps 4-10 take longer than 20 s, the app gets gRPC 321 while the handler keeps running and stays busy (`cyberdog_app.cpp:753-780,2924-2966`).
    - Ids above the cap on their own: 124/138 (20 s), 139/151 (30 s), 140 (60 s), 176-178 and 501-505 (120 s).
    - Any id that needs the 111 pre-motion adds up to 15 s.
12. **Servo-type ids through 1004.** A min-0 id sent with duration 0 (for example 303 [11,27] with a vel_des) succeeds within 500 ms, because the locomotion bar is 100. The command stays active under the heartbeat, so **the dog keeps walking** until the next command. [code + inferred consequence]

### 2.3 1002 (MotionServoCmd stream), shown in code

**Frame**
- Fields: motion_id, cmd_type (0 START, 1 DATA, 2 END), cmd_source (APP 0, AUDIO 1, VIS 2), value, and arrays.
- There is **no duration and no contact** (`bridges/protocol/ros/msg/MotionServoCmd.msg`).
- The gRPC handler does not set cmd_source (`cyberdog_app.cpp:2900-2922`).

**Admission**
- Decision layer: the ESTOP latch applies (only 0 and 111 pass), then priority (`motion_decision.cpp:67-84`).
- Handler (`motion_handler.cpp:164-232`):
  - rejected with kTargetBusy while a result command executes (`:168-177`);
  - motion_id must be in the map; there is no duration check (`:83-91`);
  - then the status becomes kExecutingServoCmd.

**START or DATA** (`motion_handler.cpp:189-226`)
1. If `AllowServoCmd(id)` is false, a full result command runs first: 111, or 118 = [80,1] RlReset for id 309 (`:193-216`).
   - `AllowServoCmd` is true when `post_motion_checked_` is set and id != 309; otherwise it falls back to `CheckPostMotion` (`:891-896`).
   - If the pre-motion fails: the handler goes idle, sets `exec_servo_pre_motion_failed_ = true`, and returns the pre-motion's code. On success it sets `post_motion_checked_ = true`.
2. Then `exec_servo_pre_motion_failed_ = false` and `last_servo_cmd_ = msg` (`:217-218`).
3. `Execute(servo)` (`motion_action.cpp:31-63`):
   - mode and gait from the map;
   - **contact = 0** (`:44`), **duration = 0** (`:46`);
   - value and arrays copied;
   - `life_count++` on **every frame** (`:56`);
   - publish (`:57`).
4. Then `TickServoCmd` and `SetServoNeedCheck(true)`.

**END**
- END calls StopServoCmd (`motion_handler.cpp:227-229`).

**Watchdog** (`motion_handler.cpp:234-249`; `ServoClick` at `motion_macros.hpp:146-164`) [code]
- Every 50 ms, Tock returns whether a frame arrived since the last check, and clears the flag.
- The counter resets on a tick. 4 consecutive misses (`motion_macros.hpp:83`) call StopServoCmd.
- The tolerated gap is therefore about 150-200 ms.

**StopServoCmd** (`motion_handler.cpp:251-268`)
- Runs WalkStand if `last_servo_cmd_` is set, its id is not 201 (FORCECONTROL_DEFINITIVELY), and the manager FSM is not TearDown or OTA.
- Then goes idle and sets `post_motion_checked_ = false`.
- `last_servo_cmd_` is **never reset** (`motion_handler.hpp:214`). Any later END frame republishes 112 from that stale frame, even with no active stream.
- WalkStand (`:281-300`) is fire-and-forget:
  - it does nothing if the servo pre-motion failed;
  - otherwise it publishes 112 = [11,1] with the last frame's `value` and `step_height`, contact 15 (0 replaced by 15) and duration 0;
  - there is no completion wait.

**Response to the app**
- 1003 `MOTION_SERVO_RESPONSE` {motion_id, order_process_bar, status, result, code} (`cyberdog_app.cpp:741-751`).
- It is produced by `MotionDecision::ServoResponseThread` (`motion_decision.cpp:91-121`; queue at `motion_decision.hpp:237-260`):
  - one response per frame, after the next LCM feedback (up to 200 ms);
  - fields: motion_id (reverse-mapped), bar, status = switch_status, code from CheckMotionResult, result from the handler;
  - on feedback timeout: motion_id -1, bar -1, status -1, result false, kComLcmTimeout.

**Posture through 1002**
- 211/212 are in the map, so frames are accepted.
- But duration 0 makes PoseCtrl use 1800000 ticks (`fsm_state_posectrl.cpp:125-126`). Each tick moves about 1/1800000 of the target, so the effect is negligible.
- Use 1004 with duration > 0 for postures. [code + inferred consequence]

### 2.4 Heartbeat, shown in code

- `WriteLcm` republishes the **last frame unchanged, with the same life_count**, every `1000/20 = 50 ms`, once any command has been sent (`motion_action.cpp:251-333,474-483`; `motion_macros.hpp:82`).
- The controller treats it as a duplicate (`command_interface.cpp:983`), but it refreshes the 0.5 s timeout (`:80-83`).
- Incrementing life_count on the heartbeat would be wrong. Every new life_count is processed again (`:1023-1043,1070`), so timed motions would repeat. [code + inferred consequence]
- The doc examples resend about every 105 ms (`cyberdog_loco_cn.md:518-526`), or every 0.2 s in sequences (`:548-619`). The doc's user-gait example increments life_count in its heartbeat (`:726-727`); follow motion_action instead.
- motion_action logs "Cannot read LCM from MR813" when no response arrives within 200 ms (`motion_action.cpp:251-333`). [code]

### 2.5 Sequences and user gaits, shown in code

**Visual-programming path** (`MotionSequenceShow`; `motion/motion_manager/src/motion_manager.cpp:340-357` goes through DecideResultCmd to HandleResultCmd with id 400):
1. `SequenceDefImpl(gait_toml)` publishes file_send_lcmt on `user_gait_file`, waits up to 2200 ms for `user_gait_result`, then sleeps 1 ms. Success means result == 0; failure gives kSequenceDefError (`motion_handler.cpp:711-722`; `motion_action.cpp:485-504`).
   - gait_toml starts with `# Gait Def`.
   - It holds `[[section]] contact=[RF,LF,RH,LH] duration=n`, where n is in units of 30 ms (`cyberdog_vp_abilityset/src/sdk/motion.cpp:198-240`).
2. ExecuteResultCmd → `Execute(MotionSequenceShow)` uploads pace_toml the same way. It publishes **no** robot_control_cmd, and a failure is only logged (`motion_action.cpp:119-132`).
   - pace_toml starts with `# Gait Params`.
   - It holds `[[step]]` entries with mode 11, gait 110, life_count 0 and all robot_control_cmd fields, encoded as in 1.2 (`motion.cpp:241-346`).
   - The request duration is the sum of the step durations (`motion.cpp:192-197,341`).
3. The second, motion_id-only 400 request publishes **one** [62,110] frame with contact 15 and duration 0 (`motion_handler.cpp:429-433`).
4. In the controller, 62/110 loads `user_gait_list_file_` through `MotionListFullCppTomlRead` (`command_interface.cpp:407-409,1177-1241`), which parses `[[step]]` with duration/2. Each [11,110] step uses the `# Gait Def` gait.
5. Completion uses min_exec_time 15000 ms. A sequence longer than 15 s reports kMotionExecuteTimeout even though it keeps running.

**Through 1004, sequence 400 is unusable.** The gRPC handler never copies gait_toml (`cyberdog_app.cpp:2924-2966`). `SequenceDefImpl("")` then gets result 1 (`command_interface.cpp:90-101`), which gives kSequenceDefError. The double [62,110] publish of the MotionResultSrv path is therefore unreachable. [code + inferred consequence]

**Manual alternative from the doc** (`cyberdog_loco_cn.md:625-741`; `loco_hl_example/customized_gait/Usergait_List.toml`):
1. Send the `# Gait Def` string and sleep 0.5 s.
2. Send the `# Gait Params` string and sleep 0.1 s.
3. Send robot_control_cmd [12,0] duration 5000, then [62,110] duration 4320, then [7,0] duration 4000, keeping the heartbeat.

**Crash guard.** Never send [62,110] before an accepted `# Gait Params` upload, and never send [11,110] (401) before an accepted `# Gait Def` upload (see 1.4).

**Other mode-62 ids** load preinstalled TOMLs by gait_id (`command_interface.cpp:251-435`):
- these have their own case: 0-6, 8-34, 70, 80-91, 96, 110 (user list), 111 and 112 (dance sets);
- **any other id, including 35, 36, 40 and 113-118, falls back to `cyberdog2_motion.toml`** (`:416-417`);
- the files are under `cyberdog_locomotion/control/motion_list/cyberdog2/preinstalled/` and `.../user/`.

### 2.6 Python translator (pseudo-code)

```python
# sim_bridge.py: pseudo-code. Needs python3-lcm plus `lcm-gen -p` output of
# cyberdog_locomotion/common/lcm_type/lcm/{robot_control_cmd_lcmt,robot_control_responce_lcmt,
#   user_gait_file_t,state_estimator_lcmt,simulator_lcmt}.lcm
import lcm, threading, time, tomllib, random
URL_CMD  = "udpm://239.255.76.67:7671?ttl=255"   # robot_control_cmd, user_gait_file, user_gait_result (motion_macros.hpp:87; simulation_bridge.hpp:29-42)
URL_RESP = "udpm://239.255.76.67:7670?ttl=255"   # robot_control_response, motor_temperature (motion_macros.hpp:88)
URL_EST  = "udpm://239.255.76.67:7669?ttl=255"   # state_estimator (robot_runner.cpp:20)
URL_SIM  = "udpm://239.255.76.67:7667?ttl=0"     # simulator_state [inferred default URL]; leg_control_data, global_to_robot
RTF = 1.0                                         # measured Gazebo real-time factor [inferred: scale waits by 1/RTF]
HEARTBEAT, FEEDBACK, TRANSIT, SEQ_ACK = 0.050, 0.200, 3.0, 2.2   # motion_macros.hpp:82,85,84; motion_action.cpp:485-504
SERVO_TICK, SERVO_MISS = 0.050, 4                                  # motion_handler.cpp:234-249; motion_macros.hpp:83
GRPC_CAP = 20.0                                                    # cyberdog_app.cpp:753-780 (app sees error 321)
FAIL_SWITCH = {2,3,4,5,6,7,8,9,10,11,12,13,14}                    # motion_handler.cpp:302-384,446-555 (1 = TRANSITIONING waits)
ARRAYS = (("vel_des",3),("rpy_des",3),("pos_des",3),("acc_des",6),("ctrl_point",3),("foot_pose",6),("step_height",2))
i8 = lambda n: ((n + 128) % 256) - 128                             # int8_t life_count_ wraps (motion_action.hpp:193)

class Bridge:
    def __init__(self, map_toml):
        e = tomllib.load(open(map_toml, "rb"))["motion_ids"]
        self.map = {x["motion_id"]: x for x in e}
        self.rev = {}
        for mid in sorted(self.map):                               # std::map order, first match wins
            self.rev.setdefault((self.map[mid]["map"][0], self.map[mid]["map"][-1]), mid)
        self.cmd = lcm.LCM(URL_CMD); self.resp = lcm.LCM(URL_RESP)
        self.cmd.subscribe("user_gait_result", self._on_file_ack)
        self.resp.subscribe("robot_control_response", self._on_resp)
        self.life = random.randrange(256)                          # [inferred] avoid colliding with a still-running controller's life_count_
        self.last, self.lk = None, threading.Lock()
        self.st = dict(motion_id=-1, bar=0, switch=0, motor_error=[0]*12, seen=0)
        self.cv = threading.Condition()
        self.busy = None; self.estop_latch = False; self.interrupt = False
        self.post_checked = False; self.pre_failed = False
        self.last_servo = None; self.ticked = False; self.need_check = False
        self.gait_def_ok = self.gait_params_ok = False             # SIM GUARD (command_interface.cpp:48-49,1186; gait.cpp:379)
        for f, a in ((self._spin, (self.cmd,)), (self._spin, (self.resp,)),
                     (self._heartbeat, ()), (self._servo_watchdog, ())):
            threading.Thread(target=f, args=a, daemon=True).start()

    # ---------- LCM primitives ----------
    def _spin(self, lc):
        while True: lc.handle_timeout(100)
    def _publish_new(self, c):                       # new frame: new life_count (motion_action.cpp:56,106)
        with self.lk:
            c.life_count = i8(self.life); self.life += 1; self.last = c
            self.cmd.publish("robot_control_cmd", c.encode())
    def _heartbeat(self):                            # same frame, same life_count, 20 Hz (motion_action.cpp:474-483)
        while True:
            with self.lk:
                if self.last is not None: self.cmd.publish("robot_control_cmd", self.last.encode())
            time.sleep(HEARTBEAT)
    def _on_resp(self, ch, data):
        r = robot_control_response_lcmt.decode(data)
        mid = 0 if r.mode == 0 else self.rev.get((r.mode, r.gait_id), -1)   # motion_action.cpp:407-459
        with self.cv:
            self.st.update(motion_id=mid, bar=r.order_process_bar, switch=r.switch_status,
                           motor_error=list(r.motor_error), seen=self.st["seen"] + 1, raw=r)
            self.cv.notify_all()
    def _wait(self, pred, timeout):
        with self.cv: return self.cv.wait_for(lambda: pred(self.st), timeout / RTF)

    def _build(self, mid, p, servo):
        m = self.map[mid]["map"]; c = robot_control_cmd_lcmt()
        c.mode, c.gait_id = m[0], m[-1]
        c.value = int(p.get("value", 0)) & ~0x04     # SIM DEVIATION: never trigger factory remount (command_interface.cpp:1058-1064)
        for f, n in ARRAYS:                          # GET_VALUE: wrong length -> zeros (motion_macros.hpp:111-121)
            v = p.get(f) or []; setattr(c, f, [float(x) for x in v] if len(v) == n else [0.0]*n)
        if servo: c.contact, c.duration = 0, 0                                   # motion_action.cpp:44,46
        else:     c.contact = int(p.get("contact", 0)) or 15; c.duration = int(p.get("duration", 0))  # :94,96
        return c

    # ---------- rules ----------
    def _fail(self, code):
        return {"motion_id": self.st["motion_id"], "result": False, "code": code}   # names; numeric codes [closed]
    def _valid(self, mid, dur):                      # motion_handler.cpp:93-119
        if mid == 400: return True
        if mid not in self.map: return False
        t = self.map[mid]["min_exec_time"]
        return dur == 0 if t > 0 else dur > 0 if t < 0 else True
    def _post_ok(self, mid):                         # motion_handler.cpp:871-889
        if mid in (111, 0): return True
        cur = self.st["motion_id"]
        if cur == -1 or cur not in self.map: return False
        return self.map[mid]["map"][0] in self.map[cur]["post_motion"]
    def _motors_ok(self):                            # motion_handler.cpp:948-984
        return all((e & 0x3FFFFFFF) == 0 for e in self.st["motor_error"])
    def _check(self):                                # CheckMotionResult (motion_handler.cpp:302-384)
        if self.st["switch"] in FAIL_SWITCH: return f"switch_{self.st['switch']}"
        return 0 if self._motors_ok() else "kHwMotor*"
    def _user_gait_guard(self, mid):                 # SIM GUARD: avoid controller crash
        m = tuple(self.map[mid]["map"]) if mid in self.map else None
        if m == (11, 110) and not self.gait_def_ok: return False
        if m == (62, 110) and not self.gait_params_ok: return False
        return True

    # ---------- 1004 ----------
    def motion_cmd_grpc(self, req):                  # app-visible behaviour incl. the 20 s cap
        out = {}; t = threading.Thread(target=lambda: out.update(self.motion_cmd(req)), daemon=True); t.start()
        t.join(GRPC_CAP)
        return out if out else {"grpc_error": 321}   # handler keeps running and stays busy
    def motion_cmd(self, req):                       # returns {"motion_id","result","code"}
        mid = req["motion_id"]
        if self.estop_latch and mid not in (0, 111): return self._fail("kEstop")   # motion_decision.hpp:161-176
        self.estop_latch = (mid == 0)                                              # motion_decision.cpp:152
        if mid == 0 and self.busy == "result":
            self.interrupt = True
            with self.cv: self.cv.notify_all()                                     # motion_handler.cpp:640-644
        if self.busy and mid != 0: return self._fail("kTargetBusy")               # :645-655 (dance/FSM cases omitted)
        if not self._valid(mid, req.get("duration", 0)):
            return self._fail("kUnSupport" if mid not in self.map else "kParametersInvalid")
        if mid != 0 and not self._motors_ok(): return self._fail("motor")
        self.busy = "result"
        try:
            if not self._post_ok(mid):
                r = self._execute({"motion_id": 111})
                if not r["result"]: return r
            if mid == 400 and not self._file_send(req.get("gait_toml", "")):   # always "" via gRPC 1004 -> fails
                return self._fail("kSequenceDefError")
            if not self._user_gait_guard(mid): return self._fail("SIM_GUARD_no_user_gait_upload")
            return self._execute(req)
        finally: self.busy = None

    def _execute(self, req, pre_published=False):  # motion_handler.cpp:401-622
        mid = req["motion_id"]; seen0 = self.st["seen"]; self.interrupt = False
        if not pre_published: self._publish_new(self._build(mid, req, servo=False))
        if mid == 400: self._publish_new(self._build(400, {}, servo=False))     # :429-433
        if not self._wait(lambda s: s["seen"] > seen0, FEEDBACK): return self._fail("kComLcmTimeout")
        time.sleep(0.015)
        sw = self.st["switch"]
        if sw in FAIL_SWITCH: return self._fail(f"switch_{sw}")
        if sw == 1 and not self._wait(lambda s: s["motion_id"] == mid and s["switch"] == 0, TRANSIT):
            return self._fail("kMotionTransitionTimeout")
        t, d = self.map[mid]["min_exec_time"], req.get("duration", 0)
        to = t if t > 0 else d + 500 if t < 0 else (500 if d == 0 else 2 * d)
        if not self._wait(lambda s: (s["motion_id"] == mid and s["bar"] > 95) or self.interrupt, to / 1000):
            return self._fail("kMotionExecuteTimeout")
        code = self._check()                         # also reached after an ESTOP interrupt (no predicate in real code)
        if code: return self._fail(code)
        return {"motion_id": self.st["motion_id"], "result": True, "code": 0}

    # ---------- 1002 ----------
    def servo(self, f):                              # cmd_type 0 START, 1 DATA, 2 END
        mid = f["motion_id"]
        if self.estop_latch and mid not in (0, 111): return self._servo_resp(False, "kEstop")
        if self.busy == "result": return self._servo_resp(False, "kTargetBusy")
        if mid not in self.map: return self._servo_resp(False, "kUnSupport")
        self.busy = "servo"
        if f.get("cmd_type", 1) != 2:
            if not ((self.post_checked and mid != 309) or self._post_ok(mid)):   # AllowServoCmd :891-896
                self.need_check = False
                r = self._execute({"motion_id": 118 if mid == 309 else 111})
                if not r["result"]:
                    self.busy, self.pre_failed = None, True; return self._servo_resp(False, r["code"])
                self.post_checked = True
            self.pre_failed = False; self.last_servo = f   # last_servo is never cleared (motion_handler.hpp:214)
            self._publish_new(self._build(mid, f, servo=True))
            self.ticked = True; self.need_check = True
        else:
            self._stop_servo()                       # faithful: a stray END re-publishes 112 from the stale frame
        return self._servo_resp(True, 0)
    def _servo_resp(self, result, code):             # 1003, motion_decision.cpp:91-121
        seen0 = self.st["seen"]
        if not self._wait(lambda s: s["seen"] > seen0, FEEDBACK):
            return dict(motion_id=-1, order_process_bar=-1, status=-1, result=False, code="kComLcmTimeout")
        s = self.st
        return dict(motion_id=s["motion_id"], order_process_bar=s["bar"], status=s["switch"],
                    result=result, code=code if not result else self._check())
    def _stop_servo(self):                           # motion_handler.cpp:251-300
        self.need_check = False
        s = self.last_servo
        if s and s["motion_id"] != 201 and not self.pre_failed:
            self._publish_new(self._build(112, {"value": s.get("value", 0),
                                                "step_height": s.get("step_height")}, servo=False))
        self.busy = None; self.post_checked = False
    def _servo_watchdog(self):                       # motion_handler.cpp:234-249
        miss = 0
        while True:
            if self.need_check:
                miss = 0 if self.ticked else miss + 1; self.ticked = False
                if miss >= SERVO_MISS: self._stop_servo()
            time.sleep(SERVO_TICK)

    # ---------- sequences (visual-programming semantics, not reachable from 1004) ----------
    def _file_send(self, text):                      # motion_action.cpp:485-504
        self._ack = None; self._ack_ev = threading.Event()
        m = file_send_lcmt(); m.data = text; self.cmd.publish("user_gait_file", m.encode())
        if not self._ack_ev.wait(SEQ_ACK): return False
        time.sleep(0.001)
        ok = self._ack == 0
        if ok and text.startswith("# Gait Def"):    self.gait_def_ok = True
        if ok and text.startswith("# Gait Params"): self.gait_params_ok = True
        return ok
    def _on_file_ack(self, ch, data):
        self._ack = file_recv_lcmt.decode(data).result; self._ack_ev.set()
    def sequence_show(self, gait_toml, pace_toml):   # motion_manager.cpp:340-357 -> motion_handler.cpp:634-776
        if self.estop_latch: return self._fail("kEstop")
        if self.busy: return self._fail("kTargetBusy")
        self.busy = "result"
        try:
            if not self._post_ok(400):
                r = self._execute({"motion_id": 111})
                if not r["result"]: return r
            if not self._file_send(gait_toml): return self._fail("kSequenceDefError")   # :711-722
            self._file_send(pace_toml)                                               # motion_action.cpp:119-132 (failure only logged)
            if not self.gait_params_ok: return self._fail("SIM_GUARD_no_gait_params")  # avoid crash (command_interface.cpp:1186)
            return self._execute({"motion_id": 400}, pre_published=True)  # only the single :429-433 [62,110] frame
        finally: self.busy = None
```

Notes on the pseudo-code:
- **Sim deviations:** masking value bit2, and the two user-gait guards.
- **Omitted:**
  - dance audio (`motion_handler.cpp:754-763,986-992`);
  - elec-skin calls (`motion_handler.cpp:723-725,770-772`; `motion_action.cpp:306-314`);
  - the `lcm-logger` spawn (`motion_action.cpp:322-325`);
  - the 101-during-dance interruption;
  - cmd_source priority;
  - the protected state [closed].
- **Concurrency:** the real stack can run an ESTOP concurrently with an in-flight result command. The bridge must accept 1004 calls on separate threads for that path to work. [inferred]

## 3. Sim state tests can assert on

**Ground truth: `simulator_state`** (simulator_lcmt, about 500 Hz). Filled in `legged_plugin.cpp:246-363`:
- **Filled fields:**
  - `p` = base world position (`:345-351`);
  - `quat` = [w,x,y,z] (`:324-327`);
  - v, vb, omega, omegab (`:345-351`);
  - q, qd, tau (`:283-285`);
  - f_foot in the order FL, FR, HL, HR, body frame (`:411-414`).
- **Not filled:** timesteps, time, R, vbd, qdd, tauAct and p_foot.
- **rpy:** it comes from Eigen `eulerAngles(2,1,0)` (`:330-333`), whose first angle is constrained to [0, pi]. Compute roll/pitch/yaw from `quat` instead. [inferred from Eigen semantics]

**Estimator: `state_estimator`** (7669, 50 Hz). This is **not** ground truth, because `cheater_mode: 0` (`robot-defaults.yaml:14`). Use contactEstimate and vBody only to check that the estimator agrees.

**`robot_control_response`** (7670):
- Fields: mode, gait_id, switch_status, order_process_bar, and contact bits (foot force > 5 N).
- Use it to assert accepted or refused and the reverse-mapped motion_id.
- After any timed command, expect the filler state (see 1.2): 21/3 → 214, 11/1 → 112, 11/31 → 113, 3/1 → 203.

**`global_to_robot`** (estimator odometry) and **`leg_control_data`** (7667, 50 Hz) carry joint q and tau, useful for posture checks. `cyberdog_visual` already forwards these and `state_estimator` to ROS (`cyberdog_simulator/cyberdog_visual/src/cyberdog_visual.cpp:44-45,75-81`).

**`motor_error`:** in the sim it is presumably all 0 [inferred]. Do not assert the hardware value INT32_MIN.

**Time:** all expected durations below are sim time. Scale wall-clock tolerances by 1/RTF. [inferred]

**Reference heights:**
- the locomotion MPC body height is 0.25 m (`cyberdog_locomotion/common/config/cyberdog2-ctrl-user-parameters.yaml:102`);
- the "motion" height parameter is 0.225 (`:103`);
- 21/5 clamps the target height to 0.10-0.30 (`fsm_state_posectrl.cpp:158-163`);
- RecoveryStand chooses stand or fold from the estimator height window `0.42*(L1+L2) < z < 0.45` (`fsm_state_recoverystand.cpp:84-108`).

Suggested assertions [inferred design; record baseline values in the sim once before tightening tolerances]:

1. **111 stand.**
   - The result succeeds within 15 s.
   - The response reads (12,0), bar > 95, switch 0.
   - `simulator_state.p[2]` is above the lying baseline by about 0.1 m or more, and all 4 contact bits are set.
2. **101 lie down.** The response reads (7,1), and p[2] returns near the lying baseline.
3. **Servo walk (303 [11,27]).** From the 111 stand, no pre-motion runs, because 111 post includes 11.
   - Stream vel_des [0.2,0,0] at 10-20 Hz for 3 s wall-clock.
   - Expect forward displacement along the initial yaw of about 0.6*RTF m (+/-50%), and a response of (11,27).
   - Send END. Expect (11,1), i.e. motion 112, and |v| < 0.05 m/s within about 1 s.
   - Repeat with vel_des [0,0,0.5] and assert the yaw change, computed from quat.
4. **Servo loss.**
   - Stop frames without END: within about 0.2 s the bridge publishes 112.
   - Kill the bridge: within about 0.5 s wall-clock the response reads mode 7 PureDamper and p[2] drops (`command_interface.cpp:162-170`).
5. **211 [21,5].**
   - Send 1004 pos_des [0,0,0.20] with duration 1000.
   - The result succeeds within 1.5 s; the response shows (21,5) with bar > 95, then the filler (21,3)/214.
   - p[2] decreases relative to the stand baseline, with roll and pitch near 0.
6. **212 [21,0].**
   - Send rpy_des [0,0.2,0] with duration 1000. Pitch should change by about 0.2 rad, spread over 500 ticks (`fsm_state_posectrl.cpp:136-141`).
   - Then the 21/3 filler holds the pose.
   - A second identical 212 adds another 0.2 rad: it is relative. Measure the sign.
7. **Posture after posture.** A servo walk right after 211/212 first runs a full 111, because 211/212 post_motion lacks 11. Assert the 111 state appears before (11,x).
8. **Refusal.** Send 126 (mode 23, absent from the open FSM). Expect switch_status 5 BAN_TRANS, mode unchanged, and the result false.
9. **Busy.** A 1004 sent during an active servo stream is rejected with kTargetBusy, except 0.
10. **ESTOP latch.** Send 0; mode becomes 0 (Off). Then 101 must return kEstop, and 111 must be accepted and clear the latch.
11. **Duration rules.** 211 with duration 0, and 111 with duration > 0, are both rejected as kParametersInvalid before any LCM traffic.
12. **Long motions.** 140 (min 60 s): an app-level call returns 321 at 20 s, and later commands get kTargetBusy until the motion completes.

## 4. Motion ids the sim cannot reproduce

The open-source FSM only has states 0, 3, 7, 9, 11, 12, 15, 16, 21, 22, 64, 80 and 81 (`control_fsm.cpp:494-546`). Each state's transition table refuses other modes with "Bad Request" [code]:

| current state | allowed next modes | source |
|---|---|---|
| 11 | 11, 12, 0, 3, 7, 21, 16, 80, 81 | `fsm_state_locomotion.cpp:109-149` |
| 21 | 21, 7, 3, 12, 0, 11, 64, 16, 22, 80 | `fsm_state_posectrl.cpp:200-231` |
| 12 | 12, 21, 7, 3, 0, 11, 64, 16, 22, 9, 80, 81 | `fsm_state_recoverystand.cpp:1120-1153` |
| 7 | 7, 15, 12, 0, 80 | `fsm_state_pure_damper.cpp:132-137` |
| Off | 0, 7, 12, 15, 80 | `fsm_state_passive.cpp:72-79` |

A refused mode gives BAN_TRANS = 5 (`control_fsm.cpp:139-146`).

**A. The mode is missing from the FSM, so BAN_TRANS and nothing moves** [code]:

| mode | what it is | motion ids |
|---|---|---|
| 17 | flips | 120, 121, 122, 127, 128, 129, 506, 507, 508, 509 |
| 23 | jump onto a stair | 126 |
| 5 | skateboard | 510, 511 |
| 6 | vision stairs | 344 |
| 25 | (unknown) | 345 |
| 65 | (unknown) | 513 |
| 82 | (unknown) | 311 |
| 86 | (unknown) | 512 |

The dog-side map still lists 5, 6, 17, 23 and 82 in 111's post_motion (`motion_id_map.toml:35-39`). So the robot's firmware probably supports them, and the open locomotion is a subset. [inferred; closed]

**B. The sub-id is missing, so the wrong motion runs** [code]:
- **Mode 11, gaits 120-123 (ids 340-343).**
  - These are >= 50, so they go to the motion-gait MPC (`fsm_state_locomotion.cpp:349-361`). Its default case is `SetDefaultParams()` = `trot_in_out_` (`convex_mpc_motion_gaits.cpp:352-458`).
  - The response echoes the commanded gait (`fsm_state_locomotion.cpp:519-520`), so the stack believes it worked.
- **Mode 16, jump ids 13 and 16 (ids 147, 500).**
  - The jump_3d default runs the +90 degree yaw jump and reports gait 0 (`fsm_state_jump_3d.cpp:191-194`).
  - Since the reported gait 0 differs from the commanded gait, switch_status stays 1 (`cyberdog_controller.hpp:79-87`). The 1004 fails with kMotionTransitionTimeout after 3 s, or with kMotionExecuteTimeout depending on timing. [inferred from code]
- **Mode 62, ids missing from the MotionId switch** (35, 36, 40, 113-118, used by 172, 179, 139, 178 and 501-505).
  - The controller silently loads the default `cyberdog2_motion.toml` (`command_interface.cpp:416-417`; `control_flags_release.hpp:181-242`).
  - The response still reports (62, id), so the stack believes it worked.

**C. Environment or plumbing is missing** [code + inferred]:
- 344 needs the perception stack.
- The dances 140, 176, 177 and 178 trigger audio through the audio service (`motion_handler.cpp:754-763,986-992`) [closed]. The robot motion itself does run.
- Sequence 400 through 1004 always fails, because gait_toml is not forwarded (`cyberdog_app.cpp:2924-2966`).
- 401 [11,110], and any user-gait use, needs a prior `# Gait Def` upload that 1004 cannot provide. Without it the sim controller is expected to crash (1.4).
- Through gRPC, ids whose min_exec_time is 20 s or more (124, 138, 139, 140, 151, 176-178, 501-505) return 321 to the app even when they succeed.
- Protected-state behaviour (low battery and similar) is not reproduced; switch statuses 6 and 7 never appear.

**D. Reproducible in the sim** [code; visual fidelity versus the robot is not verifiable]:
- 0, 101, 102, 111, 112;
- 115 ([9,1], reachable from 12; effect not verified);
- 117-119 (RlReset 80, `rl_models/cyberdog2/reset/model.txt`, `fsm_state_rl_reset.cpp:24`) and 307 (RlRapid 81, `fsm_state_rl_rapid.cpp:24-25`);
- 123 and 175 (64/0, 64/1; TwoLegStand accepts gaits 1-14, else 0, `fsm_state_two_leg_stand.cpp:35-38`);
- 124/138 (12/17, 12/18);
- 125, 130-137;
- 140-146, 148-171, 173, 174, 176, 177;
- 180-199 (user gaits 80-99, define files `control/motion_list/cyberdog2/user/define/user_gait_00..20.toml`);
- 201, 202;
- 211-214 (213/214 behave like the incremental 21 path; only 213 damps the swing leg);
- 301-306, 308-310, 390-392.

**Registration-only ids, not commands:** 113 [11,31], 114 [9,0] and 203 [3,1] (`motion_id_map.toml:49-50,58`). They appear as reverse-mapped response states.

## 5. Unknowns and closed dependencies

- Whether the motion_id_map.toml installed on the robot (`share/motion_action/preset`) matches the public copy: ids, post_motion lists, min_exec_time.
- Whether the robot's locomotion binary executes the following, which the open-source controller refuses or substitutes:
  - modes 5, 6, 17, 23, 25, 65, 82, 86;
  - gait ids 120-123;
  - jump ids 13/16;
  - motion ids 35/36/40/113-118.
- Whether the robot's controller has the empty-string substr crash, or keeps user-gait strings across boots.
- Numeric values of KeyCode/MotionCode (cyberdog_system) [closed/not read].
- Manager states (kProtected etc.) and the cmd_source priority rules in full [closed: cyberdog_machine].
- Real execution times versus the sim, and whether min_exec_time timeouts are hit on hardware.
- Real stand, lie and posture heights, and how closely the robot tracks vel_des. The sim gives ground truth; the robot only gives the estimator.
- Whether the robot's cyberdog_app is this version (1002/1004 field handling, 321/323, the 20 s cap). The local ref snapshot is identical at the cited lines.
- Wi-Fi jitter of the servo stream versus the roughly 200 ms watchdog and the 0.5 s controller timeout.
- Real motor_error values in normal operation. The sim value is presumably 0; the hardware value is presumably INT32_MIN.
- Sign convention of 212 rpy_des on hardware.
- Dance audio and LED-skin side effects on the robot for 140/176/177/178.
- The actual Gazebo RTF achievable on the target PC/WSL2, which affects every timeout.


## Unknowns that need the real dog

- Whether the motion_id_map.toml installed on the robot's firmware (share/motion_action/preset) matches the public copy (ids, post_motion lists, min_exec_time).
- Whether the robot's locomotion binary (newer or closed) actually executes modes 5, 6, 17, 23, 25, 65, 82, 86, gait ids 120-123, jump ids 13/16 and motion ids 35/36/40/113-118, which the open-source controller refuses or substitutes.
- Real execution times of each motion compared with the sim (for example how long 111 takes, flips, dances) and whether the min_exec_time timeouts are ever hit on hardware.
- Real stand, lie and posture body heights, and how closely the real robot tracks commanded vel_des. The sim gives ground truth; the real robot only gives the estimator.
- Whether the cyberdog_app gRPC server on the robot is the same version as this repo (1002/1004 field handling, error codes 321/323, the 20 s service timeout).
- Whether servo frames for posture motions (211/212 via 1002, sent with duration 0) behave on the robot as they do in the open PoseCtrl code (duration 0 is treated as 1800000 ticks).
- Jitter of the servo stream over Wi-Fi compared with the roughly 200 ms servo-loss watchdog and the 500 ms controller timeout.
- The real motor_error flag values the robot reports in normal operation (bit 31 is ignored; bits 0-29 abort commands).
- Sign convention and accumulation behaviour of 212 rpy_des on hardware, and whether 211 ignores rpy_des on the robot as it does in open code.
- Whether dance audio and LED-skin side effects fire on the robot for 140/176/177/178 when driven through the app.

## Closed or missing in the public sources

- Open cyberdog_locomotion lacks FSM modes 5, 6, 17, 23, 25, 65, 82, 86 (motion ids 120-122, 126-129, 311, 344, 345, 506-513); they return BAN_TRANS in the sim.
- Open cyberdog_locomotion lacks gait ids 120-123 (stairs, 340-343): it substitutes the default trot_in_out gait silently.
- Open cyberdog_locomotion lacks jump ids 13/16 (147, 500): the yaw+90 jump runs instead and the reported gait is 0.
- Open cyberdog_locomotion lacks MotionIds 35, 36, 40, 113-118 (139, 172, 178, 179, 501-505): the default cyberdog2_motion.toml plays instead.
- The doc's appendix motion table (mode/gait list) exists only as an image (motion_list_cn.png), not as machine-readable text.
- Sequence 400 cannot be driven via gRPC 1004: cyberdog_app.cpp does not forward gait_toml, so the sequence definition always fails. Sequences need the visual-programming MotionSequenceShow path.
- Vision-stair motion 344 needs the perception stack, which is not in the sim. Dance audio needs the audio service, which is not in the sim.
- The real motion_manager depends on cyberdog_system, manager_base, cyberdog_machine, skin_manager and bes_transmit. These are open but heavy, so the bridge re-implements the logic instead.
- Not read: the numeric KeyCode/MotionCode tables (cyberdog_system) that the app receives as 'code', and the motion_manager publisher behind the 1003 servo response.
- value bit2 (factory speed calibration) triggers a mount command in the controller and must be masked by the bridge.
- No cloud dependency was found in the app-to-motion-to-locomotion-to-sim path; everything is local LCM/ROS2/gRPC. Raw LCM access on the real robot requires a developer unlock (cyberdog_loco_cn.md:65-86), but the app path does not need it.
- Loopback multicast routing for LCM on WSL2/Ubuntu is not documented in the repos and must be configured (inferred standard LCM setup).

## Verifier corrections (16)

- **Claim:** Key claim and table rows: robot_control_cmd on 7671, robot_control_response on 7670, state_estimator on 7669, and global_to_robot/leg_control_data on 7667, all cited to motion/motion_action/include/motion_action/motion_macros.hpp:87-98.
  - **Correct:** 7671 and 7670 are at motion_macros.hpp:87-88. state_estimator on 7669 is state_lcm_ at cyberdog_locomotion/control/src/robot_runner.cpp:20, and motion_action subscribes to it at motion_action.cpp:306-314. global_to_robot and leg_control_data go out on the 7667 instances at robot_runner.cpp:19,21, published at :217-237.
  - Source: motion/motion_action/include/motion_action/motion_macros.hpp:87-98; cyberdog_locomotion/control/src/robot_runner.cpp:19-22,217-237; motion/motion_action/src/motion_action.cpp:306-314
- **Claim:** Loopback multicast setup ('multicast on' plus a 224.0.0.0/4 route) 'appears nowhere in the repos'.
  - **Correct:** loco_hl_example/scripts/auto_lcm_init.sh:11-12 and cyberdog_locomotion/scripts/config_network_lcm.sh:21-35 (option -I <iface>) do this; doc cyberdog_loco_cn.md:85 refers to it. Both scripts target a NIC. Running the same commands on `lo` for a single WSL2 host is [inferred].
  - Source: loco_hl_example/scripts/auto_lcm_init.sh:11-12; cyberdog_locomotion/scripts/config_network_lcm.sh:21-35; blogs/docs/cn/cyberdog_loco_cn.md:85
- **Claim:** In mode 21, gait_id 3 means 'task-space interpolation' (spec cites fsm_state_posectrl.cpp:129-181).
  - **Correct:** Mode 21 sub-modes as the code implements them: 5 = absolute height transform (joint interpolation to IK); 2 = incremental with a damped swing leg; any other value (0, 3, ...) = incremental rpy/pos/foot offsets spread over the duration.
  - Source: cyberdog_locomotion/control/src/fsm_states/fsm_state_posectrl.cpp:129-152,153-181; motion/motion_action/preset/motion_id_map.toml:790
- **Claim:** Manual user-gait alternative: send '# Gait Def', sleep 0.5 s, send '# Gait Params', 'then send [11,110] steps with duration > 0'.
  - **Correct:** After both uploads, send robot_control_cmd [12,0] duration 5000, then [62,110] duration 4320, then [7,0] duration 4000. In the controller, [62,110] loads user_gait_list_file_ through MotionListFullCppTomlRead.
  - Source: loco_hl_example/customized_gait/Usergait_List.toml; blogs/docs/cn/cyberdog_loco_cn.md:625-741; cyberdog_locomotion/common/src/command_interface/command_interface.cpp:407-409,1177-1241
- **Claim:** robot_control_response is published when switch_status changes, every 10 iterations, and 'when the bar reaches 95'.
  - **Correct:** It also publishes on every 2 ms tick while the bar is in 95..99, plus the first tick at 100. This adds to the switch-change and every-10-iterations (50 Hz) publishes.
  - Source: cyberdog_locomotion/control/src/robot_runner.cpp:390-391
- **Claim:** switch_status = 'ban_flag if > 0; TRANSITIONING if the gait differs', with values 0-7 including 6 OVER_HEAT and 7 LOW_BAT as controller outputs.
  - **Correct:** Ban flags in open code: 5 BAN_TRANS (next state != commanded mode); 6 and 7 never occur. Values 1-4 also come from the FSM operating mode. The .lcm comment '4:BAN_TRANS' is stale.
  - Source: cyberdog_locomotion/control/include/control/cyberdog_controller.hpp:79-87; cyberdog_locomotion/control/src/fsm_states/control_fsm.cpp:104-150,599-618; control_fsm.hpp:37; robot_control_responce_lcmt.lcm:6
- **Claim:** Servo watchdog: exact Tick/Tock semantics are 'inferred from motion_handler.hpp:132-139'.
  - **Correct:** [code] Every 50 ms one Tock. 4 consecutive misses trigger StopServoCmd, so the tolerated gap is about 150-200 ms.
  - Source: motion/motion_action/include/motion_action/motion_macros.hpp:146-164; motion/motion_manager/src/motion_handler.cpp:234-249
- **Claim:** The table cites motor_temperature (1 Hz, every 500 iterations) to robot_runner.cpp:217-237.
  - **Correct:** motor_temperature is published at robot_runner.cpp:395-404 when lcm_iterations_ % 500 == 3, i.e. 1 Hz at the 2 ms control step.
  - Source: cyberdog_locomotion/control/src/robot_runner.cpp:395-404
- **Claim:** 117-119 are 'RL 80/81; policy weights ... fsm_state_rl_rapid.cpp:24-26'.
  - **Correct:** 117/118/119 = [80,2]/[80,1]/[80,0], RlReset, using rl_models/cyberdog2/reset/model.txt. 307 = [81,0], RlRapid, using the rapid model.
  - Source: motion/motion_action/preset/motion_id_map.toml (117-119, 307); cyberdog_locomotion/control/src/fsm_states/fsm_state_rl_reset.cpp:24; fsm_state_rl_rapid.cpp:24-25
- **Claim:** `_execute` for 400 publishes [62,110] twice 'mirroring :428 and :432', and sequence_show uses it, so the visual-programming path publishes [62,110] twice.
  - **Correct:** Visual-programming sequence: upload gait_toml, upload pace_toml (failure only logged), then publish exactly one [62,110] frame (contact 15, duration 0).
  - Source: motion/motion_action/src/motion_action.cpp:119-132; motion/motion_manager/src/motion_handler.cpp:401-433,711-722
- **Claim:** 147 and 500 (mode 16, jump ids 13/16): 'the stack times out waiting for 147 or 500'.
  - **Correct:** [inferred from code] The 1004 fails with kMotionTransitionTimeout after 3000 ms. If the first response sampled 15 ms after the publish predates the mode change, it fails with kMotionExecuteTimeout instead (147: 15000 ms; 500: 1000 ms). The dog still performs the default +90 degree yaw jump.
  - Source: cyberdog_locomotion/control/include/control/cyberdog_controller.hpp:79-87; cyberdog_locomotion/control/src/fsm_states/fsm_state_jump_3d.cpp:191-194; motion/motion_manager/src/motion_handler.cpp:538-551,589-609
- **Claim:** 1003 MOTION_SERVO_RESPONSE: 'The motion_manager publisher that feeds it was not read.'
  - **Correct:** MotionDecision::ServoResponseThread publishes one response per servo frame after the next LCM feedback (up to 200 ms). Fields: motion_id (reverse-mapped), order_process_bar, status = switch_status, code from CheckMotionResult, and result from the handler. On feedback timeout it publishes motion_id -1, bar -1, status -1, result false, code kComLcmTimeout.
  - Source: motion/motion_manager/src/motion_decision.cpp:91-121; motion/motion_manager/include/motion_manager/motion_decision.hpp:237-260
- **Claim:** User-gait step_height is packed as `int(R*1e3) + int(L*1e3)*1e3`, so 35035 means 0.035/0.035.
  - **Correct:** step_height[0] = int(RF*1e3) + int(LF*1e3)*1e3 (front pair). step_height[1] = int(RH*1e3) + int(LH*1e3)*1e3 (hind pair). The doc uses ceil() rather than int().
  - Source: interaction/cyberdog_vp/cyberdog_vp_abilityset/src/sdk/motion.cpp:328-335; blogs/docs/cn/cyberdog_loco_cn.md:678-679
- **Claim:** Key claim: '111 maps to [12,0] min 15000; 211 to [21,5] and 212 to [21,0] with min -1', cited to motion_id_map.toml:35-39.
  - **Correct:** 111 at :35-39; 211 at :768-773; 212 at :776-781.
  - Source: motion/motion_action/preset/motion_id_map.toml:35,768,776
- **Claim:** Section 4D lists as reproducible commands 113 (11/31), 114/115, 201-203 and 401.
  - **Correct:** 113/114/203 are expected response states, not test commands. 401, or any [11,110], must be preceded by an accepted '# Gait Def' upload; otherwise the sim controller is expected to crash [inferred from code]. 1004 cannot provide that upload.
  - Source: motion/motion_action/preset/motion_id_map.toml:49-50,58; cyberdog_locomotion/control/src/fsm_states/fsm_state_locomotion.cpp:459-463; cyberdog_locomotion/control/src/convex_mpc/convex_mpc_motion_gaits.cpp:255-262; cyberdog_locomotion/control/src/convex_mpc/gait.cpp:361-379; cyberdog_locomotion/common/src/command_interface/command_interface.cpp:48,90-101,460; cyberdog_locomotion/simbridge/src/simulation_bridge.cpp:71-76
- **Claim:** The pseudo-code is complete as sketched: it starts a thread on self._spin and calls self._fail and self._servo_resp.
  - **Correct:** Run one handle_timeout loop per LCM instance. Define _fail, _servo_resp (per motion_decision.cpp:91-121) and the ESTOP latch. See the corrected pseudo-code.
  - Source: spec section 2.6; motion/motion_manager/src/motion_decision.cpp:67-121,140-155; motion/motion_manager/include/motion_manager/motion_decision.hpp:161-176

## Verifier additions

- ESTOP latch at decision level: after a 0 (ESTOP), every servo frame and result command other than 0 or 111 is refused with kEstop. Every result command resets the latch to (motion_id == 0), so the next 111 clears it whatever its outcome: motion/motion_manager/include/motion_manager/motion_decision.hpp:161-176; motion/motion_manager/src/motion_decision.cpp:67-84,140-155
- An ESTOP (or a 101 during a dance) notifies execute_cv_. That condition variable is waited on without a predicate, so the interrupted motion skips its completion check, goes straight to the final CheckMotionResult, and can report result=true: motion/motion_manager/src/motion_handler.cpp:637-644,600-621
- The gRPC 1004 call has a hard 20 s cap (wait_for_service 3 s, then future wait 20 s; failure returns error 321). Ids that need longer: 124/138 (20000 ms), 139/151 (30000 ms), 140 (60000 ms), 176-178 and 501-505 (120000 ms), plus any automatic 111 pre-motion (up to 15 s). The app gets 321 while the handler stays busy (kTargetBusy) until the real end: bridges/cyberdog_grpc/src/cyberdog_app.cpp:753-780,2924-2966; motion/motion_action/preset/motion_id_map.toml
- A 1004 carrying a servo-type id (min_exec_time 0, e.g. 303 [11,27]) with duration 0 publishes a continuous walk command. The 50 ms heartbeat keeps it alive, the reply succeeds after at most 500 ms (bar is 100 for duration-0 locomotion), and the dog keeps walking until another command: motion/motion_manager/src/motion_handler.cpp:589-598; motion/motion_action/src/motion_action.cpp:474-483; cyberdog_locomotion/control/src/fsm_states/fsm_state_locomotion.cpp:298-313
- last_servo_cmd_ is never reset. An END frame, valid only if its motion_id is in the map, therefore publishes 112 [11,1] built from a stale frame even when no stream is active, for example after the watchdog already stopped the stream or after a later 1004: motion/motion_manager/src/motion_handler.cpp:186-188,218,227-229,251-268; motion/motion_manager/include/motion_manager/motion_handler.hpp:214
- PrepareCmd precedence with use_rc=1: rc/udp timers first, then the MotorCtrl flag, then robot_control_cmd (under 0.5 s), then exec_request, then the timeout fallback. A single motor_ctrl message sets MotorCtrl_mode_flag_ permanently, after which LCM commands are ignored, so the bridge must never publish motor_ctrl: cyberdog_locomotion/common/src/command_interface/command_interface.cpp:45,84-88,133-171
- cyberdog_example/cyberdogmsg_sender also sets use_rc=0 and control_mode over yaml_parameter, not only keybroad_commander: cyberdog_simulator/cyberdog_example/src/cyberdogmsg_sender.cpp:36,46,57,95
- An extra controller input channel 'motion-list' (trajectory_command_lcmt) exists on 7671: cyberdog_locomotion/simbridge/include/simulation_bridge.hpp:29-42
- Controller crash on [62,110] before any accepted '# Gait Params' upload. user_gait_list_file_ starts as '' (command_interface.cpp:49), MotionListFullCppTomlRead('') calls substr(length()-5) outside its try block (command_interface.cpp:1186; the try starts at :1201), and SimulationBridge::Run rethrows (simulation_bridge.cpp:71-76) [inferred: process exit]. The same bug applies to [11,110] before a '# Gait Def' upload: gait.cpp:379; fsm_state_locomotion.cpp:461-463
- Time base: controller timers (CMD_TIMEOUT 0.5 s and the gamepad/LCM timers) use CLOCK_MONOTONIC wall-clock, while durations count in 2 ms control ticks of sim time. If Gazebo runs below RTF 1, motions stretch in wall-clock time while the 0.5 s timeout does not: cyberdog_locomotion/common/include/utilities/timer.hpp:27,46; command_interface.hpp:29; command_interface.cpp:1008; cyberdog_simulator/cyberdog_gazebo/world/simple.world:6-8
- simulator_state does not fill timesteps, time, R, vbd, qdd, tauAct or p_foot. rpy comes from Eigen eulerAngles(2,1,0), whose first angle is in [0,pi], so derive yaw/pitch/roll from quat. f_foot order is FL, FR, HL, HR in the body frame: cyberdog_simulator/cyberdog_gazebo/src/legged_plugin.cpp:246-363,324-333,345-351,411-414
- Idle fillers carry duration 200 ticks (400 ms). After a timed command they appear in robot_control_response and reverse-map to: 21/3 -> 214, 3/1 -> 203, 11/1 -> 112, 11/31 -> 113. sit_down_trans is 21/5 with pos z 0.225: cyberdog_locomotion/common/src/command_interface/command_interface.cpp:17-43,206-250
- 211/212 post_motion is [0,12,7,21,5,64,62] and has no 11. A servo walk started after a posture command therefore first runs a full 111 RecoveryStand: motion/motion_action/preset/motion_id_map.toml:768-781; motion/motion_manager/src/motion_handler.cpp:193-216,871-896
- 12/17 and 12/18 sent with duration 0 get a 4-tick 12/0 step inserted before them: cyberdog_locomotion/common/src/command_interface/command_interface.cpp:1044-1051
- The controller's life_count_ starts at -1. If the bridge restarts while the controller keeps running, its first frame may repeat the last accepted life_count and be ignored [inferred]: cyberdog_locomotion/common/src/command_interface/command_interface.cpp:11,983
- Per-state transition tables. From 11: 11,12,0,3,7,21,16,80,81. From 21: 21,7,3,12,0,11,64,16,22,80. From 12: 12,21,7,3,0,11,64,16,22,9,80,81. From 7: 7,15,12,0,80. From Off: 0,7,12,15,80. So after 101 (mode 7) everything except 0/12/15/80 needs 111 first: fsm_state_locomotion.cpp:109-149; fsm_state_posectrl.cpp:200-231; fsm_state_recoverystand.cpp:1120-1153; fsm_state_pure_damper.cpp:132-137; fsm_state_passive.cpp:72-79
- over_heat_ and low_bat_ are forced false in the open controller, so switch_status 6 and 7 never appear in the sim: cyberdog_locomotion/control/src/fsm_states/control_fsm.cpp:599-618
- Posture through 1002 (servo frames always have duration 0, which PoseCtrl treats as 1800000 ticks) moves about 1/1800000 of the target per tick, which makes 211/212 servo streams effectively no-ops: motion/motion_action/src/motion_action.cpp:46; cyberdog_locomotion/control/src/fsm_states/fsm_state_posectrl.cpp:125-126,136-141,176-177
- Protected-state exceptions (only 111, 101, 102, 0 and 212 are allowed in kProtected) depend on the manager state machine (cyberdog_machine), which the bridge does not reproduce: motion/motion_manager/src/motion_manager.cpp:126-141,265-286
- 1002 gets an immediate empty-data gRPC reply. The 1003 status travels separately through the motion_servo_response subscription: bridges/cyberdog_grpc/src/cyberdog_app.cpp:142-151,741-751,2900-2922
- Sim build and launch facts: Ubuntu 20.04 + Galactic; BUILD_ROS must be ON in the cyberdog_locomotion CMakeLists; colcon build; launchsim.py; the controller is started as `./cyberdog_control m s`: cyberdog_sim/README.md:7,68,72,78,87,100; cyberdog_simulator/cyberdog_gazebo/launch/cyberdog_control_launch.py:34
- lcm_debug_switch defaults to 0, so the state channels run at the divided 50 Hz rate: cyberdog_locomotion/common/config/robot-defaults.yaml:40; cyberdog_locomotion/control/src/robot_runner.cpp:217-237
- In the sim, motor_error is presumably 0 because the simulated spine board sets the flags to 0 [inferred]. On hardware the normal value is kMotorNormal = INT32_MIN (bit 31 only): motion/motion_action/include/motion_action/motion_macros.hpp:86
- cyberdog_visual already forwards state_estimator (7669) and global_to_robot/leg_control_data (7667) to ROS. It is an alternative state source for tests: cyberdog_simulator/cyberdog_visual/src/cyberdog_visual.cpp:44-45,75-81
- The response motion_id in a 1004 reply is the current reverse-mapped id, not the requested id: motion/motion_manager/src/motion_handler.cpp:618-620
- The doc's user-gait example increments life_count in its heartbeat, unlike motion_action, which repeats it unchanged. Only motion_action's behaviour is safe for timed commands: blogs/docs/cn/cyberdog_loco_cn.md:726-727; motion/motion_action/src/motion_action.cpp:474-483