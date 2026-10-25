# Gazebo baseline for the FakeController

Measured on 2026-10-23 in the `CyberDogSim` WSL distro. The setup was `FAST=1 /root/sim_up.sh` (ODE quick solver, world `simple_quick`) with the open-source `cyberdog_control m s`. The commands went through `emulator/controllers/lcm_link.py` (LcmController), driven by `tests/gazebo/test_lcm_link.py`.

Each number below comes from three full runs of that file, except where a row says otherwise. The raw JSON is in the VM at `/root/sim_logs/lcm_baseline_run{1,2,3}.json`. Run 1 failed only the 211 check, because of a wall-clock wait that the test has since replaced with a sim-time wait. The 211 numbers therefore come from runs 2 and 3 and from a separate diagnostic run that logs every response.

To reproduce, start the sim, then run inside the VM:

```
cd /mnt/c/<path to the repository>
PYTHONDONTWRITEBYTECODE=1 LCM_BASELINE_OUT=/root/sim_logs/x.json python3 -m pytest -s tests/gazebo/test_lcm_link.py
```

## Method, common to every row

- **Ground truth** is `simulator_state` from the Gazebo plugin: `p` for the base_link world position, `v` and `omega` in the world frame.
- **Attitude** is computed from `quat` [w,x,y,z] as ZYX angles (roll about x, pitch about y, yaw about z), not from the plugin's `rpy` field.
  - Positive pitch is **nose down**: the world z of the body x axis is -sin(pitch).
  - Positive roll is **left side up**.
- **Sim time** is the count of `simulator_state` frames x 2 ms, because the plugin leaves `time` empty. The 50 Hz response stream confirms this clock: 54-56 responses per sim-second, 50 Hz plus the extra frames at bar >= 95.
- **Heights** are averaged over 0.4-0.5 s of sim time after the stated settle time.
- **Durations** run from the publish of the new frame (a new life_count) to the first `robot_control_response` that matches.
- **The heartbeat** republishes the last frame at 20 Hz wall-clock with the same life_count, as in sim_bridge.md 2.4.
- **RTF** is the frame-count sim time divided by wall time over a sliding 2 s window, read by `LcmController.rtf()`.

## Real-time factor

| run | at start | at end |
|---|---|---|
| 1 | 0.39 | 0.44 |
| 2 | 0.63 | 0.38 |
| 3 | 0.66 | 0.55 |

The RTF moved between 0.38 and 0.66 because another job loaded the CPU, so it is not a constant. contracts.md 5 says "0.6 with FAST=1"; use `rtf()` live instead of that figure. A separate 3 s smoke reading gave 0.50.

## Heights

| state | body z (m) | other | method |
|---|---|---|---|
| Boot spawn, mode 0 (Off) | 0.073 | pitch +0.166, contact 0 | one sample after a fresh `sim_up.sh`, before any command |
| Lying after 101 [7,1] | **0.0542** | roll 0, pitch 0, contact bits 0 (no foot above 5 N) | 1.5 s sim after bar 100, 3 runs identical |
| Standing after 111 [12,0] | **0.2353** | contact 15, roll and pitch below 0.002, switch 0 | 1.0 s sim after bar 100, 3 runs identical; rise over lying 0.181 |
| Walk stand 112 [11,1] / walking 303 | 0.248-0.251 | | end of the 303 window; the locomotion MPC height is 0.25 |
| 211 [21,5] pos_des z 0.20 | **0.1924-0.1927** | roll and pitch below 0.005 | 0.5 s sim after the 21/3 filler begins; about 8 mm below the commanded leg height |
| 201 [3,0] pos_des z 0.22 | 0.2207-0.2213 | | 1.5 s sim after the first (3,0) response |
| 201 [3,0] pos_des z 0.25 | 0.2493 | | same |
| PureDamper from stand (timeout) | 0.237 -> 0.099-0.104 | | 1.5 s sim after mode 7 appears; still sinking, not yet at the lying 0.054 |

## Durations (sim seconds; wall = sim / RTF)

| motion | measured | method |
|---|---|---|
| 111 [12,0] stand from lying (7/1) | bar 100 at **2.02 s**; bar 80 at 1.22 s, 99 at 2.01 s; z reaches 95 % of the rise at 1.88 s | 3 runs (2.018-2.020); wall 2.9-4.4 s |
| 101 [7,1] lie down from 12/0 | bar rises linearly to 100 at **1.20 s**; z reaches 95 % of the drop at 1.50-1.57 s | 3 runs; matches `fsm_state_pure_damper.cpp` (600 ticks; the state is switchable at 2.0 s) |
| 211 [21,5] duration 1000 from 12/0 | (21,5) bar runs 0 -> 99 over 1.00 s; (21,3) filler at 1.000 s | diagnostic run |
| 211 [21,5] duration 1000 from 11/1 | switch_status 1 for **0.30 s** while locomotion ends its cycle; the response reads (11,1), then (11,5) at bar 99. Then (21,5) bar 0 -> 99 over 1.0 s; bar >= 96 at 1.26 s; (21,3) filler at 1.30 s | 2 runs plus the diagnostic run |
| Idle filler 21/3 | repeats every **0.40 s** (200 ticks), bar 0 -> 100 each cycle | diagnostic run |
| 212 [21,0] duration 1000 | bar >= 96 at 0.80-0.85 s | 3 runs |
| BAN_TRANS for [23,0] | switch_status 5 in the first response after the publish (same 2 ms frame) | 3 runs |
| Stop: 112 [11,1] after a 0.3 m/s walk | v_xy below 0.05 and wz below 0.1 after 0.35-0.63 s; coasts 0.03-0.05 m | 3 runs |
| Stop: 112 [11,1] after a 0.5 rad/s turn | still after 0.25-0.47 s | 3 runs |

## Locomotion tracking (303 [11,27], duration 0, step_height 0.05)

Each test starts from a standing robot (12/0 or 11/1) and runs for 4.0 s wall-clock, which is 1.7-2.6 s of sim time.

| command | measured | method |
|---|---|---|
| vel_des [0.3, 0, 0] | mean 0.279-0.291 m/s over the whole window (93-97 %, start-up included). Steady **0.32 m/s** (107 %) | Forward = displacement projected on the initial heading. Steady = second half of the window. Lateral drift -0.03 m, yaw drift under 0.01 rad. The response reads (11,27) with switch 0 throughout |
| vel_des [0, 0, 0.5] | mean 0.37-0.38 rad/s (75 %); steady **0.40 rad/s** (81 %, run 3) | yaw change unwrapped from the quat trace; translation drift 0.02-0.03 m |

## Posture commands

**212 [21,0], rpy_des [0, 0.2, 0], duration 1000** (starting from 21/3 at z 0.192):
- ZYX pitch moves -0.003 -> -0.206, a change of **-0.203**.
- **A positive pitch_des raises the nose** (the body x axis gains +0.20 of world z per metre).
- The height is unchanged.
- A second 212 with -0.2 brings the pitch back to -0.005, so 212 is **relative**.

**201 [3,0], duration 0, streamed by the heartbeat** (absolute; measured after 1.5 s of sim):

| pos_des z | rpy_des | z | roll | pitch |
|---|---|---|---|---|
| 0.22 | [0, 0.15, 0] | 0.2207 | 0.000 | **+0.145** (nose down) |
| 0.22 | [0.15, 0, 0] | 0.2213 | **+0.136** (left side up) | 0.007 |
| 0.25 | [0, 0, 0] | 0.2493 | 0.018 | 0.000 |

**The pitch signs disagree.** A positive pitch in 201 (QpStand) lowers the nose, and a positive pitch in 212 (PoseCtrl) raises it.
- 201 tracks z to within 1 mm and rpy at 90-97 %.
- From the code only, not measured: in 201, pos_des z 0 commands 0.15 m (the clamp at 0.15-0.35 in `fsm_state_balancestand.cpp`). At z 0.15 and below, and at 0.26 and above, `RpyCmdRescale` scales the rpy command to 0.

## Command timeout (PureDamper)

- **Time to PureDamper:** after the last heartbeat frame from 12/0, the response reads mode 7, gait 0 after **0.50-0.555 s wall-clock** (0.555, 0.501, 0.523 s). CMD_TIMEOUT is a wall-clock timer.
- **Restarting the heartbeat with the same frame does not resume the old command.** The response stays at (7,0) until a frame with a new life_count arrives.

## Calibration deltas for `emulator/controllers/fake.py` (its owner decides)

| fake value | Gazebo |
|---|---|
| `LIE_Z` 0.09 | 0.054 |
| `STAND_Z` 0.235 | 0.235 (matches) |
| `WALK_Z` 0.25 | 0.25 (matches) |
| 111 length 6.0 s | bar 100 at 2.02 s of sim time |
| 101 length 3.0 s | bar 100 at 1.2 s of sim time |
| 303 tracks exactly | 0.32 m/s for 0.3; 0.40 rad/s for 0.5 |
| 21/5 settles at pos_des z | about 8 mm lower |
| 21/0 pitch moves by `+rpy_des[1]` | ZYX pitch moves by **-rpy_des[1]**; fake.py has the opposite sign |
| 3/0 with pos_des z 0 goes to STAND_Z | the code commands 0.15 m (not measured) |
| 11 -> 21 is immediate | 0.30 s of switch_status 1 first |
| | the 21/3 filler repeats every 0.4 s with its bar cycling 0 -> 100 |

## End to end: app2 -> SDK -> emulator (LCM) -> Gazebo (2026-10-24)

Measured with `tests/gazebo/test_gz_e2e.py`, against the same sim and the open-source controller as above. The path is `app2/server.py` (HTTP), then `cyberdog_sdk`, then the emulator (`--controller lcm`), all on 127.0.0.1. The ground truth is the emulator's `/state`, polled at 20 Hz wall-clock, so every wall time below has a resolution of about 50 ms.

The sources are runs 4 to 7 of the whole file (8 tests each) and one run of test_02 and test_03 alone. Raw JSON is in the VM at `/root/sim_logs/gz_e2e_run{4,5,6,7}.json` and `gz_e2e_t0203alone.json`. Where a row says runs 1-7, the previous agent's runs 1-3 also count.

Every run shared the machine with other agents. The Windows CPU load was 63 % in run 5, 57-99 % in run 6, 83 % then 54 % in run 7, and 52 % then 85 % in the alone run. Run 6's turn is left out: only 4 servo frames reached the emulator in 0.86 s, so the watchdog stopped the walk. test_02 and test_03 now fail with "starved (machine load?)" when frames arrive below 15 Hz.

To reproduce from Windows, with the sim and the emulator up:

```
GZ_E2E_OUT=<file>.json python -m pytest -q tests/gazebo/test_gz_e2e.py
```

To reproduce test_09 inside the VM, with the emulator down:

```
python3 -m pytest -s tests/gazebo/test_gz_e2e.py -k test_09
```

### Real-time factor during the e2e runs

| run | overall | min / max over 2 s windows |
|---|---|---|
| 4 | 0.58 | 0.46 / 0.73 |
| 5 | 0.58 | 0.27 / 0.81 |
| 6 | 0.45 | 0.24 / 0.69 |
| 7 | 0.61 | 0.33 / 0.82 |
| test_02 + test_03 alone | 0.63 | 0.57 / 0.70 |
| final pass (`gz_e2e_final.json`) | 0.57 | 0.28 / 0.82 |

### Motion through the app

| step | measured | runs |
|---|---|---|
| Stand 111 from lying (7/1) | z reaches 95 % of the rise at **1.90-1.95 s** sim (wall 2.67-3.24 s); the result returns at 2.87-3.46 s wall | 1, 3-7 |
| Recovery 111 from the ESTOP pose (mode 0, z 0.064-0.098) | 95 % of the rise at **5.70-5.74 s** sim (wall 8.6-13.3 s), with outliers of 7.81 s (run 5) and 11.3 s (run 2); then 101 lies down to z 0.054 in 1.53 s sim (run 7) | 1-7 |
| Drive forward, default x 1.0, POST every 150 ms for 2 s wall | steady **0.216-0.222 m/s** (the 0.2 m/s default plus tracking error); 0.188-0.209 m in 1.14-1.25 s sim at RTF 0.57-0.63 (run 7, RTF 0.34: 0.094 m in 0.68 s sim) | 4-7, alone |
| Release (the app's END) | the emulator publishes WalkStand 11/1 **0.022-0.055 s** wall after the release POST | 4-7, alone |
| Stop after release | the response reports 112 (11/1, switch 0) 0.54-0.69 s sim after release; still (v below 0.05, wz below 0.1) after **0.17-0.59 s** sim; at rest (v below 0.02, wz below 0.05) after **0.62-0.70 s** sim; coasts **0.019-0.045 m** | 4-7, alone |
| Turn left, default | yaw rises (counter-clockwise): 0.25-0.40 rad over 0.92-1.37 s sim, 0.27-0.29 rad/s with the ramp (run 5 at RTF 0.32: 0.15 rad over 0.62 s) | 4, 5, 7, alone x2 |
| Dead-man (POSTs stop) | END reaches the emulator **0.41-0.45 s** wall after the last POST (app DEADMAN_S 0.4); still 0.17-0.63 s sim after it; at rest 0.55-0.81 s sim after it (0.67-1.12 s sim after the last POST); coasts 0.002-0.008 m | 4, 5, 7, alone x2 |
| Trick 146 (keymap) | mode 62/6 at 3.38-3.45 s sim, done at 4.53-4.76 s sim; moves 0.017-0.020 m; max abs pitch 0.31, roll 0.22 (run 7) | 1-7 |
| Sit 143 from 3/0 | 4.6-7.4 s wall, ends at 21/3 with z 0.176-0.177, pitch -0.439 to -0.444 | 1-7 |

The final pass (8 of 8 passed, CPU 45 % then 20 %) drove and turned at RTF 0.32 and 0.37, so it moved less: 0.079 m in 0.64 s sim and 0.19 rad in 0.73 s sim. Its other numbers fall inside the ranges above, except two. The dead-man END came 0.397 s after the last POST, and the turn reached rest 1.02 s sim after the END.

### Pitch sign for "testa su"

The app's "testa su" sends API pitch **-0.15**. The ZYX pitch goes from about 0 to **-0.1405 to -0.1409**, which is nose up: the nose rises 0.140 m per metre of body length. So `PITCH_UP_SIGN` is -1, as in the deltas table above. Reset brings the pitch back to between -0.017 and -0.030 within 0.48-0.78 s wall. Measured in runs 1-7.

### ESTOP during a walk

All of this was measured with the open-source `cyberdog_control` from the Xiaomi sources. **The dog's firmware may behave differently, and that is not verified.**

**Through the app** (runs 1-7, walking 11/27 at 0.21-0.26 m/s):

- The controller reports mode 0 at **0.12-0.19 s** wall after the POST.
- The SDK sends its second ESTOP 0.318-0.346 s after the POST.
- The emulator's servo watchdog sends WalkStand 11/1 once between the two ESTOPs, and never after the second.
- The controller stays at (0,0) and reports switch_status 5 (BAN_TRANS) once the WalkStand arrives.
- The second ESTOP is therefore refused with code **3022** (runs 2-7). `estop()` still returns the first, successful result.
- z falls below 0.15 m at 0.22-0.28 s sim and ends at 0.064-0.098 m. The robot slides **0.10-0.26 m**.

**ESTOP latency, POST to the LCM frame** (`estop_app_lat.json`, matched against an LCM capture):

- The first ESTOP of an app process takes **0.109 s**. Later ones take 0.006-0.007 s.
- The cause is that `app2/server.py` never calls `dog.connect()`, so the priority channel stays IDLE until it is first used.
- An SDK client that does call `connect()` takes 0.004-0.015 s, even after 10 s idle.

**Raw LCM** (`test_09`, emulator down, `raw_a_run1.json`). Walk 11/27 at 0.10 m/s with step_height 0.05, then ESTOP 0/0, then WalkStand 11/1 after a wall-clock delay. Both the 11/1 and the walk were kept up by the 20 Hz heartbeat, except in the "fresh" row, which sent 21 new frames over 1 s. Results were recorded for 2 s sim (3.2-4.1 s wall).

| 11/1 after ESTOP (wall) | 11/1 seen at (sim) | modes after the first 0 | switch | z below 0.15 at (sim) | z end (m) |
|---|---|---|---|---|---|
| 0.225 s | 0.098 s | (0,0) only | 0 -> 5 | 0.160 s | 0.0764 |
| 0.020 s | 0.014 s | (0,0) only | 0 -> 5 | 0.158 s | 0.0756 |
| 0.100 s | 0.064 s | (0,0) only | 0 -> 5 | 0.154 s | 0.0700 |
| 0.400 s | 0.226 s | (0,0) only | 0 -> 5 | 0.154 s | 0.0777 |
| 0.225 s, standing in place (walk vx 0) | 0.116 s | (0,0) only | 0 -> 5 | 0.152 s | 0.0895, still sinking |
| 0.225 s, fresh frames | 0.120 s | (0,0) only | 0 -> 5 | 0.154 s | 0.0641 |

Mode 0 was reported 0-14 ms sim after the ESTOP was published. The controller log prints "Cannot transition from 0 to 11", but its printing is throttled: it showed 0 or 1 lines per trial, so the log is not a count of refusals. A second run in the final pass (`raw_a_final.json`, RTF 0.64 at the end) repeated it. Again only (0,0) appeared after the first 0 in all six trials. The 11/1 was seen at 0.016-0.334 s sim, z fell below 0.15 at 0.152-0.168 s sim, and z ended at 0.067-0.074 m. The in-place trial ended at 0.112 m, still sinking.

Conclusion for this controller: **a WalkStand sent after an ESTOP does not lift it** (fsm_state_passive.cpp:65-93).

**Through the emulator** (`estop_b1.json`, 3 trials): a raw 1002 stream (303, vx 0.1) at 25 Hz, ONE ESTOP 1004 on the priority channel, then the stream stops with no END.

- In the LCM capture, the last walk frame went out 3-7 ms after the ESTOP call, and the ESTOP 0/0 frame 5-8 ms after it.
- The ESTOP reply came after 18-43 ms with result true.
- The watchdog's WalkStand 11/1 went out **222, 253 and 233 ms** after the ESTOP (walk_stands +1, watchdog_stops +1).
- The controller answered 2-3 ms later with (0,0) and switch 5.
- The latch held, the mode stayed 0 for 2 s sim, and z ended at 0.0675, 0.0784 and 0.0642 m.
