**Part C report (LCM controller link)**

**Files written**
- `emulator/controllers/lcm_link.py`: `LcmController` with `publish`, `on_response`, `ground_truth` and `close`.
  - URLs: cmd 7671, resp 7670, sim 7667 `?ttl=0`.
  - The only write is `robot_control_cmd`; it never publishes `motor_ctrl`.
  - It masks value bit 0x04 and wraps `life_count` to int8. Arrays that are the wrong size are zero-filled.
  - Attitude comes from `quat` (ZYX); velocities `v` and `omega` are in the world frame.
- `tests/gazebo/test_lcm_link.py`: 10 tests, marker `gazebo`.
  - It loads `lcm_link` by path and has a 20 Hz heartbeat that resends the same `life_count`.
  - Waits are budgeted in sim time, except the PureDamper timeout, which is wall-clock.
  - It skips on Windows, without lcm, without the multicast route on `lo`, without sim traffic, or when another process is already publishing `robot_control_cmd`.
  - Setting `LCM_BASELINE_OUT=<json>` dumps the measurements.
- `docs/design/sim_baseline.md`.
- In the VM: `/root/emu_up.sh` and `/root/emu_down.sh`. Raw data is in `/root/sim_logs/lcm_baseline_run{1,2,3}.json`.

**Verified**
- **Compile and lint:** both files compile on Python 3.12 (Windows) and 3.8 (WSL); `pyflakes` is clean.
- **Encoding, checked offline in WSL with the publish captured:**
  - Every frame is 124 bytes.
  - `life_count` wraps correctly: 126→126, 127→127, 128→-128, 255→-1, 256→0, -1→-1.
  - `value` 7 goes out as 3.
  - A short `step_height` becomes `[0,0]`.
  - `mode=200` raises `ValueError`.
  - A publish after `close()` is dropped.
  - `ground_truth()` returns all None before the first frame.
- **Without the sim:**
  - On Windows, importing the module gives a clear `ImportError` and `python -m pytest tests/gazebo/test_lcm_link.py` gives 10 skipped.
  - In WSL with the sim down, all 10 are skipped with "no simulator_state / robot_control_response".
- **Gazebo,** with `wsl.exe -d CyberDogSim -e bash -lc 'cd <repo> && PYTHONDONTWRITEBYTECODE=1 LCM_BASELINE_OUT=... python3 -m pytest -v tests/gazebo/test_lcm_link.py'` (pytest 4.6.9 was already installed, so nothing was pip-installed):
  - Run 1: 9 passed. `test_posture_211` failed because its wait was wall-clock; I changed it to a sim-time budget.
  - Run 2: 10 passed in 62 s.
  - Run 3: 10 passed in 64 s.
- **Emulator scripts:**
  - Both pass `bash -n`.
  - I ran a copy of `emu_up.sh` pointed at a dummy `emulator/__main__.py` in `/root/emu_script_check`, then deleted it. It started detached in its own session (pid = pgid = sid), got the right arguments, saw the control port answer, and refused a second start. `emu_down.sh` killed it and removed the pid file.
  - The real emulator was not tested because `emulator/__main__.py` does not exist yet.
- **Cleanup:** I stopped Gazebo with `/root/sim_down.sh` and confirmed no gzserver, controller or python3 process is left. Nothing was committed and I created no `__pycache__` in the repo. `tests/sdk/__pycache__` (cpython-38, 19:50) belongs to another engineer.

**Baseline**

Sim time, three runs; method for each row is in `sim_baseline.md`.

| Item | Measured |
|---|---|
| RTF (`FAST=1`, under load) | varied 0.38–0.66 |
| Lying after 101 [7,1] | z 0.0542 m, contact bits 0 |
| Boot spawn (mode 0) | z 0.073 m, pitch +0.166 |
| Standing after 111 [12,0] | z 0.2353 m, contact 15 |
| Walking stance | z 0.25 m |
| 111 from lying | bar 100 at 2.02 s; z at 95 % of the rise at 1.88 s |
| 101 lie-down | bar 100 at 1.20 s; z at 95 % at 1.50–1.57 s |
| 303 walk, vel_des 0.3 m/s | mean 0.28–0.29 m/s over the window, steady 0.32 m/s |
| 303 turn, wz 0.5 rad/s | mean 0.37–0.38 rad/s, steady 0.40 rad/s |
| Stop with 112 | still after 0.35–0.63 s |
| 211 [21,5], z 0.20 | settles at z 0.1925 |
| 211 sent from 11/1 | 0.30 s of switch_status 1 first; the 21/3 filler then appears at 1.30 s and repeats every 0.4 s with its bar cycling 0→100 |
| 212 [21,0], pitch +0.2 | ZYX pitch changes by -0.203, so the nose goes up; the command is relative (sending -0.2 restores it) |
| 201 [3,0], absolute | z tracks within 1 mm; pitch +0.15 → +0.145 (nose down); roll +0.15 → +0.136 |
| Mode 23 | BAN_TRANS (5) in the next response; mode and height unchanged |
| PureDamper | mode 7 at 0.50–0.555 s wall after the last frame |

**The 201 and 212 pitch signs are opposite.** A positive pitch lowers the nose in 201 and raises it in 212.

If the heartbeat restarts with the same `life_count` after the timeout, the controller stays in 7/0.

**Deviations from contracts.md**
1. `LcmController` adds a random `life_offset` (mod 256) to every `life_count`. The controller remembers its last `life_count` across emulator restarts, and the manager always restarts at 0, so a restart's first frame could be ignored. Equal counts stay equal, so heartbeat duplicates are still duplicates.
2. `ground_truth()["t"]` is sim seconds since the link opened (frame count × 2 ms), because the plugin leaves `time` empty. All values are None before the first frame.
3. Methods beyond 4.1: `last_response()`, `sim_time()`, `rtf()` and `stats()`. There is no `reset()`; the manager's `hasattr` check skips it.
4. `publish` raises `ValueError` on int8/int32 overflow and masks bit 0x04 itself as well as the manager doing it.
5. `emu_up.sh` also reapplies the `lo` multicast route, sets `PYTHONDONTWRITEBYTECODE=1` so no cpython-38 bytecode lands in the checkout, accepts `PORT` and `CPORT` overrides, and truncates `emu.log` at each start.

**Open issues**
1. **Manager timeouts versus RTF.** contracts.md 4.1 puts the manager timeouts on wall-clock. A 211 or 212 with duration 1000 gets a 1.5 s wall timeout, but needs 1.0–1.3 s of sim time, which is 2–3.4 s of wall time at these RTFs. With `--controller lcm` they will fail with 3028 (kMotionExecuteTimeout). The manager should divide its timeouts by `ctl.rtf()`, as the `timeout / RTF` in the sim_bridge.md §2.6 pseudo-code does. That is for the owner of `emulator/manager.py` or `__main__.py`.
2. **`fake.py` has the wrong sign for 21/0 pitch** (it moves by +rpy_des[1]; Gazebo moves by -rpy_des[1]). Other calibration gaps:
   - `LIE_Z` is 0.09 against 0.054 measured.
   - The 111 length is 6.0 s against 2.0 s; 101 is 3.0 s against 1.2 s.
   - 3/0 with pos_des z 0 goes to STAND_Z in the fake; the code commands 0.15 m (not measured).
   - The fake moves from 11 to 21 at once; Gazebo shows 0.3 s of switch_status 1 first.
   - The repeating 21/3 filler is missing from the fake.

   The full table is in `sim_baseline.md`.
3. **SDK and app posture sign.** `Posture` streams 201, where a positive pitch lowers the nose. 212 is the opposite. The SDK, the app labels and the emulator have to agree on this.
4. **The tests and the emulator cannot share the sim.** The tests skip when `robot_control_cmd` traffic is present, so run `emu_down.sh` first.
5. **RTF is lower than the contract says.** It ran 0.38–0.66 while another job loaded the CPU, against the 0.6 in contracts.md §5.

**sim_bridge.md versus the real controller:** nothing was contradicted. These were confirmed:
- 0.5 s wall-clock timeout to PureDamper.
- BAN_TRANS 5 for mode 23.
- Filler 21/3 after a timed 21/5.
- 212 is relative and changes pitch by about 0.2 rad.
- 50 Hz responses.
- `simulator_state` arrives on the inferred `ttl=0` URL.

These things are new, beyond what the spec says:
- The idle filler repeats every 0.4 s with its bar cycling, rather than being inserted once.
- A switch from 11 to 21 reports switch_status 1, with a brief (11,5), for 0.3 s.
- The pitch sign: positive pitch in 21/0 raises the nose, while in 3/0 it lowers it.
- A heartbeat that resumes after the timeout does not restart the old command.