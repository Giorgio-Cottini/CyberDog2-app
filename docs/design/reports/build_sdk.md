**CyberDog v2 part A report: `cyberdog_sdk` and `tests/sdk`**

All 316 SDK unit tests pass on Windows (Python 3.12), and the WSL Python 3.8 compile check is clean. No test opens a connection to the dog. The SDK tests read only one file outside `cyberdog_sdk/` and `tests/sdk/`: `emulator/data/motion_id_map.json`.

**Files written**
- `cyberdog_sdk/__init__.py`
- `cyberdog_sdk/transport.py`
- `cyberdog_sdk/codes.py`
- `cyberdog_sdk/motions.py`
- `cyberdog_sdk/client.py`
- `cyberdog_sdk/servo.py`
- `cyberdog_sdk/camera.py`
- `cyberdog_sdk/vp.py`
- `cyberdog_sdk/nav.py`
- `tests/sdk/conftest.py`: FakeTransport. It keeps the real guards, records every request, and returns scripted replies through `wire`.
- `tests/sdk/test_sdk_wire.py`
- `tests/sdk/test_sdk_transport.py`
- `tests/sdk/test_sdk_motions.py`
- `tests/sdk/test_sdk_client.py`
- `tests/sdk/test_sdk_servo.py`
- `tests/sdk/test_sdk_vp.py`
- `tests/sdk/test_sdk_nav.py`
- `tests/sdk/test_sdk_camera.py`

**Tests and checks**
- `python -m pytest tests/sdk -q` from the repo root: 316 passed, 0 failed, about 11 s, three runs in a row. The servo timing tests were also stable over 5 more runs.
- `wsl.exe -d CyberDogSim -e bash -lc 'cd /mnt/c/<path to the repository> && python3 -m compileall -q cyberdog_sdk'`: printed nothing, exit 0.
- Python 3.8 run in WSL. grpcio was missing there, so a stub `grpc` module (not in the repository) stood in: `PYTHONPATH=<repo>:<stub folder> python3 -m pytest tests/sdk -q --confcutdir=tests/sdk -p no:cacheprovider` gives 316 passed.
- pyflakes 2.1.1 in WSL on `cyberdog_sdk` and `tests/sdk`: clean.
- The generated VP task files (engine header from utils.py:171-198, plus the LED and sensor bodies, plus the footer) are pyflakes-clean under Python 3.8 in WSL. Python 3.6 itself was not available to test.

**Deviations from contracts.md (all additions; no binding signature changed)**
1. **Transport extras:**
   - `call(..., priority=False)` and `call_json(..., force=False, priority=False)`;
   - `call_one()`, which returns the first JSON reply or raises DogError on `error_code` or on no reply;
   - `stream()`, for the 6004/6010 streams;
   - `get_file(..., channel=None)`, so camera workers use their own channels;
   - `new_channel()`, plus module-level `prepare()`, `error_code()`, `as_int()` and `as_float()`;
   - `FileResult` is exported from the package. `error_code` is -1 when no chunk arrives.
2. **More guards in transport.py than §2 assigns to it (§2 lists 1, 5, 6):**
   - transport also enforces rules 2, 3, 4 and 7; rules 2 and 3 are enforced again in nav.py;
   - the 6004 type whitelist cannot be bypassed with `force`; the 6002 delete-all guard can, and `confirm_delete_all` maps to it;
   - two extra guards, with reasons from the specs: 4002 needs an int `command` 0-10 (otherwise the dog defaults to START_RECORDING), and 4004 also refuses `""`, `"."` and names containing NUL (the delete would target `/home/mi/Camera` itself).
3. **Catalog:** besides every §6 id, it also holds the gaits from §5.1 (`needs=("servo",)`) and the postures 201/202/211-214 (`needs=("pose",)`). Registration-only ids (113-115, 203, 390-392, 400) and the custom ids 180-199 are filed as R4, so they never appear in an R0-R2 menu.
4. **`still_running`:** True on 321 when `typical_s > 20` **or** `min_ms >= 20000`. The second condition comes from sim_bridge.md:684, which lists 124 and 138 as returning 321 even when they succeed.
5. **`motion()` checks before sending:**
   - the 3010 duration rule (ValueError);
   - 201/202/211-214 need `pos_des` or `rpy_des`;
   - gaits are refused with "use dog.drive";
   - unexpected keyword arguments raise TypeError;
   - the deadline is `max(timeout, 25)`;
   - `motion(0)` goes on the priority channel;
   - 202 with duration 0 raises ValueError: the dog never pops it and adds `rpy_des` again every 2 s (fsm_state_balancestand.cpp:338-396, command_interface.cpp:222), found by the emulator fidelity review;
   - while `estop()` runs, a motion other than 0 is not sent and returns code `NOT_SENT_ESTOP` (-2).
6. **`trick()`** also refuses servo and pose ids. Because the rule is applied as written, `trick(143)` while already sitting sends 111 and then 143. No step goes out once an ESTOP was sent, or `estop()` began, after `trick()` was called (step-7 review: an ESTOP during its status read let a 111 clear the latch, then the trick ran).
7. **`estop()` always sends ESTOP twice.**
   - It signals both streams to halt (no END), then sends the first ESTOP immediately.
   - A second thread joins the Driver thread that the halt stopped (never one started later), then sends the second ESTOP at `max(first sent, that thread's end) + 0.3 s`. It does not wait for the first reply.
   - Reason: the servo watchdog sends WalkStand 112 about 200-250 ms after the last frame, and that call bypasses motion_manager's ESTOP latch (motion_handler.cpp:281-300). The open-source locomotion controller then refuses mode 11 out of ESTOP's passive state (fsm_state_passive.cpp:65-93), so there one ESTOP holds; the dog's firmware is unverified, so the SDK assumes the WalkStand can overwrite the ESTOP. The second ESTOP also covers a first ESTOP lost on the link. The SDK cannot know every frame that reached the dog: a frame past the last halt check, a stream that a `move()` started during `estop()`, or a lost dead-man END. Review round 3 reproduced each of these against the conditional repeat, so the repeat became unconditional.
   - Cost: `estop()` returns after both replies, so it takes at least 0.3 s; a slow first reply (ESTOP's 1004 has a 1000 ms min_exec_time) no longer delays the second ESTOP. The second ESTOP is redundant after a Posture stream (WalkStand is skipped after 201) and harmless: the latch accepts 0.
   - Interrupts (fix-check round, 2026-10-24): a KeyboardInterrupt or SystemExit can land on any line of `estop()`, including the halt, the repeat thread's start and the wait for it. Every step now runs shielded: a BaseException is recorded and raised only after both ESTOPs went out, a send it interrupts is repeated up to `SHIELD_TRIES` (3) times, and when the repeat thread cannot start (or a Ctrl+C lands while it starts) the calling thread sends the repeat itself at the same time point. A lock-free claim makes sure exactly one of the two sends it. Tests: `tests/sdk/test_sdk_estop_round4.py` (`test_ctrl_c_while_the_repeat_thread_starts`, `test_ctrl_c_while_halting_the_stream_still_sends_both`).
   - Main-lane fallback (review round 5): after a link loss each channel reconnects on its own backoff timer, and a channel in backoff fails every RPC at once. The main channel could be back (status reads, arming) while every ESTOP failed on the priority channel. An ESTOP the priority channel refuses with UNAVAILABLE now goes again at once on the main lane; a DEADLINE_EXCEEDED does not, since the link did not answer in 3 s. All channels also retry every 0.5-1 s (`grpc.initial_reconnect_backoff_ms` 500, `grpc.max_reconnect_backoff_ms` 1000; the default grows to 120 s). Tests: `tests/sdk/test_sdk_estop_races.py` (the two `estop` lane tests at the end).
   - Third ESTOP (2026-10-26, after the probe on the real dog): the probe's watchdog trail showed the dog still in mode 303 0.61 s after the last walk frame and in WalkStand 112 at 0.72 s, later than the 200-250 ms in the source. The 10001 status cache may lag, so the true time is unknown. `estop()` now sends a third ESTOP from the repeat thread at the same time point + `ESTOP_LATE_S` (1.0 s), and returns after all three replies (at least 1.0 s). The probe sends its own third ESTOP 1.0 s after the first. Tested on the real dog later on 2026-10-26: an emergency stop while the dog walked kept the dog down.
8. **Driver and Posture:**
   - the 0.15 s pause also applies when a stream restarts after its own END, not only on a Driver/Posture switch;
   - `set_gait` accepts only the four `Gait` members;
   - extras: `Driver.velocity()`, `halt()`, `join()`, `Posture.stop()`, `Posture.active`, `Posture.stats` and `Posture.target`;
   - Posture sends the target directly with no ramp, because the spec says the firmware limits speed;
   - when the Driver starts, Posture's target resets to z 0.235, rpy 0.
9. **`save_map(name)`** returns a NavTask, as in the contract. It wraps the 6009 `{"type":5,"map_name":name}` call (up to 65 s) on the task's thread, and the outcome is in `NavTask.result`. Other Nav extras: `attach(task_type)` (6010), `state(timeout=3.0)`, an `outdoor=False` keyword on the start methods, and `decode_state()`.
10. **`device_info()`:**
    - with no flags it sends bat_info, stand and wifi;
    - flags are accepted with or without the `is_` prefix;
    - `uid` is refused;
    - the timeout rises to at least 30 s when "version" is asked.
11. **`MotionResult.message` is in English.** Italian is available through `codes.describe(code, "it")`.
12. **`led()` effect names:**
    - head/tail effects that take a color: `on`, `blink`, `blink_fast`, `breath`, `breath_fast`, `one_by_one`, `one_by_one_fast`, `back_and_forth`, `trailing_race`; without a color they play white;
    - head/tail effects that take no color: `off` and `<red|blue|yellow>_<style>`, for example `red_blink`;
    - the mini LED takes `circular_breath` (also reached with "on") or `circular_ring` with a color, and `off`, `rectangle_color`, `centre_color`, `three_circular` or `one_by_one` without;
    - `seconds` is clamped to 0.1-600, since the task's `shutdown()` frees the LED.
13. **`read_sensors()`:**
    - each sensor read uses `get_data(2)`;
    - the result file is written as `.tmp` and then renamed;
    - 4004 is polled every 0.75 s;
    - it raises DogError carrying the last getFile error code on timeout;
    - task ids are `pc_<ms>_<seq>_<rand>` and are validated against `[A-Za-z0-9_]+`.
14. **Timing clock:** servo timing uses `time.perf_counter`, because `time.monotonic()` on Windows 3.12 is GetTickCount64 with 15.6 ms resolution.
15. **Camera stop (fix-check round):** `stop()` no longer waits behind a START, which can take 10 s on the dog. A START that answers after `stop()` is followed by another STOP, so STOP is always the last 4002 command, and `start()` counts fps from zero again. Review round 5: `start()` drops the frame of the previous run, and a pull that was in flight across `stop()`/`start()` publishes nothing, so a viewer never gets an old image as live; `wait()` with no frame of this run waits instead of returning at once. A failed `start()` (cert files unreadable, no thread left) sends STOP and leaves `running` false, so the next `start()` tries again. A STOP the dog refuses (result other than 0 or 5) is sent again after 0.5 s, 3 sends in all, and the result goes to `last_error`. Fix-check of round 5: each STOP send first checks that no `start()` came in the gap, since a STOP after it would end the new stream. Tests: `tests/sdk/test_sdk_camera.py`.
16. **Probe (`tools/probe_dog.py`, fix-check round):** Ctrl+C during the moving part sends ESTOP twice, the second timed from the first send, and retries for up to 6 s until one is delivered. If none is delivered, the probe exits with code 4 and tells the person to keep away and stop the dog with the v1 app. Any "no" answer or refused move stops the moving part. An unread battery skips it. The floor question takes only the word "libero", and accents and case are ignored in answers. The skin and camera steps clean up on Ctrl+C, and the results file is written atomically. Review round 5: before each moving step the probe checks that the ESTOP channel is connected (2 s), and stops with code 3 when it is not; after Ctrl+C it warns as soon as the second ESTOP is not confirmed; the final message is printed before the save, and a save that fails (OneDrive or the antivirus holding the file) is reported and does not end the run; ESTOP timing uses the monotonic clock; waits on Windows run in short slices so a Ctrl+C lands within 0.2 s; an error in `main()` still ends through `os._exit`. Fix-check of round 5: nothing is printed before the first ESTOP, and every print on the stop path ignores a broken stdout. With stdout piped into a reader that died with the same Ctrl+C (`| tee`, `| more`, `| findstr`), the earlier "ARRESTO DI EMERGENZA: invio..." print raised OSError and no ESTOP was sent (reproduced 3 of 3 with a real console Ctrl+C during a 1004). `--help` and argument errors keep their exit codes 0 and 2. Tests: `tests/tools/test_probe_flow.py`.

**Open issues**
1. **Resolved 2026-10-24: the Driver has a dead-man.** Without a new `move()` for `deadman_s` (default 0.5 s, counted from the later of the last `move()` and the stream start) it sends one END and stops. `deadman_s = None` turns it off for scripts. Tests: the dead-man block in `tests/sdk/test_sdk_servo.py`. An adversarial review (three lenses) then found and I fixed: a gait change straddling the deadline sent a new-gait frame after expiry; a pending gait made a restarted stream open with a spurious END; an END could follow halt(); a stream that sent no frame still sent END (WalkStand 112); a `set_gait()` during the switch pause was lost (pre-existing); `estop()` sent one ESTOP after a recent stream end whose END may have been lost. The END can come up to FRAME_DEADLINE after the deadline on a slow link (documented). A second round found and I fixed: halt() was lost while move() or Posture.set() was restarting a stream (older than the dead-man, but the dead-man made restarts routine), now cancelled through a halt generation; Posture.set() was dropped when the handover took longer than POSE_IDLE_END. A third round (generation lens and sweep lens) found and I fixed: a `move()` that began while `halt()` was preempted was eaten by the doomed thread; a cancelled `move()` or `set()` still wrote its velocity or target; `estop()` could join a stream started after the halt, or a thread not yet started (RuntimeError); the repeat window was measured after the first reply; a bad `step_height` or any exception in the stream thread killed it with no END and no error recorded; `close()` did not stop a stream that a concurrent `move()` was starting, and a `move()` after `close()` streamed into a closed channel; the walking reset of the held pose could be undone by a concurrent `set()`; `max_gap_ms` counted failed frames as sent. Tests: `tests/sdk/test_sdk_estop_races.py` (22 tests; 19 fail on the round-2 code, the 3 that pass check behaviour round 2 already had). Kept and documented: after a stall of one to two periods one frame follows the slow one closely (the loop resyncs only when more than one period behind), and no END goes out at interpreter exit (the stream threads are daemons; the dog's watchdog stops the walk about 250 ms later). A fourth round (estop-timing, generation and lifecycle lenses, each finding re-run by a skeptic) found and I fixed: a stream started during `estop()` kept feeding the watchdog until the first ESTOP latched, so a first ESTOP 0.15 s late or lost let a WalkStand land after the second (now no stream starts or sends while `estop()` runs); `close()` during `estop()` closed the channel under the second ESTOP (now it waits); a Ctrl+C during the first ESTOP's reply, or during the wait for the second, left the repeat to a daemon thread that died at exit (now the repeat thread is not a daemon, and the interrupt is raised after both ESTOPs); a `set()` or `move()` cancelled inside the stream start still left its target or velocity; a `set()` during a Driver start was wiped by the walking reset; a `halt()` preempted after reading the old stream's flags could flood or silently stop a stream started after it (the flags now belong to each stream run); a failing crash END left `last_error` empty; a `move()` during the crash END was lost; `step_height` and `move()` accepted bool, str and bytes, and raised OverflowError on huge ints. The skeptic rated the four estop-timing findings medium or low under the WalkStand-overwrite threat model and harmless under the open-source controller. Rejected by the skeptics: `estop()` returning None (needs a BaseException no worker thread can get) and suppressing END after a frame that timed out (an END after a lost DATA is safer). Tests: `tests/sdk/test_sdk_estop_round4.py` (40 tests; 28 fail on the round-3 code, 12 are controls).
2. **ESTOP during a stream relies on unverified firmware behaviour.** The watchdog's WalkStand bypasses motion_manager's latch (from the code); the open-source controller refuses it out of ESTOP (fsm_state_passive.cpp:65-93); whether the dog's firmware does the same is unverified. Verify on the dog.
3. **`typical_s` is unknown for many ids.** `still_running` falls back to `min_ms`.
4. **VP depends on the dog:**
   - the engine must be running and initialised (at, cron and pyflakes present);
   - whether the at-job user can write `/home/mi/Camera` is unverified;
   - whether a new debug task preempts one still running is unverified, which affects `led_off()` during a `led()` hold;
   - LED response codes are not observable from the PC.
5. **Sensor polling and the camera share the dog's transfer limit.** Each 4004 poll counts toward it, so `read_sensors()` while the 4-worker camera runs can briefly push camera pulls to error 10.
6. **Resolved: grpcio 1.70.0 is installed in the CyberDogSim WSL distro.** `tests/sdk` passes on Python 3.8.10 there (419 tests, 2026-10-24).
7. **`Posture.stop()` right after `set()` can lose the new target.** It may send END before the target was streamed (END frames are not executed). Only internal paths call it.
8. **Pitch sign is unresolved** (motion.md §4 conflict). Posture passes pitch through unchanged.
9. **1001 with many flags plus version can take about 45 s** on the dog; the SDK default stays 15 s, or 30 s with version.
10. **The "return to dock from afar" path is not reachable.** It is 6004 type 1 without goalX/goalY, which contract rule 2 refuses.
11. **Partly resolved 2026-10-24: the ESTOP channel stays connected.** A channel connects only when first used, so a client that never called `connect()` sent its first ESTOP over a fresh TCP + mTLS connection: 84-134 ms against the emulator and about 109 ms in Gazebo, against 4-12 ms on a connected channel (`tests/integration/test_int_safety.py::test_connect_leaves_the_estop_channel_ready`). Now `transport.py` sets `grpc.client_idle_timeout_ms` to its maximum, so a connected channel never falls idle, and `app2/server.py` calls `dog.connect()` from a warm-up thread at start, after each client rebuild and every 30 s. Still open: no keepalive, so a connection the dog drops silently is found only by the next RPC; a keepalive needs the dog's gRPC ping policy, which is unknown. Scripts must call `connect()` themselves.
12. **A 1004 already on the link when `estop()` begins can clear the latch.** If a 111 sent just before the ESTOP reaches the dog after both ESTOPs (the main channel more than about 0.3 s slower than the priority channel), the dog stands up from ESTOP. `trick()` and `motion()` refuse to send once `estop()` began, which closes the window the step-7 review measured; this residue needs a reply-ordered ESTOP and is not handled.
13. **Emulator gaps from the fidelity review (low):** 3002 and 8013 are not modelled; 6009 type 7 ends relocalization in the emulator, a no-op on the dog; the VP build check misses unused AUTO imports and unused locals; VP `get_data` returns at once with code 0, where the dog waits and gives 7944 on timeout; motion 202 ignores the `rpy_des` yaw and the `pos_des` offsets; a 6009 type 0 (ResetNav) answers at once, where the dog takes 2 s or more, so the app's late nav-stop path runs only in the stub tests.
14. **Map yaw is unverified.** The UI now turns the grid by the 6003 yaw (nav_map.md: yaw = 2*atan2(qz, qw)); every map seen so far has yaw 0, and the yaml writer was not read.
15. **A Ctrl+C can still land in a few unshielded bytecodes of `estop()` (review round 5, LOW).** On the main thread, a KeyboardInterrupt raised at the entry of `_shielded`, of `_estop_once` before its `try`, or inside `threading.Event()` leaves `estop()` with zero ESTOPs sent (or one, from the repeat thread, 0.3 s later). The window is a few bytecodes inside a call of 0.3-1 s (about 1 in 100,000 presses). It concerns only a script that calls `estop()` on its main thread: app2 calls it from HTTP handler threads, which never get KeyboardInterrupt, and the probe has its own ESTOP code. Closing it needs a SIGINT handler installed for the duration of `estop()`, which changes process-wide state; not done.
16. **Encoding a large map holds the GIL (review round 5, LOW).** Python's JSON encoder and decoder never release the GIL: about 20-30 ms each for a 160,000-cell home map, about 100 ms for 1,000,000 cells. An ESTOP that arrives during `/api/map` waits that long. Only the active tab reads the map, and only on load, on a change into or out of mapping, after a save and on retry. Not changed.
