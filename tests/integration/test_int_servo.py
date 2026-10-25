"""1002 servo streams: Driver gaits, gait change, direction, END -> WalkStand, watchdog, dead-man, ESTOP
during a walk, and Posture (201) against the emulator's ground truth (motion.md 3-5)."""
from __future__ import annotations

import math
import time

from cyberdog_sdk import Gait
from cyberdog_sdk.codes import SERVO_DATA, SERVO_END
from cyberdog_sdk.servo import SWITCH_PAUSE


def walk(dog, vx, vy, wz, seconds):
    """Hold a key: move() every 0.1 s (the app resends every <= 150 ms), then stop()."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        dog.drive.move(vx, vy, wz)
        time.sleep(0.1)
    dog.drive.stop()


def pose(ctl):
    g = ctl.state()["ground_truth"]
    return g["x"], g["y"], g["yaw"]


def test_each_gait_walks_forward_and_stops_with_walkstand(standing, ctl):
    for n, gait in enumerate((Gait.SLOW, Gait.MEDIUM, Gait.FAST, Gait.AUTO), 1):
        standing.drive.set_gait(gait)
        x0 = ctl.state()["ground_truth"]["x"]
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            standing.drive.move(0.3, 0.0, 0.0)
            time.sleep(0.1)
        s = ctl.wait(lambda s: s["motion"]["motion_id"] == int(gait), timeout=2)
        assert s["servo"]["last_frame"]["vel_des"] == [0.3, 0.0, 0.0]
        assert s["servo"]["last_frame"]["step_height"] == [0.05, 0.05]
        standing.drive.stop()
        assert not standing.drive.active
        s = ctl.wait(lambda s: s["servo"]["walk_stands"] == n and s["motion"]["motion_id"] == 112, timeout=2)
        assert s["ground_truth"]["x"] - x0 > 0.1, gait
        assert s["servo"]["watchdog_stops"] == 0 and not s["motion"]["busy"]
    assert standing.drive.stats["errors"] == 0


def test_gait_change_sends_end_pauses_then_restarts(standing, ctl):
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        standing.drive.move(0.2, 0.0, 0.0)
        time.sleep(0.1)
    ctl.wait(lambda s: s["motion"]["motion_id"] == 303, timeout=2)
    standing.drive.set_gait(Gait.MEDIUM)
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        standing.drive.move(0.2, 0.0, 0.0)
        time.sleep(0.1)
    s = ctl.wait(lambda s: s["motion"]["motion_id"] == 308, timeout=2)
    assert s["servo"]["walk_stands"] == 1               # the old gait's END ran WalkStand 112 (MH:251-300)
    standing.drive.stop()

    frames = standing.rec.frames()
    i = next(k for k, f in enumerate(frames) if f[1:] == (303, SERVO_END))
    assert all(f[1:] == (303, SERVO_DATA) for f in frames[:i])
    assert frames[i + 1][1:] == (308, SERVO_DATA)
    assert frames[i + 1][0] - frames[i][0] >= SWITCH_PAUSE - 0.005
    assert all(f[1] == 308 for f in frames[i + 1:])
    assert frames[-1][1:] == (308, SERVO_END)
    ctl.wait(lambda s: s["servo"]["walk_stands"] == 2, timeout=2)


def test_pose_moves_in_the_commanded_direction(standing, ctl):
    x0, y0, _ = pose(ctl)
    walk(standing, 0.3, 0.0, 0.0, 0.8)                  # W: forward
    x1, y1, yaw1 = pose(ctl)
    assert x1 - x0 > 0.1 and abs(y1 - y0) < 0.03

    walk(standing, 0.0, 0.0, 0.5, 0.8)                  # A: turn left (+wz)
    x2, y2, yaw2 = pose(ctl)
    assert yaw2 - yaw1 > 0.15

    walk(standing, 0.0, 0.15, 0.0, 0.8)                 # Q: strafe left (+vy), in the body frame
    x3, y3, yaw3 = pose(ctl)
    left = -math.sin(yaw3) * (x3 - x2) + math.cos(yaw3) * (y3 - y2)
    assert left > 0.04

    walk(standing, -0.2, 0.0, 0.0, 0.8)                 # S: back
    x4, y4, yaw4 = pose(ctl)
    fwd = math.cos(yaw4) * (x4 - x3) + math.sin(yaw4) * (y4 - y3)
    assert fwd < -0.05


def test_watchdog_when_frames_stop(standing, ctl):
    standing.drive.deadman_s = None
    standing.drive.move(0.2, 0.0, 0.0)
    ctl.wait(lambda s: s["servo"]["frames"] >= 5 and s["motion"]["motion_id"] == 303, timeout=3)
    standing.drive.halt()                               # the link drops: no END reaches the dog
    standing.drive.join(1.0)
    s = ctl.wait(lambda s: s["servo"]["watchdog_stops"] == 1, timeout=2)
    assert s["servo"]["walk_stands"] == 1               # 4 missed 50 ms ticks: StopServoCmd + WalkStand
    ctl.wait(lambda s: s["motion"]["motion_id"] == 112 and not s["motion"]["busy"], timeout=2)
    assert all(f[2] == SERVO_DATA for f in standing.rec.frames())


def test_deadman_sends_one_end(standing, ctl):
    standing.drive.move(0.2, 0.0, 0.0)                  # then the caller goes silent
    t0 = time.perf_counter()
    s = ctl.wait(lambda s: s["servo"]["walk_stands"] == 1, timeout=3)
    assert s["servo"]["watchdog_stops"] == 0            # an END, not the watchdog, stopped it
    standing.drive.join(1.0)
    assert not standing.drive.active and standing.drive.stats["deadman_stops"] == 1
    ends = [f for f in standing.rec.frames() if f[2] == SERVO_END]
    assert len(ends) == 1 and ends[0][0] - t0 >= standing.drive.deadman_s - 0.01


def test_estop_during_a_walk_then_recovery(standing, ctl):
    standing.drive.move(0.3, 0.0, 0.0)
    ctl.wait(lambda s: s["servo"]["frames"] >= 3 and s["motion"]["motion_id"] == 303, timeout=3)
    t0 = time.monotonic()
    r = standing.estop()
    assert time.monotonic() - t0 < 2.0              # the third ESTOP goes at +1.0 s
    assert (r.motion_id, r.result, r.code) == (0, True, 0)
    assert not standing.drive.active
    # ESTOP three times: the watchdog's WalkStand 112 overwrites the first one 200-250 ms later (motion.md 3)
    t_estop = [t for t, p in standing.rec.of(1004) if p["motion_id"] == 0]
    assert len(t_estop) == 3
    frames = standing.rec.frames()
    assert all(f[2] == SERVO_DATA for f in frames)      # no END after an ESTOP
    assert all(f[0] < t_estop[0] for f in frames)       # nothing streamed after the first ESTOP
    s = ctl.wait(lambda s: s["servo"]["watchdog_stops"] == 1, timeout=2)
    assert s["motion"]["estop_latched"] and s["motion"]["motion_id"] == 0
    time.sleep(0.3)
    assert ctl.state()["motion"]["motion_id"] == 0      # still in ESTOP, not walking or standing

    assert standing.stand().result
    s = ctl.wait(lambda s: s["motion"]["motion_id"] == 111 and not s["motion"]["busy"])
    assert not s["motion"]["estop_latched"]
    walk(standing, 0.2, 0.0, 0.0, 0.4)                  # walking works again after the recovery
    ctl.wait(lambda s: s["servo"]["walk_stands"] == 2, timeout=2)


def test_posture_set_and_reset_in_ground_truth(standing, ctl):
    standing.posture.set(z=0.20, pitch=0.15)
    standing.posture.join(3.0)                          # streams 0.6 s after the last change, then END
    assert not standing.posture.active
    s = ctl.state()
    g = s["ground_truth"]
    assert s["motion"]["motion_id"] == 201
    assert abs(g["z"] - 0.20) < 0.01
    assert 0.12 < g["pitch"] < 0.16                     # +201 pitch lowers the nose (contracts.md 6.1, sim_baseline)
    assert abs(g["roll"]) < 0.01
    assert s["servo"]["walk_stands"] == 0               # END after 201 skips WalkStand: the pose is held
    frames = standing.rec.frames()
    assert {f[1] for f in frames} == {201} and frames[-1][2] == SERVO_END
    last = standing.rec.of(1002)[-1][1]
    assert last["pos_des"] == [0.0, 0.0, 0.2] and last["rpy_des"] == [0.0, 0.15, 0.0]

    standing.posture.set(roll=0.2)                      # absolute: z and pitch keep their targets
    standing.posture.join(3.0)
    g = ctl.state()["ground_truth"]
    assert g["roll"] > 0.1 and abs(g["z"] - 0.20) < 0.01 and g["pitch"] > 0.1

    standing.posture.set(z=0.5)                         # clamped by the SDK to 0.30
    standing.posture.join(3.0)
    assert ctl.state()["servo"]["last_frame"]["pos_des"] == [0.0, 0.0, 0.3]
    assert abs(ctl.state()["ground_truth"]["z"] - 0.30) < 0.01

    standing.posture.reset()
    standing.posture.join(3.0)
    g = ctl.state()["ground_truth"]
    assert abs(g["z"] - 0.235) < 0.01 and abs(g["roll"]) < 0.01 and abs(g["pitch"]) < 0.01


def test_driver_and_posture_never_stream_together(standing, ctl):
    standing.drive.move(0.2, 0.0, 0.0)
    ctl.wait(lambda s: s["motion"]["motion_id"] == 303, timeout=3)
    standing.posture.set(z=0.25)                        # stops the Driver first: END, pause, then 201
    assert not standing.drive.active
    standing.posture.join(3.0)
    frames = standing.rec.frames()
    i = next(k for k, f in enumerate(frames) if f[1] == 201)
    assert frames[i - 1][1:] == (303, SERVO_END)
    assert frames[i][0] - frames[i - 1][0] >= SWITCH_PAUSE - 0.005
    assert all(f[1] == 201 for f in frames[i:])
    s = ctl.wait(lambda s: s["motion"]["motion_id"] == 201, timeout=2)
    assert s["servo"]["walk_stands"] == 1
    assert abs(s["ground_truth"]["z"] - 0.25) < 0.01


def test_estop_during_a_posture_stream(standing, ctl):
    standing.posture.set(z=0.22, pitch=0.1)
    ctl.wait(lambda s: s["motion"]["motion_id"] == 201, timeout=3)
    r = standing.estop()
    assert (r.motion_id, r.result, r.code) == (0, True, 0)
    assert not standing.posture.active
    assert [p["motion_id"] for _, p in standing.rec.of(1004)].count(0) == 3   # always three times
    s = ctl.wait(lambda s: s["servo"]["watchdog_stops"] == 1, timeout=2)
    assert s["servo"]["walk_stands"] == 0
    assert s["motion"]["motion_id"] == 0 and s["motion"]["estop_latched"]
    assert all(f[2] == SERVO_DATA for f in standing.rec.frames())
