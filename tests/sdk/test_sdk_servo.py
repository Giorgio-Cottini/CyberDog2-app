"""Driver (1002 walk stream) and Posture (201 stream): framing, rate, clamping, END rules."""
from __future__ import annotations

import json
import sys
import threading
import time

import pytest

from cyberdog_sdk import Gait
from cyberdog_sdk.servo import (DEADMAN_S, FRAME_DEADLINE, POSE_DEFAULT, POSE_IDLE_END, SWITCH_PAUSE, Driver,
                                clock)


def frames(fake, cmd_type=None):
    out = fake.of(1002)
    return [s for s in out if cmd_type is None or s.params["cmd_type"] == cmd_type]


def wait_until(cond, timeout=2.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


# ---- Driver -------------------------------------------------------------------------------
def test_driver_frame_is_exact(dog, fake):
    dog.drive.move(0.3, 0, 0)
    assert wait_until(lambda: frames(fake))
    dog.drive.stop()
    first = frames(fake)[0]
    assert first.raw == ('{"motion_id":303,"cmd_type":1,"vel_des":[0.3,0.0,0.0],"rpy_des":[0.0,0.0,0.0],'
                         '"pos_des":[0.0,0.0,0.0],"acc_des":[0.0,0.0,0.0,0.0,0.0,0.0],'
                         '"ctrl_point":[0.0,0.0,0.0],"foot_pose":[0.0,0.0,0.0,0.0,0.0,0.0],'
                         '"step_height":[0.05,0.05],"value":0}')
    assert first.timeout == 0.25 and first.lane == "main"


def test_driver_rate_is_25_hz(dog, fake):
    dog.drive.deadman_s = None            # one move() for a 1 s stream
    dog.drive.move(0.2, 0.0, 0.0)
    time.sleep(1.0)
    dog.drive.stop()
    data = frames(fake, 1)
    assert 21 <= len(data) <= 28, len(data)
    span = data[-1].t - data[0].t
    rate = (len(data) - 1) / span
    assert 22.0 <= rate <= 27.5, rate
    gaps = [b.t - a.t for a, b in zip(data, data[1:])]
    assert max(gaps) < 0.1
    st = dog.drive.stats
    assert st["frames"] == len(data) and st["ends"] == 1 and st["errors"] == 0
    assert 0 < st["max_gap_ms"] < 100


def test_one_rpc_per_frame(dog, fake):
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.2)
    dog.drive.stop()
    assert all(len(json.loads(s.raw)) == 10 for s in frames(fake))   # one frame per request


@pytest.mark.parametrize("gait,cmd,expected", [
    (Gait.SLOW, (5, 5, 5), [0.6, 0.2, 1.3]),
    (Gait.SLOW, (-5, -5, -5), [-0.6, -0.2, -1.3]),
    (Gait.MEDIUM, (5, -5, 5), [1.2, -0.4, 2.25]),
    (Gait.FAST, (5, 5, -5), [1.6, 0.55, -2.5]),
    (Gait.FAST, (-5, -5, 5), [-1.2, -0.55, 2.5]),
    (Gait.AUTO, (5, 5, 5), [1.6, 0.6, 2.5]),
    (Gait.AUTO, (-5, -5, -5), [-1.2, -0.6, -2.5]),
    (Gait.SLOW, (0.2, -0.15, 0.4), [0.2, -0.15, 0.4]),
])
def test_driver_clamps_to_gait_limits(dog, fake, gait, cmd, expected):
    dog.drive.set_gait(gait)
    dog.drive.move(*cmd)
    assert wait_until(lambda: frames(fake))
    dog.drive.stop()
    f = frames(fake, 1)[0].params
    assert f["motion_id"] == int(gait) and f["vel_des"] == pytest.approx(expected)


@pytest.mark.parametrize("h,sent", [(0.2, 0.06), (-1.0, 0.0), (0.03, 0.03)])
def test_step_height_clamped_and_doubled(dog, fake, h, sent):
    dog.drive.step_height = h
    dog.drive.move(0.1, 0, 0)
    assert wait_until(lambda: frames(fake))
    dog.drive.stop()
    assert frames(fake, 1)[0].params["step_height"] == [sent, sent]


def test_driver_refuses_bad_input(dog, fake):
    with pytest.raises(ValueError):
        dog.drive.move(float("nan"), 0, 0)
    with pytest.raises(ValueError):
        dog.drive.set_gait(301)
    assert not dog.drive.active and fake.sent == []


def test_stop_sends_exactly_one_end(dog, fake):
    dog.drive.stop()                      # idle: nothing
    assert fake.sent == []
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.15)
    dog.drive.stop()
    dog.drive.stop()
    ends = frames(fake, 2)
    assert len(ends) == 1 and not dog.drive.active
    assert frames(fake)[-1].params["cmd_type"] == 2
    end = ends[0].params
    assert end["motion_id"] == 303 and end["vel_des"] == [0.0, 0.0, 0.0] and end["step_height"] == [0.05, 0.05]
    time.sleep(0.1)
    assert len(frames(fake)) == len(fake.of(1002))          # nothing after stop
    n = len(fake.sent)
    time.sleep(0.1)
    assert len(fake.sent) == n


def test_move_updates_running_stream(dog, fake):
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    dog.drive.move(0.0, 0, 0.4)
    time.sleep(0.1)
    dog.drive.stop()
    vels = [tuple(s.params["vel_des"]) for s in frames(fake, 1)]
    assert vels[0] == (0.1, 0.0, 0.0) and vels[-1] == (0.0, 0.0, 0.4)
    assert len(frames(fake, 2)) == 1


def test_gait_change_sends_end_then_pauses(dog, fake):
    dog.drive.deadman_s = None            # 0.6 s on one move(): the final END must come from stop()
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.2)
    dog.drive.set_gait(Gait.MEDIUM)
    time.sleep(0.4)
    dog.drive.stop()
    seq = [(s.params["motion_id"], s.params["cmd_type"], s.t) for s in frames(fake)]
    kinds = [(m, c) for m, c, _ in seq]
    i = kinds.index((303, 2))
    assert all(k == (303, 1) for k in kinds[:i]) and i >= 3
    assert kinds[i + 1] == (308, 1)
    assert seq[i + 1][2] - seq[i][2] >= SWITCH_PAUSE - 0.01
    assert kinds[-1] == (308, 2) and kinds.count((308, 2)) == 1
    assert dog.drive.gait == Gait.MEDIUM and dog.drive.stats["deadman_stops"] == 0


def test_gait_set_while_idle_is_immediate(dog, fake):
    dog.drive.set_gait(305)
    dog.drive.move(0.1, 0, 0)
    assert wait_until(lambda: frames(fake))
    dog.drive.stop()
    assert {s.params["motion_id"] for s in frames(fake)} == {305}


def test_frame_errors_are_counted_and_stream_continues(dog, fake):
    def boom(rec):
        raise RuntimeError("link down")
    fake.replies[1002] = boom
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.2)
    assert dog.drive.active
    dog.drive.stop()
    st = dog.drive.stats
    assert st["errors"] >= 4 and st["frames"] == 0 and "link down" in st["last_error"]


# ---- Posture ------------------------------------------------------------------------------
def test_posture_frames_and_idle_end(dog, fake):
    t0 = time.perf_counter()
    dog.posture.set(z=0.25, pitch=0.1)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    elapsed = time.perf_counter() - t0
    assert 0.55 <= elapsed <= 1.0
    data, ends = frames(fake, 1), frames(fake, 2)
    assert len(ends) == 1 and 7 <= len(data) <= 11
    f = data[0]
    assert f.raw == ('{"motion_id":201,"cmd_type":1,"vel_des":[0.0,0.0,0.0],"rpy_des":[0.0,0.1,0.0],'
                     '"pos_des":[0.0,0.0,0.25],"acc_des":[0.0,0.0,0.0,0.0,0.0,0.0],'
                     '"ctrl_point":[0.0,0.0,0.0],"foot_pose":[0.0,0.0,0.0,0.0,0.0,0.0],'
                     '"step_height":[0.0,0.0],"value":0}')
    assert ends[0].params["motion_id"] == 201 and ends[0].params["pos_des"] == [0.0, 0.0, 0.25]
    rate = (len(data) - 1) / (data[-1].t - data[0].t)
    assert 12.0 <= rate <= 17.0
    time.sleep(0.2)
    assert len(frames(fake, 2)) == 1 and not dog.posture.active


@pytest.mark.parametrize("kw,key,expected", [
    ({"z": 0.5}, "z", 0.30), ({"z": 0.0}, "z", 0.18), ({"roll": 1.0}, "roll", 0.35),
    ({"roll": -1.0}, "roll", -0.35), ({"pitch": 1.0}, "pitch", 0.25), ({"pitch": -1.0}, "pitch", -0.2),
    ({"yaw": 2.0}, "yaw", 0.5), ({"yaw": -2.0}, "yaw", -0.5), ({"z": 0.2}, "z", 0.2),
])
def test_posture_clamps(dog, fake, kw, key, expected):
    assert dog.posture.set(**kw)[key] == pytest.approx(expected)
    assert wait_until(lambda: frames(fake))
    p = frames(fake, 1)[0].params
    got = {"z": p["pos_des"][2], "roll": p["rpy_des"][0], "pitch": p["rpy_des"][1], "yaw": p["rpy_des"][2]}
    assert got[key] == pytest.approx(expected)
    dog.posture.stop()


def test_posture_set_keeps_other_axes_and_extends_stream(dog, fake):
    dog.posture.set(z=0.25)
    time.sleep(0.4)
    dog.posture.set(yaw=0.3)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    assert len(frames(fake, 2)) == 1
    last = frames(fake, 1)[-1].params
    assert last["pos_des"][2] == 0.25 and last["rpy_des"][2] == 0.3
    # the stream ran ~0.4 s + 0.6 s: no END in between
    data = frames(fake, 1)
    assert data[-1].t - data[0].t >= 0.85


def test_posture_reset(dog, fake):
    dog.posture.set(z=0.2, roll=0.2)
    assert dog.posture.reset() == POSE_DEFAULT
    time.sleep(0.15)
    dog.posture.stop()
    assert frames(fake, 1)[-1].params["pos_des"] == [0.0, 0.0, 0.235]


def test_posture_set_after_end_restarts(dog, fake):
    dog.posture.set(z=0.25)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    dog.posture.set(z=0.22)
    assert wait_until(lambda: len(frames(fake, 2)) == 2, timeout=2.0)
    assert frames(fake, 1)[-1].params["pos_des"][2] == 0.22


# ---- exclusivity --------------------------------------------------------------------------
def test_posture_stops_driver_first(dog, fake):
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.2)
    dog.posture.set(z=0.25)
    assert not dog.drive.active
    seq = frames(fake)
    end_i = next(i for i, s in enumerate(seq) if s.params["cmd_type"] == 2)
    first_pose = next(i for i, s in enumerate(seq) if s.params["motion_id"] == 201)
    assert seq[end_i].params["motion_id"] == 303 and end_i < first_pose
    assert all(s.params["motion_id"] == 303 for s in seq[:end_i + 1])
    assert seq[first_pose].t - seq[end_i].t >= SWITCH_PAUSE - 0.01
    dog.posture.stop()


def test_driver_stops_posture_first_and_resets_pose(dog, fake):
    dog.posture.set(z=0.25, yaw=0.3)
    time.sleep(0.2)
    dog.drive.move(0.2, 0, 0)
    assert not dog.posture.active
    seq = frames(fake)
    end_i = next(i for i, s in enumerate(seq) if s.params["cmd_type"] == 2)
    assert seq[end_i].params["motion_id"] == 201
    walk = [s for s in seq if s.params["motion_id"] == 303]
    assert walk and walk[0].t - seq[end_i].t >= SWITCH_PAUSE - 0.01
    assert all(s.params["motion_id"] == 201 for s in seq[:end_i + 1])
    dog.drive.stop()
    assert dog.posture.target == POSE_DEFAULT


def test_never_both_streaming(dog, fake):
    for _ in range(3):
        dog.drive.move(0.1, 0, 0)
        assert not dog.posture.active
        time.sleep(0.05)
        dog.posture.set(z=0.24)
        assert not dog.drive.active
        time.sleep(0.05)
    dog.posture.stop()
    seq = frames(fake)
    # between two different motion ids there is always an END of the earlier one
    for a, b in zip(seq, seq[1:]):
        if a.params["motion_id"] != b.params["motion_id"]:
            assert a.params["cmd_type"] == 2


def test_restart_after_stop_keeps_the_pause(dog, fake):
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    seq = frames(fake)
    i = next(k for k, s in enumerate(seq) if s.params["cmd_type"] == 2)
    assert seq[i + 1].params["cmd_type"] == 1 and seq[i + 1].t - seq[i].t >= SWITCH_PAUSE - 0.01
    assert len(frames(fake, 2)) == 2


def test_gait_change_cancelled_and_carried_over(dog, fake):
    dog.drive.move(0.1, 0, 0)
    dog.drive.set_gait(Gait.FAST)
    dog.drive.set_gait(Gait.SLOW)          # back to the current gait: nothing pending
    time.sleep(0.3)
    assert len(frames(fake, 2)) == 0 and dog.drive.gait == Gait.SLOW
    dog.drive.stop()
    dog.drive.set_gait(Gait.MEDIUM)        # idle: immediate
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    after_first_end = frames(fake)[[s.params["cmd_type"] for s in frames(fake)].index(2) + 1:]
    assert [s.params["motion_id"] for s in after_first_end] == [308] * len(after_first_end)
    assert [s.params["cmd_type"] for s in after_first_end].count(2) == 1


# ---- dead-man -----------------------------------------------------------------------------
def test_deadman_is_on_by_default(dog):
    assert dog.drive.deadman_s == DEADMAN_S == 0.5


def test_deadman_stops_after_silence(dog, fake):
    t0 = time.perf_counter()
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    end = frames(fake, 2)[0]
    assert 0.45 <= end.t - t0 <= 0.75, end.t - t0
    assert end.params["motion_id"] == 303 and end.params["vel_des"] == [0.0, 0.0, 0.0]
    assert wait_until(lambda: not dog.drive.active)
    assert dog.drive.stats["deadman_stops"] == 1 and dog.drive.stats["ends"] == 1
    n = len(fake.sent)
    time.sleep(0.15)
    assert len(fake.sent) == n                     # nothing after the END
    dog.drive.stop()                               # idle: no second END
    assert len(frames(fake, 2)) == 1


def test_move_keeps_the_stream_alive(dog, fake):
    t_end = time.perf_counter() + 1.3
    while time.perf_counter() < t_end:
        dog.drive.move(0.2, 0, 0)
        time.sleep(0.2)
    assert dog.drive.active and frames(fake, 2) == []
    dog.drive.stop()
    assert len(frames(fake, 2)) == 1 and dog.drive.stats["deadman_stops"] == 0


def test_deadman_off_for_scripts(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.8)
    assert dog.drive.active and frames(fake, 2) == []
    dog.drive.stop()
    assert len(frames(fake, 2)) == 1 and dog.drive.stats["deadman_stops"] == 0


def test_deadman_from_constructor(fake):
    d = Driver(fake, deadman_s=None)
    assert d.deadman_s is None
    d = Driver(fake, deadman_s=0.25)
    t0 = time.perf_counter()
    d.move(0.1, 0, 0)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    assert 0.2 <= frames(fake, 2)[0].t - t0 <= 0.5
    d.stop()


def test_deadman_change_while_streaming(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.6)
    assert dog.drive.active
    dog.drive.deadman_s = 0.5                      # already 0.6 s since move(): ends now
    assert wait_until(lambda: not dog.drive.active, timeout=0.5)
    assert len(frames(fake, 2)) == 1 and dog.drive.stats["deadman_stops"] == 1


def test_move_after_deadman_restarts_with_pause(dog, fake):
    dog.drive.move(0.1, 0, 0)
    assert wait_until(lambda: not dog.drive.active, timeout=2.0)
    dog.drive.move(0.0, 0, 0.3)
    assert wait_until(lambda: frames(fake, 1)[-1].params["vel_des"] == [0.0, 0.0, 0.3])
    dog.drive.stop()
    seq = frames(fake)
    i = next(k for k, s in enumerate(seq) if s.params["cmd_type"] == 2)
    assert seq[i + 1].params["cmd_type"] == 1 and seq[i + 1].t - seq[i].t >= SWITCH_PAUSE - 0.01
    assert len(frames(fake, 2)) == 2 and dog.drive.stats["deadman_stops"] == 1


@pytest.mark.parametrize("bad", [0, -0.1, float("nan"), float("inf"), "x"])
def test_deadman_refuses_bad_values(dog, bad):
    with pytest.raises(ValueError):
        dog.drive.deadman_s = bad
    assert dog.drive.deadman_s == DEADMAN_S


def test_moves_around_the_deadman_expiry_reach_the_wire(dog, fake):
    # move() lands just before and just after the expiry, so it alternately updates a live stream
    # and restarts a stopped one; each velocity must reach the wire. The narrow race where move()
    # lands while the loop is deciding to end is covered deterministically by the stall tests below.
    dog.drive.deadman_s = 0.08
    for k, delay in enumerate([0.07, 0.08, 0.09, 0.075, 0.085, 0.08, 0.095, 0.065] * 3):
        vx = round(0.01 * (k + 1), 2)
        dog.drive.move(vx, 0, 0)
        assert wait_until(lambda: any(s.params["vel_des"][0] == vx for s in frames(fake, 1)), timeout=1.0), k
        time.sleep(delay)
    assert wait_until(lambda: not dog.drive.active, timeout=1.0)
    seq = frames(fake)
    assert seq[-1].params["cmd_type"] == 2
    for a, b in zip(seq, seq[1:]):                 # never two ENDs in a row: one END per stream
        assert not (a.params["cmd_type"] == 2 and b.params["cmd_type"] == 2)


def test_deadman_counts_from_the_stream_start(dog, fake):
    # the handover from Posture (END + pause) must not eat the first move()'s dead-man time
    dog.posture.set(z=0.25)
    time.sleep(0.1)
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: [s for s in frames(fake, 2) if s.params["motion_id"] == 303], timeout=2.0)
    walk = [s for s in frames(fake, 1) if s.params["motion_id"] == 303]
    end = [s for s in frames(fake, 2) if s.params["motion_id"] == 303][0]
    assert end.t - walk[0].t >= 0.45


def test_deadman_counts_from_every_stream_start(dog, fake):
    dog.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    dog.posture.set(z=0.25)
    time.sleep(0.1)
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len([s for s in frames(fake, 2) if s.params["motion_id"] == 303]) == 2, timeout=2.0)
    pose_end = [s for s in frames(fake, 2) if s.params["motion_id"] == 201][0]
    walk = [s for s in frames(fake, 1) if s.params["motion_id"] == 303 and s.t > pose_end.t]
    end = [s for s in frames(fake, 2) if s.params["motion_id"] == 303][1]
    assert end.t - walk[0].t >= 0.45, end.t - walk[0].t


@pytest.mark.parametrize("bad", [0, -0.1, float("nan"), float("inf"), "x"])
def test_deadman_constructor_refuses_bad_values(fake, bad):
    with pytest.raises(ValueError):
        Driver(fake, deadman_s=bad)


def test_deadman_end_equals_stop_end(dog, fake):
    dog.drive.step_height = 0.03
    dog.drive.set_gait(Gait.FAST)
    dog.drive.move(0.4, 0.1, 0.2)
    time.sleep(0.1)
    dog.drive.stop()
    stop_end = frames(fake, 2)[-1].raw
    dog.drive.move(0.4, 0.1, 0.2)
    assert wait_until(lambda: len(frames(fake, 2)) == 2, timeout=2.0)
    assert frames(fake, 2)[-1].raw == stop_end


def test_deadman_latency_with_slow_rpc(dog, fake):
    # the check runs at frame boundaries: on a slow link the END comes at most FRAME_DEADLINE late
    def slow(rec):
        time.sleep(0.2)
        return [""]
    fake.replies[1002] = slow
    dog.drive.move(0.2, 0, 0)
    t_move = dog.drive._last_move
    assert wait_until(lambda: frames(fake, 2), timeout=3.0)
    assert DEADMAN_S <= frames(fake, 2)[0].t - t_move <= DEADMAN_S + FRAME_DEADLINE + 0.05


# ---- dead-man races: each forces the interleaving instead of hoping for it ------------------
def stall_first_end_decision(drive, seconds):
    """Make the loop pause right after _want() said False, still under _plock (a preemption)."""
    deciding = threading.Event()
    orig = drive._want

    def slow_want():
        r = orig()
        if not r and not deciding.is_set():
            deciding.set()
            time.sleep(seconds)
        return r
    drive._want = slow_want
    return deciding


def slow_end(fake, seconds):
    """The first END RPC takes `seconds`; returns an Event set while that END is in flight."""
    in_end = threading.Event()

    def reply(rec):
        if rec.params["cmd_type"] == 2 and not in_end.is_set():
            in_end.set()
            time.sleep(seconds)
        return [""]
    fake.replies[1002] = reply
    return in_end


def sent_vx(fake, vx):
    return any(s.params["vel_des"][0] == vx for s in frames(fake, 1))


def no_double_end(fake):
    k = [(s.params["motion_id"], s.params["cmd_type"]) for s in frames(fake)]
    return [(a, b) for a, b in zip(k, k[1:]) if a[1] == 2 and b[1] == 2] == []


def test_move_while_the_loop_decides_to_end_is_not_lost(dog, fake):
    dog.drive.deadman_s = 0.2
    deciding = stall_first_end_decision(dog.drive, 0.1)
    dog.drive.move(0.1, 0, 0)
    assert deciding.wait(2.0)
    dog.drive.move(0.3, 0, 0)               # lands while the loop decides to end
    assert wait_until(lambda: sent_vx(fake, 0.3), timeout=1.0), "move() lost at the dead-man instant"
    dog.drive.stop()


def test_move_while_the_deadman_end_is_in_flight_restarts(dog, fake):
    in_end = slow_end(fake, 0.1)
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.1, 0, 0)
    assert in_end.wait(2.0)
    assert dog.drive.active                 # the END RPC is still running
    dog.drive.move(0.3, 0, 0)
    assert wait_until(lambda: sent_vx(fake, 0.3), timeout=1.0), "move() lost while the END was in flight"
    dog.drive.stop()
    assert no_double_end(fake)


def test_gait_and_move_while_the_deadman_end_is_in_flight(dog, fake):
    # a single-threaded script: the new stream must start in the new gait, without a second END
    in_end = slow_end(fake, 0.1)
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert in_end.wait(2.0)
    dog.drive.set_gait(Gait.MEDIUM)         # thread still alive: stored as pending
    dog.drive.move(0.3, 0, 0)
    assert wait_until(lambda: sent_vx(fake, 0.3), timeout=1.0)
    dog.drive.stop()
    assert no_double_end(fake)
    assert [s.params["motion_id"] for s in frames(fake, 1) if s.params["vel_des"][0] == 0.3][0] == 308


def test_pending_gait_survives_a_deadman_stop(dog, fake):
    in_end = slow_end(fake, 0.1)
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert in_end.wait(2.0)
    dog.drive.set_gait(Gait.MEDIUM)
    assert wait_until(lambda: not dog.drive.active)
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: any(s.params["motion_id"] == 308 for s in frames(fake, 1)))
    dog.drive.stop()
    assert [s.params["motion_id"] for s in frames(fake, 2)] == [303, 308]


@pytest.mark.parametrize("deadman,before_gait", [(0.2, 0.1), (DEADMAN_S, 0.42)])
def test_deadman_inside_gait_pause_sends_no_walk_frame(dog, fake, deadman, before_gait):
    # the caller changes gait and then hangs: the dead-man expires during the END + pause,
    # so no frame of the new gait may follow, and the old gait's END is the only END
    dog.drive.deadman_s = deadman
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    time.sleep(before_gait)
    dog.drive.set_gait(Gait.MEDIUM)
    assert wait_until(lambda: not dog.drive.active, timeout=2.0)
    seq = [(s.params["motion_id"], s.params["cmd_type"]) for s in frames(fake)]
    assert (308, 1) not in seq and seq.count((303, 2)) == 1 and len(frames(fake, 2)) == 1, seq
    assert dog.drive.stats["deadman_stops"] == 1


def test_stop_inside_gait_pause_sends_one_end_and_no_new_gait(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    dog.drive.set_gait(Gait.MEDIUM)
    assert wait_until(lambda: frames(fake, 2))       # the gait-change END: the 0.15 s pause starts
    dog.drive.stop()
    seq = [(s.params["motion_id"], s.params["cmd_type"]) for s in frames(fake)]
    assert (308, 1) not in seq and len(frames(fake, 2)) == 1, seq


def test_set_gait_during_the_switch_pause_is_kept(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.1)
    dog.drive.set_gait(Gait.MEDIUM)
    assert wait_until(lambda: frames(fake, 2), timeout=1.0)    # END of 303 sent, pause running
    time.sleep(0.03)
    dog.drive.set_gait(Gait.SLOW)            # the caller changes its mind during the pause
    time.sleep(0.4)
    gait = dog.drive.gait
    dog.drive.stop()
    assert gait == Gait.SLOW, gait
    assert no_double_end(fake)


def test_a_stalled_start_sends_neither_frame_nor_end(dog, fake):
    # the loop stalls longer than deadman_s before its first frame: the stale velocity must not go
    # out, and an END without a frame would make the dog run WalkStand 112
    orig = dog.drive._want
    first = threading.Event()

    def stalled_want():
        if not first.is_set():
            first.set()
            time.sleep(dog.drive.deadman_s + 0.05)
        return orig()
    dog.drive._want = stalled_want
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: not dog.drive.active, timeout=2.0)
    assert frames(fake) == [] and dog.drive.stats["deadman_stops"] == 1


def test_no_end_after_halt_while_the_loop_waits_for_plock(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.2)
    with dog.drive._plock:                   # what move() holds for a few microseconds
        time.sleep(0.06)                     # > one period: the loop passed the halt check, now blocks here
        n_before = len(frames(fake))
        assert dog.drive.halt()              # the ESTOP path
    dog.drive.join(1.0)
    assert frames(fake)[n_before:] == []     # neither END nor a walking frame after halt()


def test_stream_after_halt_without_frame_sends_no_end(dog, fake):
    # halt() leaves the last command a DATA frame; the next stream must start from scratch
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    assert dog.drive.halt()
    dog.drive.join(1.0)
    n = len(frames(fake))
    orig = dog.drive._want
    first = threading.Event()

    def stalled_want():
        if not first.is_set():
            first.set()
            time.sleep(0.25)
        return orig()
    dog.drive._want = stalled_want
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: first.is_set() and not dog.drive.active, timeout=2.0)
    assert frames(fake)[n:] == []


def test_halt_while_the_stop_end_is_built_sends_no_end(dog, fake):
    dog.drive.deadman_s = None
    building, release = threading.Event(), threading.Event()
    orig = dog.drive._end_frame

    def slow_end_frame():
        building.set()
        release.wait(2.0)
        return orig()
    dog.drive._end_frame = slow_end_frame
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    th = threading.Thread(target=dog.drive.stop)
    th.start()
    assert building.wait(2.0)
    assert dog.drive.halt()
    release.set()
    th.join(2.0)
    assert frames(fake, 2) == []


def test_halt_while_a_walk_frame_is_built_sends_nothing(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    building, release = threading.Event(), threading.Event()
    orig = dog.drive.velocity

    def slow_velocity():
        if threading.current_thread() is dog.drive._thread and not building.is_set():
            building.set()
            release.wait(2.0)
        return orig()
    dog.drive.velocity = slow_velocity
    assert building.wait(2.0)
    n = len(frames(fake))
    assert dog.drive.halt()
    release.set()
    dog.drive.join(2.0)
    assert frames(fake)[n:] == []


class GateLock:
    """Wraps _plock: when armed, the next acquire from _step waits until go is set."""

    def __init__(self, inner):
        self.inner = inner
        self.armed, self.waiting, self.go = threading.Event(), threading.Event(), threading.Event()

    def __enter__(self):
        if self.armed.is_set() and sys._getframe(1).f_code.co_name == "_step":
            self.armed.clear()
            self.waiting.set()
            self.go.wait(2.0)
        return self.inner.__enter__()

    def __exit__(self, *a):
        return self.inner.__exit__(*a)


def test_stop_while_step_waits_for_plock_sends_only_the_end(dog, fake):
    dog.drive.deadman_s = None
    gate = GateLock(dog.drive._plock)
    dog.drive._plock = gate
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    gate.armed.set()
    assert gate.waiting.wait(2.0)
    th = threading.Thread(target=dog.drive.stop)
    th.start()
    assert wait_until(lambda: dog.drive._exit == "end")
    n = len(frames(fake))
    gate.go.set()
    th.join(2.0)
    assert [s.params["cmd_type"] for s in frames(fake)[n:]] == [2]


def test_deadman_counts_from_a_late_thread_start(dog, fake):
    orig = dog.drive._loop

    def late_loop():
        time.sleep(0.3)
        orig()
    dog.drive._loop = late_loop
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 2), timeout=2.0)
    walk = frames(fake, 1)
    assert walk and frames(fake, 2)[0].t - walk[0].t >= 0.45, frames(fake, 2)[0].t - walk[0].t


def test_deadman_boundary_is_exact(fake):
    d = Driver(fake, deadman_s=0.5)
    now = clock()
    d.started_at = now - 10.0
    d._last_move = now - 0.45
    assert d._want() is True
    d._last_move = now - 0.55
    assert d._want() is False
    d._last_move = now - 10.0
    d.started_at = now - 0.45
    assert d._want() is True
    assert d.stats["deadman_stops"] == 1


# ---- halt() against a stream that is (re)starting -----------------------------------------
def test_halt_cancels_a_move_waiting_in_claim(dog, fake):
    # move() right after stop(): claim() sleeps SWITCH_PAUSE before the thread starts, and no
    # Driver thread is alive when halt() comes
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    th = threading.Thread(target=dog.drive.move, args=(0.3, 0, 0))
    th.start()
    time.sleep(0.05)
    assert not dog.drive.active
    n = len(frames(fake))
    assert dog.drive.halt() is False
    th.join(2.0)
    time.sleep(0.3)
    assert frames(fake)[n:] == [] and not dog.drive.active


def test_halt_cancels_a_move_joining_a_closing_stream(dog, fake):
    in_end = slow_end(fake, 0.15)
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert in_end.wait(2.0)                  # dead-man END in flight
    th = threading.Thread(target=dog.drive.move, args=(0.3, 0, 0))
    th.start()
    time.sleep(0.03)                         # move() is joining the finishing thread
    assert dog.drive.halt()
    th.join(2.0)
    time.sleep(0.3)
    assert not sent_vx(fake, 0.3) and not dog.drive.active


def test_move_after_halt_starts_a_new_stream(dog, fake):
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake, 1))
    assert dog.drive.halt()
    dog.drive.join(1.0)
    dog.drive.move(0.3, 0, 0)
    assert wait_until(lambda: sent_vx(fake, 0.3))
    dog.drive.stop()
    assert frames(fake)[-1].params["cmd_type"] == 2


def test_posture_set_survives_a_slow_driver_handover(dog, fake):
    def reply(rec):
        if rec.params["motion_id"] != 201:
            time.sleep(0.24)                 # slow link, inside FRAME_DEADLINE
        return [""]
    fake.replies[1002] = reply
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: frames(fake))
    n = len(frames(fake))
    assert wait_until(lambda: len(frames(fake)) > n)     # a DATA RPC has just begun
    dog.posture.set(z=0.27)
    time.sleep(POSE_IDLE_END + 0.3)
    pose = [s for s in frames(fake) if s.params["motion_id"] == 201]
    assert pose and pose[0].params["pos_des"][2] == 0.27, "posture.set() silently dropped"
