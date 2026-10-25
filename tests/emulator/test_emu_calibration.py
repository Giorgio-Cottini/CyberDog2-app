"""FakeController against the Gazebo baseline (docs/design/sim_baseline.md), and the manager's RTF waits.

The controller runs at time scale S, so a Gazebo duration of d sim seconds is expected at d * S wall
seconds. The 0.5 s command timeout and the 50 ms heartbeat stay wall-clock."""
from __future__ import annotations

import math
import threading
import time

import pytest

from emulator.controllers.fake import FakeController
from emulator.manager import Manager

S = 0.3
GRACE = 0.02


def near(dt, sim_s, slack=0.035):
    return abs(dt - sim_s * S) < slack + 0.1 * sim_s * S


def frame(mode, gait, life, dur=0, vel=None, rpy=None, pos=None):
    return {"mode": mode, "gait_id": gait, "contact": 15, "life_count": life, "value": 0, "duration": dur,
            "vel_des": vel or [0.0] * 3, "rpy_des": rpy or [0.0] * 3, "pos_des": pos or [0.0] * 3,
            "acc_des": [0.0] * 6, "ctrl_point": [0.0] * 3, "foot_pose": [0.0] * 6, "step_height": [0.0] * 2}


class Bench:
    """Drives a FakeController as motion_manager does: a new frame gets a new life_count and a 20 Hz
    heartbeat repeats the last frame. Responses are logged with their wall time."""

    def __init__(self):
        self.c = FakeController(time_scale=S)
        self.log = []
        self.c.on_response(lambda r: self.log.append((time.monotonic(), r)))
        self.lk = threading.Lock()
        self.life, self.last, self.beat = 0, None, True
        self.stop = threading.Event()
        threading.Thread(target=self._hb, daemon=True).start()

    def _hb(self):
        while not self.stop.wait(0.05):
            with self.lk:
                if self.beat and self.last is not None:
                    self.c.publish(self.last)
                    self.t_last = time.monotonic()

    def send(self, mode, gait, **kw):
        with self.lk:
            self.life += 1
            self.last = frame(mode, gait, self.life, **kw)
            t = time.monotonic()
            self.c.publish(self.last)
            self.t_last = time.monotonic()
        return t

    def after(self, t0, pred, timeout=4.0):
        """Wall seconds from t0 to the first response at or after t0 that matches pred."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, r in list(self.log):
                if t >= t0 and pred(r):
                    return t - t0
            time.sleep(0.005)
        raise AssertionError("no matching response in %.1f s" % timeout)

    def since(self, t0, t1=None):
        return [(t - t0, r) for t, r in list(self.log) if t >= t0 and (t1 is None or t < t1)]

    def settled(self, t0, t1=None):
        """Responses after t0 + GRACE: one computed just before a publish can be delivered just after it."""
        return self.since(t0 + GRACE, t1)

    def stand(self):
        t = self.send(12, 0)
        self.after(t, lambda r: (r["mode"], r["order_process_bar"]) == (12, 100))

    def gt(self):
        return self.c.ground_truth()


@pytest.fixture
def bench():
    b = Bench()
    yield b
    b.stop.set()
    b.c.close()


def is_(mode, gait=None, bar=None, sw=None):
    return lambda r: (r["mode"] == mode and (gait is None or r["gait_id"] == gait)
                      and (bar is None or r["order_process_bar"] == bar)
                      and (sw is None or r["switch_status"] == sw))


def test_stand_and_lie_heights_and_lengths(bench):
    b = bench
    assert abs(b.gt()["z"] - 0.054) < 1e-3                # boot: lying after 101
    t = b.send(12, 0)
    dt = b.after(t, is_(12, 0, bar=100))
    assert near(dt, 2.0), dt                               # Gazebo: bar 100 at 2.02 s
    assert abs(b.gt()["z"] - 0.2353) < 1e-3
    t = b.send(7, 1)
    assert near(b.after(t, is_(7, 1, bar=100)), 1.2)      # Gazebo: 1.20 s
    time.sleep(1.7 * S)                                   # the body keeps sinking after bar 100
    assert abs(b.gt()["z"] - 0.054) < 1e-3


def test_walk_to_posectrl_switch_1_then_repeating_filler(bench):
    b = bench
    b.stand()
    t = b.send(11, 1)
    b.after(t, is_(11, 1, sw=0))
    t = b.send(21, 5, dur=1000, pos=[0.0, 0.0, 0.20])
    d21 = b.after(t, is_(21, 5))
    assert near(d21, 0.3), d21                            # Gazebo: 0.30 s of switch_status 1 first
    before = b.settled(t, t + d21)
    assert before and all(r["mode"] == 11 and r["switch_status"] == 1 for _, r in before)
    dfill = b.after(t, is_(21, 3))
    assert near(dfill, 1.3), dfill                        # Gazebo: 21/3 filler at 1.30 s
    assert abs(b.gt()["z"] - 0.1925) < 1e-3               # Gazebo: 0.1924-0.1927 for 0.20
    t1 = t + dfill
    time.sleep(4.3 * 0.4 * S)
    fill = [(x, r["order_process_bar"]) for x, r in b.since(t1) if (r["mode"], r["gait_id"]) == (21, 3)]
    starts = [x for (x, bar), (_, prev) in zip(fill[1:], fill) if prev - bar > 50]
    assert len(starts) >= 3, fill
    gaps = [b2 - a for a, b2 in zip(starts, starts[1:])]
    per = 0.4 * S                                         # repeats every 0.40 s; under load a cycle can go unseen
    ks = [round(g / per) for g in gaps]
    assert 1 in ks and all(k >= 1 and abs(g - k * per) < 0.05 for g, k in zip(gaps, ks)), gaps
    assert max(bar for _, bar in fill) == 100


def test_posture_pitch_signs(bench):
    b = bench
    b.stand()
    p0 = b.gt()["pitch"]
    t = b.send(21, 0, dur=1000, rpy=[0.0, 0.2, 0.0])     # 212 is relative: nose up
    b.after(t, is_(21, 3))
    p1 = b.gt()["pitch"]
    assert abs((p1 - p0) - (-0.203)) < 0.005, (p0, p1)
    t = b.send(21, 0, dur=1000, rpy=[0.0, -0.2, 0.0])   # queued behind the 0.4 s filler (command_interface.cpp:1031-1041)
    d = b.after(t, is_(21, 0))
    b.after(t + d, is_(21, 3))
    assert abs(b.gt()["pitch"] - p0) < 0.005
    b.send(3, 0, pos=[0.0, 0.0, 0.22], rpy=[0.0, 0.15, 0.0])   # 201 is absolute: nose down
    time.sleep(12 * 0.15 * S)
    g = b.gt()
    assert abs(g["pitch"] - 0.145) < 0.003 and abs(g["z"] - 0.22) < 1e-3, g
    b.send(3, 0, pos=[0.0, 0.0, 0.22], rpy=[0.15, 0.0, 0.0])
    time.sleep(12 * 0.15 * S)
    g = b.gt()
    assert abs(g["roll"] - 0.136) < 0.003 and abs(g["pitch"]) < 0.003, g


def test_walk_tracking_and_walkstand_stop(bench):
    b = bench
    b.stand()
    t = b.send(11, 1)
    b.after(t, is_(11, 1, sw=0))
    b.send(11, 27, vel=[0.3, 0.0, 0.0], dur=0)
    time.sleep(12 * 0.2 * S)
    assert 0.29 <= b.gt()["vx"] <= 0.32
    t = b.send(11, 1)                                     # 112
    while math.hypot(b.gt()["vx"], b.gt()["vy"]) >= 0.05:
        assert time.monotonic() - t < 2.0
        time.sleep(0.003)
    dt = time.monotonic() - t
    assert 0.35 * S - 0.035 < dt < 0.63 * S + 0.035, dt  # Gazebo: still after 0.35-0.63 s
    b.send(11, 27, vel=[0.0, 0.0, 0.5])
    time.sleep(12 * 0.2 * S)
    assert abs(b.gt()["wz"] - 0.40) < 0.01


def test_mode_23_is_ban_trans_with_mode_and_height_unchanged(bench):
    b = bench
    b.stand()
    z0 = b.gt()["z"]
    t = b.send(23, 0)
    assert b.after(t, lambda r: r["switch_status"] == 5) < 0.2   # Gazebo: the next response; slack for load
    time.sleep(0.3)
    after = b.since(t + 0.1)
    assert after and all((r["mode"], r["gait_id"], r["switch_status"]) == (12, 0, 5) for _, r in after)
    assert abs(b.gt()["z"] - z0) < 1e-4


def test_resumed_heartbeat_does_not_restart_the_command(bench):
    b = bench
    b.stand()
    with b.lk:
        b.beat = False
        t = b.t_last                                      # the timeout runs from the last frame
    assert 0.45 < b.after(t, is_(7, 0)) < 0.7             # wall-clock CMD_TIMEOUT
    with b.lk:
        b.beat = True
    time.sleep(0.4)
    assert all(r["mode"] == 7 for _, r in b.since(time.monotonic() - 0.3))
    t = b.send(12, 0)                                     # a new life_count does
    b.after(t, is_(12, 0))


# ---- motion_manager over the dog ------------------------------------------------------------

def test_1004_mode_23_is_3022_and_nothing_moves(dog):
    dog.j(1004, {"motion_id": 111})
    s0 = dog.state()
    r = dog.j(1004, {"motion_id": 126})                  # [23,0]: a post motion of 111, banned by RecoveryStand
    assert r == {"motion_id": 111, "result": False, "code": 3022}
    s = dog.state()
    assert s["motion"]["mode"] == 12 and abs(s["ground_truth"]["z"] - s0["ground_truth"]["z"]) < 1e-3


def test_wrong_size_array_is_sent_as_zeros(dog):
    dog.j(1004, {"motion_id": 111})
    p0 = dog.state()["ground_truth"]["pitch"]
    r = dog.j(1004, {"motion_id": 212, "duration": 1000, "rpy_des": [0.0, 0.2]})
    assert r["result"] is True, r
    last = dog.emu.mgr.last
    assert last["mode"] == 21 and last["rpy_des"] == [0.0, 0.0, 0.0]
    s = dog.state()
    assert abs(s["ground_truth"]["pitch"] - p0) < 1e-3
    assert any("2 floats, 3 expected" in b["bug"] for b in s["client_bugs"])
    dog.j(1004, {"motion_id": 212, "duration": 1000, "rpy_des": [0.0, 0.2, 0.0]})
    dog.wait(lambda s: s["ground_truth"]["pitch"] - p0 < -0.19, timeout=2)


# ---- manager waits divided by the real-time factor ---------------------------------------------

class SlowSim:
    """A controller at real-time factor 0.25 that does not report it: a 21/x command completes after
    duration / 0.25 wall seconds; anything else answers as a standing robot."""
    SLOW = 0.25

    def __init__(self):
        self.cbs, self.frames, self.life = [], [], None

    def on_response(self, cb):
        self.cbs.append(cb)

    def ground_truth(self):
        return {}

    def close(self):
        pass

    def _emit(self, mode, gait, bar):
        for cb in self.cbs:
            cb({"mode": mode, "gait_id": gait, "contact": 15, "order_process_bar": bar, "switch_status": 0,
                "ori_error": 0, "footpos_error": 0, "motor_error": [0] * 12})

    def publish(self, cmd):
        self.frames.append(time.monotonic())
        if cmd["life_count"] == self.life:
            return
        self.life = cmd["life_count"]
        if cmd["mode"] == 21:
            self._emit(21, cmd["gait_id"], 0)
            t = threading.Timer(cmd["duration"] / 1000.0 / self.SLOW, self._emit, (21, cmd["gait_id"], 100))
            t.daemon = True
            t.start()
        else:
            self._emit(12, 0, 100)


class SlowSimWithRtf(SlowSim):
    def __init__(self):
        super().__init__()
        self.value = self.SLOW

    def rtf(self):
        return self.value


class EmuStub:
    time_scale = 1.0

    def sc(self, t):
        return t

    def protected(self):
        return False

    def client_bug(self, *a, **k):
        pass

    def refuse(self, *a, **k):
        pass


def _manager(ctl):
    m = Manager(ctl, EmuStub())
    time.sleep(0.05)
    return m


def test_manager_waits_are_divided_by_the_rtf_and_the_heartbeat_is_not():
    ctl = SlowSimWithRtf()
    m = _manager(ctl)
    try:
        n0, t0 = len(ctl.frames), time.monotonic()
        r = m.motion_cmd({"motion_id": 212, "duration": 300})   # 1.2 s wall; the wait is 0.8 / 0.25 s
        assert r["result"] is True and r["code"] == 0, r
        el = time.monotonic() - t0
        rate = (len(ctl.frames) - n0) / el
        assert 8 < rate < 25, rate                        # 20 Hz wall-clock (5 Hz if it were divided)
        ctl.value = None                                  # no simulator_state yet: the 0.2 floor
        assert m._rt(1.0) == pytest.approx(5.0)
        ctl.value = 0.1
        assert m._rt(1.0) == pytest.approx(5.0)
        ctl.value = 0.5
        assert m._rt(1.0) == pytest.approx(2.0)
    finally:
        m.close()


def test_manager_without_rtf_times_out_on_a_slow_sim():
    m = _manager(SlowSim())
    try:
        assert m._rt(1.0) == 1.0
        r = m.motion_cmd({"motion_id": 212, "duration": 300})   # waits 0.8 s, the sim needs 1.2 s
        assert r["result"] is False and r["code"] == 3028, r
    finally:
        m.close()
