"""Review round 4: estop() against the dog's servo watchdog, the halt generation against stream
restarts, and stream thread failures. Each test reproduced a defect on the round-3 code.

DogModel puts the firmware facts of motion.md section 3 on top of the fake transport (no network):
- a 1002 frame that the dog gets before the first ESTOP that reached it is accepted. A walk DATA
  frame (motion_id != 201) arms the watchdog, which runs WalkStand 112 200-250 ms later, unless
  an accepted END came after it (END runs WalkStand at once, before the latch);
- once an ESTOP reached the dog (latch), 1002 frames get 3040 and do not tick the watchdog;
- a WalkStand that fires after the latch overwrites the ESTOP; only a later ESTOP undoes it.
The last point is the design's threat model: the open-source controller refuses mode 11 out of
ESTOP (fsm_state_passive.cpp:65-93), the dog's firmware is not verified.
"Reached the dog" is when the fake's reply callable runs."""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time

import pytest

from cyberdog_sdk.client import CyberDog
from cyberdog_sdk.servo import SWITCH_PAUSE, Gait, clock
from cyberdog_sdk.transport import DogUnreachable

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WATCHDOG_MAX = 0.25      # the watchdog fires 200-250 ms after the last accepted frame


def wait_until(cond, timeout=2.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        if cond():
            return True
        time.sleep(0.002)
    return False


def data(fake):
    return [s for s in fake.of(1002) if s.params["cmd_type"] == 1]


def ends(fake):
    return [s for s in fake.of(1002) if s.params["cmd_type"] == 2]


def pose_data(fake):
    return [s for s in data(fake) if s.params["motion_id"] == 201]


def estop_ok(rec):
    return [{"motion_id": rec.params["motion_id"], "result": True, "code": 0}]


class DogModel:
    def __init__(self, fake, estop_delay=None, estop_fail=None):
        self.fake = fake
        self.lock = threading.Lock()
        self.accepted = []          # (t, motion_id, cmd_type) of frames accepted before the latch
        self.latched = []           # times an ESTOP reached the dog
        self.n = 0
        self.estop_delay = estop_delay or (lambda n: 0.0)
        self.estop_fail = estop_fail or (lambda n: None)
        fake.replies[1002] = self.servo
        fake.replies[1004] = self.motion

    def _closed(self):
        if self.fake.closed:        # what grpc does after channel.close()
            raise ValueError("Cannot invoke RPC: Channel closed!")

    def servo(self, rec):
        self._closed()
        t = clock()
        with self.lock:
            if not self.latched:
                self.accepted.append((t, rec.params["motion_id"], rec.params["cmd_type"]))
        return [""]

    def motion(self, rec):
        self._closed()
        with self.lock:
            self.n += 1
            n = self.n
        d = self.estop_delay(n)
        if d:
            time.sleep(d)
        exc = self.estop_fail(n)
        if exc is not None:
            raise exc
        if rec.params["motion_id"] == 0:
            with self.lock:
                self.latched.append(clock())
        return estop_ok(rec)

    def walkstand_by(self):
        """Latest time the watchdog's WalkStand fires, or None when no walk frame arms it."""
        with self.lock:
            acc = list(self.accepted)
        if not acc:
            return None
        t, mid, cmd = acc[-1]
        if cmd == 2 or mid == 201:
            return None
        return t + WATCHDOG_MAX

    def report(self):
        w = self.walkstand_by()
        with self.lock:
            lat = list(self.latched)
            last = self.accepted[-1][0] if self.accepted else None
        t0 = lat[0] if lat else 0.0
        return ("last accepted frame %+.3f s, WalkStand by %+.3f s, ESTOPs reached the dog at %s "
                "(relative to the first latch)"
                % ((last or 0) - t0, (w or 0) - t0, ["%+.3f" % (e - t0) for e in lat]))

    def overwritten(self):
        """True when the watchdog's WalkStand can land after the latch with no ESTOP after it."""
        w = self.walkstand_by()
        if w is None:
            return False
        with self.lock:
            lat = list(self.latched)
        return bool(lat) and not any(e >= w for e in lat)


class Joystick:
    """move() every 40 ms from its own thread, as an app joystick or a control loop does."""

    def __init__(self, dog):
        self.dog = dog
        self.stop = threading.Event()
        self.th = threading.Thread(target=self.run, daemon=True, name="joystick")

    def run(self):
        while not self.stop.is_set():
            self.dog.drive.move(0.2, 0, 0)
            time.sleep(0.04)

    def start(self):
        if self.th.ident is None and not self.stop.is_set():
            self.th.start()
        return self

    def end(self):
        self.stop.set()
        if self.th.ident is not None:
            self.th.join(2.0)
        self.dog.drive.halt()


def halt_then_joystick(dog, fake, j):
    """Posture.halt() replacement: the joystick's first move() begins right after the halts."""
    orig = dog.posture.halt

    def hook():
        r = orig()
        j.start()
        time.sleep(0.2)             # estop() loses the CPU here; the joystick keeps calling move()
        return r
    dog.posture.halt = hook


# ---- estop() against the watchdog -------------------------------------------------------------
def test_plain_estop_while_walking_is_not_overwritten(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    r = dog.estop()
    assert r.result and len(model.latched) == 3
    assert not model.overwritten(), model.report()


def test_joystick_during_estop_is_not_overwritten(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    j = Joystick(dog).start()
    assert wait_until(lambda: len(data(fake)) > 3)
    r = dog.estop()
    j.end()
    assert r.result
    assert not model.overwritten(), model.report()


@pytest.mark.parametrize("walking,start_delay", [(True, 0.2), (False, 0.12)])
def test_a_slow_repeat_thread_start_leaves_no_window(dog, fake, monkeypatch, walking, start_delay):
    # a slow Thread.start() of the repeat delays ESTOP #1 but not ESTOP #2's schedule; a joystick
    # move() during estop() must not get a walk frame to the dog in between
    model = DogModel(fake)
    dog.drive.deadman_s = None
    j = Joystick(dog)
    if walking:
        j.start()
        assert wait_until(lambda: len(data(fake)) > 3)
    else:
        halt_then_joystick(dog, fake, j)
    orig_start = threading.Thread.start

    def slow_start(self):
        if self.name == "estop-repeat":
            time.sleep(start_delay)
        return orig_start(self)
    monkeypatch.setattr(threading.Thread, "start", slow_start)
    r = dog.estop()
    j.end()
    est = fake.of(1004)
    assert r.result and len(est) == 3
    assert not model.overwritten(), "%s; ESTOP #1 -> #2 sent %.3f s apart" % (model.report(), est[1].t - est[0].t)


def test_a_first_estop_slow_to_reach_the_dog_is_covered(dog, fake):
    # an idle priority channel reconnects (TCP + TLS): ESTOP #1 reaches the dog 0.15 s late
    model = DogModel(fake, estop_delay=lambda n: 0.15 if n == 1 else 0.0)
    dog.drive.deadman_s = None
    j = Joystick(dog)
    halt_then_joystick(dog, fake, j)
    r = dog.estop()
    j.end()
    assert r.result and len(fake.of(1004)) == 3
    assert not model.overwritten(), model.report()


def test_a_failed_first_estop_with_a_joystick_is_covered_by_the_second(dog, fake):
    model = DogModel(fake, estop_fail=lambda n: DogUnreachable("1004: StatusCode.UNAVAILABLE")
                     if n == 1 else None)
    dog.drive.deadman_s = None
    j = Joystick(dog).start()
    assert wait_until(lambda: len(data(fake)) > 3)
    r = dog.estop()
    j.end()
    assert r.result, r
    assert len(model.latched) == 2
    assert not model.overwritten(), model.report()


def test_a_failed_first_estop_after_a_halted_stream_is_covered(dog, fake):
    model = DogModel(fake, estop_fail=lambda n: DogUnreachable("1004: UNAVAILABLE") if n == 1 else None)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    r = dog.estop()
    assert r.result
    assert not model.overwritten(), model.report()


def test_close_during_estop_waits_for_the_repeat(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    out = []
    e = threading.Thread(target=lambda: out.append(dog.estop()))
    e.start()
    assert wait_until(lambda: fake.of(1004))
    time.sleep(0.05)
    dog.close()                             # another thread shuts down (app exit, reconnect)
    e.join(3.0)
    assert out and out[0].result, out
    assert len(model.latched) == 3, "an ESTOP never reached the dog; " + model.report()
    assert not model.overwritten(), model.report()


class StallNextFrame:
    """Transport proxy for the Driver: once armed, the next 1002 waits `stall` s before the wire
    (it passed the halt check already) and its reply takes `reply` s more."""

    def __init__(self, inner, stall, reply=0.0):
        self.inner, self.stall, self.reply = inner, stall, reply
        self.arm, self.stalled = threading.Event(), threading.Event()

    def call(self, code, params=None, timeout=10.0, **kw):
        hit = code == 1002 and self.arm.is_set() and not self.stalled.is_set()
        if hit:
            self.stalled.set()
            time.sleep(self.stall)
        out = self.inner.call(code, params, timeout=timeout, **kw)
        if hit and self.reply:
            time.sleep(self.reply)
        return out

    def __getattr__(self, n):
        return getattr(self.inner, n)


@pytest.mark.parametrize("stall,reply", [(0.0, 0.25), (1.1, 0.0), (0.85, 0.3)])
def test_a_halted_frame_stuck_in_flight_is_covered(dog, fake, stall, reply):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    proxy = StallNextFrame(dog.drive.t, stall, reply)
    dog.drive.t = proxy
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    proxy.arm.set()
    assert proxy.stalled.wait(1.0)
    r = dog.estop()
    assert r.result
    assert not model.overwritten(), model.report()


def test_estop_in_the_gait_change_pause(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    dog.drive.set_gait(305)
    assert wait_until(lambda: ends(fake))
    time.sleep(0.03)
    r = dog.estop()
    assert r.result and not model.overwritten(), model.report()
    assert not [s for s in data(fake) if s.params["motion_id"] == 305]


def test_two_estops_at_once(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    dog.motion(111)
    out = []
    ts = [threading.Thread(target=lambda: out.append(dog.estop())) for _ in range(2)]
    for t in ts:
        t.start()
        time.sleep(0.1)
    for t in ts:
        t.join(3)
    assert len(out) == 2 and all(r.result for r in out)
    assert len(fake.of(1004)) == 7 and dog._estops == 6 and dog.last_motion == 0
    assert not dog.drive.arbiter.estopping
    assert not model.overwritten(), model.report()


def test_estop_during_the_posture_to_driver_handover(dog, fake):
    model = DogModel(fake)
    dog.drive.deadman_s = None
    dog.posture.set(z=0.25)
    assert wait_until(lambda: dog.posture.active and fake.of(1002))
    m = threading.Thread(target=dog.drive.move, args=(0.2, 0, 0))
    m.start()
    assert wait_until(lambda: ends(fake))       # Posture END: claim() is in its pause
    r = dog.estop()
    m.join(2)
    assert r.result and not dog.drive.active
    assert not [s for s in data(fake) if s.params["motion_id"] != 201]
    assert not model.overwritten(), model.report()


# ---- a BaseException in an ESTOP RPC ---------------------------------------------------------------
def test_ctrl_c_during_the_first_estop_raises_after_the_repeat(dog, fake):
    # ESTOP's 1004 can take up to 1 s; a Ctrl+C in that wait reaches the main thread inside the RPC
    def motion(rec):
        if len(fake.of(1004)) == 1:
            time.sleep(0.05)
            raise KeyboardInterrupt
        return estop_ok(rec)
    fake.replies[1004] = motion
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    with pytest.raises(KeyboardInterrupt):
        dog.estop()
    # the interrupted ESTOP is sent again (it may not have left), then the two repeats
    assert len(fake.of(1004)) == 4 and not dog.drive.arbiter.estopping


def test_ctrl_c_while_waiting_for_the_repeat_is_raised_after_it(dog, fake, monkeypatch):
    # a second Ctrl+C lands in the main thread's wait for the repeat; a close() in the caller's
    # finally would then shut the channel under the repeat
    orig_wait = threading.Event.wait
    hit = []

    def wait(self, timeout=None):
        if (threading.current_thread() is threading.main_thread() and not hit
                and any(t.name == "estop-repeat" for t in threading.enumerate())):
            hit.append(1)
            raise KeyboardInterrupt
        return orig_wait(self, timeout)
    monkeypatch.setattr(threading.Event, "wait", wait)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    with pytest.raises(KeyboardInterrupt):
        dog.estop()
    assert hit and len(fake.of(1004)) == 3 and not dog.drive.arbiter.estopping


@pytest.mark.parametrize("runs", [False, True])
def test_ctrl_c_while_the_repeat_thread_starts(dog, fake, monkeypatch, runs):
    # the Ctrl+C lands in Thread.start(): before the thread exists (runs=False), or in the wait
    # for it to begin (runs=True). Either way all three ESTOPs go out, once each, then it is raised.
    orig_start = threading.Thread.start

    def start(self):
        if self.name == "estop-repeat":
            if runs:
                orig_start(self)
            raise KeyboardInterrupt
        return orig_start(self)
    monkeypatch.setattr(threading.Thread, "start", start)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    with pytest.raises(KeyboardInterrupt):
        dog.estop()
    est = fake.of(1004)
    assert [s.params["motion_id"] for s in est] == [0, 0, 0] and not dog.drive.arbiter.estopping
    assert est[1].t - est[0].t >= 0.15                  # the repeat still waits out the watchdog


def test_ctrl_c_while_halting_the_stream_still_sends_all(dog, fake, monkeypatch):
    orig = dog.drive.halt_thread
    hit = []

    def halt_thread():
        if not hit:
            hit.append(1)
            raise KeyboardInterrupt
        return orig()
    monkeypatch.setattr(dog.drive, "halt_thread", halt_thread)
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    with pytest.raises(KeyboardInterrupt):
        dog.estop()
    assert len(fake.of(1004)) == 3 and hit and not dog.drive.active


def test_the_repeat_thread_is_not_a_daemon(dog, fake):
    seen = []

    def motion(rec):
        seen.extend(t.daemon for t in threading.enumerate() if t.name == "estop-repeat")
        return estop_ok(rec)
    fake.replies[1004] = motion
    dog.estop()
    assert seen and not any(seen)


EXIT_SCRIPT = r'''
import sys, time
sys.path[:0] = [HERE, ROOT]
from conftest import FakeTransport
from cyberdog_sdk.client import CyberDog
fake = FakeTransport()
def motion(rec):
    n = len(fake.of(1004))
    print("ESTOP %d reached the dog" % n, flush=True)
    if n == 1:
        time.sleep(0.05)
        raise KeyboardInterrupt        # Ctrl+C while waiting for the first reply
    return [{"motion_id": 0, "result": True, "code": 0}]
fake.replies[1004] = motion
dog = CyberDog(transport=fake)
dog.drive.deadman_s = None
dog.drive.move(0.2, 0, 0)
time.sleep(0.3)
dog.estop()
'''


def test_ctrl_c_during_estop_then_exit_still_sends_the_repeat():
    code = "HERE = %r; ROOT = %r\n" % (HERE, ROOT) + EXIT_SCRIPT
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30)
    assert "ESTOP 1 reached the dog" in p.stdout, (p.stdout, p.stderr[-500:])
    assert "ESTOP 2 reached the dog" in p.stdout, "only ESTOP #1 went out: %r" % p.stdout


def test_estop_never_returns_none(dog, fake):
    old = threading.excepthook
    threading.excepthook = lambda a: None

    def motion(rec):
        if len(fake.of(1004)) == 1:
            return [{"motion_id": 0, "result": False, "code": 3012}]
        raise SystemExit("synthetic")
    fake.replies[1004] = motion
    try:
        try:
            r = dog.estop()
        except BaseException as e:          # raising is allowed: the last one raised
            r = e
    finally:
        threading.excepthook = old
    assert r is not None and not dog.drive.arbiter.estopping


# ---- the halt generation: a cancelled move()/set() leaves no state behind -----------------------
class Gate:
    """Wraps a bound method: the first call stalls (before or after the original) until go."""

    def __init__(self, fn, after=False):
        self.fn, self.after = fn, after
        self.stalled, self.go = threading.Event(), threading.Event()

    def __call__(self, *a, **k):
        if not self.after and not self.stalled.is_set():
            self.stalled.set()
            self.go.wait(3.0)
        r = self.fn(*a, **k)
        if self.after and not self.stalled.is_set():
            self.stalled.set()
            self.go.wait(3.0)
        return r


def test_set_cancelled_in_the_handover_leaves_the_target_alone(dog, fake):
    # Posture.set() while walking: claim() ENDs the Driver and keeps the pause; estop() lands there
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    gate = Gate(dog.drive.on_other_started)        # claim() calls it after END + pause
    dog.drive.on_other_started = gate
    s = threading.Thread(target=dog.posture.set, kwargs={"roll": 0.3})
    s.start()
    assert gate.stalled.wait(2.0)
    dog.estop()
    gate.go.set()
    s.join(2.0)
    assert not dog.posture.active and pose_data(fake) == []
    assert dog.posture.target["roll"] == 0.0, dog.posture.target


def test_a_set_cancelled_by_estop_does_not_reach_a_later_set(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    gate = Gate(dog.drive.on_other_started)
    dog.drive.on_other_started = gate
    s = threading.Thread(target=dog.posture.set, kwargs={"roll": 0.3})
    s.start()
    assert gate.stalled.wait(2.0)
    dog.estop()
    gate.go.set()
    s.join(2.0)
    t_after = clock()
    dog.posture.set(z=0.25)
    assert wait_until(lambda: pose_data(fake))
    dog.posture.halt()
    rolls = [s.params["rpy_des"][0] for s in pose_data(fake) if s.t > t_after]
    assert 0.3 not in rolls, rolls[:3]


def test_set_cancelled_while_waiting_for_the_arbiter_leaves_the_target_alone(dog, fake):
    dog.drive.deadman_s = None
    dog.posture.set(z=0.25)
    assert wait_until(lambda: pose_data(fake))
    gate = Gate(dog.posture.on_other_started, after=True)   # move()'s claim, holding arbiter.lock
    dog.posture.on_other_started = gate
    m = threading.Thread(target=dog.drive.move, args=(0.2, 0, 0))
    m.start()
    assert gate.stalled.wait(2.0)
    s = threading.Thread(target=dog.posture.set, kwargs={"roll": 0.3})
    s.start()
    time.sleep(0.05)                               # set() now waits for arbiter.lock
    dog.estop()
    gate.go.set()
    m.join(2.0)
    s.join(2.0)
    assert not dog.posture.active
    assert dog.posture.target["roll"] == 0.0, dog.posture.target


def test_move_cancelled_in_the_handover_leaves_no_velocity(dog, fake):
    dog.posture.set(z=0.25)
    assert wait_until(lambda: pose_data(fake))
    gate = Gate(dog.posture.on_other_started)
    dog.posture.on_other_started = gate
    m = threading.Thread(target=dog.drive.move, args=(0.55, 0, 0))
    m.start()
    assert gate.stalled.wait(2.0)
    dog.estop()
    gate.go.set()
    m.join(2.0)
    assert not dog.drive.active and data(fake) == pose_data(fake)
    assert dog.drive.velocity() == (0.0, 0.0, 0.0), dog.drive.velocity()


def test_set_cancelled_in_the_handover_no_hooks(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    s = threading.Thread(target=dog.posture.set, kwargs={"roll": 0.3})
    s.start()
    assert wait_until(lambda: ends(fake), 1.0)     # claim() sent the Driver END: now in the pause
    dog.estop()
    s.join(2.0)
    assert not dog.posture.active and pose_data(fake) == []
    assert dog.posture.target["roll"] == 0.0, dog.posture.target


def test_set_during_a_driver_start_is_not_wiped_by_the_walking_reset(dog, fake):
    # no halt: a set() that arrives while move() is in claim() applies on top of the walking reset
    dog.drive.deadman_s = None
    dog.posture.set(z=0.25)
    assert wait_until(lambda: pose_data(fake))
    gate = Gate(dog.posture.on_other_started)          # stalls before the reset
    dog.posture.on_other_started = gate
    m = threading.Thread(target=dog.drive.move, args=(0.2, 0, 0))
    m.start()
    assert gate.stalled.wait(2.0)
    out = {}
    s = threading.Thread(target=lambda: out.update(r=dog.posture.set(roll=0.3)))
    s.start()
    time.sleep(0.05)                                   # set() now waits for arbiter.lock
    t_set = clock()
    gate.go.set()
    m.join(2.0)
    s.join(2.0)
    assert wait_until(lambda: [p for p in pose_data(fake) if p.t > t_set])
    time.sleep(0.1)
    dog.posture.halt()
    rolls = [p.params["rpy_des"][0] for p in pose_data(fake) if p.t > t_set]
    assert out["r"]["roll"] == 0.3
    assert rolls and rolls[-1] == 0.3, "set() returned roll 0.3, the stream sends %r" % rolls[:3]


def test_set_during_a_driver_start_no_hooks(dog, fake):
    dog.drive.deadman_s = None
    dog.posture.set(z=0.25)
    assert wait_until(lambda: pose_data(fake))
    m = threading.Thread(target=dog.drive.move, args=(0.2, 0, 0))
    m.start()
    assert wait_until(lambda: ends(fake), 1.0)        # claim() sent the Posture END: now in the pause
    t_set = clock()
    r = dog.posture.set(roll=0.3)                     # waits for the Driver start, then takes over
    m.join(2.0)
    assert wait_until(lambda: [p for p in pose_data(fake) if p.t > t_set])
    time.sleep(0.1)
    dog.posture.halt()
    rolls = [p.params["rpy_des"][0] for p in pose_data(fake) if p.t > t_set]
    assert r["roll"] == 0.3
    assert rolls and rolls[-1] == 0.3, "set() returned roll 0.3, the stream sends %r" % rolls[:3]


def test_deadman_expiry_racing_a_move_restarts(dog, fake):
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 2)
    orig = dog.drive._want
    stalled, go = threading.Event(), threading.Event()

    def want():
        r = orig()
        if not r and not stalled.is_set():
            stalled.set()
            go.wait(2.0)
        return r
    dog.drive._want = want
    assert stalled.wait(2.0)
    m = threading.Thread(target=dog.drive.move, args=(0.37, 0, 0))
    m.start()                       # blocks on _plock, held by the loop
    time.sleep(0.05)
    go.set()
    m.join(2.0)
    assert wait_until(lambda: any(s.params["vel_des"][0] == 0.37 for s in data(fake)))
    dog.drive.halt()


# ---- a halt() that loses the CPU after reading the old thread writes only to that thread ----------
class LateSet:
    """The stream's wake Event; set() from thread `who` stalls before setting the flag."""

    def __init__(self, real, who):
        self.real, self.who = real, who
        self.stalled, self.go = threading.Event(), threading.Event()

    def __getattr__(self, n):
        return getattr(self.real, n)

    def set(self):
        if threading.current_thread().name == self.who and not self.stalled.is_set():
            self.stalled.set()
            self.go.wait(3.0)
        return self.real.set()


def _halt_preempted_at_wake_set_then_move(dog, fake, v=0.31):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    old = dog.drive._thread
    late = LateSet(dog.drive._wake, "estopper")
    dog.drive._wake = late
    e = threading.Thread(target=dog.drive.halt, name="estopper")
    e.start()
    assert late.stalled.wait(2.0)
    assert wait_until(lambda: not old.is_alive())     # the old stream exits on the generation
    dog.drive.move(v, 0, 0)                           # began after halt(): a new stream
    assert wait_until(lambda: any(s.params["vel_des"][0] == v for s in data(fake)))
    late.go.set()
    e.join(2.0)


def test_late_wake_from_halt_leaves_the_new_stream_at_25_hz(dog, fake):
    _halt_preempted_at_wake_set_then_move(dog, fake)
    t0 = clock()
    time.sleep(0.4)
    n = len([s for s in data(fake) if t0 < s.t < t0 + 0.4])
    dog.drive.halt()
    assert n <= 14, "%d frames in 0.4 s (25 Hz is 10)" % n


def test_late_wake_from_halt_keeps_the_gait_switch_pause(dog, fake):
    _halt_preempted_at_wake_set_then_move(dog, fake)
    t0 = clock()
    dog.drive.set_gait(Gait.MEDIUM)
    assert wait_until(lambda: [s for s in data(fake) if s.params["motion_id"] == 308])
    dog.drive.halt()
    end = [s for s in ends(fake) if s.t > t0][0]
    first = [s for s in data(fake) if s.params["motion_id"] == 308][0]
    assert first.t - end.t >= SWITCH_PAUSE - 0.01, "END -> new gait in %.1f ms" % ((first.t - end.t) * 1000)


def test_late_exit_from_halt_does_not_kill_a_stream_started_after_it(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    th_old = dog.drive._thread
    orig_alive = th_old.is_alive
    stalled, go = threading.Event(), threading.Event()

    def alive_then_preempted():
        r = orig_alive()
        if threading.current_thread().name == "estopper" and not stalled.is_set():
            stalled.set()
            go.wait(3.0)
        return r
    th_old.is_alive = alive_then_preempted
    e = threading.Thread(target=dog.drive.halt, name="estopper")
    e.start()
    assert stalled.wait(2.0)
    assert wait_until(lambda: not orig_alive())
    dog.drive.move(0.31, 0, 0)
    assert wait_until(lambda: any(s.params["vel_des"][0] == 0.31 for s in data(fake)))
    go.set()
    e.join(2.0)
    time.sleep(0.3)
    alive = dog.drive.active
    dog.drive.halt()
    assert alive, "the stream of a move() that began after halt() was stopped silently"


# ---- stream thread failures -------------------------------------------------------------------------
def test_crash_is_recorded_even_when_the_crash_end_fails(dog, fake):
    def servo(rec):
        if rec.params["cmd_type"] == 2:
            raise RuntimeError("StatusCode.DEADLINE_EXCEEDED")     # the END is lost on the link
        return [""]
    fake.replies[1002] = servo
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) >= 3)
    dog.drive.gait = 999                # a bug: velocity() fails inside the thread
    assert wait_until(lambda: not dog.drive.active)
    assert len(ends(fake)) == 1         # the END was attempted
    assert "stream stopped" in (dog.drive.stats["last_error"] or ""), dog.drive.stats


def test_base_exception_on_data_then_failing_end_is_recorded(dog, fake):
    old = threading.excepthook
    threading.excepthook = lambda a: None

    def servo(rec):
        if rec.params["cmd_type"] == 2:
            raise RuntimeError("StatusCode.UNAVAILABLE")
        if len(data(fake)) == 3:
            raise SystemExit("synthetic")
        return [""]
    fake.replies[1002] = servo
    try:
        dog.drive.deadman_s = None
        dog.drive.move(0.2, 0, 0)
        assert wait_until(lambda: not dog.drive.active)
        assert "stream stopped" in (dog.drive.stats["last_error"] or ""), dog.drive.stats
    finally:
        threading.excepthook = old


def test_move_during_the_crash_end_restarts_the_stream(dog, fake):
    slow_end = threading.Event()

    def servo(rec):
        if rec.params["cmd_type"] == 2:
            slow_end.set()
            time.sleep(0.15)
        return [""]
    fake.replies[1002] = servo
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) >= 3)
    dog.drive.gait = 999                # crash
    assert slow_end.wait(2.0)
    dog.drive.gait = 303                # the caller fixes it and moves again while the crash END is in flight
    dog.drive.move(0.35, 0, 0)
    time.sleep(0.5)
    assert any(s.params["vel_des"][0] == 0.35 for s in data(fake)), "move() during the crash END was lost"
    dog.drive.halt()


@pytest.mark.parametrize("bad", ["0.05", b"0.05", True, 10 ** 400])
def test_step_height_refuses_non_numbers(dog, bad):
    with pytest.raises(ValueError):
        dog.drive.step_height = bad
    with pytest.raises(ValueError):
        dog.drive.move(bad, 0, 0)
    assert dog.drive.step_height == 0.05


def test_close_twice(fake):
    d = CyberDog(transport=fake)
    d.drive.deadman_s = None
    d.drive.move(0.2, 0, 0)
    assert wait_until(lambda: data(fake))
    d.close()
    d.close()
    assert len(ends(fake)) == 1 and not d.drive.active


def test_close_during_the_deadman_end_counts_the_deadman_stop(fake):
    d = CyberDog(transport=fake)
    in_end = threading.Event()

    def servo(rec):
        if rec.params["cmd_type"] == 2:
            in_end.set()
            time.sleep(0.2)
        return [""]
    fake.replies[1002] = servo
    d.drive.deadman_s = 0.15
    d.drive.move(0.2, 0, 0)
    assert in_end.wait(2.0)
    d.close()
    assert not d.drive.active and len(ends(fake)) == 1 and fake.closed
    assert d.drive.stats["deadman_stops"] == 1


def test_close_during_a_gait_switch(fake):
    d = CyberDog(transport=fake)
    d.drive.deadman_s = None
    d.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) >= 3)
    d.drive.set_gait(308)
    assert wait_until(lambda: ends(fake))
    time.sleep(0.03)
    d.close()
    n = len(fake.of(1002))
    time.sleep(0.3)
    assert len(fake.of(1002)) == n and not d.drive.active
    assert [s.params["motion_id"] for s in ends(fake)] == [303], [s.params for s in ends(fake)]
