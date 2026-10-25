"""ESTOP and stream races found in review round 3: the always-repeated ESTOP, the halt
generation against restarting streams, close(), and failures inside the stream thread."""
from __future__ import annotations

import threading
import time

import pytest

from cyberdog_sdk.client import CyberDog
from cyberdog_sdk.servo import ESTOP_LATE_S, SWITCH_PAUSE, WATCHDOG_S, clock
from cyberdog_sdk.transport import DogUnreachable


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


def estop_ok(rec):
    return [{"motion_id": rec.params["motion_id"], "result": True, "code": 0}]


# ---- the second ESTOP --------------------------------------------------------------------
def test_second_estop_does_not_wait_for_the_first_reply(dog, fake):
    # ESTOP's 1004 can take up to its 1000 ms min_exec_time to answer; the watchdog's WalkStand
    # comes 200-250 ms after the last frame, so the repeat must not queue behind that reply
    def motion(rec):
        if len(fake.of(1004)) == 1:
            time.sleep(0.8)
        return estop_ok(rec)
    fake.replies[1004] = motion
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    r = dog.estop()
    est = fake.of(1004)
    assert r.result and len(est) == 3
    # the repeat is timed from the SDK's clock and the halted stream's end, so measure it from the
    # last walk frame; the fake's receive time of ESTOP #1 can lag the SDK's under load
    assert est[1].t - data(fake)[-1].t >= WATCHDOG_S - 0.01, est[1].t - data(fake)[-1].t
    assert est[1].t - est[0].t < WATCHDOG_S + 0.15, est[1].t - est[0].t


@pytest.mark.parametrize("delay_s,reply_s", [(0.0, 0.0), (0.0, 0.35), (0.12, 0.0), (0.12, 0.2)])
def test_repeat_after_a_lost_drive_end_does_not_depend_on_the_first_reply(dog, fake, delay_s, reply_s):
    def servo(rec):
        if rec.params["cmd_type"] == 2:
            raise RuntimeError("deadline exceeded")     # the dead-man END is lost on the link
        return [""]

    def motion(rec):
        time.sleep(reply_s)
        return estop_ok(rec)
    fake.replies[1002] = servo
    fake.replies[1004] = motion
    dog.drive.deadman_s = 0.15
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: not dog.drive.active)
    time.sleep(delay_s)
    dog.estop()
    est = fake.of(1004)
    assert len(est) == 3
    assert est[1].t - dog.drive.ended_at >= WATCHDOG_S - 0.01


def test_a_move_during_estop_is_dropped(dog, fake):
    # the estop() thread loses the CPU after the halts and a move() begins then: while estop()
    # runs no stream starts (round 4), so no walk frame can arm the watchdog before ESTOP #1
    # lands; a move() after estop() returns starts a stream as usual
    dog.drive.deadman_s = None
    orig = dog.posture.halt
    moved = threading.Event()

    def halt_then_preempted():
        r = orig()
        threading.Thread(target=lambda: (dog.drive.move(0.3, 0, 0), moved.set()), daemon=True).start()
        assert moved.wait(1.0)          # move() returned at once: dropped
        return r
    dog.posture.halt = halt_then_preempted
    dog.estop()
    est = fake.of(1004)
    assert data(fake) == [] and not dog.drive.active and dog.drive.velocity() == (0.0, 0.0, 0.0)
    assert len(est) == 3 and est[1].t - est[0].t >= WATCHDOG_S - 0.01
    dog.drive.move(0.3, 0, 0)
    assert wait_until(lambda: data(fake))
    dog.drive.halt()


def test_joystick_during_estop_neither_blocks_it_nor_delays_the_repeat(dog, fake):
    # a move() during a slow ESTOP #1 restarts the stream; estop() must wait only for the
    # stream it halted, and repeat WATCHDOG_S after that stream's last frame
    def motion(rec):
        if len(fake.of(1004)) == 1:
            time.sleep(0.2)
        return estop_ok(rec)
    fake.replies[1004] = motion
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    stop = threading.Event()

    def joystick():
        while not stop.is_set():
            dog.drive.move(0.2, 0, 0)
            time.sleep(0.04)
    th = threading.Thread(target=dog.estop)
    j = threading.Thread(target=joystick)
    t0 = clock()
    th.start()
    time.sleep(0.01)
    j.start()
    th.join(3)
    dt = clock() - t0
    stop.set()
    j.join(2)
    dog.drive.halt()
    est = fake.of(1004)
    last_old = max(s.t for s in data(fake) if s.t < est[0].t)
    assert dt < ESTOP_LATE_S + 0.5, dt
    assert len(est) == 3 and est[1].t - last_old < WATCHDOG_S + 0.15, est[1].t - last_old


def test_estop_while_a_stream_thread_is_being_created(dog, fake, monkeypatch):
    # _start publishes the Thread before start() returns; join() on it would raise
    orig_start = threading.Thread.start
    in_start = threading.Event()

    def slow_start(self):
        if self.name == "Driver":
            in_start.set()
            time.sleep(0.05)
        return orig_start(self)
    monkeypatch.setattr(threading.Thread, "start", slow_start)
    m = threading.Thread(target=dog.drive.move, args=(0.2, 0, 0))
    m.start()
    assert in_start.wait(2.0)
    r = dog.estop()
    m.join(2.0)
    assert r.result and len(fake.of(1004)) == 3
    dog.drive.join(1.0)                 # the public join() is safe in that window too
    dog.drive.halt()


def test_estop_while_walking_and_a_restart_is_being_created(dog, fake, monkeypatch):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    orig_start = threading.Thread.start

    def slow_start(self):
        if self.name == "Driver":
            time.sleep(0.05)
        return orig_start(self)
    monkeypatch.setattr(threading.Thread, "start", slow_start)

    def motion(rec):
        if len(fake.of(1004)) == 1:
            threading.Thread(target=dog.drive.move, args=(0.3, 0, 0), daemon=True).start()
            time.sleep(0.17)            # the move restarts the stream at +0.15 s
        return estop_ok(rec)
    fake.replies[1004] = motion
    r = dog.estop()
    assert r.result and len(fake.of(1004)) == 3
    dog.drive.halt()


def test_a_failing_estop_rpc_does_not_stop_the_other(dog, fake):
    def motion(rec):
        if len(fake.of(1004)) == 1:
            raise ValueError("Cannot invoke RPC on closed channel!")
        return estop_ok(rec)
    fake.replies[1004] = motion
    r = dog.estop()
    assert r.result and len(fake.of(1004)) == 3


# ---- the halt generation -----------------------------------------------------------------
def test_move_streams_when_it_begins_while_halt_is_preempted(dog, fake):
    # halt() has counted the halt but not yet marked the old stream; the old stream exits on
    # the generation, and a move() that began after the count must start a new one
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) > 3)
    th_old = dog.drive._thread
    orig_alive = th_old.is_alive
    in_halt, resume = threading.Event(), threading.Event()

    def stalled_is_alive():
        if threading.current_thread().name == "estopper" and not in_halt.is_set():
            in_halt.set()
            resume.wait(2.0)
        return orig_alive()
    th_old.is_alive = stalled_is_alive
    e = threading.Thread(target=dog.drive.halt, name="estopper")
    e.start()
    assert in_halt.wait(2.0)
    assert dog.drive._halts != dog.drive._gen and dog.drive._exit is None
    dog.drive.move(0.31, 0, 0)
    resume.set()
    e.join()
    time.sleep(0.3)
    assert any(s.params["vel_des"][0] == 0.31 for s in data(fake)) and dog.drive.active
    dog.drive.halt()


def test_move_after_halt_is_not_eaten_by_a_doomed_thread(dog, fake, monkeypatch):
    # halt() lands between _start's last check and thread.start(): the new thread is doomed
    # but starts slowly; a move() after halt() must not take it for a live stream
    from cyberdog_sdk import servo
    dog.drive.deadman_s = None
    hit = []

    class HookedRun(servo._Run):        # _start creates the stream's _Run right there
        __slots__ = ()

        def __init__(self):
            super().__init__()
            if not hit:
                hit.append(1)
                t = threading.Thread(target=dog.drive.halt)
                t.start()
                t.join()
    monkeypatch.setattr(servo, "_Run", HookedRun)
    orig_loop = dog.drive._loop

    def late_loop():
        time.sleep(0.2)
        orig_loop()
    dog.drive._loop = late_loop
    dog.drive.move(0.2, 0, 0)           # its thread is doomed by the halt at _Run()
    assert hit
    dog.drive.move(0.33, 0, 0)
    time.sleep(0.6)
    assert any(s.params["vel_des"][0] == 0.33 for s in data(fake))
    assert not any(s.params["vel_des"][0] == 0.2 for s in data(fake))
    dog.drive.halt()


class StallFirst:
    """Wraps _plock: the first acquire from thread `who` waits until go is set."""

    def __init__(self, inner, who):
        self.inner, self.who = inner, who
        self.stalled, self.go = threading.Event(), threading.Event()

    def __enter__(self):
        if threading.current_thread().name == self.who and not self.stalled.is_set():
            self.stalled.set()
            self.go.wait(2.0)
        return self.inner.__enter__()

    def __exit__(self, *a):
        return self.inner.__exit__(*a)


def test_a_cancelled_move_leaves_no_velocity_behind(dog, fake):
    dog.drive.deadman_s = None
    gate = StallFirst(dog.drive._plock, "m1")
    dog.drive._plock = gate
    m1 = threading.Thread(target=dog.drive.move, args=(0.55, 0, 0), name="m1")
    m1.start()                          # read the generation, then waits for the lock
    assert gate.stalled.wait(2.0)
    dog.estop()
    dog.drive.move(0.05, 0, 0)
    assert wait_until(lambda: data(fake))
    gate.go.set()
    m1.join(2.0)
    time.sleep(0.3)
    assert 0.55 not in [s.params["vel_des"][0] for s in data(fake)]
    assert dog.drive.velocity()[0] == 0.05
    dog.drive.halt()


def test_a_cancelled_posture_set_leaves_the_target_alone(dog, fake):
    gate = StallFirst(dog.posture._plock, "p1")
    dog.posture._plock = gate
    p1 = threading.Thread(target=dog.posture.set, kwargs={"z": 0.2}, name="p1")
    p1.start()
    assert gate.stalled.wait(2.0)
    dog.estop()
    gate.go.set()
    p1.join(2.0)
    time.sleep(0.2)
    assert dog.posture.target["z"] == 0.235 and not dog.posture.active
    assert [s for s in fake.of(1002) if s.params["motion_id"] == 201] == []


# ---- Posture against the walking reset -----------------------------------------------------
class SlowFloat:
    def __init__(self, v, delay):
        self.v, self.delay = v, delay

    def __float__(self):
        time.sleep(self.delay)
        return self.v


def test_walking_reset_of_the_held_pose_is_not_undone_by_a_concurrent_set(dog, fake):
    dog.posture.set(roll=0.3)
    dog.posture.stop()                  # pose held: roll 0.3
    time.sleep(SWITCH_PAUSE)
    p = threading.Thread(target=dog.posture.set, kwargs={"z": SlowFloat(0.25, 0.3)})
    p.start()                           # set() is converting its argument
    time.sleep(0.05)
    dog.drive.move(0.1, 0, 0)           # claim(): walking resets the held pose
    time.sleep(0.1)
    p.join(2)
    assert wait_until(lambda: dog.posture.active)
    time.sleep(0.1)
    last = [s for s in fake.of(1002) if s.params["motion_id"] == 201][-1].params
    assert last["rpy_des"][0] == 0.0 and last["pos_des"][2] == 0.25
    dog.posture.halt()


# ---- close() -------------------------------------------------------------------------------
def test_close_while_a_move_waits_in_claim_leaves_no_stream(fake):
    d = CyberDog(transport=fake)
    d.drive.deadman_s = None
    d.posture.set(z=0.25)
    time.sleep(0.1)
    th = threading.Thread(target=d.drive.move, args=(0.2, 0, 0))
    th.start()
    time.sleep(0.05)                    # move() is inside claim(): Posture END sent, pausing
    d.close()
    t_closed = clock()
    th.join(2)
    time.sleep(0.3)
    assert not d.drive.active and [s for s in data(fake) if s.t > t_closed] == []


def test_move_after_close_does_nothing(fake):
    d = CyberDog(transport=fake)
    d.drive.deadman_s = None
    d.close()
    d.drive.move(0.2, 0, 0)
    d.posture.set(z=0.25)
    time.sleep(0.3)
    assert not d.drive.active and not d.posture.active and fake.of(1002) == []


def test_close_sends_one_end_for_a_walking_stream(fake):
    d = CyberDog(transport=fake)
    d.drive.deadman_s = None
    d.drive.move(0.2, 0, 0)
    assert wait_until(lambda: data(fake))
    d.close()
    assert not d.drive.active and len(ends(fake)) == 1


# ---- failures inside the stream thread -------------------------------------------------------
def test_step_height_is_checked_when_set(dog, fake):
    with pytest.raises(TypeError):
        dog.drive.step_height = None
    with pytest.raises(ValueError):
        dog.drive.step_height = float("nan")
    assert dog.drive.step_height == 0.05
    dog.drive.step_height = 0.5         # accepted; clamped to the doc range when sent
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: data(fake))
    dog.drive.stop()
    assert data(fake)[0].params["step_height"] == [0.06, 0.06]


def test_a_crash_in_the_stream_sends_one_end_and_reports_it(dog, fake):
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: len(data(fake)) >= 3)
    dog.drive.gait = 999                # not a gait: velocity() fails inside the thread
    assert wait_until(lambda: not dog.drive.active)
    e = ends(fake)
    assert len(e) == 1 and e[0].params["motion_id"] == 303 and e[0].params["vel_des"] == [0.0, 0.0, 0.0]
    st = dog.drive.stats
    assert st["errors"] == 1 and "stream stopped" in st["last_error"]


def test_a_base_exception_from_the_transport_still_ends_with_an_end(dog, fake):
    old = threading.excepthook
    threading.excepthook = lambda a: None

    def servo(rec):
        if rec.params["cmd_type"] == 1 and len(data(fake)) == 3:
            raise SystemExit("synthetic")
        return [""]
    fake.replies[1002] = servo
    try:
        dog.drive.deadman_s = None
        dog.drive.move(0.2, 0, 0)
        assert wait_until(lambda: not dog.drive.active)
        assert len(ends(fake)) == 1
    finally:
        threading.excepthook = old


def test_max_gap_counts_only_frames_that_went_through(dog, fake):
    n = [0]

    def servo(rec):
        if rec.params["cmd_type"] == 1:
            n[0] += 1
            if n[0] % 2 == 0:
                raise RuntimeError("unavailable")     # never reached the dog
        return [""]
    fake.replies[1002] = servo
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.6)
    dog.drive.stop()
    ok = [s.t for i, s in enumerate(data(fake)) if i % 2 == 0]
    real = max(b - a for a, b in zip(ok, ok[1:])) * 1000
    assert dog.drive.stats["max_gap_ms"] >= real - 5


def _refused_on(lane_refused, status):
    def reply(rec):
        if rec.lane == lane_refused:
            raise DogUnreachable("1004: %s" % status.name, status=status, name_code=1004)
        return [{"motion_id": 0, "result": True, "code": 0}]
    return reply


def test_an_estop_the_priority_channel_refuses_goes_on_the_main_lane(fake, dog):
    import grpc
    fake.replies[1004] = _refused_on("priority", grpc.StatusCode.UNAVAILABLE)
    r = dog.estop()
    assert r.result is True
    assert [s.lane for s in fake.of(1004)] == ["priority", "main"] * 3


def test_an_estop_deadline_is_not_sent_again_on_the_main_lane(fake, dog):
    import grpc
    fake.replies[1004] = _refused_on("priority", grpc.StatusCode.DEADLINE_EXCEEDED)
    with pytest.raises(DogUnreachable):
        dog.estop()
    assert [s.lane for s in fake.of(1004)] == ["priority"] * 3
