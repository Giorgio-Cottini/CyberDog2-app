"""CyberDog high-level client: motion rules, trick sit rules, ESTOP, status and info."""
from __future__ import annotations

import threading
import time

import pytest

from cyberdog_sdk import (CyberDog, DogError, DogUnreachable, ForbiddenCommand, MotionResult, RiskyMotion,
                          Status)
from cyberdog_sdk.client import NOT_SENT_ESTOP, trick_steps
from cyberdog_sdk.servo import ESTOP_LATE_S, WATCHDOG_S


def wait_until(cond, timeout=2.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def status_reply(motion_id):
    return [{"motion_status": {"motion_id": motion_id},
             "task_status": {"task_status": 101, "task_sub_status": 0},
             "self_check_status": {"code": 0, "description": ""},
             "state_switch_status": {"state": 0, "code": 0},
             "charging_status": {"wired_charging": False, "wireless_charging": True}}]


def motions_sent(fake):
    return [s.params["motion_id"] for s in fake.of(1004)]


# ---- motion() -----------------------------------------------------------------------------
def test_motion_exact_json_and_deadline(dog, fake):
    r = dog.motion(111)
    assert r == MotionResult(111, True, 0, "OK", False)
    s = fake.of(1004)[0]
    assert s.raw == '{"motion_id":111,"duration":0}' and s.timeout >= 25.0 and s.lane == "main"


def test_stand_sit_lie_down(dog, fake):
    dog.stand()
    dog.sit()
    dog.lie_down()
    assert motions_sent(fake) == [111, 143, 101]


@pytest.mark.parametrize("mid", [130, 136, 153, 166, 126, 500, 121, 139, 172, 506, 512, 185])
def test_r3_r4_refused_without_allow_risky(dog, fake, mid):
    with pytest.raises(RiskyMotion):
        dog.motion(mid)
    with pytest.raises(RiskyMotion):
        dog.trick(mid)
    assert fake.sent == []


def test_r3_allowed_with_allow_risky(dog, fake):
    assert dog.motion(130, allow_risky=True).result
    assert motions_sent(fake) == [130]


def test_unknown_id_refused(dog, fake):
    with pytest.raises(RiskyMotion):
        dog.motion(999)
    assert fake.sent == []
    dog.motion(999, allow_risky=True)
    assert motions_sent(fake) == [999]


@pytest.mark.parametrize("mid", [113, 114, 115, 400, 203, 390])
def test_never_ids_refused_even_with_allow_risky(dog, fake, mid):
    with pytest.raises(ForbiddenCommand):
        dog.motion(mid, allow_risky=True)
    assert fake.sent == []


@pytest.mark.parametrize("mid", [303, 304, 305, 308])
def test_gaits_refused_on_1004(dog, fake, mid):
    with pytest.raises(ForbiddenCommand):
        dog.motion(mid)
    with pytest.raises(ForbiddenCommand):
        dog.trick(mid)
    assert fake.sent == []


def test_duration_rule(dog, fake):
    with pytest.raises(ValueError):
        dog.motion(143, duration=1000)          # min 15000 > 0: duration must be 0
    with pytest.raises(ValueError):
        dog.motion(211, pos_des=[0, 0, 0.2])     # min -1: duration must be > 0
    assert fake.sent == []
    dog.motion(211, duration=1000, pos_des=[0, 0, 0.2])
    dog.motion(201, rpy_des=[0, 0.1, 0], pos_des=[0, 0, 0.25])
    raws = [s.raw for s in fake.of(1004)]
    assert raws == ['{"motion_id":211,"duration":1000,"pos_des":[0.0,0.0,0.2]}',
                    '{"motion_id":201,"duration":0,"rpy_des":[0.0,0.1,0.0],"pos_des":[0.0,0.0,0.25]}']


def test_pose_ids_need_arrays_and_unknown_kwargs_refused(dog, fake):
    with pytest.raises(ValueError):
        dog.motion(201)
    with pytest.raises(TypeError):
        dog.motion(111, speed=3)
    assert fake.sent == []


def test_untimed_relative_posture_is_refused(dog, fake):
    # 202 with duration 0 is never popped on the dog: it adds rpy_des again every 2 s
    with pytest.raises(ValueError, match="never ends"):
        dog.motion(202, rpy_des=[0.0, 0.1, 0.0])
    assert fake.sent == []
    dog.motion(202, rpy_des=[0.0, 0.1, 0.0], duration=1000)
    assert fake.of(1004)[0].params["duration"] == 1000


def test_motion_value_masked(dog, fake):
    dog.motion(111, value=0x06)
    assert fake.of(1004)[0].params["value"] == 0x02


def test_error_reply(dog, fake):
    fake.replies[1004] = [{"motion_id": 141, "result": False, "code": 3011}]
    r = dog.motion(141)
    assert r == MotionResult(141, False, 3011, "busy: another motion is running", False)
    fake.replies[1004] = []
    assert dog.motion(141).code == -1


@pytest.mark.parametrize("mid,still", [(140, True), (151, True), (176, True), (501, True), (124, True),
                                       (139, True), (143, False), (111, False), (174, False)])
def test_still_running_on_321(dog, fake, mid, still):
    fake.replies[1004] = [{"error_code": 321}]
    r = dog.motion(mid, allow_risky=True)
    assert (r.result, r.code, r.still_running) == (False, 321, still)


def test_322_is_not_still_running(dog, fake):
    fake.replies[1004] = [{"error_code": 322}]
    assert dog.motion(140).still_running is False


# ---- trick() sit rules -------------------------------------------------------------------
@pytest.mark.parametrize("current,target,expected", [
    (143, 141, [111, 141]),       # after a sit: stand first
    (214, 146, [111, 146]),
    (148, 123, [111, 123]),
    (143, 143, [111, 143]),
    (143, 148, [148]),            # sit swings straight from a sit
    (143, 111, [111]),
    (143, 101, [101]),
    (111, 148, [143, 148]),       # sit swings need a sit
    (101, 150, [143, 150]),
    (111, 141, [141]),            # standing: the manager handles it
    (101, 141, [141]),
])
def test_trick_sit_rules(dog, fake, current, target, expected):
    fake.replies[10001] = status_reply(current)
    dog.trick(target)
    assert motions_sent(fake) == expected
    assert fake.codes()[0] == 10001


def test_trick_pre_motion_failure_stops(dog, fake):
    fake.replies[10001] = status_reply(143)
    fake.replies[1004] = lambda rec: [{"motion_id": rec.params["motion_id"], "result": False, "code": 3040}]
    r = dog.trick(141)
    assert motions_sent(fake) == [111] and r.motion_id == 111 and r.code == 3040


def test_estop_during_the_trick_status_read_sends_no_step(dog, fake):
    # a 111 after the ESTOP would clear the latch and the trick would follow (motion.md §2.4)
    def status(rec):
        dog.estop()
        return status_reply(143)
    fake.replies[10001] = status
    r = dog.trick(141)
    assert motions_sent(fake) == [0, 0, 0]
    assert (r.motion_id, r.result, r.code) == (111, False, NOT_SENT_ESTOP)


def test_no_step_while_estop_runs(dog, fake):
    th = []

    def status(rec):
        th.append(threading.Thread(target=dog.estop))
        th[0].start()
        assert wait_until(lambda: dog.drive.arbiter.estopping)
        return status_reply(143)
    fake.replies[10001] = status
    r = dog.trick(141)
    th[0].join(3)
    assert r.code == NOT_SENT_ESTOP and motions_sent(fake) == [0, 0, 0]
    fake.replies[10001] = status_reply(143)
    assert dog.trick(141).code == 0 and motions_sent(fake)[3:] == [111, 141]   # a new trick() goes out


def test_motion_is_not_sent_while_estop_runs(dog, fake):
    gate = threading.Event()
    fake.replies[1004] = lambda rec: (gate.wait(3), [{"motion_id": rec.params["motion_id"], "result": True,
                                                       "code": 0}])[1]
    t = threading.Thread(target=dog.estop)
    t.start()
    assert wait_until(lambda: dog.drive.arbiter.estopping)
    r = dog.motion(111)
    gate.set()
    t.join(3)
    assert r.code == NOT_SENT_ESTOP and 111 not in motions_sent(fake)
    assert dog.motion(111).result is True                           # recovery after estop() returned


def test_unfilled_motion_reply_is_not_ok(dog, fake):
    fake.replies[1004] = [{"motion_id": 0, "result": False, "code": 0}]
    r = dog.motion(141)
    assert r.result is False and r.message != "OK"


def test_trick_refuses_pose_ids(dog, fake):
    with pytest.raises(ForbiddenCommand):
        dog.trick(201)


@pytest.mark.parametrize("status,last,target,expected", [
    (214, None, 148, [148]),          # last motion unknown: the status-only rule
    (214, 143, 148, [148]),           # sat with 143: sitting
    (214, 140, 149, [149]),           # the dance ends sitting (VP SDK motion.cpp:566-571)
    (214, 150, 148, [148]),
    (214, 141, 148, [143, 148]),      # 214 is the filler after a non-sit trick: sit first
    (214, 111, 150, [143, 150]),
    (214, 0, 148, [143, 148]),        # after an ESTOP
    (143, 141, 148, [148]),           # status 143 or 148-150: sitting whatever the last motion
    (148, 146, 149, [149]),
    (111, 143, 148, [143, 148]),      # a standing status wins over a stale last motion
    (214, 141, 146, [111, 146]),      # other targets: 214 still stands first (conservative, ~2 s)
    (214, None, 141, [111, 141]),
    (214, 141, 101, [101]),
    (111, 141, 141, [141]),
])
def test_trick_steps(status, last, target, expected):
    assert trick_steps(target, status, last) == expected


def test_trick_148_after_a_non_sit_trick_sends_143(dog, fake):
    assert dog.last_motion is None
    dog.trick(141)                             # status 111: no pre-motion
    assert dog.last_motion == 141
    fake.replies[10001] = status_reply(214)    # the 21/3 filler after a mode-62 trick
    dog.trick(148)
    assert motions_sent(fake) == [141, 143, 148] and dog.last_motion == 148
    dog.trick(149)                             # sitting since the 143: no second one
    assert motions_sent(fake)[3:] == [149]


def test_trick_148_after_a_sit_skips_143(dog, fake):
    dog.sit()
    fake.replies[10001] = status_reply(214)
    dog.trick(148)
    assert motions_sent(fake) == [143, 148]


def test_last_motion_counts_only_result_true(dog, fake):
    dog.sit()
    fake.replies[1004] = lambda rec: [{"motion_id": rec.params["motion_id"], "result": False, "code": 3040}]
    assert not dog.motion(146).result
    fake.replies[1004] = [{"error_code": 321}]
    assert dog.motion(140).still_running       # still running is not completed
    assert dog.last_motion == 143


def test_estop_sets_last_motion_to_zero(dog, fake):
    dog.stand()
    assert dog.last_motion == 111
    dog.estop()
    assert dog.last_motion == 0
    fake.replies[10001] = status_reply(214)
    dog.trick(148)
    assert motions_sent(fake)[-2:] == [143, 148]


def test_failed_estop_still_sets_last_motion_to_zero(dog, fake):
    dog.sit()

    def down(rec):
        raise DogUnreachable("down")
    fake.replies[1004] = down
    with pytest.raises(DogError):
        dog.estop()
    assert dog.last_motion == 0


def test_motion_replying_after_an_estop_does_not_count(dog, fake):
    gate = threading.Event()

    def slow(rec):
        if rec.params["motion_id"] != 0:
            gate.wait(5)
        return [{"motion_id": rec.params["motion_id"], "result": True, "code": 0}]
    fake.replies[1004] = slow
    th = threading.Thread(target=dog.motion, args=(143,))
    th.start()
    time.sleep(0.05)
    dog.estop()
    gate.set()
    th.join(5)
    assert dog.last_motion == 0


# ---- estop ---------------------------------------------------------------------------------
def test_estop_priority_channel(dog, fake):
    r = dog.estop()
    assert r.result and r.motion_id == 0
    s = fake.of(1004)
    assert len(s) == 3                     # always three times: the repeats outlast any watchdog stop
    assert all(e.lane == "priority" and e.timeout == 3.0 and e.raw == '{"motion_id":0}' for e in s)
    assert s[1].t - s[0].t >= WATCHDOG_S - 0.01 and s[2].t - s[0].t >= ESTOP_LATE_S - 0.01


def test_estop_halts_driver_without_end_and_repeats(dog, fake):
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.2)
    t0 = time.perf_counter()
    dog.estop()
    assert not dog.drive.active
    servo = fake.of(1002)
    assert all(s.params["cmd_type"] == 1 for s in servo)          # no END: halted
    estops = fake.of(1004)
    assert len(estops) == 3 and all(s.lane == "priority" for s in estops)
    assert estops[0].t - t0 < 0.05                                 # first ESTOP is immediate
    assert estops[1].t - servo[-1].t >= WATCHDOG_S - 0.01          # second after the watchdog window
    time.sleep(0.1)
    assert len(fake.of(1002)) == len(servo)


def test_estop_at_the_deadman_decision_sends_no_end(dog, fake):
    # the loop has just decided to end (dead-man) and is preempted when estop() halts it
    deciding = threading.Event()
    orig = dog.drive._want

    def slow_want():
        r = orig()
        if not r and not deciding.is_set():
            deciding.set()
            time.sleep(0.05)
        return r
    dog.drive._want = slow_want
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.1, 0, 0)
    assert deciding.wait(2.0)
    dog.estop()
    first_estop = fake.of(1004)[0].t
    assert [s for s in fake.of(1002) if s.params["cmd_type"] == 2 and s.t > first_estop] == []


def test_estop_after_a_slow_frame_past_the_deadman_sends_no_end(dog, fake):
    slow = threading.Event()

    def reply(rec):
        if rec.params["cmd_type"] == 1 and not slow.is_set():
            slow.set()
            time.sleep(0.3)                  # longer than the dead-man
        return [""]
    fake.replies[1002] = reply
    dog.drive.deadman_s = 0.2
    dog.drive.move(0.1, 0, 0)
    assert slow.wait(2.0)
    time.sleep(0.05)
    dog.estop()
    assert [s for s in fake.of(1002) if s.params["cmd_type"] == 2] == []


def test_estop_repeats_after_a_recent_drive_end_that_failed(dog, fake):
    # the dead-man END is lost on the link: the dog's watchdog is still armed and would
    # overwrite a single ESTOP with WalkStand 112 (motion.md section 3)
    def reply(rec):
        if rec.params["cmd_type"] == 2:
            raise RuntimeError("deadline exceeded")
        return [""]
    fake.replies[1002] = reply
    dog.drive.deadman_s = 0.15
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: not dog.drive.active)
    st = dog.drive.stats
    assert st["ends"] == 0 and st["errors"] == 1 and st["deadman_stops"] == 1
    dog.estop()
    est = fake.of(1004)
    assert len(est) == 3
    assert est[1].t - dog.drive.ended_at >= WATCHDOG_S - 0.01


def test_estop_repeats_inside_the_whole_watchdog_window(dog, fake):
    def reply(rec):
        if rec.params["cmd_type"] == 2:
            raise RuntimeError("deadline exceeded")
        return [""]
    fake.replies[1002] = reply
    dog.drive.deadman_s = 0.15
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: not dog.drive.active)
    time.sleep(0.12)
    dog.estop()
    assert len(fake.of(1004)) == 3
    assert fake.of(1004)[1].t - dog.drive.ended_at >= WATCHDOG_S - 0.01


def test_estop_while_move_waits_in_claim_streams_nothing_after(dog, fake):
    # move() right after stop(): Arbiter.claim() sleeps SWITCH_PAUSE before the thread starts;
    # estop() lands in that sleep, when no Driver thread is alive
    dog.drive.deadman_s = None
    dog.drive.move(0.2, 0, 0)
    time.sleep(0.1)
    dog.drive.stop()
    th = threading.Thread(target=dog.drive.move, args=(0.3, 0, 0))
    th.start()
    time.sleep(0.05)
    assert not dog.drive.active
    dog.estop()
    th.join()
    time.sleep(0.3)
    first = fake.of(1004)[0].t
    after = [s for s in fake.of(1002) if s.t > first]
    assert after == [] and not dog.drive.active, (len(after), dog.drive.active)


def test_estop_during_posture_to_driver_handover(dog, fake):
    dog.drive.deadman_s = None
    dog.posture.set(z=0.25)
    time.sleep(0.1)
    th = threading.Thread(target=dog.drive.move, args=(0.3, 0, 0))
    th.start()
    time.sleep(0.05)                       # claim(): Posture END sent, pause running
    dog.estop()
    th.join()
    time.sleep(0.3)
    first = fake.of(1004)[0].t
    assert [s for s in fake.of(1002) if s.t > first] == [] and not dog.drive.active


def test_estop_long_after_a_drive_end_still_repeats(dog, fake):
    # the SDK cannot know every stream on the dog (another client, a frame in flight), so the
    # repeat does not depend on its own stream history; it waits WATCHDOG_S after the first
    dog.drive.deadman_s = 0.1
    dog.drive.move(0.2, 0, 0)
    assert wait_until(lambda: not dog.drive.active)
    time.sleep(WATCHDOG_S + 0.05)
    dog.estop()
    est = fake.of(1004)
    # the repeat is timed from just before the first send, which a loaded machine can record 10 ms late
    assert len(est) == 3 and WATCHDOG_S - 0.05 <= est[1].t - est[0].t < WATCHDOG_S + 0.1


def test_close_during_the_deadman_end(fake):
    in_end = threading.Event()

    def reply(rec):
        if rec.params["cmd_type"] == 2 and not in_end.is_set():
            in_end.set()
            time.sleep(0.1)
        return [""]
    fake.replies[1002] = reply
    d = CyberDog(transport=fake)
    d.drive.deadman_s = 0.15
    d.drive.move(0.2, 0, 0)
    assert in_end.wait(2.0)
    d.close()
    assert not d.drive.active and len([s for s in fake.of(1002) if s.params["cmd_type"] == 2]) == 1


def test_estop_halts_posture(dog, fake):
    dog.posture.set(z=0.25)
    time.sleep(0.1)
    dog.estop()
    assert not dog.posture.active
    assert all(s.params["cmd_type"] == 1 for s in fake.of(1002))
    assert len(fake.of(1004)) == 3


def test_estop_not_behind_a_running_motion(dog, fake):
    import threading
    gate = threading.Event()

    def slow(rec):
        if rec.params["motion_id"] != 0:
            gate.wait(5)
        return [{"motion_id": rec.params["motion_id"], "result": True, "code": 0}]
    fake.replies[1004] = slow
    th = threading.Thread(target=dog.motion, args=(140,))
    th.start()
    time.sleep(0.05)
    t0 = time.perf_counter()
    assert dog.estop().result
    assert time.perf_counter() - t0 < ESTOP_LATE_S + 0.5
    gate.set()
    th.join()


def test_estop_failure_raises(dog, fake):
    def down(rec):
        raise DogUnreachable("down")
    fake.replies[1004] = down
    with pytest.raises(DogError):
        dog.estop()


# ---- status and info -----------------------------------------------------------------------
def test_status(dog, fake):
    fake.replies[10001] = status_reply(143)
    st = dog.status()
    assert isinstance(st, Status)
    assert (st.motion_id, st.task_status, st.task_sub_status, st.state, st.state_code,
            st.wired_charging, st.wireless_charging, st.self_check_code) == (143, 101, 0, 0, 0, False, True, 0)
    assert st.raw == status_reply(143)[0] and fake.sent[0].raw == "{}" and fake.sent[0].timeout == 3.0


def test_status_error_raises(dog, fake):
    fake.replies[10001] = [{"error_code": 322}]
    with pytest.raises(DogError):
        dog.status()


def test_device_info_flags(dog, fake):
    fake.replies[1001] = [{"bat_info": {"power": 80}, "stand": True}]
    assert dog.device_info()["stand"] is True
    assert fake.sent[0].params == {"is_bat_info": True, "is_stand": True, "is_wifi": True}
    dog.device_info("is_motor_temper", "volume")
    assert fake.sent[1].params == {"is_motor_temper": True, "is_volume": True}
    assert fake.sent[1].timeout == 15.0
    dog.device_info("version")
    assert fake.sent[2].params == {"is_version": True} and fake.sent[2].timeout >= 30.0
    for s in fake.sent:
        assert "is_uid" not in s.params and ("is_version" not in s.params or s is fake.sent[2])
    with pytest.raises(ValueError):
        dog.device_info("uid")


def test_set_volume(dog, fake):
    assert dog.set_volume(150) is True
    dog.set_volume(-3)
    dog.set_volume(42.7)
    assert [s.raw for s in fake.of(1012)] == ['{"volume":100}', '{"volume":0}', '{"volume":42}']
    assert all(s.timeout >= 10.0 for s in fake.of(1012))


def test_skin(dog, fake):
    assert dog.skin(2) is True
    assert [(s.name_code, s.raw) for s in fake.sent] == [(11001, '{"data":true}'),
                                                         (11002, '{"mode":2,"wave_cycle_time":3000}')]
    with pytest.raises(ValueError):
        dog.skin(7)


def test_close_stops_streams_and_transport(fake):
    d = CyberDog(transport=fake)
    d.drive.move(0.1, 0, 0)
    time.sleep(0.1)
    d.close()
    assert not d.drive.active and fake.closed
    assert [s.params["cmd_type"] for s in fake.of(1002)][-1] == 2
