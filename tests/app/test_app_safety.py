"""Layer (a): the safety rules of contracts.md section 1 as the app enforces them (stub SDK)."""
from __future__ import annotations

import itertools
import re
import threading
import time

import pytest

from apphelp import SERVER_PY, StubDog, clock, server

GATED = [("/api/drive", {"x": 1, "y": 0, "z": 0}), ("/api/action", {"name": "stand"}),
         ("/api/motion", {"id": 141, "confirm": False}), ("/api/posture", {"pitch": 0.1}),
         ("/api/posture/reset", {}), ("/api/nav/start", {"task": "mapping"}),
         ("/api/nav/save_map", {"name": "casa"})]


def wait_idle(h, timeout=3.0):
    h.wait(lambda: not h.state()["motion"]["busy"], timeout, "motion finished")
    return h.state()["motion"]["last_result"]


# ---- the movement gate ------------------------------------------------------
def test_gate_is_off_at_start(h):
    assert h.state()["armed"] is False and h.app.armed is False


@pytest.mark.parametrize("path,body", GATED)
def test_every_gated_endpoint_is_403_while_disarmed(h, path, body):
    s, b = h.post(path, body)
    assert s == 403 and b["ok"] is False and b["code"] is None and b["armed"] is False
    assert "disabilitato" in b["error"]
    time.sleep(0.05)
    d = h.dog
    assert d.motions == [] and d.drive.moves == [] and d.posture.sets == [] and d.nav.started == []
    assert d.nav.saves == []


def test_gate_checked_before_input_validation(h):
    assert h.post("/api/motion", {"id": "nonsense"})[0] == 403
    assert h.post("/api/drive", {"x": "fast"})[0] == 403


def test_disarm_stops_drive_posture_and_queued_steps(h):
    h.arm()
    assert h.post("/api/drive", {"x": 1})[0] == 200
    assert h.dog.drive.active
    s, b = h.post("/api/arm", {"armed": False})
    assert (s, b) == (200, {"ok": True, "armed": False})
    assert not h.dog.drive.active and len(h.dog.drive.stops) == 1
    # posture stream
    h.arm()
    assert h.post("/api/posture", {"z": 0.25})[0] == 200
    assert h.dog.posture.active
    h.post("/api/arm", {"armed": False})
    assert not h.dog.posture.active and h.dog.posture.stops == 1
    # queued motion: sitting, so 141 needs 111 first; disarm during 111 -> 141 never goes out
    h.dog.motion_id = 143
    gate = h.dog.block("motion")
    h.arm()
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert h.dog.entered["motion"].wait(2)
    h.post("/api/arm", {"armed": False})
    gate.set()
    res = wait_idle(h)
    assert h.dog.motions == [111]
    assert res["ok"] is False and res["message"] == "annullato: movimento disabilitato"


def test_motion_needs_the_gate_again_after_disarm(h):
    h.arm()
    h.post("/api/arm", {"armed": False})
    assert h.post("/api/action", {"name": "stand"})[0] == 403


# ---- ESTOP -------------------------------------------------------------------
def test_estop_never_gated_disarms_and_uses_the_sdk_estop(h):
    s, b = h.post("/api/estop", {})
    assert s == 200 and b["ok"] is True and b["armed"] is False and b["result"]["id"] == 0
    h.arm()
    h.post("/api/drive", {"x": 1})
    s, b = h.post("/api/estop", {})
    assert s == 200 and h.state()["armed"] is False
    assert len(h.dog.estops) == 2 and h.dog.drive.halts >= 2
    assert 0 not in h.dog.motions               # ESTOP is CyberDog.estop(), not a queued motion(0)
    assert h.state()["motion"]["last_result"]["id"] == 0


def test_estop_not_blocked_by_a_slow_call_holding_the_dog_lock(h):
    h.arm()
    gate = h.dog.block("posture")
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("r", h.post("/api/posture", {"pitch": 0.1})))
    th.start()
    assert h.dog.entered["posture"].wait(2)
    assert h.app.dog_lock.locked()
    t0 = clock()
    s, b = h.post("/api/estop", {})
    dt = clock() - t0
    assert s == 200 and b["ok"] is True and dt < 0.5, dt
    assert h.app.dog_lock.locked()               # the slow call still holds it
    assert len(h.dog.estops) == 1
    gate.set()
    th.join(3)
    assert out["r"][0] == 403                    # the posture call returns into a disarmed app
    assert not h.dog.posture.active


def test_estop_not_queued_behind_a_running_motion(h):
    h.arm()
    h.dog.block("motion")
    assert h.post("/api/motion", {"id": 141})[0] == 200
    assert h.dog.entered["motion"].wait(2)
    t0 = clock()
    s, b = h.post("/api/estop", {})
    assert s == 200 and clock() - t0 < 0.5
    st = h.state()
    assert st["motion"]["busy"] is False and st["armed"] is False
    assert st["motion"]["last_result"]["id"] == 0
    # the 1004 of 141 returns (3021) after the ESTOP and does not overwrite the state
    h.wait(lambda: any("dopo l'arresto" in e["msg"] for e in h.app.logs), what="late reply logged")
    assert h.state()["motion"]["last_result"]["id"] == 0
    # recovery: 111 is accepted at once (no 409 busy)
    h.arm()
    h.dog.blocks.pop("motion")
    h.dog._abort.clear()
    assert h.post("/api/action", {"name": "stand"}) == (200, {"ok": True, "queued": True})
    assert wait_idle(h)["id"] == 111


# ---- the drive dead-man and speed limits ------------------------------------
def test_drive_deadman_stops_after_400_ms(h):
    h.arm()
    t0 = clock()
    assert h.post("/api/drive", {"x": 1})[0] == 200
    h.wait(lambda: h.dog.drive.stops, 2.0, "dead-man END")
    dt = h.dog.drive.stops[0] - t0
    assert 0.38 <= dt <= 0.6, dt
    assert h.state()["drive"]["active"] is False
    assert any("400 ms" in e["msg"] for e in h.app.logs)


def test_drive_keeps_walking_while_resent_every_150_ms(h):
    h.arm()
    end = clock() + 1.0
    while clock() < end:
        assert h.post("/api/drive", {"x": 0.5})[0] == 200
        time.sleep(0.15)
    assert h.dog.drive.stops == [] and h.dog.drive.active
    assert h.state()["drive"]["active"] is True
    h.wait(lambda: h.dog.drive.stops, 1.0, "dead-man END")


def test_drive_all_zeros_stops_at_once(h):
    h.arm()
    h.post("/api/drive", {"x": 1, "z": 1})
    t0 = clock()
    s, b = h.post("/api/drive", {"x": 0, "y": 0, "z": 0})
    assert (s, b) == (200, {"ok": True, "active": False, "vx": 0.0, "vy": 0.0, "wz": 0.0})
    assert len(h.dog.drive.stops) == 1 and h.dog.drive.stops[0] - t0 < 0.1


def test_drive_default_speed_never_above_v1(h):
    h.arm()
    for x, y, z in itertools.product((-1, 0, 1), repeat=3):
        if (x, y, z) == (0, 0, 0):
            continue
        s, b = h.post("/api/drive", {"x": x, "y": y, "z": z})
        assert s == 200
        _, vx, vy, wz = h.dog.drive.moves[-1]
        assert abs(vx) <= 0.2 + 1e-9 and abs(vy) <= 0.15 + 1e-9 and abs(wz) <= 0.4 + 1e-9, (vx, vy, wz)
    h.post("/api/drive", {"x": 1, "y": 1, "z": 1})
    assert h.dog.drive.moves[-1][1:] == pytest.approx((0.2, 0.08, 0.4))      # 0.2 m/s forward by default


def test_drive_clamps_sticks_and_scales_by_gait_and_speed(h):
    h.arm()
    s, b = h.post("/api/drive", {"x": 5, "y": -7, "z": 2})
    assert s == 200 and (b["vx"], b["vy"], b["wz"]) == pytest.approx((0.2, -0.08, 0.4))
    h.post("/api/speed", {"level": 1.0})
    h.post("/api/drive", {"x": -3, "y": 1, "z": -1})
    assert h.dog.drive.moves[-1][1:] == pytest.approx((-0.5, 0.2, -1.0))
    h.post("/api/gait", {"gait": "fast"})
    h.post("/api/drive", {"x": 1})
    assert h.dog.drive.moves[-1][1:] == pytest.approx((1.6, 0.0, 0.0))
    h.post("/api/drive", {"x": -1})
    assert h.dog.drive.moves[-1][1:] == pytest.approx((-1.2, 0.0, 0.0))      # the gait's own reverse limit
    h.post("/api/speed", {"level": 0.25})
    h.post("/api/drive", {"x": 0.5, "y": -0.5})
    assert h.dog.drive.moves[-1][1:] == pytest.approx((0.2, -0.06875, 0.0))
    st = h.state()["drive"]
    assert st["active"] is True and st["gait"] == "fast" and st["vx"] == pytest.approx(0.2)
    assert h.post("/api/drive", {"x": "1"})[0] == 400
    assert h.post("/api/drive", {"x": True})[0] == 400


# ---- the motion catalog -----------------------------------------------------
@pytest.mark.parametrize("bad", [130, 136, 121, 139, 172, 303, 201, 400, 0, 9999, "141", True, 141.5, None])
def test_motion_refuses_ids_outside_the_catalog(h, bad):
    h.arm()
    s, b = h.post("/api/motion", {"id": bad, "confirm": True})
    assert s == 400 and b["ok"] is False and b["code"] is None
    time.sleep(0.05)
    assert h.dog.motions == [] and h.dog.calls == [c for c in h.dog.calls if c[0] != "motion"]


def test_r2_needs_confirm(h):
    h.arm()
    s, b = h.post("/api/motion", {"id": 140})
    assert s == 409 and b["need_confirm"] is True and b["id"] == 140 and b["ok"] is False
    assert "pavimento libero?" in b["error"]
    assert h.post("/api/motion", {"id": 140, "confirm": False})[0] == 409
    assert h.dog.motions == []
    assert h.post("/api/motion", {"id": 140, "confirm": True}) == (200, {"ok": True, "queued": True})
    assert wait_idle(h)["ok"] is True and h.dog.motions == [140]
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})     # R0: no confirm


def test_never_force_or_allow_risky(h):
    h.arm()
    for body in ({"id": 141}, {"id": 140, "confirm": True}, {"id": 148}):
        h.post("/api/motion", body)
        wait_idle(h)
    h.post("/api/action", {"name": "stand_sit_toggle"})
    wait_idle(h)
    motion_calls = [c for c in h.dog.calls if c[0] == "motion"]
    assert motion_calls and all(c[2]["allow_risky"] is False for c in motion_calls)
    assert all(set(c[2]) == {"duration", "allow_risky"} and c[2]["duration"] == 0 for c in motion_calls)
    src = open(SERVER_PY, encoding="utf-8").read()
    code = "\n".join(line.split("#")[0] for line in src.splitlines())
    assert not re.search(r"allow_risky\s*=\s*True|force\s*=\s*True|confirm_delete_all|set_labels|\.trick\(",
                         code)
    assert ".transport" not in code             # no raw access around the SDK guards


def test_motion_busy_409_and_last_result(h):
    h.arm()
    gate = h.dog.block("motion")
    t0 = clock()
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert clock() - t0 < 0.3                    # never waits for the 1004 result
    assert h.dog.entered["motion"].wait(2)
    assert h.state()["motion"]["busy"] is True
    for path, body in (("/api/motion", {"id": 123}), ("/api/action", {"name": "sit"}),
                       ("/api/drive", {"x": 1}), ("/api/posture", {"z": 0.2})):
        s, b = h.post(path, body)
        assert s == 409 and b["busy"] is True and b["ok"] is False, path
    s, b = h.post("/api/nav/stop", {})
    assert s == 200                              # nav stop and ESTOP are always accepted
    assert h.post("/api/drive", {"x": 0, "y": 0, "z": 0})[0] == 200     # a key release is never refused
    gate.set()
    res = wait_idle(h)
    assert res == {"id": 141, "ok": True, "code": 0, "message": "OK", "still_running": False, "seq": 1}


def test_last_result_seq_counts_every_result_estop_included(h):
    h.arm()
    seqs = []
    for _ in range(2):                           # the same trick twice: only seq tells the two results apart
        assert h.post("/api/motion", {"id": 141})[0] == 200
        seqs.append(wait_idle(h)["seq"])
    s, b = h.post("/api/estop", {})
    assert s == 200 and b["result"]["id"] == 0
    seqs.append(b["result"]["seq"])
    assert h.state()["motion"]["last_result"]["seq"] == seqs[-1]
    assert h.post("/api/motion", {"id": 141})[0] == 403         # refusals are not results
    h.arm()
    h.dog._abort.clear()
    h.dog.results[111] = (False, 3040)
    assert h.post("/api/action", {"name": "stand"})[0] == 200
    res = wait_idle(h)
    seqs.append(res["seq"])
    assert res["ok"] is False and res["code"] == 3040
    assert seqs == [1, 2, 3, 4]


# ---- a 321 "still running" (motion.md: later 1004s get 3011 until the motion ends) ----
def _start_321(h, mid=140):
    h.arm()
    h.dog.motion_id = 111
    h.dog.still.add(mid)
    assert h.post("/api/motion", {"id": mid, "confirm": True}) == (200, {"ok": True, "queued": True})
    h.wait(lambda: any("il movimento continua" in e["msg"] for e in h.app.logs), 3.0, "321 logged")


def test_321_keeps_busy_until_the_poll_shows_the_dog_left_the_motion(h):
    _start_321(h)
    assert any(e["level"] == "warn" and "il movimento continua" in e["msg"] for e in h.app.logs)
    time.sleep(0.3)                              # about 6 polls while the dog still reports 140
    st = h.state()["motion"]
    assert st["busy"] is True and st["id"] == 140 and st["name"] == "raccolta di balli"
    assert st["last_result"] is None             # one result per job, when it really ends
    for path, body in (("/api/motion", {"id": 141}), ("/api/action", {"name": "lie"}),
                       ("/api/drive", {"x": 1}), ("/api/posture", {"z": 0.2})):
        s, b = h.post(path, body)
        assert s == 409 and b["busy"] is True, path
    assert h.dog.motions == [140]                # nothing else reached the dog
    h.dog.motion_id = 214                        # the dance ends (sit filler)
    res = wait_idle(h)
    assert res["id"] == 140 and res["ok"] is False and res["code"] == 321 and res["still_running"] is False
    assert "senza conferma" in res["message"] and res["seq"] == 1
    assert any(e["level"] == "warn" and "terminato" in e["msg"] for e in h.app.logs)
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert wait_idle(h)["ok"] is True and h.dog.motions == [140, 111, 141]


def test_321_wait_needs_two_polls_away_from_the_motion(make_harness):
    h = make_harness(poll_s=0.3)
    _start_321(h)
    job = h.app.job
    h.dog.motion_id = 214
    h.wait(lambda: job.away == 1, 2.0, "one poll away from 140")
    h.dog.motion_id = 140                        # a single odd sample does not end the wait
    time.sleep(0.7)
    assert h.state()["motion"]["busy"] is True and job.away == 0
    h.dog.motion_id = 214
    h.wait(lambda: not h.state()["motion"]["busy"], 2.0, "busy cleared")


def test_321_busy_ends_at_the_cap_with_a_warning(make_harness):
    assert server.linger_cap_s(140) == 90.0 and server.linger_cap_s(151) == 45.0
    assert server.linger_cap_s(176) == 180.0 and server.linger_cap_s(124) == 30.0
    h = make_harness(linger_margin=0.02)         # 140: 0.02 x 60 s = 1.2 s from the 1004 send
    t0 = clock()
    _start_321(h)                                # the dog never leaves 140
    h.wait(lambda: not h.state()["motion"]["busy"], 4.0, "busy cleared at the cap")
    dt = clock() - t0
    assert 1.1 <= dt <= 2.5, dt
    res = h.state()["motion"]["last_result"]
    assert res["id"] == 140 and res["ok"] is False and res["code"] == 321 and res["still_running"] is True
    assert "potrebbe essere ancora in corso" in res["message"]
    assert any(e["level"] == "warn" and "potrebbe essere ancora in corso" in e["msg"] for e in h.app.logs)


def test_estop_clears_a_321_busy_at_once(h):
    _start_321(h)
    assert h.state()["motion"]["busy"] is True
    t0 = clock()
    s, b = h.post("/api/estop", {})
    assert s == 200 and b["ok"] is True and clock() - t0 < 0.5
    st = h.state()["motion"]
    assert st["busy"] is False and st["last_result"]["id"] == 0 and st["last_result"]["seq"] == b["result"]["seq"]
    h.dog.motion_id = 214
    time.sleep(0.3)                              # the worker leaves without touching the state
    assert h.state()["motion"]["last_result"]["id"] == 0
    assert not any("terminato" in e["msg"] for e in h.app.logs)


def test_disarm_keeps_a_321_busy_the_dog_is_still_moving(h):
    _start_321(h)
    assert h.post("/api/arm", {"armed": False})[0] == 200
    time.sleep(0.2)
    assert h.state()["motion"]["busy"] is True
    h.dog.motion_id = 214
    assert wait_idle(h)["code"] == 321


def test_321_wait_lets_the_client_be_rebuilt(h):
    """Losing the link during the wait must not block the client rebuild until the cap."""
    _start_321(h)
    nxt = StubDog()
    nxt.motion_id = 140                          # the same dog, still dancing, after the rebuild
    h.next_dog = nxt
    h.dog.reachable = False
    h.wait(lambda: h.dog is nxt and h.state()["connected"], 3.0, "rebuilt and reconnected")
    time.sleep(0.2)
    assert h.state()["motion"]["busy"] is True
    nxt.motion_id = 214
    assert wait_idle(h)["code"] == 321


def test_motion_failure_is_reported_in_italian(h):
    h.arm()
    h.dog.results[141] = (False, 3011)
    h.post("/api/motion", {"id": 141})
    res = wait_idle(h)
    assert res["ok"] is False and res["code"] == 3011 and res["message"] == "occupato: un altro movimento è in corso"
    assert any(e["level"] == "error" and "occupato" in e["msg"] for e in h.app.logs)


def test_sit_rules_and_stand_sit_toggle(h):
    h.arm()
    d = h.dog

    def run(body, motion_id):
        d.motion_id = motion_id
        start = len(d.motions)
        assert h.post("/api/action" if "name" in body else "/api/motion", body)[0] == 200
        wait_idle(h)
        return d.motions[start:]
    assert run({"name": "stand_sit_toggle"}, 101) == [111]      # lying -> stand
    assert run({"name": "stand_sit_toggle"}, 0) == [111]        # after ESTOP -> stand
    assert run({"name": "stand_sit_toggle"}, 111) == [143]      # standing -> sit
    assert run({"name": "stand_sit_toggle"}, 214) == [111]      # sitting -> stand
    assert run({"id": 141}, 143) == [111, 141]                  # after a sit, 111 first
    assert run({"id": 148}, 111) == [143, 148]                  # 148-150 need a sit first
    assert run({"id": 149}, 143) == [149]
    assert run({"id": 101}, 143) == [101]
    assert run({"name": "lie"}, 111) == [101]
    assert run({"name": "sit"}, 111) == [143]
    assert h.post("/api/action", {"name": "jump"})[0] == 400


def test_148_after_a_standing_trick_sits_first(h):
    """The dog reports 214 after every mode-62 trick, sitting or not (SDK trick_steps)."""
    h.arm()
    d = h.dog
    for prev, want in ((141, [143, 148]), (143, [148])):
        assert h.post("/api/motion", {"id": prev})[0] == 200
        wait_idle(h)
        assert d.last_motion == prev
        d.motion_id = 214
        start = len(d.motions)
        assert h.post("/api/motion", {"id": 148})[0] == 200
        wait_idle(h)
        assert d.motions[start:] == want, prev


def test_stream_stops_before_a_motion(h):
    h.arm()
    h.post("/api/drive", {"x": 1})
    h.post("/api/action", {"name": "sit"})
    wait_idle(h)
    assert len(h.dog.drive.stops) == 1 and not h.dog.drive.active


# ---- posture -----------------------------------------------------------------
def test_posture_passed_unmodified_in_radians_and_metres(h):
    h.arm()
    s, b = h.post("/api/posture", {"pitch": 0.1, "z": 0.25})
    assert s == 200 and h.dog.posture.sets[-1] == {"pitch": 0.1, "z": 0.25}
    assert b["posture"] == {"z": 0.25, "roll": 0.0, "pitch": 0.1, "yaw": 0.0}
    s, b = h.post("/api/posture", {"pitch": -0.9, "roll": 0.2, "yaw": -0.45})
    assert h.dog.posture.sets[-1] == {"pitch": -0.9, "roll": 0.2, "yaw": -0.45}     # the server does not clamp
    assert b["posture"]["pitch"] == -0.2 and b["posture"]["roll"] == 0.2           # the SDK does
    assert h.state()["posture"] == b["posture"]
    assert h.post("/api/posture", {})[0] == 400
    assert h.post("/api/posture", {"pitch": "up"})[0] == 400
    assert h.post("/api/posture/reset", {})[0] == 200 and h.dog.posture.resets == 1
    assert h.state()["posture"] == {"z": 0.235, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def test_posture_takes_over_from_the_drive(h):
    h.arm()
    h.post("/api/drive", {"x": 1})
    h.post("/api/posture", {"z": 0.2})
    assert not h.dog.drive.active and h.state()["drive"]["active"] is False


# ---- navigation ----------------------------------------------------------------
def test_nav_start_tasks_and_types(h):
    h.arm()
    nav = h.dog.nav
    assert h.post("/api/nav/start", {"task": "goto", "x": 1.0})[0] == 400
    assert h.post("/api/nav/start", {"task": "goto", "y": 1.0})[0] == 400
    assert h.post("/api/nav/start", {"task": "goto", "x": "a", "y": 1})[0] == 400
    assert nav.started == []
    assert h.post("/api/nav/start", {"task": "goto", "x": 1.5, "y": -0.5}) == (200, {"ok": True, "accepted": 1})
    assert nav.started[-1] == (1, (1.5, -0.5))
    st = h.state()["nav"]
    assert st["task"] == "goto" and st["goal"] == [1.5, -0.5]
    for task, typ in (("mapping", 5), ("relocalize", 7), ("dock", 9)):
        assert h.post("/api/nav/start", {"task": task}) == (200, {"ok": True, "accepted": 1})
        assert nav.started[-1][0] == typ
    for bad in ("reset", "0", 0, None, "track"):
        assert h.post("/api/nav/start", {"task": bad})[0] == 400
    assert {t for t, _ in nav.started} <= {1, 5, 7, 9} and 0 not in {t for t, _ in nav.started}


def test_nav_start_rejected_and_unanswered(h):
    h.arm()
    h.dog.nav.accept = 2
    s, b = h.post("/api/nav/start", {"task": "goto", "x": 1, "y": 1})
    assert s == 502 and b["accepted"] == 2 and "rilocalizza" in b["error"]
    h.dog.nav.accept, h.dog.nav.error = 1, 322
    s, b = h.post("/api/nav/start", {"task": "mapping"})
    assert s == 502 and b["code"] == 322 and b["error"] == "creazione della mappa: servizio non disponibile"
    h.dog.nav.error, h.dog.nav.delay = None, None          # never answers
    t0 = clock()
    s, b = h.post("/api/nav/start", {"task": "dock"})
    assert s == 502 and b["code"] is None and 0.5 <= clock() - t0 < 1.5
    assert h.state()["nav"]["task"] is None


def test_nav_save_map(h):
    h.arm()
    assert h.post("/api/nav/save_map", {"name": "casa"})[0] == 409       # not mapping: fake success avoided
    h.dog.task_status = 5
    h.wait(lambda: h.app.task_status == 5)
    for bad in ("", "a/b", "../x", "x" * 33, "città", 5, None):
        assert h.post("/api/nav/save_map", {"name": bad})[0] == 400, bad
    assert h.post("/api/nav/save_map", {"name": "casa_2"}) == (200, {"ok": True, "accepted": 1, "result": 0})
    assert h.dog.nav.saves == ["casa_2"]
    h.dog.nav.save_result = 1
    s, b = h.post("/api/nav/save_map", {"name": "casa"})
    assert s == 502 and b["code"] == 1
    h.dog.nav.save_delay = 2.0
    s, b = h.post("/api/nav/save_map", {"name": "lento"})
    assert (s, b) == (200, {"ok": True, "accepted": 1, "pending": True})


def test_nav_stop_types_and_never_gated(h):
    d = h.dog
    # an AB task before feedback 300 needs type 0: type 1 then is a no-op and the goal still goes out
    for status, sub, stop_type in ((1, 307, 1), (1, 1001, 0), (21, 300, 1), (8, 0, 0), (7, 0, 0), (9, 0, 9),
                                   (5, 0, 5)):
        d.task_status, d.task_sub = status, sub
        s, b = h.post("/api/nav/stop", {})
        assert s == 200 and b["stopped"] is True and d.nav.stops[-1] == stop_type, (status, sub)
    assert any("prima della partenza" in e["msg"] for e in h.app.logs)
    d.task_status, d.task_sub = 101, 0
    assert h.post("/api/nav/stop", {}) == (200, {"ok": True, "stopped": False})
    d.task_status = 0
    s, b = h.post("/api/nav/stop", {})
    assert s == 502 and "riavvio" in b["error"]


def test_no_delete_all_path(h):
    for path in server.POST_ROUTES:
        assert "delete" not in path and "label" not in path
    for path in ("/api/map/delete", "/api/nav/delete", "/api/map/labels", "/api/labels"):
        assert h.post(path, {"confirm": True})[0] == 404
    assert h.dog.nav.labels_calls == []


def test_no_drive_or_posture_stream_during_autonomous_navigation(h):
    h.arm()
    for status in (1, 9, 11, 13):           # goto, dock, tracking: nav_map.md "do not stream 1002"
        h.dog.task_status = status
        h.wait(lambda: h.app.task_status == status)
        s, b = h.post("/api/drive", {"x": 1})
        assert s == 409 and b["navigating"] is True and "navigazione" in b["error"], status
        assert h.post("/api/posture", {"pitch": 0.1})[0] == 409
        assert h.post("/api/drive", {"x": 0, "y": 0, "z": 0})[0] == 200    # a stop always goes through
    assert h.dog.drive.moves == [] and h.dog.posture.sets == []
    for status in (5, 7, 8, 101):           # mapping and relocalization are driven by hand
        h.dog.task_status = status
        h.wait(lambda: h.app.task_status == status)
        assert h.post("/api/drive", {"x": 1})[0] == 200, status
    h.post("/api/drive", {"x": 0})
    h.dog.task_status = 8
    h.wait(lambda: h.app.task_status == 8)
    assert h.post("/api/nav/start", {"task": "goto", "x": 1, "y": 1})[0] == 200
    assert h.post("/api/drive", {"x": 1})[0] == 409        # accepted goto, before the poller sees it run
    assert h.post("/api/nav/stop", {})[0] == 200
    assert h.post("/api/drive", {"x": 1})[0] == 200


def test_motion_and_action_refused_during_autonomous_navigation(h):
    """What the dog does with a 1004 during goto, dock or tracking is unverified: the app refuses it."""
    h.arm()
    for status in (1, 9, 11, 13):
        h.dog.task_status = status
        h.wait(lambda: h.app.task_status == status)
        for path, body in (("/api/motion", {"id": 141}), ("/api/motion", {"id": 140}),
                           ("/api/action", {"name": "sit"}), ("/api/action", {"name": "stand_sit_toggle"})):
            s, b = h.post(path, body)
            assert s == 409 and b["ok"] is False and b["navigating"] is True, (status, path)
            assert "«Ferma navigazione»" in b["error"] and "need_confirm" not in b, b
        assert h.state()["motion"]["busy"] is False
        assert h.post("/api/nav/stop", {})[0] == 200        # never gated
    s, b = h.post("/api/estop", {})                          # never gated either
    assert s == 200 and b["ok"] is True
    assert h.dog.motions == [] and len(h.dog.estops) == 1
    h.arm()
    h.dog._abort.clear()
    for status in (5, 7, 8, 101):                            # mapping and relocalization allow motions
        h.dog.task_status = status
        h.wait(lambda: h.app.task_status == status)
        assert h.post("/api/action", {"name": "stand"}) == (200, {"ok": True, "queued": True}), status
        wait_idle(h)
    h.dog.task_status = 8
    h.wait(lambda: h.app.task_status == 8)
    assert h.post("/api/nav/start", {"task": "goto", "x": 1, "y": 1})[0] == 200
    assert h.post("/api/motion", {"id": 141})[0] == 409      # accepted goto, before the poller sees it run
    assert h.post("/api/nav/stop", {})[0] == 200
    assert h.post("/api/motion", {"id": 141})[0] == 200


def test_nav_start_refused_while_a_motion_runs(h):
    h.arm()
    h.dog.task_status = 8
    h.wait(lambda: h.app.task_status == 8)
    gate = h.dog.block("motion")
    assert h.post("/api/action", {"name": "sit"})[0] == 200
    assert h.dog.entered["motion"].wait(2)
    for body in ({"task": "goto", "x": 1, "y": 1}, {"task": "dock"}, {"task": "mapping"},
                 {"task": "relocalize"}):
        s, b = h.post("/api/nav/start", body)
        assert s == 409 and b["busy"] is True, (body, s, b)
    assert h.dog.nav.started == []
    gate.set()
    wait_idle(h)
    assert h.post("/api/nav/start", {"task": "goto", "x": 1, "y": 1})[0] == 200


def test_no_motion_while_a_goto_waits_for_the_accept(h):
    h.arm()
    h.dog.task_status = 8
    h.wait(lambda: h.app.task_status == 8)
    h.dog.nav.delay = 0.4           # the dog's accept takes seconds; FAST nav_accept_s is 0.6
    out = {}
    th = threading.Thread(target=lambda: out.setdefault(
        "r", h.post("/api/nav/start", {"task": "goto", "x": 1, "y": 1})))
    th.start()
    h.wait(lambda: h.dog.nav.started, 2, "goto sent")
    s, b = h.post("/api/motion", {"id": 141})
    assert s == 409 and b["navigating"] is True, (s, b)
    assert h.post("/api/drive", {"x": 1})[0] == 409
    th.join(3)
    assert out["r"][0] == 200 and h.dog.motions == []
    assert h.app.nav_starting == 0


def test_safety_and_status_paths_never_wait_on_the_dog_lock(h):
    """A move() that restarts a stream holds dog_lock (about 0.4 s in the SDK); here it hangs."""
    h.arm()
    gate = h.dog.block("move")
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("r", h.post("/api/drive", {"x": 1})))
    th.start()
    assert h.dog.entered["move"].wait(2) and h.app.dog_lock.locked()
    for method, path, body, want in (("GET", "/api/state", None, 200), ("POST", "/api/nav/stop", {}, 200),
                                     ("POST", "/api/motion", {"id": 141}, 200),
                                     ("POST", "/api/arm", {"armed": False}, 200), ("POST", "/api/estop", {}, 200)):
        t0 = clock()
        s, _, b = h.request(method, path, body)
        dt = clock() - t0
        assert s == want and dt < 0.5, (path, s, b, dt)
    assert h.app.dog_lock.locked()                           # the slow move() still holds it
    gate.set()
    th.join(3)
    assert out["r"][0] == 403 and not h.dog.drive.active     # the drive returns into a disarmed app
    time.sleep(0.2)
    assert h.dog.motions == []                               # the queued 141 was cancelled before its 1004
