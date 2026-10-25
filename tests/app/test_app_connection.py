"""Layer (a): the state poller (connection, latency, battery, emulator flag), reconnection
with a fresh client, and the timing rules (POSTs answer within about 1 s, sensors in the
background, motions on the worker)."""
from __future__ import annotations

import threading

import pytest

from cyberdog_sdk import DogError

from apphelp import StubDog, clock


def test_connected_latency_battery(h):
    h.wait(lambda: h.app.soc == 80, what="1001 battery")
    st = h.state()
    assert st["connected"] is True and isinstance(st["latency_ms"], int) and st["latency_ms"] >= 0
    assert st["battery"] == {"soc": 80, "charging": False}
    h.dog.wired = True
    h.dog.soc = 55
    h.wait(lambda: h.state()["battery"] == {"soc": 55, "charging": True}, what="battery update")
    assert any("connesso al cane" in e["msg"] for e in h.app.logs)


def test_emulator_flag_from_the_serial_number(make_harness):
    emu = make_harness()
    h_emu = emu.wait(lambda: emu.app.emulator is not None, what="sn")
    assert h_emu and emu.state()["emulator"] is True
    real = StubDog()
    real.sn = "TZ2241CN0000123"
    h = make_harness(dog=real)
    h.wait(lambda: h.app.emulator is not None, what="sn")
    assert h.state()["emulator"] is False
    assert ("device_info", ("bat_info", "sn", "wifi"), {"timeout": 8.0}) in real.calls


class _WifiOnly(StubDog):
    """An older image: 1001 gives no "sn"; the wifi name is the fallback."""

    def __init__(self, wifi):
        super().__init__()
        self.wifi = wifi

    def device_info(self, *flags, timeout=15.0):
        self._check()
        self._rec("device_info", *flags)
        return {"bat_info": {"power": 70, "is_charging": False}, "wifi": {"name": self.wifi}}


class _NoInfo(StubDog):
    def device_info(self, *flags, timeout=15.0):
        self._rec("device_info", *flags)
        return {"bat_info": {"power": 70, "is_charging": False}}


def test_emulator_flag_fallbacks(make_harness):
    h = make_harness(dog=_WifiOnly("emulator"))
    h.wait(lambda: h.app.emulator is not None, what="wifi fallback")
    assert h.state()["emulator"] is True
    h2 = make_harness(dog=_WifiOnly("HOTSPOT"))
    h2.wait(lambda: h2.app.emulator is not None, what="wifi fallback")
    assert h2.state()["emulator"] is False
    # neither answer: asked 3 times, then "not the emulator" (the safe label is the real dog)
    d = _NoInfo()
    h3 = make_harness(dog=d)
    h3.wait(lambda: h3.app.emulator is not None, 3.0, "sn retries")
    assert h3.state()["emulator"] is False
    assert sum(1 for c in d.calls if c[0] == "device_info" and "sn" in c[1]) == 3


def test_loopback_target_is_not_assumed_to_be_the_emulator(make_harness):
    real = StubDog()
    real.sn = "TZ2241CN0000123"         # e.g. an SSH tunnel to the real dog on 127.0.0.1
    h = make_harness(dog=real)
    h.app.target = "127.0.0.1:50052"
    h.wait(lambda: h.app.emulator is not None)
    assert h.state()["emulator"] is False


def test_lost_connection_rebuild_and_reconnect(h):
    first = h.dog
    first.reachable = False
    h.wait(lambda: not h.state()["connected"], 2.0, "disconnected")
    st = h.state()
    assert st["latency_ms"] is None and st["battery"]["soc"] is None and st["nav"]["state"] == "unknown"
    assert st["motion"]["id"] is None and st["emulator"] is False
    assert any("connessione con il cane persa" in e["msg"] for e in h.app.logs)
    h.wait(lambda: len(h.dogs) >= 2 and h.state()["connected"], 3.0, "rebuilt and reconnected")
    assert h.dog is not first and h.app.rebuilds >= 1
    h.wait(lambda: first.closed, 2.0, "old client closed")
    h.wait(lambda: h.app.emulator is True, 2.0, "emulator flag asked again")
    assert h.dog.drive.gaits[-1] == 303          # the gait is re-applied on the new client


def test_channels_are_warmed_at_start_and_after_a_rebuild(h):
    # without connect() the first ESTOP of each client waits for TCP + mTLS (Gazebo: about 0.1 s)
    h.wait(lambda: h.dog.connects >= 1, 2.0, "warm-up at start")
    first = h.dog
    first.reachable = False
    h.wait(lambda: len(h.dogs) >= 2 and h.dog is not first, 4.0, "rebuilt")
    h.wait(lambda: h.dog.connects >= 1, 2.0, "warm-up of the new client")


def test_gate_and_motion_refused_while_unreachable(h):
    h.arm()
    h.dog.reachable = False
    h.post("/api/motion", {"id": 141})
    h.wait(lambda: not h.state()["motion"]["busy"])
    res = h.state()["motion"]["last_result"]
    assert res["ok"] is False and "non risponde" in res["message"]


def test_quick_posts_answer_within_about_1_s(make_harness):
    """A dog that does not answer: the POST still returns after QUICK_S (1 s) with "pending"."""
    h = make_harness(quick_s=1.0)
    gate = h.dog.block("led")
    t0 = clock()
    s, b = h.post("/api/led", {"target": "head", "color": "#ff0000"})
    assert (s, b) == (200, {"ok": True, "pending": True}) and 0.9 <= clock() - t0 < 1.4, clock() - t0
    gate.set()
    h.dog.task_status = 1
    h.wait(lambda: h.app.task_status == 1)
    gate = h.dog.block("nav_stop")
    t0 = clock()
    s, b = h.post("/api/nav/stop", {})                  # nav stop: never gated, never stuck
    assert (s, b) == (200, {"ok": True, "stopped": True, "pending": True}) and clock() - t0 < 1.4
    gate.set()
    h.wait(lambda: any("in ritardo" in e["msg"] for e in h.app.logs)
           and any(e["msg"] == "navigazione fermata" for e in h.app.logs), 2.0, "late outcomes logged")


def test_a_second_nav_stop_while_one_runs_sends_no_second_6009(h):
    h.dog.task_status, h.dog.task_sub = 1, 307              # goal sent: a type 1 stop
    gate = h.dog.block("nav_stop")
    assert h.post("/api/nav/stop", {}) == (200, {"ok": True, "stopped": True, "pending": True})
    assert h.dog.entered["nav_stop"].is_set()
    t0 = clock()
    assert h.post("/api/nav/stop", {}) == (200, {"ok": True, "stopped": True, "pending": True})
    assert clock() - t0 < 0.2                               # answered at once: no second 6009 waits
    gate.set()
    h.wait(lambda: any(e["msg"] == "navigazione fermata" for e in h.app.logs), 2.0, "late outcome logged")
    assert h.dog.nav.stops == [1]
    assert h.post("/api/nav/stop", {})[0] == 200
    assert h.dog.nav.stops == [1, 1]                        # the flag clears when the stop call ends


def test_a_failed_nav_stop_clears_the_flag(h):
    h.dog.task_status = 0                                   # wedged: refused before any 6009
    assert h.post("/api/nav/stop", {})[0] == 502
    h.dog.task_status, h.dog.task_sub = 1, 307
    assert h.post("/api/nav/stop", {}) == (200, {"ok": True, "stopped": True, "result": 0})


@pytest.mark.parametrize("result, level, text", [
    (1, "error", "non è riuscito a fermare il compito"),
    (10, "warn", "ancora in corso"),
])
def test_a_late_nav_stop_outcome_is_logged_by_result(h, result, level, text):
    h.dog.task_status, h.dog.task_sub = 1, 307
    nav, gate = h.dog.nav, threading.Event()

    def stop(task_type):
        gate.wait(5)
        nav.stops.append(task_type)
        return result
    nav.stop = stop
    assert h.post("/api/nav/stop", {})[1].get("pending") is True
    gate.set()
    h.wait(lambda: any(e["level"] == level and text in e["msg"] for e in h.app.logs), 2.0, "late outcome logged")


def _start_in_thread(h, body):
    out = {}
    th = threading.Thread(target=lambda: out.update(r=h.post("/api/nav/start", body)), daemon=True)
    th.start()
    return th, out


def test_a_dock_accepted_after_an_estop_is_stopped(h):
    h.arm()
    h.dog.nav.delay = 0.5
    th, out = _start_in_thread(h, {"task": "dock"})
    h.wait(lambda: h.dog.nav.started, 1.0, "6004 sent")
    assert h.post("/api/estop", {})[0] == 200
    th.join(5)
    status, body = out["r"]
    assert status == 409 and body["cancelled"] is True and "annullato da STOP" in body["error"]
    assert h.dog.nav.stops == [9]
    assert h.app.nav_task is None


def test_a_goto_accepted_after_a_nav_stop_waits_for_that_stop(h):
    # the stop of the task before still runs (ResetNav) when the accept lands: the second 6009 waits for it
    h.arm()
    h.dog.task_status, h.dog.task_sub = 1, 307
    h.dog.nav.delay = 0.4
    gate = h.dog.block("nav_stop")
    sent, stop = [], h.dog.nav.stop
    h.dog.nav.stop = lambda t: (sent.append(t), stop(t))[1]     # counted on entry: the stub blocks first
    th, out = _start_in_thread(h, {"task": "goto", "x": 1.0, "y": 0.5})
    h.wait(lambda: h.dog.nav.started, 1.0, "6004 sent")
    assert h.post("/api/nav/stop", {})[1].get("pending") is True
    h.wait(lambda: any("dopo «Ferma navigazione»" in e["msg"] for e in h.app.logs), 2.0, "late accept seen")
    clock_end = clock() + 0.3
    while clock() < clock_end:
        assert sent == [1]                              # no type 0 into the running stop
    h.dog.task_sub = 0                                  # the new goto has no feedback 300 yet
    gate.set()
    th.join(5)
    status, body = out["r"]
    assert status == 409 and body["cancelled"] is True and "annullato da «Ferma navigazione»" in body["error"]
    assert body["error"].endswith("e l'ho fermato")
    assert h.dog.nav.stops == [1, 0]
    assert h.app.nav_task is None and not h.app.nav_stopping


def test_a_late_stop_that_finds_nothing_running_is_not_an_error(h):
    # the stop before ended the accepted goto too: the second 6009 answers FAILED (status 101), which is fine
    h.arm()
    h.dog.nav.delay = 0.5
    stop = h.dog.nav.stop
    h.dog.nav.stop = lambda t: (stop(t), 1)[1]          # every 6009 answers FAILED
    th, out = _start_in_thread(h, {"task": "goto", "x": 1.0, "y": 0.5})
    h.wait(lambda: h.dog.nav.started, 1.0, "6004 sent")
    assert h.post("/api/estop", {})[0] == 200
    th.join(5)
    status, body = out["r"]
    assert status == 409 and body["error"].endswith("e l'arresto precedente l'ha fermato")
    assert h.dog.nav.stops == [0]
    assert not [e for e in h.app.logs if e["level"] == "error"]


def test_a_late_stop_the_dog_refuses_says_so(h):
    h.arm()
    h.dog.nav.delay = 0.5
    th, out = _start_in_thread(h, {"task": "dock"})
    h.wait(lambda: h.dog.nav.started, 1.0, "6004 sent")
    h.dog.task_status = 9                               # the dock runs
    stop = h.dog.nav.stop
    h.dog.nav.stop = lambda t: (stop(t), 1)[1]
    assert h.post("/api/estop", {})[0] == 200
    th.join(5)
    status, body = out["r"]
    assert status == 409 and "non sono riuscito a fermarlo" in body["error"]
    assert h.dog.nav.stops == [9]
    assert any(e["level"] == "error" and "non è riuscito a fermarlo" in e["msg"] for e in h.app.logs)


def test_nav_start_checks_the_gate_where_it_reads_the_stop_counters(h, monkeypatch):
    # an ESTOP between _gate() and the counters disarms: the start is refused, not sent
    monkeypatch.setattr(h.app, "_gate", lambda: None)
    status, body = h.post("/api/nav/start", {"task": "dock"})
    assert status == 403 and body["armed"] is False
    assert h.dog.nav.started == []


def test_a_nav_stop_whose_thread_cannot_start_clears_the_flag(h, monkeypatch):
    from apphelp import server
    h.dog.task_status, h.dog.task_sub = 1, 307
    real = server.threading.Thread.start

    def refuse(self):
        if self.name == "call-ferma navigazione":
            raise RuntimeError("can't start new thread")
        return real(self)
    monkeypatch.setattr(server.threading.Thread, "start", refuse)
    assert h.post("/api/nav/stop", {})[0] == 502
    assert not h.app.nav_stopping
    monkeypatch.setattr(server.threading.Thread, "start", real)
    assert h.post("/api/nav/stop", {}) == (200, {"ok": True, "stopped": True, "result": 0})


def test_a_step_not_sent_for_an_estop_says_so(h):
    from cyberdog_sdk import MotionResult
    from apphelp import server
    gate, inside = threading.Event(), threading.Event()

    def motion(motion_id, **kw):
        inside.set()
        gate.wait(5)
        return MotionResult(motion_id, False, server.NOT_SENT_ESTOP, "not sent: ESTOP", False)
    h.dog.motion = motion
    h.arm()
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert inside.wait(2)
    assert h.post("/api/estop", {})[0] == 200
    gate.set()
    h.wait(lambda: any("non inviato, arresto di emergenza in corso" in e["msg"] for e in h.app.logs), 2.0,
           "not-sent log")
    assert not any("risposta arrivata dopo" in e["msg"] for e in h.app.logs)


def test_quick_posts_pending_bodies(make_harness):
    h = make_harness(quick_s=0.2)
    for name, path, body, extra in (("skin", "/api/skin", {"mode": 1}, {"mode": 1}),
                                    ("volume", "/api/volume", {"level": 20}, {"level": 20})):
        gate = h.dog.block(name)
        t0 = clock()
        s, b = h.post(path, body)
        assert s == 200 and b == dict({"ok": True, "pending": True}, **extra), (path, b)
        assert clock() - t0 < 0.6
        gate.set()


def test_sensors_refresh_returns_at_once(h):
    h.dog.sensor_delay = 0.5
    t0 = clock()
    assert h.post("/api/sensors/refresh", {}) == (200, {"ok": True, "pending": True})
    assert clock() - t0 < 0.3
    assert h.state()["sensors"]["pending"] is True
    assert h.post("/api/sensors/refresh", {}) == (200, {"ok": True, "pending": True})    # no second read
    h.wait(lambda: not h.state()["sensors"]["pending"], 3.0, "sensor data")
    sen = h.state()["sensors"]
    assert sen["t"] is not None and sen["error"] is None
    assert sen["bms"]["batt_soc"] == 80 and sen["ultrasonic"]["range"] is None      # NaN -> null
    assert sum(1 for c in h.dog.calls if c[0] == "read_sensors") == 1
    assert ("read_sensors", (), {"timeout": 2.0}) in h.dog.calls


def test_sensors_failure_is_reported(h):
    h.dog.sensor_error = DogError("vp: no result")
    h.post("/api/sensors/refresh", {})
    h.wait(lambda: not h.state()["sensors"]["pending"], 3.0)
    sen = h.state()["sensors"]
    assert sen["t"] is None and "sensori" in sen["error"]
    assert any(e["level"] == "error" and e["msg"].startswith("sensori:") for e in h.app.logs)


def test_motion_post_returns_before_the_motion_ends(h):
    h.arm()
    h.dog.motion_delay = 0.5
    t0 = clock()
    assert h.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert clock() - t0 < 0.3
    assert h.state()["motion"]["busy"] is True
    h.wait(lambda: not h.state()["motion"]["busy"], 3.0)
    assert h.state()["motion"]["last_result"]["ok"] is True


def test_map_and_nav_start_wait_for_the_accept_reply(h):
    h.arm()
    h.dog.nav.delay = 0.3
    t0 = clock()
    assert h.post("/api/nav/start", {"task": "mapping"}) == (200, {"ok": True, "accepted": 1})
    assert 0.25 <= clock() - t0 < 0.6
