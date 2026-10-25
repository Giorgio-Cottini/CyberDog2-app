"""Layer (b): app2/server.py against the real emulator, both as subprocesses on free ports.

    python -m emulator --port <free> --control-port <free> --time-scale 0.05
    python app2/server.py --dog 127.0.0.1:<port> --port <free>

The emulator's control endpoint (/state, /reset) is the ground truth: what the
dog actually received. If the emulator does not start, every test here fails
with its traceback."""
from __future__ import annotations

import http.client
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

import pytest

from apphelp import ROOT, RawStream, clock

FIXED = {50052, 50080, 8794}
STARTUP_S = 25.0


def free_port():
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        if port not in FIXED:
            return port


def http_json(port, method, path, body=None, timeout=10.0):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=timeout)
    try:
        hdrs = {}
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            hdrs["Content-Type"] = "application/json"
        conn.request(method, path, body=data, headers=hdrs)
        r = conn.getresponse()
        return r.status, json.loads(r.read() or b"null")
    finally:
        conn.close()


def _read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


class Env:
    def __init__(self, tmp):
        self.tmp = tmp
        self.procs = []
        self.fs_root = None
        self.env = dict(os.environ)
        paths = ([os.path.join(ROOT, "pylibs")] if os.name == "nt" else []) + [ROOT]   # pylibs: Windows wheels
        self.env["PYTHONPATH"] = os.pathsep.join(paths)
        self.env["PYTHONIOENCODING"] = "utf-8"
        self.env["PYTHONUNBUFFERED"] = "1"

    def spawn(self, name, args):
        log = os.path.join(self.tmp, name + ".log")
        f = open(log, "w", encoding="utf-8")
        p = subprocess.Popen([sys.executable] + args, cwd=ROOT, env=self.env, stdout=f, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
        f.close()
        self.procs.append(p)
        return p, log

    def start(self):
        self.emu_port, self.ctl_port, self.app_port = free_port(), free_port(), free_port()
        p, self.emu_log = self.spawn("emulator", ["-m", "emulator", "--port", str(self.emu_port), "--control-port",
                                                  str(self.ctl_port), "--time-scale", "0.05"])
        end = clock() + STARTUP_S
        while "emulator: gRPC" not in _read(self.emu_log):
            if p.poll() is not None or clock() > end:
                return "the emulator did not start:\n" + _read(self.emu_log)
            time.sleep(0.05)
        line = [ln for ln in _read(self.emu_log).splitlines() if ln.startswith("emulator: gRPC")][0]
        if ", files " in line:
            self.fs_root = line.split(", files ", 1)[1].strip()
        q, self.app_log = self.spawn("app", [os.path.join(ROOT, "app2", "server.py"), "--dog",
                                             "127.0.0.1:%d" % self.emu_port, "--port", str(self.app_port)])
        while True:
            if q.poll() is not None:
                return "app2/server.py exited:\n" + _read(self.app_log)
            try:
                s, st = http_json(self.app_port, "GET", "/api/state", timeout=2.0)
                if s == 200 and st["connected"] and st["emulator"]:
                    return None
            except OSError:
                pass
            if clock() > end:
                raise AssertionError("app2/server.py never connected to the emulator:\n" + _read(self.app_log))
            time.sleep(0.1)

    def stop(self):
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(5)
                except subprocess.TimeoutExpired:
                    p.kill()
        # the emulator's temporary file tree (a killed process cannot remove it itself)
        tmp = os.path.realpath(tempfile.gettempdir())
        if self.fs_root and os.path.realpath(self.fs_root).startswith(tmp + os.sep):
            shutil.rmtree(self.fs_root, ignore_errors=True)

    # helpers
    def app(self, method, path, body=None, timeout=10.0):
        return http_json(self.app_port, method, path, body, timeout)

    def post(self, path, body=None):
        return self.app("POST", path, {} if body is None else body)

    def state(self):
        s, st = self.app("GET", "/api/state")
        assert s == 200
        return st

    def emu(self):
        s, st = http_json(self.ctl_port, "GET", "/state")
        assert s == 200
        return st

    def wait(self, pred, timeout=5.0, what="condition", period=0.02):
        end = clock() + timeout
        while True:
            v = pred()
            if v:
                return v
            if clock() > end:
                raise AssertionError("%s not met within %.1f s\n--- app log ---\n%s"
                                     % (what, timeout, _read(self.app_log)[-3000:]))
            time.sleep(period)

    def arm(self):
        assert self.post("/api/arm", {"armed": True}) == (200, {"ok": True, "armed": True})

    def idle(self, timeout=8.0):
        self.wait(lambda: not self.state()["motion"]["busy"], timeout, "motion finished")
        return self.state()["motion"]["last_result"]

    def stand(self):
        self.arm()
        assert self.post("/api/action", {"name": "stand"}) == (200, {"ok": True, "queued": True})
        res = self.idle()
        assert res["id"] == 111 and res["ok"] is True, res


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = Env(str(tmp_path_factory.mktemp("emu")))
    try:
        why = e.start()
    except BaseException:
        e.stop()
        raise
    if why is not None:
        e.stop()
        pytest.fail("the emulator did not start: %s" % why)
    yield e
    e.stop()


@pytest.fixture
def emu(env):
    """Each test starts from a reset emulator (lying, idle) and a disarmed app."""
    env.post("/api/camera", {"on": False})
    env.post("/api/arm", {"armed": False})
    s, _ = http_json(env.ctl_port, "POST", "/reset", {})
    assert s == 200
    env.wait(lambda: env.state()["motion"]["id"] == 101 and env.state()["nav"]["state"] == "idle", 5.0,
             "app sees the reset dog")
    yield env
    st = env.emu()
    assert st["dangerous_calls"] == [] and st["client_bugs"] == [], (st["dangerous_calls"], st["client_bugs"])
    assert st["crashed"] in (None, False, [], {}), st["crashed"]


# ---- the tests --------------------------------------------------------------
def test_connected_to_the_emulator(emu):
    st = emu.state()
    assert st["connected"] is True and st["emulator"] is True
    assert st["target"] == "127.0.0.1:%d" % emu.emu_port and isinstance(st["latency_ms"], int)
    emu.wait(lambda: emu.state()["battery"]["soc"] is not None, 8.0, "battery")
    s, cat = emu.app("GET", "/api/catalog")
    assert s == 200 and {m["risk"] for m in cat["motions"]} <= {"R0", "R1", "R2"}


def test_gate_refuses_while_disarmed(emu):
    before = emu.emu()["calls"].get("1004", 0)
    for path, body in (("/api/action", {"name": "stand"}), ("/api/drive", {"x": 1}),
                       ("/api/posture", {"z": 0.2}), ("/api/nav/start", {"task": "mapping"})):
        s, b = emu.post(path, body)
        assert s == 403 and b["ok"] is False, path
    time.sleep(0.3)
    st = emu.emu()
    assert st["calls"].get("1004", 0) == before and st["servo"]["frames"] == 0
    assert st["nav"]["task"] is None


def test_stand_trick_and_last_result(emu):
    emu.stand()
    emu.wait(lambda: emu.state()["motion"]["id"] == 111, 2.0, "status poll shows standing")
    assert emu.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert emu.state()["motion"]["busy"] is True
    assert emu.post("/api/motion", {"id": 123})[1].get("busy") is True       # 409 while busy
    res = emu.idle()
    seq = res.pop("seq")
    assert res == {"id": 141, "ok": True, "code": 0, "message": "OK", "still_running": False}
    assert emu.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    res = emu.idle()
    assert res["ok"] is True and res["seq"] == seq + 1          # the same trick again: a new seq


def _after_321(emu, mid):
    """Queue a long dance; return once the app has the 1004's 321 (time scale 0.05: the bridge cap
    is 1 s, the 120 s dances run about 3.6 s in the fake controller)."""
    emu.stand()
    log0 = len(_read(emu.app_log))
    assert emu.post("/api/motion", {"id": mid, "confirm": True}) == (200, {"ok": True, "queued": True})
    emu.wait(lambda: "il movimento continua" in _read(emu.app_log)[log0:], 5.0, "the 321 reached the app")


def test_321_keeps_busy_until_the_dance_ends(emu):
    _after_321(emu, 176)
    assert emu.emu()["motion"]["motion_id"] == 176               # the dance goes on in the emulator
    st = emu.state()["motion"]
    assert st["busy"] is True and st["id"] == 176 and st["name"] == "ballo 1"
    calls = emu.emu()["calls"].get("1004", 0)
    s, b = emu.post("/api/motion", {"id": 141})
    assert s == 409 and b["busy"] is True                       # the app refuses it: no 3011 from the dog
    assert emu.emu()["calls"].get("1004", 0) == calls
    early = []

    def cleared():
        busy = emu.state()["motion"]["busy"]
        if not busy and emu.emu()["motion"]["motion_id"] == 176:
            early.append("busy cleared while the emulator still runs 176")
        return not busy
    emu.wait(cleared, 10.0, "busy cleared after the dance", period=0.05)
    assert early == []
    res = emu.state()["motion"]["last_result"]
    assert res["id"] == 176 and res["ok"] is False and res["code"] == 321 and res["still_running"] is False
    emu.wait(lambda: not emu.emu()["motion"]["busy"], 2.0, "emulator manager idle")
    assert emu.post("/api/motion", {"id": 141}) == (200, {"ok": True, "queued": True})
    assert emu.idle()["ok"] is True


def test_estop_during_the_321_wait(emu):
    _after_321(emu, 176)
    assert emu.state()["motion"]["busy"] is True
    t0 = clock()
    s, b = emu.post("/api/estop", {})
    assert s == 200 and b["ok"] is True and clock() - t0 < 2.0, b      # the third ESTOP goes at +1.0 s
    st = emu.state()["motion"]
    assert st["busy"] is False and st["last_result"]["id"] == 0
    assert emu.emu()["motion"]["estop_latched"] is True
    emu.stand()


def test_drive_scaled_and_deadman_stop(emu):
    emu.stand()
    s, b = emu.post("/api/drive", {"x": 1})
    t_last = clock()
    assert s == 200 and b["vx"] == pytest.approx(0.2)
    frame = lambda: emu.emu()["servo"]["last_frame"] or {}      # noqa: E731 (the client's last 1002 frame)
    emu.wait(lambda: frame().get("vel_des") == [0.2, 0.0, 0.0], 2.0, "1002 frames at 0.2 m/s")
    # END (cmd_type 2) is a frame the client sends: the dog's own watchdog would not show here
    emu.wait(lambda: frame().get("cmd_type") == 2, 2.0, "dead-man END", period=0.01)
    dt = clock() - t_last
    assert 0.3 <= dt <= 1.0, dt         # 400 ms plus polling; the exact bound is in test_app_safety
    assert emu.state()["drive"]["active"] is False


def test_held_key_is_never_cut(emu):
    """The UI resends every 150 ms: neither the app dead-man (400 ms) nor the SDK one (500 ms) may stop it."""
    emu.stand()
    end = clock() + 1.5
    seen = []
    while clock() < end:
        assert emu.post("/api/drive", {"x": 0.5})[0] == 200
        seen.append((emu.emu()["servo"]["last_frame"] or {}).get("cmd_type"))
        time.sleep(0.12)
    assert 1 in seen and 2 not in seen, seen          # frames flowed and no END was sent while held
    assert emu.state()["drive"]["active"] is True
    assert emu.post("/api/drive", {"x": 0})[0] == 200


def test_drive_all_zeros_stops(emu):
    emu.stand()
    emu.post("/api/drive", {"z": -1})
    emu.wait(lambda: (emu.emu()["servo"]["last_frame"] or {}).get("vel_des", [0, 0, 0])[2] == pytest.approx(-0.4),
             1.0)
    assert emu.post("/api/drive", {"x": 0, "y": 0, "z": 0})[1]["active"] is False
    emu.wait(lambda: emu.emu()["servo"]["last_frame"]["cmd_type"] == 2, 0.5, "END at once")


def test_posture_reaches_the_dog_unmodified(emu):
    emu.stand()
    s, b = emu.post("/api/posture", {"pitch": 0.1, "z": 0.25})
    assert s == 200 and b["posture"]["pitch"] == 0.1 and b["posture"]["z"] == 0.25
    emu.wait(lambda: (emu.emu()["servo"]["last_frame"] or {}).get("motion_id") == 201, 2.0, "posture frames")
    f = emu.emu()["servo"]["last_frame"]
    assert f["rpy_des"][1] == pytest.approx(0.1) and f["pos_des"][2] == pytest.approx(0.25)
    assert emu.post("/api/arm", {"armed": False})[0] == 200
    emu.wait(lambda: emu.emu()["servo"]["last_frame"]["cmd_type"] == 2, 0.5, "END on disarm")


def test_disarm_stops_the_drive(emu):
    emu.stand()
    for _ in range(4):
        emu.post("/api/drive", {"x": 1})
        time.sleep(0.1)
    t0 = clock()
    assert emu.post("/api/arm", {"armed": False}) == (200, {"ok": True, "armed": False})
    emu.wait(lambda: emu.emu()["servo"]["last_frame"]["cmd_type"] == 2, 0.5, "END on disarm", period=0.01)
    assert clock() - t0 < 0.5
    assert emu.post("/api/drive", {"x": 1})[0] == 403


def test_led(emu):
    s, b = emu.post("/api/led", {"target": "head", "color": "#ff0000", "effect": "on", "seconds": 2})
    assert s == 200 and b["ok"] is True and (b.get("task_id") or b.get("pending"))
    emu.wait(lambda: (emu.emu()["leds"]["head"] or {}).get("r") == 255, 3.0, "head LED red")
    led = emu.emu()["leds"]["head"]
    assert (led["g"], led["b"]) == (0, 0)
    assert emu.state()["leds"]["head"]["color"] == "#ff0000"


def test_sensors_refresh(emu):
    t0 = clock()
    assert emu.post("/api/sensors/refresh", {}) == (200, {"ok": True, "pending": True})
    assert clock() - t0 < 1.0
    assert emu.state()["sensors"]["pending"] is True

    def outcome():
        sen = emu.state()["sensors"]
        if not sen["pending"]:
            return "done"
        logs = json.dumps(emu.emu()["vp"].get("logs", {}))
        if "Traceback" in logs:
            return logs
        return None
    r = emu.wait(outcome, 25.0, "sensor read-back", period=0.2)
    assert r == "done", "the sensor task failed in the emulator: %s" % r[:300]
    sen = emu.state()["sensors"]
    assert sen["error"] is None and sen["t"] is not None and sen["bms"]


def test_map(emu):
    end = time.monotonic() + 15.0
    while True:                         # pending while the 6003 runs; the page asks again 1 s later
        s, m = emu.app("GET", "/api/map", timeout=5.0)
        if not (s == 200 and m.get("pending")) or time.monotonic() > end:
            break
        time.sleep(0.5)
    assert s == 200 and m["ok"] is True and m["name"] == "casa"
    assert m["width"] > 0 and m["height"] > 0 and len(m["data"]) == m["width"] * m["height"]
    assert m["resolution"] > 0 and len(m["origin"]) == 2
    assert {lb["name"] for lb in m["labels"]} == {"cucina", "divano"}


def test_mapping_and_save(emu):
    emu.arm()
    assert emu.post("/api/nav/save_map", {"name": "prova_app"})[0] == 409      # nothing being mapped
    assert emu.post("/api/nav/start", {"task": "mapping"}) == (200, {"ok": True, "accepted": 1})
    emu.wait(lambda: emu.state()["nav"]["state"] == "mapping", 3.0, "mapping")
    s, b = emu.post("/api/nav/save_map", {"name": "prova_app"})
    assert s == 200 and b["ok"] is True and b["accepted"] == 1 and (b.get("result") == 0 or b.get("pending"))
    emu.wait(lambda: "prova_app" in emu.emu()["nav"]["maps"], 5.0, "map saved on the dog")


def test_nav_relocalize_goto_and_stop(emu):
    emu.arm()
    s, b = emu.post("/api/nav/start", {"task": "goto", "x": 1.0})
    assert s == 400 and "x e y" in b["error"]
    assert emu.post("/api/nav/start", {"task": "relocalize"}) == (200, {"ok": True, "accepted": 1})
    emu.wait(lambda: emu.state()["nav"]["state"] == "localized", 8.0, "localized", period=0.1)
    assert emu.post("/api/nav/start", {"task": "goto", "x": 1.0, "y": 0.5}) == (200, {"ok": True, "accepted": 1})
    st = emu.wait(lambda: (lambda s: s if s["nav"]["state"] == "navigating" else None)(emu.state()), 3.0,
                  "navigating")
    assert st["nav"]["task"] == "goto" and st["nav"]["goal"] == [1.0, 0.5]
    calls = emu.emu()["calls"].get("1004", 0)
    s, b = emu.post("/api/motion", {"id": 141})                     # no 1004 during autonomous navigation
    assert s == 409 and b["navigating"] is True and "Ferma navigazione" in b["error"]
    assert emu.post("/api/action", {"name": "sit"})[1].get("navigating") is True
    assert emu.emu()["calls"].get("1004", 0) == calls
    emu.wait(lambda: emu.emu()["nav"]["sub_status"] == 307, 3.0, "goal sent")     # a type 1 stop keeps the map
    s, b = emu.post("/api/nav/stop", {})
    assert s == 200 and b["stopped"] is True
    emu.wait(lambda: emu.state()["nav"]["state"] == "localized", 3.0, "back to localized")
    assert emu.state()["nav"]["task"] is None
    st = emu.emu()
    assert st["nav"]["task"] in (None, "", {}) or st["nav"]["status"] == 8


def test_estop_during_a_motion(emu):
    emu.stand()
    assert emu.post("/api/motion", {"id": 140})[1]["need_confirm"] is True         # R2: 409
    assert emu.post("/api/motion", {"id": 130, "confirm": True})[0] == 400          # R3: not in the app
    assert emu.post("/api/motion", {"id": 140, "confirm": True}) == (200, {"ok": True, "queued": True})
    emu.wait(lambda: emu.emu()["motion"]["motion_id"] == 140, 2.0, "dance started", period=0.005)
    t0 = clock()
    s, b = emu.post("/api/estop", {})
    dt = clock() - t0
    assert s == 200 and b["ok"] is True and b["armed"] is False and b["result"]["id"] == 0, b
    assert dt < 2.0, dt                 # the third ESTOP goes at +1.0 s
    st = emu.state()
    assert st["armed"] is False and st["motion"]["busy"] is False
    m = emu.emu()["motion"]
    assert m["estop_latched"] is True and m["motion_id"] == 0
    assert emu.post("/api/motion", {"id": 141})[0] == 403
    emu.stand()                                                                     # recovery: 111
    assert emu.emu()["motion"]["estop_latched"] is False


def test_camera_stream(emu):
    r = RawStream(emu.app_port, "/stream.mjpg")
    assert r.status == 409
    r.close()
    assert emu.post("/api/camera", {"on": True}) == (200, {"ok": True, "on": True})
    r = RawStream(emu.app_port, "/stream.mjpg")
    try:
        assert r.status == 200
        ctype, jpeg = r.part(8.0)
        assert ctype == "image/jpeg" and jpeg[:2] == b"\xff\xd8"
        assert emu.state()["camera"]["viewers"] == 1
    finally:
        r.close()
    emu.wait(lambda: emu.state()["camera"]["viewers"] == 0, 3.0, "viewer gone")
    assert emu.post("/api/camera", {"on": False})[0] == 200
