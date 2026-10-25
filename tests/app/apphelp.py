"""Shared helpers for the app server tests (imported by conftest.py and the test files).

Layer (a): `harness` runs app2/server.py's App and HTTP server in-process with a
StubDog injected through the client factory: no network, no gRPC. The stub
records every SDK call, can block chosen calls on an Event (to simulate a dog
that does not answer) and keeps the SDK's own guards where they matter
(check_risk for 1004, vp.led_body for LED arguments, Posture clamps).

Layer (b) helpers live in test_app_emulator.py.
"""
from __future__ import annotations

import http.client
import importlib.util
import json
import os
import socket
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SERVER_PY = os.path.join(ROOT, "app2", "server.py")


def _load_server():
    name = "app2_server"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SERVER_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


server = _load_server()

from cyberdog_sdk import DogUnreachable, Gait, MotionResult, Status  # noqa: E402
from cyberdog_sdk import vp  # noqa: E402
from cyberdog_sdk.client import check_risk  # noqa: E402
from cyberdog_sdk.codes import describe  # noqa: E402
from cyberdog_sdk.nav import NAV_TYPES, MapData, decode_state  # noqa: E402
from cyberdog_sdk.servo import POSE_DEFAULT, POSE_LIMITS, clamp  # noqa: E402

clock = time.perf_counter


# ---- the stub SDK client ----------------------------------------------------
class StubDriver:
    def __init__(self, dog):
        self.dog = dog
        self.gait = Gait.SLOW
        self.moves = []         # (t, vx, vy, wz)
        self.stops = []         # t of each END
        self.halts = 0
        self.gaits = []
        self._active = False
        self.ended_at = 0.0

    @property
    def active(self):
        return self._active

    @property
    def stats(self):
        return {"frames": len(self.moves), "ends": len(self.stops)}

    def set_gait(self, gait):
        self.gait = Gait(int(gait))
        self.gaits.append(int(gait))

    def move(self, vx, vy, wz):
        self.dog.enter("move")
        if self.dog.posture.active:
            self.dog.posture.stop()
        self.moves.append((clock(), vx, vy, wz))
        self._active = True

    def stop(self):
        if self._active:
            self.stops.append(clock())
            self._active = False
            self.ended_at = clock()

    def halt(self):
        was, self._active = self._active, False
        self.halts += 1
        return was

    def join(self, timeout=None):
        pass


class StubPosture:
    def __init__(self, dog):
        self.dog = dog
        self.target = dict(POSE_DEFAULT)
        self.sets = []          # the raw keyword arguments of every set()
        self.resets = 0
        self.stops = 0
        self._active = False

    @property
    def active(self):
        return self._active

    def set(self, z=None, roll=None, pitch=None, yaw=None):
        self.dog.enter("posture")
        raw = {k: v for k, v in (("z", z), ("roll", roll), ("pitch", pitch), ("yaw", yaw)) if v is not None}
        self.sets.append(raw)
        for k, v in raw.items():
            self.target[k] = clamp(float(v), *POSE_LIMITS[k])
        if self.dog.drive.active:          # the SDK Arbiter: Posture stops the Driver
            self.dog.drive.stop()
        self._active = True
        return dict(self.target)

    def reset(self):
        self.resets += 1
        return self.set(**POSE_DEFAULT)

    def stop(self):
        if self._active:
            self.stops += 1
            self._active = False

    def halt(self):
        was, self._active = self._active, False
        return was

    def join(self, timeout=None):
        pass


class StubCamera:
    def __init__(self):
        self.starts = self.stops = 0
        self.running = False
        self.fps = 0.0
        self._seq = 0
        self._latest = None
        self._cond = threading.Condition()

    def start(self):
        self.starts += 1
        if self.running:
            return
        self.running = True
        self.fps = 20.0
        threading.Thread(target=self._frames, daemon=True).start()

    def _frames(self):
        while self.running:
            with self._cond:
                self._seq += 1
                self._latest = (self._seq, b"\xff\xd8stub-jpeg-%d\xff\xd9" % self._seq)
                self._cond.notify_all()
            time.sleep(0.05)

    def stop(self):
        self.stops += 1
        self.running = False
        self.fps = 0.0

    def close(self):
        self.running = False

    def latest(self):
        with self._cond:
            return self._latest

    def wait(self, after_seq=0, timeout=5.0):
        with self._cond:
            if self._seq <= after_seq:
                self._cond.wait(timeout)
            return self._latest


class StubTask:
    """NavTask look-alike: accepted / result arrive after `delay`."""

    def __init__(self, task_type, kind, accepted=1, delay=0.02, result=0, error=None):
        self.task_type, self.kind = task_type, kind
        self.accepted = None
        self.feedback = []
        self.result = None
        self.error = None
        self.done = threading.Event()
        self._out = (accepted, result, error)
        if delay is not None:
            t = threading.Timer(delay, self._fire)
            t.daemon = True
            t.start()

    def _fire(self):
        accepted, result, error = self._out
        if error is not None:
            self.error = error
            self.done.set()
            return
        if self.kind == "save":
            self.result = result
            self.done.set()
        else:
            self.accepted = accepted
            if accepted != 1:
                self.done.set()

    def wait(self, timeout=None):
        return self.done.wait(timeout)


class StubNav:
    def __init__(self, dog):
        self.dog = dog
        self.started = []       # (type, args)
        self.stops = []
        self.saves = []
        self.labels_calls = []
        self.accept = 1
        self.delay = 0.02
        self.error = None
        self.save_result = 0
        self.save_delay = 0.02
        self.map_success = 0
        self.map_reads = 0      # 6003 calls

    def _start(self, typ, *args):
        assert typ in NAV_TYPES, "6004 type %r" % typ
        self.started.append((typ, args))
        return StubTask(typ, "start", self.accept, self.delay, error=self.error)

    def start_mapping(self, outdoor=False):
        return self._start(5)

    def relocalize(self, outdoor=False):
        return self._start(7)

    def go_to(self, x, y, outdoor=False):
        return self._start(1, x, y)

    def dock(self):
        return self._start(9)

    def save_map(self, name):
        self.saves.append(name)
        return StubTask(5, "save", delay=self.save_delay, result=self.save_result)

    def stop(self, task_type):
        self.dog.enter("nav_stop")
        self.stops.append(task_type)
        return 0

    def state(self, timeout=3.0):
        self.dog._check()
        return decode_state(self.dog.task_status, self.dog.task_sub)

    def get_map(self, name=""):
        self.dog._check()
        self.map_reads += 1
        self.dog.enter("get_map")
        if self.map_success:
            return MapData("", self.map_success, False, 0.0, 0, 0, (0.0, 0.0), 0.0, [], [])
        return MapData("casa", 0, False, 0.05, 4, 3, (-2.0, -1.5), 0.0, [-1, 0, 100, 0] * 3,
                       [("cucina", 1.3, 0.7), ("divano", -1.1, -0.6)])

    def set_labels(self, *a, **kw):     # the app must never call this (no delete-all path)
        self.labels_calls.append((a, kw))
        raise AssertionError("set_labels called")


class StubDog:
    def __init__(self):
        self.calls = []                 # (name, args, kwargs) of every request/response call
        self.connects = 0               # connect() calls (the app's channel warm-up)
        self.reachable = True
        self.motion_id = 101
        self.task_status = 101
        self.task_sub = 0
        self.wired = self.wireless = False
        self.fsm_state = 0
        self.soc = 80
        self.sn = "EMU2241CN0000001"
        self.motion_delay = 0.02
        self.results = {}               # motion id -> (result, code)
        self.still = set()              # motion ids whose 1004 gives 321 still_running: the dog goes on
                                        # (status keeps the id until the test sets another motion_id)
        self.motions = []               # motion ids in the order sent
        self.last_motion = None         # as CyberDog.last_motion: last id with result true, 0 after ESTOP
        self.estops = []                # clock() of each estop()
        self.sensor_delay = 0.05
        self.sensor_error = None
        self.blocks = {}                # call name -> Event the call waits on
        self.entered = {}               # call name -> Event set when the call starts
        self.closed = False
        self._abort = threading.Event()
        self.drive = StubDriver(self)
        self.posture = StubPosture(self)
        self.camera = StubCamera()
        self.nav = StubNav(self)

    # test helpers
    def block(self, name):
        ev = threading.Event()
        self.blocks[name] = ev
        self.entered[name] = threading.Event()
        return ev

    def enter(self, name):
        if name in self.entered:
            self.entered[name].set()
        ev = self.blocks.get(name)
        if ev is not None:
            ev.wait(10)

    def _check(self):
        if not self.reachable:
            raise DogUnreachable("stub: unreachable")

    def connect(self, timeout=5.0):     # not in calls: it runs from the app's warm loop at any time
        self.connects += 1
        self._check()

    def _rec(self, name, *args, **kw):
        self.calls.append((name, args, kw))

    def names(self):
        return [c[0] for c in self.calls]

    # the CyberDog surface used by the server
    def status(self, timeout=3.0):
        self._check()
        return Status(self.motion_id, self.task_status, self.task_sub, self.fsm_state, 0, self.wired,
                      self.wireless, 0, {})

    def device_info(self, *flags, timeout=15.0):
        self._check()
        self._rec("device_info", *flags, timeout=timeout)
        out = {}
        if "bat_info" in flags:
            out["bat_info"] = {"power": self.soc, "is_charging": self.wired}
        if "sn" in flags:
            out["sn"] = self.sn
        if "wifi" in flags:
            out["wifi"] = {"name": "emulator" if self.sn.startswith("EMU") else "HOTSPOT"}
        return out

    def set_volume(self, level):
        self._check()
        self._rec("set_volume", level)
        self.enter("volume")
        return True

    def motion(self, motion_id, *, duration=0, timeout=25.0, allow_risky=False, **arrays):
        self._rec("motion", motion_id, duration=duration, allow_risky=allow_risky, **arrays)
        check_risk(motion_id, allow_risky)         # the SDK's own catalog gate
        self._check()
        self.motions.append(motion_id)
        if "motion" in self.entered:
            self.entered["motion"].set()
        ev = self.blocks.get("motion")
        if ev is not None:
            end = clock() + 10
            while not ev.is_set() and not self._abort.is_set() and clock() < end:
                ev.wait(0.01)
        else:
            time.sleep(self.motion_delay)
        if self._abort.is_set():
            return MotionResult(motion_id, False, 3021, describe(3021, "en"), False)
        if motion_id in self.still:
            self.motion_id = motion_id
            return MotionResult(motion_id, False, 321, describe(321, "en"), True)
        ok, code = self.results.get(motion_id, (True, 0))
        if ok:
            self.motion_id = self.last_motion = motion_id
        return MotionResult(motion_id, ok, code, describe(code, "en"), False)

    def trick(self, motion_id, allow_risky=False):
        self._rec("trick", motion_id, allow_risky=allow_risky)
        raise AssertionError("the server plans tricks itself")

    def estop(self):
        self._rec("estop")
        self.estops.append(clock())
        self.drive.halt()
        self.posture.halt()
        self._abort.set()
        self.motion_id = self.last_motion = 0
        return MotionResult(0, True, 0, "OK", False)

    def led(self, target, color=None, effect="on", seconds=3.0):
        self._rec("led", target, color, effect, seconds)
        vp.led_body(target, color, effect, seconds)     # the SDK's argument checks
        self._check()
        self.enter("led")
        return "pc_stub_led"

    def led_off(self, target):
        self._rec("led_off", target)
        self._check()
        return "pc_stub_off"

    def skin(self, mode, wave_ms=3000):
        self._rec("skin", mode, wave_ms)
        self._check()
        self.enter("skin")
        return True

    def read_sensors(self, kinds=vp.SENSORS, timeout=20.0):
        self._rec("read_sensors", timeout=timeout)
        self.enter("sensors")
        time.sleep(self.sensor_delay)
        if self.sensor_error is not None:
            raise self.sensor_error
        return {"task": "pc_stub", "time": time.time(), "bms": {"batt_soc": 80, "code": 0},
                "tof": {"code": 0}, "ultrasonic": {"range": float("nan"), "code": 0},
                "odom": {"position": [0.0, 0.0, 0.0]}}

    def close(self):
        self.closed = True
        self.camera.close()


# ---- HTTP harness -----------------------------------------------------------
FAST = dict(poll_s=0.05, keepalive_s=10.0, quick_s=0.3, nav_accept_s=0.6, info_every_s=0.2,
            rebuild_after_s=0.3, rebuild_every_s=0.3, sensor_timeout=2.0)


class Harness:
    def __init__(self, tmp_path, ui_text="<!doctype html><title>stub ui</title>", dog=None, **app_kw):
        self.dogs = []
        self.next_dog = dog             # a prepared StubDog for the next factory call

        def factory():
            d = self.next_dog or StubDog()
            self.next_dog = None
            self.dogs.append(d)
            return d
        ui_path = str(tmp_path / "ui.html")
        if ui_text is not None:
            with open(ui_path, "w", encoding="utf-8") as f:
                f.write(ui_text)
        kw = dict(FAST)
        kw.update(app_kw)
        self.app = server.App("stub:50052", factory, ui_path=ui_path, **kw)
        self.httpd = server.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        self.host = "127.0.0.1:%d" % self.port
        self.app.start()
        self.thread = threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.05},
                                       daemon=True)
        self.thread.start()
        self.wait(lambda: self.app.connected, 3.0, "stub never connected")

    @property
    def dog(self) -> StubDog:
        return self.app.dog

    def request(self, method, path, body=None, headers=None, raw=None, timeout=5.0):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        hdrs = {}
        data = raw
        if body is not None:
            data = json.dumps(body).encode()
            hdrs["Content-Type"] = "application/json"
        if headers:
            hdrs.update(headers)
        hdrs = {k: v for k, v in hdrs.items() if v is not None}
        try:
            conn.request(method, path, body=data, headers=hdrs)
            r = conn.getresponse()
            payload = r.read()
            ctype = r.getheader("Content-Type") or ""
            out = json.loads(payload) if ctype.startswith("application/json") else payload
            return r.status, dict(r.getheaders()), out
        finally:
            conn.close()

    def get(self, path, **kw):
        s, _, b = self.request("GET", path, **kw)
        return s, b

    def post(self, path, body=None, **kw):
        s, _, b = self.request("POST", path, {} if body is None else body, **kw)
        return s, b

    def state(self):
        s, b = self.get("/api/state")
        assert s == 200, b
        return b

    def arm(self):
        s, b = self.post("/api/arm", {"armed": True})
        assert (s, b) == (200, {"ok": True, "armed": True}), (s, b)

    def wait(self, pred, timeout=3.0, what="condition"):
        end = clock() + timeout
        while True:
            v = pred()
            if v:
                return v
            if clock() > end:
                raise AssertionError("%s not met within %.1f s" % (what, timeout))
            time.sleep(0.01)

    def close(self):
        for d in self.dogs:
            for ev in d.blocks.values():
                ev.set()
        self.httpd.shutdown()
        self.app.close()
        self.httpd.server_close()


# ---- raw streaming client (SSE, MJPEG, odd headers) ---------------------------
class RawStream:
    """One HTTP request on a plain socket, kept open: for /api/events and /stream.mjpg,
    and for requests whose Host header http.client would not send as given."""

    def __init__(self, port, path, host="", method="GET", headers=(), body=b"", rcvbuf=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if rcvbuf:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.sock.settimeout(5.0)
        self.sock.connect(("127.0.0.1", port))
        if host == "":
            host = "127.0.0.1:%d" % port
        lines = ["%s %s HTTP/1.1" % (method, path)]
        if host is not None:
            lines.append("Host: %s" % host)
        lines += ["%s: %s" % kv for kv in headers]
        if body:
            lines.append("Content-Length: %d" % len(body))
        self.sock.sendall(("\r\n".join(lines) + "\r\n\r\n").encode("latin-1") + body)
        self.buf = b""
        self.eof = False
        head = self.read_until(b"\r\n\r\n", 5.0)
        assert head is not None, "no response head"
        first, *rest = head.decode("latin-1").split("\r\n")
        self.status = int(first.split()[1])
        self.headers = {}
        for ln in rest:
            if ":" in ln:
                k, v = ln.split(":", 1)
                self.headers[k.strip().lower()] = v.strip()

    def _fill(self, deadline) -> bool:
        if self.eof:
            return False
        left = deadline - clock()
        if left <= 0:
            return False
        self.sock.settimeout(left)
        try:
            chunk = self.sock.recv(65536)
        except socket.timeout:
            return False
        except OSError:
            chunk = b""
        if not chunk:
            self.eof = True
            return False
        self.buf += chunk
        return True

    def read_until(self, sep: bytes, timeout: float):
        deadline = clock() + timeout
        while sep not in self.buf:
            if not self._fill(deadline):
                return None
        out, self.buf = self.buf.split(sep, 1)
        return out

    def read_n(self, n: int, timeout: float):
        deadline = clock() + timeout
        while len(self.buf) < n:
            if not self._fill(deadline):
                return None
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def body_json(self, timeout=3.0):
        data = self.read_n(int(self.headers.get("content-length", "0")), timeout)
        return json.loads(data)

    # SSE
    def frame(self, timeout=3.0):
        """The next SSE frame as (t, kind, data): kind is the event name, "comment" or "retry"."""
        raw = self.read_until(b"\n\n", timeout)
        if raw is None:
            return None
        t = clock()
        text = raw.decode("utf-8")
        if text.startswith(":"):
            return t, "comment", text[1:].strip()
        if text.startswith("retry:"):
            return t, "retry", text[6:].strip()
        fields = dict(ln.split(": ", 1) for ln in text.split("\n"))
        return t, fields["event"], json.loads(fields["data"])

    def frames(self, duration: float):
        out, end = [], clock() + duration
        while clock() < end:
            f = self.frame(max(0.01, end - clock()))
            if f is not None:
                out.append(f)
        return out

    def until(self, pred, timeout=3.0):
        end = clock() + timeout
        while clock() < end:
            f = self.frame(max(0.01, end - clock()))
            if f is not None and pred(f):
                return f
        return None

    # MJPEG
    def part(self, timeout=3.0):
        head = self.read_until(b"\r\n\r\n", timeout)
        if head is None:
            return None
        hdrs = dict(ln.split(": ", 1) for ln in head.decode("latin-1").split("\r\n")[1:])
        data = self.read_n(int(hdrs["Content-Length"]) + 2, timeout)
        assert head.startswith(b"--cyberdogframe") and data is not None and data.endswith(b"\r\n")
        return hdrs["Content-Type"], data[:-2]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
