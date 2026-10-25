"""End to end in Gazebo: app2/server.py over HTTP -> cyberdog_sdk -> emulator (LCM controller) -> Gazebo.

Needs, in the CyberDogSim WSL distro, the sim and the emulator up:
    wsl.exe -d CyberDogSim -e bash -lc 'FAST=1 /root/sim_up.sh && /root/emu_up.sh'
then, from the repo root on Windows (or inside the VM):
    python -m pytest -s -m gazebo tests/gazebo/test_gz_e2e.py
Skipped when the emulator's control endpoint (127.0.0.1:50080) does not answer, when it runs the fake
controller, or when no simulator_state arrives. test_lcm_link.py needs the opposite (emulator down), so
the two files never run against the same sim at once.

The app runs as a subprocess on a free port with --dog 127.0.0.1:50052: every connection stays on this
machine. Ground truth is the emulator's GET /state (Gazebo simulator_state through LcmController),
sampled at 20 Hz. Expected sizes are in simulated time (ground_truth.t, frame count x 2 ms); wall-clock
limits are divided by the RTF measured over the same window. GZ_E2E_OUT=<file.json> saves every
measurement; -s prints them.

The tests run in file order and share one robot and one app, as one operator session: arm, stand, drive
and release, turn and dead-man, a keymap trick, "testa su" and reset, sit, stand, ESTOP during a walk,
recovery, lie down. Each test first brings the robot to the state it needs, so a single test also runs
alone.

test_09 is the opposite case: raw LCM to the controller, inside the VM, with the emulator DOWN (like
test_lcm_link.py). It checks whether the servo watchdog's WalkStand (11/1), sent after an ESTOP the way
motion_action.cpp would, can lift the open-source controller out of Off. Run it with
    cd /mnt/c/<path to the repository> && python3 -m pytest -s tests/gazebo/test_gz_e2e.py -k test_09
It skips on Windows and whenever another process publishes robot_control_cmd (the emulator).
"""
from __future__ import annotations

import http.client
import importlib.util
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time

import pytest

pytestmark = [pytest.mark.gazebo, pytest.mark.slow]

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SERVER_PY = os.path.join(ROOT, "app2", "server.py")
UI_HTML = os.path.join(ROOT, "app2", "ui.html")
HOST = "127.0.0.1"                                # the emulator only, never the dog
EMU_PORT = int(os.environ.get("GZ_EMU_PORT", "50052"))
CTL_PORT = int(os.environ.get("GZ_EMU_CONTROL_PORT", "50080"))
FIXED = {50052, 50080, 8793, 8794}

SAMPLE_S = 0.05                                   # ground-truth poll period (wall)
DRIVE_PERIOD = 0.15                               # the UI resends /api/drive at least this often
V_STILL, W_STILL, STILL_HOLD = 0.05, 0.1, 0.3     # still: |v_xy| < 0.05 m/s and |wz| < 0.1 rad/s for 0.3 s wall
V_REST, W_REST = 0.02, 0.05                       # at rest: the stricter pair (the gait change creeps at 0.03-0.04)
LIE_Z, STAND_Z = 0.054, 0.2353                    # sim_baseline.md
APP_DEADMAN = 0.4                                 # app2 DEADMAN_S
SDK_DEADMAN, FRAME_DEADLINE = 0.5, 0.25           # cyberdog_sdk.servo
RESULT_WAIT = 60.0                                # wall cap for one queued motion result
SERVO_MIN_HZ = 15.0                               # 1002 frames reaching the emulator; below: starved (load)
POSE_PITCH = 0.15                                 # "testa su" amount, radians
TRICK = 146                                       # keymap "4", stretch: R0, mode 62/6, in place

MEAS = {}                                         # measurements, printed and dumped at module teardown
clock = time.perf_counter


# ---- HTTP -------------------------------------------------------------------------------
def http_json(port, method, path, body=None, timeout=5.0):
    conn = http.client.HTTPConnection(HOST, port, timeout=timeout)
    try:
        hdrs, data = {}, None
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            hdrs["Content-Type"] = "application/json"
        conn.request(method, path, body=data, headers=hdrs)
        r = conn.getresponse()
        return r.status, json.loads(r.read() or b"null")
    finally:
        conn.close()


def free_port():
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, 0))
        port = s.getsockname()[1]
        s.close()
        if port not in FIXED:
            return port


def sim_not_ready():
    """Why the test must skip, or None."""
    try:
        s, st = http_json(CTL_PORT, "GET", "/state", timeout=2.0)
    except (OSError, ValueError) as e:
        return ("emulator control endpoint %s:%d does not answer (%s): start FAST=1 /root/sim_up.sh, "
                "then /root/emu_up.sh" % (HOST, CTL_PORT, e))
    if s != 200 or not isinstance(st, dict):
        return "emulator control endpoint answered HTTP %s" % s
    if st.get("controller") != "lcm":
        return "the emulator on %d runs the %r controller, not lcm (Gazebo)" % (CTL_PORT, st.get("controller"))
    t0 = (st.get("ground_truth") or {}).get("t")
    time.sleep(0.5)
    try:
        t1 = (http_json(CTL_PORT, "GET", "/state", timeout=2.0)[1].get("ground_truth") or {}).get("t")
    except (OSError, ValueError, AttributeError):
        t1 = None
    if t0 is None or t1 is None or t1 <= t0:
        return "no simulator_state reaches the emulator (Gazebo is not running)"
    return None


def pitch_up_sign():
    """PITCH_UP_SIGN from app2/ui.html: API pitch = PITCH_UP_SIGN * "testa su" (contracts.md 6.1)."""
    with open(UI_HTML, encoding="utf-8") as f:
        m = re.search(r"const\s+PITCH_UP_SIGN\s*=\s*(-?1)\s*;", f.read())
    assert m, "PITCH_UP_SIGN not found in app2/ui.html"
    return int(m.group(1))


# ---- ground truth -------------------------------------------------------------------------
def _flat(st, w):
    g, m, sv, calls = st["ground_truth"], st["motion"], st["servo"], st.get("calls") or {}
    if g.get("t") is None:
        return None
    return {"w": w, "t": g["t"], "x": g["x"], "y": g["y"], "z": g["z"], "roll": g["roll"], "pitch": g["pitch"],
            "yaw": g["yaw"], "vx": g["vx"], "vy": g["vy"], "wz": g["wz"], "mode": m["mode"], "gait": m["gait_id"],
            "mid": m["motion_id"], "sw": m["switch_status"], "bar": m["bar"], "busy": m["busy"],
            "estop": m["estop_latched"], "ws": sv["walk_stands"], "wd": sv["watchdog_stops"],
            "frames": sv["frames"], "c1004": calls.get("1004", 0), "c1002": calls.get("1002", 0)}


class Sampler:
    """Polls the emulator's GET /state every SAMPLE_S; each sample is stamped with the wall midpoint."""

    def __init__(self):
        self.samples = []
        self.lk = threading.Lock()
        self.errors = 0
        self.stop_ev = threading.Event()
        self.th = threading.Thread(target=self._loop, name="gz-e2e-sampler", daemon=True)
        self.th.start()

    def _loop(self):
        nxt = clock()
        while not self.stop_ev.is_set():
            t0 = clock()
            try:
                s, st = http_json(CTL_PORT, "GET", "/state", timeout=2.0)
            except (OSError, ValueError):
                s, st = None, None
                self.errors += 1
            if s == 200:
                rec = _flat(st, (t0 + clock()) / 2.0)
                if rec is not None:
                    with self.lk:
                        self.samples.append(rec)
            nxt += SAMPLE_S
            d = nxt - clock()
            if d < -SAMPLE_S:
                nxt = clock()
            elif d > 0:
                self.stop_ev.wait(d)

    def close(self):
        self.stop_ev.set()
        self.th.join(3.0)

    def since(self, w0, w1=None):
        with self.lk:
            return [s for s in self.samples if s["w"] >= w0 and (w1 is None or s["w"] <= w1)]

    def latest(self):
        with self.lk:
            return self.samples[-1] if self.samples else None

    def at(self, w):
        """The last sample taken at or before wall time w (else the first one after it)."""
        with self.lk:
            best = None
            for s in self.samples:
                if s["w"] <= w:
                    best = s
                else:
                    return best or s
            return best

    def fresh(self, timeout=3.0):
        """The first sample taken after now."""
        w = clock()
        s = self.wait(lambda s: True, timeout, since=w)
        assert s is not None, "no ground-truth sample for %.1f s" % timeout
        return s

    def wait(self, pred, timeout, since=None):
        """First sample at or after `since` (default now) matching pred, or None after timeout wall s."""
        w0 = clock() if since is None else since
        end = clock() + timeout
        i = 0
        while True:
            with self.lk:
                new = [s for s in self.samples[i:] if s["w"] >= w0]
                i = len(self.samples)
            for s in new:
                if pred(s):
                    return s
            if clock() > end:
                return None
            time.sleep(SAMPLE_S / 2)

    def rtf(self, w0, w1=None):
        ss = self.since(w0, w1)
        if len(ss) < 2 or ss[-1]["w"] - ss[0]["w"] < 0.5:
            return None
        return (ss[-1]["t"] - ss[0]["t"]) / (ss[-1]["w"] - ss[0]["w"])


def still(s, v=V_STILL, w=W_STILL):
    return math.hypot(s["vx"], s["vy"]) < v and abs(s["wz"]) < w


def first_still(samples, v=V_STILL, w=W_STILL):
    """The first sample from which the body stays still for STILL_HOLD wall seconds (complete window)."""
    for i, s in enumerate(samples):
        if not still(s, v, w):
            continue
        for q in samples[i:]:
            if not still(q, v, w):
                break
            if q["w"] - s["w"] >= STILL_HOLD:
                return s
    return None


def wait_still(gz, w0, timeout, v=V_STILL, w=W_STILL):
    end = clock() + timeout
    while clock() < end:
        s = first_still(gz.gt.since(w0), v, w)
        if s is not None:
            return s
        time.sleep(0.1)
    return first_still(gz.gt.since(w0), v, w)


def stop_info(gz, ref, budget):
    """How the walk ended after the sample `ref`: still (V_STILL), at rest (V_REST), and the first
    response that reports WalkStand 112 (11/1, switch 0: the controller has finished the gait cycle)."""
    stop = wait_still(gz, ref["w"], budget)
    rest = wait_still(gz, ref["w"], budget, V_REST, W_REST)
    rep = gz.gt.wait(lambda q: q["mid"] == 112 and q["sw"] == 0, budget, since=ref["w"])
    m = {"still_sim_s": round(stop["t"] - ref["t"], 2) if stop else None,
         "still_wall_s": round(stop["w"] - ref["w"], 2) if stop else None,
         "rest_sim_s": round(rest["t"] - ref["t"], 2) if rest else None,
         "rest_wall_s": round(rest["w"] - ref["w"], 2) if rest else None,
         "walkstand_reported_sim_s": round(rep["t"] - ref["t"], 2) if rep else None,
         "coast_m": r4(math.hypot(rest["x"] - ref["x"], rest["y"] - ref["y"])) if rest else None,
         "dyaw_after": r4(unwrap_dyaw(gz.gt.since(ref["w"], rest["w"]))) if rest else None}
    return m, rest


def servo_hz(gz, w0, w1):
    """1002 frames per wall second that reached the emulator between wall times w0 and w1."""
    a, b = gz.gt.at(w0), gz.gt.at(w1)
    return (b["frames"] - a["frames"]) / max(b["w"] - a["w"], 1e-6)


def unwrap_dyaw(samples):
    d = 0.0
    for a, b in zip(samples, samples[1:]):
        d += math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
    return d


def heading_split(s0, s1):
    """Displacement s0 -> s1 split along the heading at s0: (forward, left)."""
    dx, dy = s1["x"] - s0["x"], s1["y"] - s0["y"]
    c, n = math.cos(s0["yaw"]), math.sin(s0["yaw"])
    return dx * c + dy * n, -dx * n + dy * c


def r4(v):
    return None if v is None else round(v, 4)


# ---- the app -------------------------------------------------------------------------------
class Holder:
    """A held drive key: POST /api/drive every DRIVE_PERIOD until stop()."""

    def __init__(self, gz, x=0.0, y=0.0, z=0.0):
        self.gz, self.body = gz, {"x": x, "y": y, "z": z}
        self.posts = []                           # (t_sent, t_answered, status, reply)
        self.stop_ev = threading.Event()
        self.th = threading.Thread(target=self._loop, name="gz-e2e-drive", daemon=True)
        self.th.start()

    def _loop(self):
        nxt = clock()
        while not self.stop_ev.is_set():
            t0 = clock()
            try:
                s, r = self.gz.post("/api/drive", self.body)
            except (OSError, ValueError) as e:
                s, r = None, str(e)
            self.posts.append((t0, clock(), s, r))
            nxt += DRIVE_PERIOD
            d = nxt - clock()
            if d < -DRIVE_PERIOD:
                nxt = clock()
            elif d > 0:
                self.stop_ev.wait(d)

    def stop(self):
        self.stop_ev.set()
        self.th.join(5.0)
        return self.posts


class Gz:
    def __init__(self, tmp):
        self.tmp = tmp
        self.proc = None
        self.gt = Sampler()
        self.t_start = clock()

    def start_app(self):
        self.port = free_port()
        env = dict(os.environ)
        paths = [ROOT] + ([os.path.join(ROOT, "pylibs")] if os.name == "nt" else [])   # pylibs: Windows wheels
        env["PYTHONPATH"] = os.pathsep.join(paths)
        env["PYTHONIOENCODING"] = "utf-8"
        env["PYTHONUNBUFFERED"] = "1"
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        self.log = os.path.join(self.tmp, "app.log")
        with open(self.log, "w", encoding="utf-8") as f:
            self.proc = subprocess.Popen([sys.executable, SERVER_PY, "--dog", "%s:%d" % (HOST, EMU_PORT),
                                          "--port", str(self.port)], cwd=ROOT, env=env, stdout=f,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        end = clock() + 30.0
        while True:
            if self.proc.poll() is not None:
                raise AssertionError("app2/server.py exited:\n" + self.app_log())
            try:
                s, st = self.app("GET", "/api/state", timeout=2.0)
                if s == 200 and st["connected"] and st["emulator"]:
                    return st
            except (OSError, ValueError):
                pass
            if clock() > end:
                raise AssertionError("app2/server.py never connected to the emulator:\n" + self.app_log())
            time.sleep(0.2)

    def app_log(self):
        try:
            with open(self.log, encoding="utf-8", errors="replace") as f:
                return f.read()[-4000:]
        except OSError:
            return ""

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.post("/api/arm", {"armed": False})
            except (OSError, ValueError):
                pass
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(5)
        self.gt.close()

    # app API
    def app(self, method, path, body=None, timeout=10.0):
        return http_json(self.port, method, path, body, timeout)

    def post(self, path, body=None):
        return self.app("POST", path, {} if body is None else body)

    def state(self):
        s, st = self.app("GET", "/api/state")
        assert s == 200, st
        return st

    def emu(self):
        s, st = http_json(CTL_PORT, "GET", "/state")
        assert s == 200, st
        return st

    def arm(self):
        s, r = self.post("/api/arm", {"armed": True})
        assert s == 200 and r["armed"] is True, (s, r)

    def wait_app(self, pred, timeout, period=0.05):
        end = clock() + timeout
        while True:
            st = self.state()
            if pred(st):
                return st
            if clock() > end:
                return None
            time.sleep(period)

    def run(self, path, body):
        """A queued motion or action: (last_result, t_post, t_result) once the app holds its result."""
        st = self.state()
        seq0 = (st["motion"]["last_result"] or {}).get("seq", 0)
        t0 = clock()
        s, r = self.post(path, body)
        assert s == 200 and r.get("queued") is True, (path, body, s, r)
        st = self.wait_app(lambda st: (st["motion"]["last_result"] or {}).get("seq", 0) > seq0
                           and not st["motion"]["busy"], RESULT_WAIT)
        assert st is not None, "%s %s: no result in %.0f s\n%s" % (path, body, RESULT_WAIT, self.app_log())
        return st["motion"]["last_result"], t0, clock()

    def action(self, name):
        return self.run("/api/action", {"name": name})

    def rtf(self, w0, w1=None):
        r = self.gt.rtf(w0, w1)
        assert r is not None and r > 0.05, "RTF unknown or stalled (%r)" % r
        return r

    def ensure_standing(self, strict=True):
        """Standing on four legs (111/112, z near STAND_Z), armed, ESTOP latch clear. strict=False also
        accepts any upright pose that is not lying, ESTOP or a sit."""
        s = self.gt.fresh()
        self.arm()
        ok = (not s["estop"] and s["mode"] not in (0, 7) and s["z"] > 0.21 and abs(s["pitch"]) < 0.1
              and abs(s["roll"]) < 0.1)
        if strict:
            ok = ok and s["mid"] in (111, 112)
        if not ok:
            res, _, _ = self.action("stand")
            assert res["ok"], "stand (111) failed: %r" % res
        s = wait_still(self, clock(), 6.0 / self.rtf(clock() - 3.0))
        assert s is not None, "the robot does not settle"
        return s

    def ensure_lying(self):
        """Lying after 101, through the app, then disarmed. Not the emulator's POST /reset: that
        publishes 101 even after an ESTOP, and 101 (PureDamper lie-down) from the legs-splayed pose an
        ESTOP during a walk leaves pitched the robot about 1.3 rad onto its nose in Gazebo. The app
        path is the real one: after ESTOP only 111 is accepted (3040 otherwise), then 101."""
        s = self.gt.fresh()
        if not (s["mid"] == 101 and s["z"] < LIE_Z + 0.02 and not s["estop"]):
            self.arm()
            if s["estop"] or s["mode"] not in (12, 11, 21, 3, 62):
                res, _, _ = self.action("stand")
                assert res["ok"], "stand (111) failed: %r" % res
            res, _, _ = self.action("lie")
            assert res["ok"], "lie down (101) failed: %r" % res
        s, r = self.post("/api/arm", {"armed": False})
        assert s == 200 and r["armed"] is False, (s, r)
        s = self.gt.wait(lambda q: q["mid"] == 101 and q["z"] < LIE_Z + 0.02, 15.0)
        assert s is not None, "the robot does not lie down"
        # "still" ignores the vertical speed: the drop ends 1.5-1.6 s sim after 101 starts
        s = self.gt.wait(lambda q: q["t"] >= s["t"] + 1.5, 15.0, since=s["w"])
        assert s is not None and still(s) and abs(s["z"] - LIE_Z) < 0.01, "the robot does not settle lying: %r" % s
        return s


@pytest.fixture(scope="module")
def gz(tmp_path_factory):
    why = sim_not_ready()
    if why:
        pytest.skip(why)
    g = Gz(str(tmp_path_factory.mktemp("gz_e2e")))
    try:
        st = g.start_app()
        assert st["armed"] is False, "the movement gate must be off at app start"
        assert st["drive"]["gait"] == "slow" and abs(st["drive"]["speed"] - 0.4) < 1e-9, st["drive"]
        yield g
    finally:
        g.close()
        _report(g)


def _report(g):
    ss = g.gt.since(g.t_start)
    if len(ss) >= 2:
        rtfs = [g.gt.rtf(a["w"], a["w"] + 2.0) for a in ss[::20] if a["w"] + 2.0 <= ss[-1]["w"]]
        rtfs = [r for r in rtfs if r is not None]
        MEAS["rtf_run"] = {"overall": r4(g.gt.rtf(g.t_start)), "min_2s": r4(min(rtfs)) if rtfs else None,
                           "max_2s": r4(max(rtfs)) if rtfs else None, "wall_s": round(ss[-1]["w"] - ss[0]["w"], 1),
                           "sim_s": round(ss[-1]["t"] - ss[0]["t"], 2), "samples": len(ss),
                           "sample_errors": g.gt.errors}
    try:
        e = g.emu()
        MEAS["emulator_end"] = {"refused": e["refused"][-20:], "client_bugs": e["client_bugs"],
                                "dangerous_calls": e["dangerous_calls"], "calls": e["calls"]}
    except (OSError, ValueError, AssertionError):
        pass
    out = os.environ.get("GZ_E2E_OUT")
    if out:
        with open(out, "w", encoding="utf-8") as f:
            json.dump(MEAS, f, indent=1, sort_keys=True, default=str)
        with open(out + ".trace.jsonl", "w", encoding="utf-8") as f:     # every ground-truth sample
            for q in ss:
                f.write(json.dumps(dict(q, w=round(q["w"] - g.t_start, 3))) + "\n")
    print("\n[gz_e2e] " + json.dumps(MEAS, sort_keys=True, default=str))


# ---- the scenario ----------------------------------------------------------------------------
def test_01_arm_and_stand(gz):
    lying = gz.ensure_lying()                     # the boot state after emu_up.sh, and the end of a full run
    for path, body in (("/api/action", {"name": "stand"}), ("/api/drive", {"x": 1.0, "y": 0, "z": 0})):
        s, r = gz.post(path, body)
        assert s == 403, "gated commands must be refused before arming: %s %s %r" % (path, s, r)
    gz.arm()
    res, t0, t1 = gz.action("stand")
    assert res["ok"] and res["id"] == 111 and res["code"] == 0, res
    s0 = gz.gt.at(t0)
    rise = STAND_Z - lying["z"]
    z95 = gz.gt.wait(lambda s: s["z"] >= lying["z"] + 0.95 * rise, 0.1, since=t0)
    end = wait_still(gz, t1, 10.0)
    assert end is not None, "the robot does not settle after 111"
    after = gz.gt.wait(lambda s: s["w"] >= end["w"] + 0.5, 5.0, since=end["w"])
    rtf = gz.rtf(t0, t1)
    MEAS["stand_111"] = {"result_wall_s": round(t1 - t0, 2), "z_lying": r4(lying["z"]), "z": r4(after["z"]),
                         "z95_wall_s": round(z95["w"] - t0, 2) if z95 else None,
                         "z95_sim_s": round(z95["t"] - s0["t"], 2) if z95 else None,
                         "mode": [after["mode"], after["gait"]], "rtf": r4(rtf)}
    assert z95 is not None, "z never reached 95 % of the stand rise"
    assert z95["t"] - s0["t"] <= 3.5, "111 too slow in sim time (baseline 1.9 s): %r" % MEAS["stand_111"]
    assert abs(after["z"] - STAND_Z) < 0.012, after
    assert after["mid"] == 111 and (after["mode"], after["gait"]) == (12, 0), after
    assert abs(after["roll"]) < 0.05 and abs(after["pitch"]) < 0.05, after


def test_02_drive_forward_then_release(gz):
    s0 = gz.ensure_standing()
    ws0, wd0 = s0["ws"], s0["wd"]
    h = Holder(gz, x=1.0)
    time.sleep(2.0)
    posts = h.stop()
    t_rel = clock()
    s, r = gz.post("/api/drive", {"x": 0, "y": 0, "z": 0})
    assert s == 200 and r["active"] is False, (s, r)
    rel = gz.gt.at(t_rel)
    stop_m, stop = stop_info(gz, rel, 5.0 / gz.rtf(posts[0][0]))
    assert all(p[2] == 200 for p in posts), [p[2:] for p in posts if p[2] != 200]
    assert all(abs(p[3]["vx"] - 0.2) < 1e-9 and p[3]["vy"] == 0 and p[3]["wz"] == 0 for p in posts), posts[0]
    gaps = [b[0] - a[0] for a, b in zip(posts, posts[1:])]
    trip = gz.gt.since(posts[0][0], t_rel)
    fwd, left = heading_split(s0, rel)
    end_ws = gz.gt.wait(lambda s: s["ws"] > ws0, 2.0, since=t_rel)
    rtf = gz.rtf(posts[0][0], t_rel)
    sim_drive = rel["t"] - gz.gt.at(posts[0][0])["t"]
    half = [q for q in trip if q["w"] >= posts[0][0] + 1.0]
    steady = (heading_split(s0, half[-1])[0] - heading_split(s0, half[0])[0]) / max(half[-1]["t"] - half[0]["t"], 1e-6)
    m = {"posts": len(posts), "max_post_gap_s": round(max(gaps), 3), "drive_wall_s": round(t_rel - posts[0][0], 2),
         "drive_sim_s": round(sim_drive, 2), "forward_m": r4(fwd), "left_m": r4(left),
         "dyaw": r4(unwrap_dyaw(trip)), "mean_speed_sim": r4(fwd / max(sim_drive, 1e-6)),
         "steady_speed_sim": r4(steady), "walk_stand_after_release_wall_s": round(end_ws["w"] - t_rel, 3) if end_ws else None,
         "rtf": r4(rtf), "servo_hz": r4(servo_hz(gz, posts[0][0] + 0.3, t_rel))}
    m["after_release"] = stop_m
    MEAS["drive_forward"] = m
    assert m["servo_hz"] >= SERVO_MIN_HZ, "1002 frames starved (machine load?), the run says nothing: %r" % m
    assert stop is not None, "not at rest after release: %r" % m
    m.update({"forward_total_m": r4(heading_split(s0, stop)[0]), "after": [stop["mode"], stop["gait"], stop["mid"]]})
    assert max(gaps) < DRIVE_PERIOD + 0.1, gaps
    # in sim time: 0.2 m/s for drive_sim_s, minus the gait ramp (a load dip can halve the sim seconds)
    assert fwd > max(0.03, 0.3 * 0.2 * sim_drive), "the robot did not walk forward: %r" % m
    assert abs(left) < 0.05 + 0.3 * fwd and abs(m["dyaw"]) < 0.2, "the walk was not straight: %r" % m
    assert m["steady_speed_sim"] < 0.2 * 1.3, "faster than the 0.2 m/s default: %r" % m
    assert end_ws is not None and end_ws["wd"] == wd0, "the release END did not reach the dog: %r" % m
    # baseline: still 0.35-0.63 s sim after 112, coasting 0.03-0.05 m (sim_baseline.md)
    assert stop_m["still_sim_s"] is not None and stop_m["still_sim_s"] <= 1.2, "stop too slow: %r" % m
    assert stop_m["rest_sim_s"] <= 2.0 and stop_m["coast_m"] < 0.1, "stop too slow or too long: %r" % m
    assert stop_m["walkstand_reported_sim_s"] is not None and stop_m["walkstand_reported_sim_s"] <= 1.5, m


def test_03_turn_left_then_deadman(gz):
    s0 = gz.ensure_standing()
    ws0, wd0 = s0["ws"], s0["wd"]
    h = Holder(gz, z=1.0)
    time.sleep(2.0)
    posts = h.stop()                              # the key "sticks": nothing more is sent, no zero
    t_last = posts[-1][1]
    end_ws = gz.gt.wait(lambda s: s["ws"] > ws0, 3.0, since=t_last)
    assert all(p[2] == 200 for p in posts), [p[2:] for p in posts if p[2] != 200]
    assert all(abs(p[3]["wz"] - 0.4) < 1e-9 and p[3]["vx"] == 0 for p in posts), posts[0]
    last = gz.gt.at(t_last)
    fwd, left = heading_split(s0, last)
    st = gz.state()
    m = {"posts": len(posts), "turn_wall_s": round(t_last - posts[0][0], 2),
         "turn_sim_s": round(last["t"] - gz.gt.at(posts[0][0])["t"], 2), "dyaw_at_last_post": r4(unwrap_dyaw(
             gz.gt.since(posts[0][0], t_last))), "drift_m": r4(math.hypot(fwd, left)),
         "end_after_last_post_wall_s": round(end_ws["w"] - t_last, 3) if end_ws else None,
         "watchdog_stops": (end_ws or last)["wd"] - wd0, "rtf": r4(gz.rtf(posts[0][0], t_last)),
         "app_drive_active_after": st["drive"]["active"], "servo_hz": r4(servo_hz(gz, posts[0][0] + 0.3, t_last)),
         "max_post_gap_s": round(max(b[0] - a[0] for a, b in zip(posts, posts[1:])), 3)}
    MEAS["turn_deadman"] = m
    # a starved stream lets the watchdog (4 x 50 ms) stop the walk before the dead-man: no verdict then
    assert m["max_post_gap_s"] < DRIVE_PERIOD + 0.1 and m["servo_hz"] >= SERVO_MIN_HZ, \
        "drive POSTs or 1002 frames starved (machine load?), the run says nothing: %r" % m
    assert end_ws is not None and m["watchdog_stops"] == 0, "the dead-man END did not reach the dog: %r" % m
    stop_m, stop = stop_info(gz, end_ws, 5.0 / gz.rtf(posts[0][0]))
    m["after_end"] = stop_m
    assert stop is not None, "still turning after the dead-man: %r" % m
    m.update({"dyaw_total": r4(unwrap_dyaw(gz.gt.since(posts[0][0], stop["w"]))),
              "rest_after_last_post_wall_s": round(stop["w"] - t_last, 2),
              "rest_after_last_post_sim_s": round(stop["t"] - last["t"], 2),
              "after": [stop["mode"], stop["gait"], stop["mid"]]})
    # in sim time: 0.4 rad/s for turn_sim_s, minus the ramp (run 5: RTF 0.32, 0.62 s sim, 0.149 rad)
    assert m["dyaw_at_last_post"] > max(0.05, 0.3 * 0.4 * m["turn_sim_s"]), \
        "turn left must raise yaw (counter-clockwise): %r" % m
    assert m["drift_m"] < 0.1, m
    # app dead-man 0.4 s (40 Hz tick); the SDK one (0.5 s + one frame deadline) is the backstop
    assert APP_DEADMAN - 0.1 <= m["end_after_last_post_wall_s"] <= SDK_DEADMAN + FRAME_DEADLINE + 0.1, m
    # baseline: still 0.25-0.47 s sim after 112 from a 0.5 rad/s turn (sim_baseline.md)
    assert stop_m["still_sim_s"] is not None and stop_m["still_sim_s"] <= 1.2, "stop too slow: %r" % m
    assert stop_m["rest_sim_s"] <= 2.0, m
    assert stop_m["walkstand_reported_sim_s"] is not None and stop_m["walkstand_reported_sim_s"] <= 1.5, m
    assert st["drive"]["active"] is False


def test_04_keymap_trick(gz):
    s, cat = gz.app("GET", "/api/catalog")
    assert s == 200 and cat["keymap"]["4"] == TRICK
    s0 = gz.ensure_standing()
    res, t0, t1 = gz.run("/api/motion", {"id": TRICK})
    seen = gz.gt.since(t0, t1)
    in62 = [q for q in seen if (q["mode"], q["gait"]) == (62, 6)]
    end = wait_still(gz, t1, 6.0)
    m = {"result": res, "wall_s": round(t1 - t0, 2), "sim_s": round(seen[-1]["t"] - seen[0]["t"], 2) if seen else None,
         "mode62_sim_s": round(in62[-1]["t"] - in62[0]["t"], 2) if in62 else None,
         "states": sorted({(q["mode"], q["gait"]) for q in seen}), "rtf": r4(gz.rtf(t0, t1))}
    MEAS["trick_%d" % TRICK] = m
    assert res["ok"] and res["id"] == TRICK and res["code"] == 0, m
    assert in62, "the controller never ran the mode-62 list for %d: %r" % (TRICK, m)
    assert end is not None, m
    m.update({"travel_m": r4(math.hypot(end["x"] - s0["x"], end["y"] - s0["y"])), "z_after": r4(end["z"]),
              "after": [end["mode"], end["gait"], end["mid"]], "max_abs_roll": r4(max(abs(q["roll"]) for q in seen)),
              "max_abs_pitch": r4(max(abs(q["pitch"]) for q in seen))})
    assert m["travel_m"] < 0.15, "the trick should stay in place: %r" % m
    assert end["z"] > 0.18 and abs(end["roll"]) < 0.1, "not upright after the trick: %r" % m


def test_05_posture_testa_su_then_reset(gz):
    sign = pitch_up_sign()
    gz.ensure_standing(strict=False)              # straight after the trick (214) is the scenario
    s0 = gz.gt.fresh()
    t0 = clock()
    s, r = gz.post("/api/posture", {"pitch": sign * POSE_PITCH})
    assert s == 200 and abs(r["posture"]["pitch"] - sign * POSE_PITCH) < 1e-9, (s, r)
    lim = 12.0 / gz.rtf(t0 - 3.0)
    moved = gz.gt.wait(lambda q: q["mode"] == 3 and abs(q["pitch"] - s0["pitch"]) > 0.08, lim, since=t0)
    m = {"api_pitch": sign * POSE_PITCH, "pitch_up_sign": sign, "pitch_before": r4(s0["pitch"]),
         "from": [s0["mode"], s0["gait"], s0["mid"]]}
    MEAS["posture_testa_su"] = m
    assert moved is not None, "posture 201 never moved the pitch: %r" % m
    settle = gz.gt.wait(lambda q: q["t"] >= moved["t"] + 1.5, lim, since=moved["w"])
    assert settle is not None, m
    gz.gt.wait(lambda q: q["w"] >= settle["w"] + 0.6, 5.0, since=settle["w"])
    win = gz.gt.since(settle["w"], settle["w"] + 0.6)
    pitch = sum(q["pitch"] for q in win) / len(win)
    m.update({"pitch_zyx": r4(pitch), "nose_dz_per_m": r4(-math.sin(pitch)), "z": r4(win[-1]["z"]),
              "first_move_wall_s": round(moved["w"] - t0, 2), "first_move_sim_s": round(moved["t"] - s0["t"], 2),
              "states": sorted({(q["mode"], q["gait"]) for q in gz.gt.since(t0, settle["w"])}),
              "nose": "up" if pitch < 0 else "down"})
    # ZYX pitch > 0 is nose down (sim_baseline.md): "testa su" must give a negative pitch
    assert pitch < -0.08, "«testa su» did not raise the nose in Gazebo: %r" % m
    t1 = clock()
    s, r = gz.post("/api/posture/reset", {})
    assert s == 200, (s, r)
    back = gz.gt.wait(lambda q: abs(q["pitch"]) < 0.03 and abs(q["z"] - 0.235) < 0.01, lim, since=t1)
    m["reset_wall_s"] = round(back["w"] - t1, 2) if back else None
    assert back is not None, "posture reset did not level the body: %r" % gz.gt.latest()
    m["reset_pitch"] = r4(back["pitch"])


def test_06_sit_then_stand(gz):
    gz.ensure_standing(strict=False)
    s0 = gz.gt.fresh()
    res, t0, t1 = gz.action("sit")
    seen = gz.gt.since(t0, t1)
    end = wait_still(gz, t1, 6.0)
    m = {"result": res, "wall_s": round(t1 - t0, 2), "from": [s0["mode"], s0["gait"], s0["mid"]],
         "states": sorted({(q["mode"], q["gait"]) for q in seen}), "rtf": r4(gz.rtf(t0, t1))}
    MEAS["sit_143"] = m
    assert res["ok"] and res["id"] == 143, m
    assert any((q["mode"], q["gait"]) == (62, 3) for q in seen), "no 62/3 list ran: %r" % m
    assert end is not None, m
    m.update({"z_after": r4(end["z"]), "pitch_after": r4(end["pitch"]), "after": [end["mode"], end["gait"], end["mid"]]})
    res, t0, t1 = gz.action("stand")
    end = wait_still(gz, t1, 6.0)
    assert end is not None, m
    m["stand_after_sit"] = {"result": res, "wall_s": round(t1 - t0, 2), "z": r4(end["z"]), "pitch": r4(end["pitch"]),
                            "after": [end["mode"], end["gait"], end["mid"]]}
    assert res["ok"] and res["id"] == 111, m
    assert abs(end["z"] - STAND_Z) < 0.012 and end["mid"] == 111, m


def test_07_estop_during_walk(gz):
    s0 = gz.ensure_standing()
    c0, ws0 = s0["c1004"], s0["ws"]
    refused0 = len(gz.emu()["refused"])
    h = Holder(gz, x=1.0)
    time.sleep(1.5)
    walking = gz.gt.fresh()
    epoch_e = time.time()                         # to line up with an LCM capture in the VM
    t_e = clock()
    s, r = gz.post("/api/estop", {})
    t_e1 = clock()
    time.sleep(1.0)                               # the drive key is still held after ESTOP
    posts = h.stop()
    first0 = gz.gt.wait(lambda q: q["mode"] == 0, 3.0, since=t_e)
    second = gz.gt.wait(lambda q: q["c1004"] >= c0 + 2, 3.0, since=t_e)
    time.sleep(max(0.0, 3.0 / gz.rtf(t_e - 1.0) - (clock() - t_e)))
    after = gz.gt.since(first0["w"] if first0 else t_e)
    fin = gz.gt.fresh()
    st = gz.state()
    late = [p for p in posts if p[0] > t_e1]
    at_e = gz.gt.at(t_e)
    refused = [x for x in gz.emu()["refused"][refused0:] if x.get("nameCode") == 1004]
    m = {"http": s, "reply": r, "estop_wall_s": round(t_e1 - t_e, 3), "post_epoch": round(epoch_e, 4),
         "walking_before": {"v": r4(math.hypot(walking["vx"], walking["vy"])), "mode": [walking["mode"], walking["gait"]]},
         "mode0_after_wall_s": round(first0["w"] - t_e, 3) if first0 else None,
         "second_estop_after_wall_s": round(second["w"] - t_e, 3) if second else None,
         "estops_sent": fin["c1004"] - c0, "walk_stands_between": (second or fin)["ws"] - ws0,
         "walk_stands_after_second": fin["ws"] - (second or fin)["ws"],
         "modes_after": sorted({(q["mode"], q["gait"]) for q in after}), "z_final": r4(fin["z"]),
         "switch_after": sorted({q["sw"] for q in after}), "estop_refusals_1004": [
             (x.get("motion_id"), x.get("reason")) for x in refused],
         "slide_m": r4(math.hypot(fin["x"] - at_e["x"], fin["y"] - at_e["y"])),
         "drives_after_estop": sorted({p[2] for p in late}), "estop_latched": fin["estop"],
         "app_armed": st["armed"], "app_last_result": st["motion"]["last_result"], "rtf": r4(gz.rtf(t_e, fin["w"]))}
    low = gz.gt.wait(lambda q: q["z"] < 0.15, 0.1, since=t_e)
    m["z_below_0.15_sim_s"] = round(low["t"] - at_e["t"], 2) if low else None
    MEAS["estop_walk"] = m
    assert walking["mode"] == 11 and math.hypot(walking["vx"], walking["vy"]) > 0.05, "not walking at ESTOP: %r" % m
    assert s == 200 and r["ok"] and r["armed"] is False and r["result"]["id"] == 0 and r["result"]["ok"], m
    assert first0 is not None and m["mode0_after_wall_s"] <= 1.0, "ESTOP did not reach the controller: %r" % m
    assert m["estops_sent"] == 2, "estop() sends ESTOP twice (contracts.md 3): %r" % m
    assert m["walk_stands_after_second"] == 0, "a WalkStand followed the second ESTOP: %r" % m
    assert {q[0] for q in m["modes_after"]} == {0}, "the robot left Off after ESTOP: %r" % m
    assert fin["estop"] is True and fin["z"] < 0.15, "ESTOP must leave it down: %r" % m
    assert late and all(p[2] == 403 for p in late), "a held drive key must be refused after ESTOP: %r" % m
    assert st["armed"] is False and st["motion"]["last_result"]["id"] == 0


def test_08_recover_then_lie_down(gz):
    gz.arm()
    s0 = gz.gt.fresh()
    res, t0, t1 = gz.action("stand")
    end = wait_still(gz, t1, 10.0)
    z95 = gz.gt.wait(lambda q: q["z"] >= 0.226, 0.1, since=t0)
    m = {"from": [s0["mode"], s0["gait"], s0["mid"]], "z_before": r4(s0["z"]), "result": res,
         "wall_s": round(t1 - t0, 2), "z95_sim_s": round(z95["t"] - gz.gt.at(t0)["t"], 2) if z95 else None,
         "rtf": r4(gz.rtf(t0, t1))}
    MEAS["recover_111"] = m
    assert res["ok"] and res["id"] == 111, m
    assert end is not None and abs(end["z"] - STAND_Z) < 0.012 and end["mid"] == 111 and not end["estop"], end
    m.update({"z": r4(end["z"]), "roll": r4(end["roll"]), "pitch": r4(end["pitch"])})
    res, t0, t1 = gz.action("lie")
    down = gz.gt.wait(lambda q: q["z"] < LIE_Z + 0.01, 10.0, since=t0)
    s1 = gz.gt.at(t1)
    end = gz.gt.wait(lambda q: q["t"] >= s1["t"] + 1.5, 10.0, since=t1)
    m["lie_101"] = {"result": res, "wall_s": round(t1 - t0, 2), "z": r4(end["z"]) if end else None,
                    "z_low_sim_s": round(down["t"] - gz.gt.at(t0)["t"], 2) if down else None}
    assert res["ok"] and res["id"] == 101, m
    assert down is not None and end is not None and end["mid"] == 101 and (end["mode"], end["gait"]) == (7, 1), m
    assert abs(end["z"] - LIE_Z) < 0.01, m
    s, r = gz.post("/api/arm", {"armed": False})
    assert s == 200
    e = gz.emu()
    assert not e["dangerous_calls"] and not e["client_bugs"], (e["dangerous_calls"], e["client_bugs"])


# ---- ESTOP latch against the watchdog's WalkStand: raw LCM, inside the VM, emulator down -----------
# docs/protocol/motion.md 3: after an ESTOP during a 1002 stream, the servo watchdog's StopServoCmd sends
# WalkStand 112 (11/1) through action_ptr_->Execute, past motion_decision's ESTOP latch
# (motion_handler.cpp:234-300). Whether that stands the robot up again is the controller's call:
# fsm_state_passive.cpp:65-93 leaves Off (0) only for 7, 12, motor ctrl and RL reset, and logs "Bad
# Request: Cannot transition from 0 to 11" otherwise. This drives the open-source cyberdog_locomotion of
# the sim directly, framed as motion_action.cpp frames it (a new command is a new life_count, the 20 Hz
# heartbeat repeats it). It says nothing about the dog's own firmware, which may differ.
LCM_HELPERS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_lcm_link.py")
CONTROL_LOG = os.environ.get("GZ_CONTROL_LOG", "/root/sim_logs/control.log")
BAD_REQUEST = b"Cannot transition from 0 to 11"
RAW_WINDOW = 2.0                                  # record this long after the ESTOP, in sim AND wall seconds
# (WalkStand delay after the ESTOP in wall s, walk vx, repeat): "heartbeat" = one new frame, then the
# 20 Hz heartbeat with the same life_count (motion_action.cpp); "fresh" = a new life_count every 50 ms.
WALKSTAND_TRIALS = ((0.225, 0.1, "heartbeat"), (0.02, 0.1, "heartbeat"), (0.10, 0.1, "heartbeat"),
                    (0.40, 0.1, "heartbeat"), (0.225, 0.0, "heartbeat"), (0.225, 0.1, "fresh"))


def _bad_requests():
    """Count of the controller's 'Bad Request ... 0 to 11' lines (printed at a throttled rate)."""
    try:
        with open(CONTROL_LOG, "rb") as f:
            return f.read().count(BAD_REQUEST)
    except OSError:
        return None


@pytest.fixture(scope="module")
def raw():
    if not sys.platform.startswith("linux"):
        pytest.skip("the raw LCM test runs inside the CyberDogSim WSL distro (python3 -m pytest there)")
    spec = importlib.util.spec_from_file_location("gz_e2e_lcm_helpers", LCM_HELPERS)
    lt = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lt)                   # Sim, Pilot, Recorder of test_lcm_link.py
    mod = lt._load_link()                         # skips without python3-lcm or the lcm-gen types
    try:
        route = subprocess.run(["ip", "route", "get", "239.255.76.67"], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, universal_newlines=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as e:
        route = str(e)
    if " dev lo" not in route:                    # sim_up.sh keeps the LCM group on lo: nothing leaves the VM
        pytest.skip("239.255.76.67 is not routed on lo (run /root/sim_up.sh): %s" % route.strip())
    seen = []
    probe = mod.lcm.LCM(mod.URL_CMD)
    probe.subscribe(mod.CH_CMD, lambda ch, data: seen.append(1))
    t_end = time.monotonic() + 0.6
    while time.monotonic() < t_end:
        probe.handle_timeout(100)
    del probe
    if seen:
        pytest.skip("another process publishes robot_control_cmd (the emulator): run /root/emu_down.sh first")
    s = lt.Sim(mod)
    try:
        if not lt.wait_for(lambda: s.gt()["t"] is not None and s.resp() is not None, 10.0, 0.1):
            pytest.skip("no simulator_state / robot_control_response: start FAST=1 /root/sim_up.sh")
        time.sleep(2.2)                           # fill the RTF window
        s.rtf = max(0.1, s.link.rtf() or 0.5)
        yield s
    finally:
        try:
            try:
                s.lie()                           # leave it as test_lcm_link.py and emu_up.sh expect
            finally:
                MEAS["raw_rtf_end"] = r4(s.link.rtf())
                print("\n[gz_e2e raw] " + json.dumps(MEAS, sort_keys=True, default=str))
                out = os.environ.get("GZ_E2E_OUT")
                if out:
                    with open(out, "w", encoding="utf-8") as f:
                        json.dump(MEAS, f, indent=1, sort_keys=True, default=str)
        finally:
            s.close()


def _estop_then_walkstand(sim, delay, vx, how):
    """Walk (11/27 at vx), ESTOP (0), WalkStand (11/1) `delay` wall s later; record RAW_WINDOW s."""
    sim.stand()
    t_w0 = sim.pilot.send(11, 27, vel_des=[vx, 0.0, 0.0], step_height=[0.05, 0.05])
    e = sim.rec.wait_resp(lambda r: (r["mode"], r["gait_id"]) == (11, 27), t_w0, sim_s=2.0)
    assert e is not None, "the walk 11/27 was not reported: %r" % (sim.resp(),)
    sim.sleep_sim(1.2)                            # well into the gait cycle
    g_walk = sim.gt()
    bad0 = _bad_requests()
    t_e = sim.pilot.send(0, 0)                    # ESTOP: mode 0, gait 0 (motion_id 0)
    s_e = sim.link.sim_time()
    while time.monotonic() < t_e + delay:
        time.sleep(0.001)
    t_s = sim.pilot.send(11, 1, step_height=[0.05, 0.05])   # WalkStand 112, as StopServoCmd sends it
    s_s = sim.link.sim_time()
    fresh = 1
    while how == "fresh" and time.monotonic() < t_s + 1.0:
        time.sleep(0.05)
        sim.pilot.send(11, 1, step_height=[0.05, 0.05])
        fresh += 1
    cap = time.monotonic() + 3.0 * RAW_WINDOW / max(0.05, sim.rtf / 3.0)
    while time.monotonic() < cap and (sim.link.sim_time() < s_e + RAW_WINDOW or time.monotonic() < t_e + RAW_WINDOW):
        time.sleep(0.02)
    t_end = time.monotonic()
    resps = sim.rec.resps_between(t_e, t_end)
    trace = [g for g in sim.rec.trace_between(t_e, t_end) if g["z"] is not None]
    changes = []                                  # (sim s after the ESTOP, mode, gait, switch, bar)
    for w, s, r in resps:
        k = (r["mode"], r["gait_id"], r["switch_status"])
        if not changes or tuple(changes[-1][1:4]) != k:
            changes.append((round(s - s_e, 3), r["mode"], r["gait_id"], r["switch_status"], r["order_process_bar"]))
    first0 = next(((w, s) for w, s, r in resps if r["mode"] == 0), None)
    after0 = [r for w, s, r in resps if first0 is not None and w >= first0[0]]
    low = next((g for g in trace if g["z"] < 0.15), None)
    zs = [round(g["z"], 3) for g in trace[::max(1, len(trace) // 10)]]
    bad1 = _bad_requests()
    return {"delay_wall_s": delay, "walk_vx": vx, "repeat": how, "walkstand_frames": fresh,
            "walkstand_after_estop_sim_s": round(s_s - s_e, 3),
            "walking_v": r4(math.hypot(g_walk["vx"], g_walk["vy"])), "z_walking": r4(g_walk["z"]),
            "first_mode0_sim_s": round(first0[1] - s_e, 3) if first0 else None,
            "modes_after_first_0": sorted({(r["mode"], r["gait_id"]) for r in after0}),
            "switch_after_first_0": sorted({r["switch_status"] for r in after0}),
            "response_changes": changes, "z_min": r4(min(g["z"] for g in trace)) if trace else None,
            "z_end": r4(trace[-1]["z"]) if trace else None, "z_trace_10": zs,
            "z_below_0.15_sim_s": round(low["t"] - s_e, 3) if low else None,
            "window_sim_s": round(sim.link.sim_time() - s_e, 2), "window_wall_s": round(t_end - t_e, 2),
            "bad_request_lines": None if bad0 is None or bad1 is None else bad1 - bad0}


def test_09_raw_lcm_walkstand_cannot_lift_estop(raw):
    trials = []
    MEAS["estop_vs_walkstand_raw"] = trials
    for delay, vx, how in WALKSTAND_TRIALS:
        trials.append(_estop_then_walkstand(raw, delay, vx, how))
    for m in trials:
        assert m["walking_v"] > 0.03 or m["walk_vx"] == 0.0, "not walking before the ESTOP: %r" % m
        assert m["first_mode0_sim_s"] is not None, "the controller never reported Off (0): %r" % m
        # open-source controller: once Off, a WalkStand (11/1) is refused and the robot stays down
        assert m["modes_after_first_0"] == [(0, 0)], "the controller left Off after the ESTOP: %r" % m
        assert m["z_end"] is not None and m["z_end"] < 0.15, "the robot did not stay down: %r" % m
