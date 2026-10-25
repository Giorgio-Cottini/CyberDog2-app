"""LcmController against cyberdog_locomotion in Gazebo (part C; sim_bridge.md sections 1 and 3).

Runs inside the CyberDogSim WSL distro with the sim up (FAST=1 /root/sim_up.sh) and no emulator running:
    cd /mnt/c/<path to the repository> && python3 -m pytest -s tests/gazebo/test_lcm_link.py
Skipped on Windows, without python3-lcm, without simulator_state traffic, or when another process
already publishes robot_control_cmd. The tests run in file order and share one robot; each one first
brings the robot to the state it needs, so any single test also runs alone.

The test drives the link directly: a 20 Hz heartbeat republishes the last frame with the same
life_count (sim_bridge.md 2.4). Expected motion sizes are in simulated time, measured from the
simulator_state frame count, so they do not depend on the real-time factor (RTF); wall-clock waits
are divided by the RTF measured at start. Set LCM_BASELINE_OUT=<file.json> to save every measurement.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import subprocess
import sys
import threading
import time

import pytest

pytestmark = pytest.mark.gazebo

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LINK_PATH = os.path.join(ROOT, "emulator", "controllers", "lcm_link.py")
HEARTBEAT = 0.05
BASE = {}                                         # measurements, dumped at module teardown


def i8(n):
    return ((n + 128) % 256) - 128


def _load_link():
    if not sys.platform.startswith("linux"):
        pytest.skip("LCM link tests run inside the CyberDogSim WSL distro")
    spec = importlib.util.spec_from_file_location("cyberdog_lcm_link", LINK_PATH)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)              # by path: no emulator package import (grpcio may be absent)
    except ImportError as e:
        pytest.skip(str(e))
    return mod


def wait_for(pred, timeout, step=0.01):
    t_end = time.monotonic() + timeout
    while True:
        v = pred()
        if v:
            return v
        if time.monotonic() >= t_end:
            return None
        time.sleep(step)


class Pilot:
    """Minimal motion_action: new frame = new life_count; 20 Hz heartbeat of the same frame."""

    def __init__(self, link):
        self.link, self.life, self.frame = link, 0, None
        self.lk = threading.Lock()
        self.beating = threading.Event()
        self.beating.set()
        self.stop_ev = threading.Event()
        self.t_last = 0.0                         # wall time of the last frame sent (new or heartbeat)
        threading.Thread(target=self._loop, name="pilot-heartbeat", daemon=True).start()

    def send(self, mode, gait_id, duration=0, contact=15, value=0, **arrays):
        c = {"mode": mode, "gait_id": gait_id, "contact": contact, "value": value, "duration": duration}
        for f, n in (("vel_des", 3), ("rpy_des", 3), ("pos_des", 3), ("acc_des", 6), ("ctrl_point", 3),
                     ("foot_pose", 6), ("step_height", 2)):
            c[f] = [float(x) for x in arrays.get(f, [0.0] * n)]
        with self.lk:
            c["life_count"] = i8(self.life)
            self.life += 1
            self.frame = c
            self.link.publish(c)
            self.t_last = time.monotonic()
        return self.t_last

    def _loop(self):
        while not self.stop_ev.wait(HEARTBEAT):
            if not self.beating.is_set():
                continue
            with self.lk:
                if self.frame is not None:
                    self.link.publish(self.frame)
                    self.t_last = time.monotonic()

    def pause(self):
        with self.lk:
            self.beating.clear()
            return self.t_last

    def resume(self):
        self.beating.set()

    def close(self):
        self.stop_ev.set()


class Recorder:
    """Every response with (wall, sim) stamps, and a 50 Hz ground-truth trace."""

    def __init__(self, link):
        self.link = link
        self.cv = threading.Condition()
        self.resps = []
        self.trace = []
        self.stop_ev = threading.Event()
        link.on_response(self._on_resp)
        threading.Thread(target=self._sample, name="gt-trace", daemon=True).start()

    def _on_resp(self, r):
        with self.cv:
            self.resps.append((time.monotonic(), self.link.sim_time(), r))
            self.cv.notify_all()

    def _sample(self):
        while not self.stop_ev.wait(0.02):
            g = self.link.ground_truth()
            if g["t"] is not None:
                self.trace.append((time.monotonic(), g))

    def wait_resp(self, pred, since, sim_s=None, wall_s=None):
        """First response at or after wall time `since` matching pred: (wall, sim, resp) or None.
        Gives up after sim_s simulated seconds (RTF-proof), or wall_s wall seconds for wall-clock rules.
        A sim budget also has a wall cap at RTF 0.05, so a stalled sim cannot hang the test."""
        s_end = None if sim_s is None else self.link.sim_time() + sim_s
        t_end = time.monotonic() + (wall_s if wall_s is not None else sim_s / 0.05 + 5.0)
        i = 0
        with self.cv:
            while True:
                while i < len(self.resps):
                    e = self.resps[i]
                    i += 1
                    if e[0] >= since and pred(e[2]):
                        return e
                if time.monotonic() >= t_end or (s_end is not None and self.link.sim_time() >= s_end):
                    return None
                self.cv.wait(0.05)

    def resps_between(self, t0, t1):
        with self.cv:
            return [e for e in self.resps if t0 <= e[0] <= t1]

    def trace_between(self, t0, t1):
        return [g for (w, g) in list(self.trace) if t0 <= w <= t1]


class Sim:
    def __init__(self, mod):
        self.mod = mod
        self.link = mod.LcmController()
        self.rec = Recorder(self.link)
        self.pilot = Pilot(self.link)
        self.rtf = 1.0
        self.s_pub = 0.0                          # sim time of the last command() publish

    def gt(self):
        return self.link.ground_truth()

    def resp(self):
        return self.link.last_response()

    def wall(self, sim_s, margin=1.0):
        """Wall seconds to allow for sim_s simulated seconds, at the RTF measured now (floor: 1/3 of it)."""
        rtf = min(self.rtf, self.link.rtf() or self.rtf)
        return sim_s / max(0.05, rtf / 3.0) + margin

    def sleep_sim(self, sim_s):
        t_end = self.link.sim_time() + sim_s
        if not wait_for(lambda: self.link.sim_time() >= t_end, self.wall(sim_s, 3.0)):
            pytest.fail("simulator_state stalled")

    def wait_gt(self, pred, sim_s):
        """Wait until pred(ground_truth) holds, for at most sim_s simulated seconds; returns sim seconds or None."""
        s0 = self.link.sim_time()
        ok = wait_for(lambda: pred(self.gt()) or self.link.sim_time() - s0 > sim_s, self.wall(sim_s, 3.0), 0.02)
        return self.link.sim_time() - s0 if ok and pred(self.gt()) else None

    def mean_gt(self, sim_s=0.4):
        t0 = time.monotonic()
        self.sleep_sim(sim_s)
        g = self.rec.trace_between(t0, time.monotonic())
        keys = ("x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "wz")
        return {k: sum(e[k] for e in g) / len(g) for k in keys} if g else self.gt()

    def command(self, mode, gait, done_bar=100, sim_len=1.0, **kw):
        """Send a new frame; wait for (mode, gait) with bar >= done_bar. Returns (sim seconds, wall seconds, resp)."""
        s0 = self.s_pub = self.link.sim_time()
        t0 = self.pilot.send(mode, gait, **kw)
        e = self.rec.wait_resp(lambda r: r["mode"] == mode and r["gait_id"] == gait
                               and r["order_process_bar"] >= done_bar, t0, sim_s=sim_len * 1.5 + 0.5)
        if e is None:
            return None, None, self.resp()
        return e[1] - s0, e[0] - t0, e[2]

    def is_standing(self):
        r, g = self.resp(), self.gt()
        return (r is not None and g["z"] is not None and r["switch_status"] == 0 and g["z"] > 0.2
                and (r["mode"], r["gait_id"]) in ((12, 0), (11, 1), (3, 0)) and r["order_process_bar"] >= 100)

    def stand(self):
        if self.is_standing():
            return
        dt, _, r = self.command(12, 0, sim_len=15.0)
        assert dt is not None, "111 [12,0] did not complete: %r" % (r,)
        self.sleep_sim(0.5)

    def lie(self):
        r, g = self.resp(), self.gt()
        if r is not None and r["mode"] == 7 and g["z"] is not None and g["z"] < 0.15:
            return
        if r is not None and r["mode"] not in (7, 12, 0, 15, 80):     # 7 is reachable from 11/21/12/3, not from 23
            self.stand()
        dt, _, r = self.command(7, 1, sim_len=3.0)
        assert dt is not None, "101 [7,1] did not complete: %r" % (r,)
        self.sleep_sim(1.5)

    def close(self):
        self.rec.stop_ev.set()
        self.pilot.close()
        self.link.close()


@pytest.fixture(scope="module")
def sim():
    mod = _load_link()
    try:                                          # sim_up.sh routes the LCM group on lo: nothing leaves the VM
        route = subprocess.run(["ip", "route", "get", "239.255.76.67"], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, universal_newlines=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as e:
        route = str(e)
    if " dev lo" not in route:
        pytest.skip("239.255.76.67 is not routed on lo (run /root/sim_up.sh): %s" % route.strip())
    seen = []                                     # someone else (an emulator) on robot_control_cmd?
    probe = mod.lcm.LCM(mod.URL_CMD)
    probe.subscribe(mod.CH_CMD, lambda ch, data: seen.append(1))
    t_end = time.monotonic() + 0.6
    while time.monotonic() < t_end:
        probe.handle_timeout(100)
    del probe
    if seen:
        pytest.skip("another process publishes robot_control_cmd (stop the emulator: /root/emu_down.sh)")
    s = Sim(mod)
    try:
        if not wait_for(lambda: s.gt()["t"] is not None and s.resp() is not None, 10.0, 0.1):
            pytest.skip("no simulator_state / robot_control_response: start FAST=1 /root/sim_up.sh")
        time.sleep(2.2)                           # fill the RTF window
        s.rtf = max(0.1, s.link.rtf() or 0.5)
        BASE["rtf_start"] = round(s.rtf, 3)
        yield s
    finally:
        try:
            BASE["rtf_end"] = round(s.link.rtf() or 0.0, 3)
            BASE["life_offset"] = s.link.life_offset
            BASE["link_stats"] = s.link.stats()
            print("\nLCM link measurements:\n" + json.dumps(BASE, indent=1, sort_keys=True))
            out = os.environ.get("LCM_BASELINE_OUT")
            if out:
                with open(out, "w") as f:
                    json.dump(BASE, f, indent=1, sort_keys=True)
        finally:
            s.close()


def _r(x, n=3):
    return None if x is None else round(x, n)


def _heading_disp(g0, g1):
    c, s = math.cos(g0["yaw"]), math.sin(g0["yaw"])
    dx, dy = g1["x"] - g0["x"], g1["y"] - g0["y"]
    return dx * c + dy * s, -dx * s + dy * c


def _unwrapped_yaw(trace):
    out, prev, acc = [], None, 0.0
    for g in trace:
        if prev is not None:
            d = g["yaw"] - prev
            acc += math.atan2(math.sin(d), math.cos(d))
        prev = g["yaw"]
        out.append(acc)
    return out


# ---------------------------------------------------------------------------------------------
def test_lying_baseline(sim):
    r = sim.resp()
    BASE["boot_response"] = {k: r[k] for k in ("mode", "gait_id", "switch_status", "order_process_bar", "contact")}
    sim.lie()
    g = sim.mean_gt(0.5)
    r = sim.resp()
    BASE["lying"] = {"z": _r(g["z"], 4), "roll": _r(g["roll"]), "pitch": _r(g["pitch"]),
                     "mode": r["mode"], "gait_id": r["gait_id"], "contact": r["contact"]}
    assert g["z"] < 0.15
    assert abs(g["roll"]) < 0.2 and abs(g["pitch"]) < 0.2


def test_stand_111(sim):
    sim.lie()
    z_lie = sim.mean_gt(0.3)["z"]
    t0w = time.monotonic()
    dt, dtw, r = sim.command(12, 0, sim_len=15.0)
    assert dt is not None, "111 [12,0] never reached bar 100: %r" % (r,)
    sim.sleep_sim(1.0)
    g = sim.mean_gt(0.5)
    r = sim.resp()
    tr = [(e["t"], e["z"]) for e in sim.rec.trace_between(t0w, time.monotonic())]
    z_end = g["z"]
    t_start = tr[0][0]
    t_95 = next((t for t, z in tr if z - z_lie >= 0.95 * (z_end - z_lie)), None)
    bars = sorted({e[2]["order_process_bar"] for e in sim.rec.resps_between(t0w, time.monotonic())
                   if e[2]["mode"] == 12})
    BASE["stand_111"] = {"z": _r(z_end, 4), "rise": _r(z_end - z_lie, 4), "bar100_sim_s": _r(dt),
                         "bar100_wall_s": _r(dtw), "z95_sim_s": _r(None if t_95 is None else t_95 - t_start),
                         "contact": r["contact"], "switch": r["switch_status"], "roll": _r(g["roll"]),
                         "pitch": _r(g["pitch"]), "bars_seen_min_max": [bars[0], bars[-1]] if bars else None}
    assert (r["mode"], r["gait_id"], r["switch_status"]) == (12, 0, 0)
    assert r["order_process_bar"] == 100
    assert z_end - z_lie >= 0.1
    assert r["contact"] & 0x0F == 0x0F


def _walk(sim, vel, wall_s=4.0):
    sim.stand()
    g0 = sim.gt()
    t0 = sim.pilot.send(11, 27, vel_des=vel, step_height=[0.05, 0.05])
    s0 = sim.link.sim_time()
    time.sleep(wall_s)
    g1 = sim.gt()
    t1 = time.monotonic()
    s1 = sim.link.sim_time()
    seen = {(e[2]["mode"], e[2]["gait_id"], e[2]["switch_status"]) for e in sim.rec.resps_between(t0 + 0.3, t1)}
    tr = sim.rec.trace_between(t0, t1)
    # stop: 112 [11,1], continuous
    ts = sim.pilot.send(11, 1, step_height=[0.05, 0.05])
    ss = sim.link.sim_time()
    stop = sim.rec.wait_resp(lambda r: (r["mode"], r["gait_id"]) == (11, 1), ts, sim_s=1.0)
    still = sim.wait_gt(lambda g: math.hypot(g["vx"], g["vy"]) < 0.05 and abs(g["wz"]) < 0.1, 3.0)
    t_still = sim.link.sim_time() - ss if still is not None else None
    sim.sleep_sim(0.5)
    g2 = sim.gt()
    return dict(g0=g0, g1=g1, g2=g2, sim_s=s1 - s0, wall_s=t1 - t0, seen=seen, trace=tr,
                stop_resp=stop, stop_sim_s=t_still)


def test_walk_303(sim):
    w = _walk(sim, [0.3, 0.0, 0.0])
    fwd, lat = _heading_disp(w["g0"], w["g1"])
    tr = w["trace"]
    half = [g for g in tr if g["t"] >= tr[0]["t"] + 0.5 * (tr[-1]["t"] - tr[0]["t"])]
    ss_speed = None
    if len(half) > 2:
        a, b = half[0], half[-1]
        ss_speed = _heading_disp(w["g0"], b)[0] - _heading_disp(w["g0"], a)[0]
        ss_speed /= max(1e-6, b["t"] - a["t"])
    BASE["walk_303_vx0.3"] = {"wall_s": _r(w["wall_s"], 2), "sim_s": _r(w["sim_s"], 2),
                              "forward_m": _r(fwd), "lateral_m": _r(lat),
                              "mean_speed": _r(fwd / w["sim_s"]), "steady_speed_2nd_half": _r(ss_speed),
                              "yaw_drift": _r(w["g1"]["yaw"] - w["g0"]["yaw"]),
                              "z_walking": _r(w["g1"]["z"], 4), "responses": sorted(w["seen"]),
                              "stop_to_still_sim_s": _r(w["stop_sim_s"]),
                              "coast_after_stop_m": _r(_heading_disp(w["g0"], w["g2"])[0] - fwd)}
    assert (11, 27, 0) in w["seen"]
    expected = 0.3 * w["sim_s"]
    assert 0.3 * expected <= fwd <= 1.5 * expected, (fwd, expected)
    assert w["stop_resp"] is not None, "112 [11,1] not reported"
    assert w["stop_sim_s"] is not None and w["stop_sim_s"] < 2.0


def test_turn_303(sim):
    w = _walk(sim, [0.0, 0.0, 0.5])
    yaw = _unwrapped_yaw(w["trace"])
    dyaw = yaw[-1] if yaw else 0.0
    tr = w["trace"]
    k = next((i for i, g in enumerate(tr) if g["t"] >= tr[0]["t"] + 0.5 * (tr[-1]["t"] - tr[0]["t"])), None)
    ss_wz = None if k is None or k >= len(tr) - 2 else (yaw[-1] - yaw[k]) / max(1e-6, tr[-1]["t"] - tr[k]["t"])
    fwd, lat = _heading_disp(w["g0"], w["g1"])
    BASE["turn_303_wz0.5"] = {"wall_s": _r(w["wall_s"], 2), "sim_s": _r(w["sim_s"], 2), "dyaw": _r(dyaw),
                              "mean_wz": _r(dyaw / w["sim_s"]), "steady_wz_2nd_half": _r(ss_wz),
                              "drift_m": _r(math.hypot(fwd, lat)), "stop_to_still_sim_s": _r(w["stop_sim_s"])}
    expected = 0.5 * w["sim_s"]
    assert 0.3 * expected <= dyaw <= 1.5 * expected, (dyaw, expected)


def test_posture_211(sim):
    sim.stand()
    g0 = sim.mean_gt(0.4)
    t0 = time.monotonic()
    dt, dtw, r = sim.command(21, 5, done_bar=96, sim_len=1.0, duration=1000, pos_des=[0.0, 0.0, 0.20])
    assert dt is not None, "211 [21,5] never passed bar 95: %r" % (r,)
    fill = sim.rec.wait_resp(lambda r: (r["mode"], r["gait_id"]) == (21, 3), t0, sim_s=1.5)
    sim.sleep_sim(0.5)
    g1 = sim.mean_gt(0.4)
    states = []                                   # (mode, gait, switch, sim s after publish) at each change
    for e in sim.rec.resps_between(t0, time.monotonic()):
        k = (e[2]["mode"], e[2]["gait_id"], e[2]["switch_status"])
        if not states or states[-1][:3] != k:
            states.append(k + (round(e[1] - sim.s_pub, 3),))
    BASE["posture_211_z0.20"] = {"z_before": _r(g0["z"], 4), "z_after": _r(g1["z"], 4),
                                 "bar96_sim_s": _r(dt),
                                 "filler_21_3_sim_s": _r(None if fill is None else fill[1] - sim.s_pub),
                                 "roll": _r(g1["roll"]), "pitch": _r(g1["pitch"]), "states": states}
    assert g1["z"] < g0["z"] - 0.02
    assert abs(g1["roll"]) < 0.1 and abs(g1["pitch"]) < 0.1
    assert fill is not None, "no 21/3 filler after the timed 21/5"


def test_posture_212_pitch_sign(sim):
    r = sim.resp()
    if r is None or r["mode"] != 21:
        sim.stand()
    g0 = sim.mean_gt(0.4)
    dt, _, r = sim.command(21, 0, done_bar=96, sim_len=1.0, duration=1000, rpy_des=[0.0, 0.2, 0.0])
    assert dt is not None, "212 [21,0] never passed bar 95: %r" % (r,)
    sim.sleep_sim(0.6)
    g1 = sim.mean_gt(0.4)
    dp = g1["pitch"] - g0["pitch"]
    nose = -math.sin(g1["pitch"]) + math.sin(g0["pitch"])  # world z of the body x axis (ZYX), after - before
    # undo: 212 is relative, the opposite offset brings the pitch back
    dt2, _, _ = sim.command(21, 0, done_bar=96, sim_len=1.0, duration=1000, rpy_des=[0.0, -0.2, 0.0])
    sim.sleep_sim(0.6)
    g2 = sim.mean_gt(0.4)
    BASE["posture_212_pitch0.2"] = {"pitch_before": _r(g0["pitch"]), "pitch_after": _r(g1["pitch"]),
                                    "dpitch": _r(dp), "nose_dz_per_m": _r(nose),
                                    "positive_pitch_des_moves_nose": "down" if nose < 0 else "up",
                                    "z_before": _r(g0["z"], 4), "z_after": _r(g1["z"], 4),
                                    "bar96_sim_s": _r(dt), "pitch_after_undo": _r(g2["pitch"]),
                                    "undo_bar96_sim_s": _r(dt2)}
    assert 0.1 <= abs(dp) <= 0.35, dp
    assert abs(g2["pitch"] - g0["pitch"]) < 0.08


def test_qpstand_201(sim):
    r = sim.resp()
    if r is None or r["mode"] not in (21, 12, 11, 3):
        sim.stand()
    g0 = sim.mean_gt(0.4)
    out = {"z_before": _r(g0["z"], 4)}
    for name, pos, rpy in (("pitch", [0.0, 0.0, 0.22], [0.0, 0.15, 0.0]),
                           ("roll", [0.0, 0.0, 0.22], [0.15, 0.0, 0.0]),
                           ("neutral", [0.0, 0.0, 0.25], [0.0, 0.0, 0.0])):
        t0 = sim.pilot.send(3, 0, pos_des=pos, rpy_des=rpy)
        e = sim.rec.wait_resp(lambda r: (r["mode"], r["gait_id"]) == (3, 0), t0, sim_s=1.0)
        assert e is not None, "201 [3,0] not reported: %r" % (sim.resp(),)
        sim.sleep_sim(1.5)
        g = sim.mean_gt(0.4)
        out[name] = {"pos_des_z": pos[2], "rpy_des": rpy, "z": _r(g["z"], 4), "roll": _r(g["roll"]),
                     "pitch": _r(g["pitch"]), "switch": sim.resp()["switch_status"],
                     "bar": sim.resp()["order_process_bar"]}
    BASE["qpstand_201"] = out
    assert abs(out["pitch"]["pitch"] - g0["pitch"]) >= 0.05
    assert abs(out["roll"]["roll"] - g0["roll"]) >= 0.05
    assert abs(out["pitch"]["z"] - out["neutral"]["z"]) >= 0.01


def test_ban_trans_mode_23(sim):
    sim.stand()
    if (sim.resp()["mode"], sim.resp()["gait_id"]) != (12, 0):   # after 201 / 112: a clean 12/0
        sim.command(12, 0, sim_len=15.0)
        sim.sleep_sim(0.3)
    z0 = sim.gt()["z"]
    s0 = sim.link.sim_time()
    t0 = sim.pilot.send(23, 0)
    e = sim.rec.wait_resp(lambda r: r["switch_status"] == 5, t0, sim_s=1.0)
    sim.sleep_sim(0.5)
    r = sim.resp()
    BASE["ban_trans_23"] = {"switch": None if e is None else e[2]["switch_status"],
                            "after_sim_s": _r(None if e is None else e[1] - s0),
                            "mode_kept": r["mode"], "gait_kept": r["gait_id"], "dz": _r(sim.gt()["z"] - z0, 4)}
    assert e is not None, "no BAN_TRANS (5) for mode 23: %r" % (r,)
    assert r["mode"] == 12 and r["switch_status"] == 5
    assert abs(sim.gt()["z"] - z0) < 0.03
    dt, _, r = sim.command(12, 0, sim_len=15.0)  # a valid command clears the ban
    assert dt is not None and r["switch_status"] == 0


def test_pure_damper_on_silence(sim):
    sim.stand()
    z0 = sim.gt()["z"]
    t_last = sim.pilot.pause()
    e = sim.rec.wait_resp(lambda r: r["mode"] == 7, t_last, wall_s=3.0)   # CMD_TIMEOUT is wall-clock
    took = None if e is None else e[0] - t_last
    sim.sleep_sim(1.5)
    z1 = sim.gt()["z"]
    r_damp = sim.resp()
    # the heartbeat comes back with the same (already processed) frame: does the old command resume?
    t_res = time.monotonic()
    sim.pilot.resume()
    sim.sleep_sim(1.0)
    after = sorted({(x[2]["mode"], x[2]["gait_id"]) for x in sim.rec.resps_between(t_res + 0.2, time.monotonic())})
    BASE["pure_damper"] = {"wall_s_after_last_frame": _r(took), "z_before": _r(z0, 4),
                           "z_1.5s_sim_later": _r(z1, 4), "gait_id": r_damp["gait_id"],
                           "after_duplicate_heartbeat_resumes": after}
    assert e is not None, "no PureDamper after the heartbeat stopped"
    assert 0.45 <= took <= 0.5 + 0.3 / sim.rtf, took


def test_lie_down_101(sim):
    sim.stand()
    if (sim.resp()["mode"], sim.resp()["gait_id"]) != (12, 0):
        sim.command(12, 0, sim_len=15.0)
        sim.sleep_sim(0.5)
    t0w = time.monotonic()
    dt, dtw, r = sim.command(7, 1, sim_len=3.0)
    assert dt is not None, "101 [7,1] never reached bar 100: %r" % (r,)
    sim.sleep_sim(2.0)
    g = sim.mean_gt(0.5)
    tr = [(e["t"], e["z"]) for e in sim.rec.trace_between(t0w, time.monotonic())]
    z0, z_end = tr[0][1], g["z"]
    t_95 = next((t for t, z in tr if z0 - z >= 0.95 * (z0 - z_end)), None)
    BASE["lie_101"] = {"z": _r(z_end, 4), "bar100_sim_s": _r(dt), "bar100_wall_s": _r(dtw),
                       "z95_sim_s": _r(None if t_95 is None else t_95 - tr[0][0]),
                       "mode": sim.resp()["mode"], "gait_id": sim.resp()["gait_id"]}
    lying = BASE.get("lying", {}).get("z")
    assert z_end < 0.15
    if lying is not None:
        assert abs(z_end - lying) < 0.03
