"""FakeController: the cyberdog_locomotion controller at the robot_control_response level
(sim_bridge.md sections 1.2-1.3, 3 and 4), plus a kinematic pose.

It reproduces what motion_manager can observe: the per-state transition tables and BAN_TRANS,
progress bars over motion lengths, the idle fillers after timed commands, mode-62 motion lists,
the 0.5 s command timeout to PureDamper, and switch_status. Motion lengths are multiplied by
time_scale; the 0.5 s timeout is wall-clock, as on the robot (command_interface.cpp:1008).
Heights, lengths, signs and tracking are calibrated against Gazebo (docs/design/sim_baseline.md).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

DATA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")

# Allowed next modes per FSM state (sim_bridge.md section 4; fsm_state_*.cpp CheckTransition).
TABLE = {
    11: {11, 12, 0, 3, 7, 21, 16, 80, 81},           # fsm_state_locomotion.cpp:109-149
    21: {21, 7, 3, 12, 0, 11, 64, 16, 22, 80},       # fsm_state_posectrl.cpp:200-231
    12: {12, 21, 7, 3, 0, 11, 64, 16, 22, 9, 80, 81},  # fsm_state_recoverystand.cpp:1120-1153
    7: {7, 15, 12, 0, 80},                           # fsm_state_pure_damper.cpp:132-137
    0: {0, 7, 12, 15, 80},                           # fsm_state_passive.cpp:72-79
    3: {3, 7, 12, 80, 0, 11, 21, 16},
    22: {22, 21, 7, 3, 12, 80, 0, 11, 64},
    16: {16, 12, 80, 21, 7, 0},
    9: {9, 7, 12, 0, 11, 3, 80},                     # 11/3/80 are conditional on the robot
    15: {15, 0, 7, 12, 80},
    64: {64, 3, 12, 80, 0, 7, 16, 21},
    80: {80, 21, 7, 3, 0, 11, 64, 16, 22, 9, 12, 81},
    81: {81, 0, 7, 12, 11, 80},
}
CMD_TIMEOUT = 0.5          # command_interface.hpp:29, wall-clock (Gazebo: mode 7 after 0.50-0.555 s)
TRANS = 0.04               # kTransitioning window when the state or the gait changes [INF]
# Leaving 11 for these states first ends the gait cycle, 10 MPC periods of 30 ms, while the response
# still reads mode 11 with switch_status 1 (fsm_state_locomotion.cpp:157-192). Gazebo: 0.30 s.
LOCO_EXIT, LOCO_EXIT_TO = 0.3, {3, 7, 12, 21, 80, 81}
FILLER = 0.4               # 200 ticks of 2 ms (command_interface.cpp:206-250)
# Gazebo calibration (sim_baseline.md). Attitude is ZYX of the ground truth: +pitch is nose down.
LIE_Z, STAND_Z, WALK_Z = 0.054, 0.2353, 0.25
POSE5_SAG = 0.0075         # 21/5 settles below pos_des z: 0.1925 for 0.20
PITCH21 = -1.015           # 21/x is relative and rpy_des[1] +0.2 moves pitch by -0.203 (nose up)
QP_GAIN = (0.91, 0.97, 1.0)    # 201 is absolute: roll 0.15 -> 0.136, pitch 0.15 -> 0.145 (nose down)
VEL_GAIN = (1.03, 1.0, 0.8)    # 303: vx 0.3 -> 0.29-0.32 m/s, wz 0.5 -> 0.40 rad/s
VEL_TAU = 0.2              # velocity lag: 112 stops a 0.3 m/s walk in about 0.36 s (Gazebo 0.35-0.63)
TRACK_TAU = 0.15           # setpoint lag for streamed 3/x and the 11 stance height [INF]
Z_SPAN = {(7, 1): 1.6, (7, 0): 2.0, (0, 0): 2.0}   # 101: 95 % of the drop at 1.50-1.57 s; damper: 0.237 -> 0.10 in 1.5 s
RPY_MIN, RPY_MAX = (-0.47, -0.25, -0.65), (0.47, 0.3, 0.65)   # cyberdog2-ctrl-user-parameters.yaml:127-128

# Motion (bar) lengths in seconds: Gazebo for 101/111, pure_damper.cpp:105-108 for 102, motion.md
# section 6 "doc" lower bounds for the tricks; others 0.6 x min_exec_time.
DOC_LEN = {101: 1.2, 102: 4.0, 111: 2.0, 141: 8.6, 142: 8.6, 143: 1.3, 144: 3.9, 145: 3.9, 146: 3.3,
           174: 14.0, 152: 9.0, 151: 27.0, 123: 8.7, 130: 0.79, 131: 0.92, 132: 0.79, 136: 0.76}
# Velocity limits per locomotion gait id: (vx_min, vx_max, vy, wz) (motion.md section 5.1).
VEL = {27: (-0.6, 0.6, 0.2, 1.3), 3: (-1.2, 1.2, 0.4, 2.25), 10: (-1.2, 1.6, 0.55, 2.5),
       26: (-1.4, 1.6, 0.6, 2.5), 4: (-1.2, 1.2, 0.4, 0.8), 7: (-0.3, 0.3, 0.3, 1.5),
       2: (-0.22, 0.19, 0.4, 1.5), 1: (0, 0, 0, 0), 31: (0, 0, 0, 0)}
# Mode-16 jump displacement (dx, dy, dyaw) by gait; 13 and 16 run the +90 deg default (fsm_state_jump_3d.cpp:191-194).
JUMP = {0: (0, 0, math.pi / 2), 3: (0, 0, -math.pi / 2), 1: (0.6, 0, 0), 4: (0.3, 0, 0),
        2: (0, 0.2, 0), 5: (0, -0.2, 0), 13: (0, 0, math.pi / 2), 16: (0, 0, math.pi / 2)}
ZERO3 = [0.0, 0.0, 0.0]


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _wrap(it, d):
    """PoseCtrl trans_iter_: reset once it exceeds the duration (fsm_state_posectrl.cpp:127,131-132),
    so a value of exactly d is carried, not 0."""
    return it - d * math.ceil(it / d - 1) if d > 0 and it > d else it


def _rpy_rescale(z, rpy):
    """201 attitude command: RpyCmdRescale for CYBERDOG2 on the commanded height, then the user limits
    (fsm_state_balancestand.cpp:231-241,550-596). The roll and pitch cross terms are as written there."""
    if 0.235 < z < 0.26:
        d = (0.26 - z) / 0.025
    elif 0.15 < z < 0.18:
        d = (z - 0.15) / 0.03
    else:
        d = 1.0 if 0.15 < z < 0.26 else 0.0
    r, p, y = (v * d for v in rpy)

    def damp(v, i):
        lo, hi = RPY_MIN[i], RPY_MAX[i]
        if hi / 3 < v <= hi:
            return 1 - (v - hi / 3) / (hi * 2 / 3)
        if lo <= v < lo / 3:
            return 1 - (v - lo / 3) / (lo * 2 / 3)
        return 0.0 if v > hi or v < lo else 1.0
    k = damp(y, 2)
    r, p = r * k, p * k
    r, p = r * damp(p, 1), p * damp(r, 0)
    return [_clamp(v, RPY_MIN[i], RPY_MAX[i]) * QP_GAIN[i] for i, v in enumerate((r, p, y))]


class FakeController:
    def __init__(self, time_scale: float = 1.0):
        self.s = time_scale
        self.lk = threading.RLock()
        self.cbs = []
        self.t_start = time.monotonic()
        self.lengths = {}
        with open(os.path.join(DATA, "motion_id_map.json"), encoding="utf-8") as f:
            for e in json.load(f):
                key = tuple(e["map"])
                if key in self.lengths:
                    continue
                mid, t = e["motion_id"], e["min_exec_time"]
                self.lengths[key] = DOC_LEN.get(mid, 0.6 * t / 1000.0 if t > 0 else 0.5)
        self.stop_ev = threading.Event()
        self.reset()
        threading.Thread(target=self._loop, name="fake-controller", daemon=True).start()

    # ---- Controller interface (contracts.md 4.1) ---------------------------------
    def on_response(self, cb):
        self.cbs.append(cb)

    def ground_truth(self) -> dict:
        with self.lk:
            p = dict(self.pose)
        p["t"] = round(time.monotonic() - self.t_start, 3)
        return p

    def close(self):
        self.stop_ev.set()

    def reset(self):
        """Boot state: lying down after a completed 101 (mode 7, gait 1)."""
        with self.lk:
            now = time.monotonic()
            lie = self._entry({"mode": 7, "gait_id": 1, "duration": 0}, now)
            lie["t0"] = now - 1e3
            self.run = self.cmd = lie            # what the FSM runs / what was last commanded
            self.pending = None                  # entry waiting for the FSM transition out of 11
            self.queue = []
            self.ban = 0
            self.trans_until = 0.0
            self.last_life = None
            self.last_rx = now
            self.timed_out = False
            self.final_sent = False
            self.emit_now = False
            self.pose = dict(x=0.0, y=0.0, z=LIE_Z, roll=0.0, pitch=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)
            self.p0 = dict(self.pose)
            self.bv = [0.0, 0.0, 0.0]            # body-frame vx, vy, wz

    def publish(self, cmd: dict):
        with self.lk:
            now = time.monotonic()
            self.last_rx = now                   # every frame, duplicates included, refreshes the timer
            if cmd["mode"] == -1 or cmd["life_count"] == self.last_life:
                return                           # heartbeat duplicate (command_interface.cpp:983)
            self.last_life = cmd["life_count"]
            self.timed_out = False
            self._command(self._entry(cmd, now), now)
            self.emit_now = True

    # ---- command handling (command_interface.cpp:1023-1070) ------------------------
    def _entry(self, cmd, now):
        mode, gait = cmd["mode"], cmd["gait_id"]
        e = {"mode": mode, "gait": gait, "dur": cmd.get("duration", 0) / 1000.0 * self.s,
             "vel": list(cmd.get("vel_des") or ZERO3), "rpy": list(cmd.get("rpy_des") or ZERO3),
             "pos": list(cmd.get("pos_des") or ZERO3), "t0": now, "jumped": False, "iter0": 0.0}
        e["len"] = self.lengths.get((mode, gait), 0.5) * self.s
        return e

    def _command(self, e, now):
        if e["mode"] == 62 and self.run["mode"] == 62 and self.cmd is self.run:
            return                               # a new 62 is dropped while a list runs (:1032-1033)
        if e["dur"] == 0 and e["mode"] != 62:    # continuous command: clear the queue
            self.queue = []
            if e["mode"] == 12 and e["gait"] in (17, 18):   # 4-tick 12/0 step first (:1044-1051)
                pre = dict(e, gait=0, dur=0.008 * self.s)
                self.queue = [e]
                self._start(pre, now)
                return
            self._start(e, now)
            return
        self._pop_untimed()                      # a timed command waits behind a running timed one
        self.queue.append(e)
        if not self._timed_active():
            self._next(now)

    def _timed_active(self):
        c = self.cmd
        return (c is self.run or c is self.pending) and (c["dur"] > 0 or c["mode"] == 62)

    def _pop_untimed(self):
        while self.queue and self.queue[0]["dur"] == 0 and self.queue[0]["mode"] != 62:
            self.queue.pop(0)

    def _next(self, now):
        if self.queue:
            self._start(self.queue.pop(0), now)

    def _start(self, e, now):
        """Make e the commanded entry; enter its FSM state if the current state's table allows it."""
        self.cmd = e
        self.final_sent = False
        state = self._state()
        target = 21 if e["mode"] == 62 else e["mode"]
        if target != state and target not in TABLE.get(state, ()):
            self.ban = 5                         # BAN_TRANS while this command stands (control_fsm.cpp:139-146)
            self.pending = None
            return
        self.ban = 0
        r = self.run
        if (e["mode"], e["gait"]) == (r["mode"], r["gait"]) and e["dur"] == 0 == r["dur"] and e["mode"] != 62:
            e["t0"], e["jumped"] = r["t0"], r["jumped"]   # same continuous command: new setpoints only
            self.run, self.pending = e, None
            return
        if state == 11 and target in LOCO_EXIT_TO:
            if self.pending is None:
                self.trans_until = now + LOCO_EXIT * self.s
            self.pending = e                     # the queue timer waits for the transition (:440-441)
            e["t0"] = self.trans_until
            return
        self.pending = None
        if target != state or e["gait"] != r["gait"]:
            self.trans_until = now + TRANS * self.s
        e["t0"] = now
        if e["mode"] == 21 and e["gait"] != 5 and r["mode"] == 21:
            # trans_iter_ carries over between 21 commands unless it exceeds the new duration
            # (fsm_state_posectrl.cpp:127,131-132): a 212 after the 21/3 filler starts near 40 %.
            it = _wrap(r["iter0"] + now - r["t0"], r["dur"])
            e["iter0"] = it if it <= e["dur"] else 0.0
        self._enter(e)

    def _enter(self, e):
        self.run = e
        self.p0 = dict(self.pose)

    def _state(self):
        return 21 if self.run["mode"] == 62 else self.run["mode"]

    # ---- reported values ---------------------------------------------------------
    def _reported(self, now):
        r = self.run                             # the current FSM state, the old one during a transition
        mode, gait = r["mode"], r["gait"]
        if mode == 7:
            gait = 1 if gait == 1 else 0         # PureDamper
        elif mode == 0:
            gait = 0
        elif mode == 16 and gait in (13, 16):
            gait = 0                             # jump ids missing from the open FSM
        elif mode == 64 and not 1 <= gait <= 14:
            gait = 0
        elif mode in (81, 3) and not (mode == 3 and gait in (1, 3)):
            gait = 0                             # balancestand.cpp:221-222 reports 0 for absolute 3/x
        sw = 1 if now < self.trans_until or (gait != r["gait"] and mode != 62) else 0
        return mode, gait, sw

    def _bar(self, now):
        r = self.run
        el = now - r["t0"]
        mode, gait, L, d = r["mode"], r["gait"], max(r["len"], 1e-6), r["dur"]
        if el < 0:
            return 0
        if mode == 62:
            p = el / L
            return 100 if p >= 0.9 else int(95 * p / 0.9)
        if mode in (0, 15, 81, 9):
            return 100
        if (mode, gait) == (12, 0) and d == 0:   # fold (0), stand (80), 99/100 (recoverystand.cpp:633-635,820)
            return 0 if el < 0.6 * L else 80 if el < L else 100
        if (mode, gait) == (7, 0) and d == 0:    # pd_iter_ 3000 ticks, 100 after 2000 (pure_damper.cpp:105-108)
            return 100 if el >= L else int(100 * el / (1.5 * L))
        if mode in (7, 12, 80):
            return 100 if d == 0 and el >= L else min(99, int(100 * el / (d or L)))
        if (mode, gait) == (3, 3) and d == 0:    # 202 untimed: 1000-tick periods, the bar restarts each one
            P = 2.0 * self.s                     # (balancestand.cpp:338-341,371,386-396); SIM DEVIATION: kept
            return int(95 * (el % P) / P)        # at 95 or less, so the manager's 500 ms wait always gives 3028
        if mode in (11, 3):
            return 100 if d == 0 else min(100, int(100 * el / d))
        if mode == 21:
            return 0 if d == 0 else min(100, int(100 * _wrap(r["iter0"] + el, d) / d))
        return 100 if el >= L else 0             # 16 / 64 / 22: 100 after the move

    def _response(self, now, bar=None):
        mode, gait, sw = self._reported(now)
        if self.ban:
            sw = self.ban
        return {"mode": mode, "gait_id": gait, "contact": 15,
                "order_process_bar": self._bar(now) if bar is None else bar,
                "switch_status": sw, "ori_error": 0, "footpos_error": 0, "motor_error": [0] * 12}

    # ---- loop ----------------------------------------------------------------------
    def _loop(self):
        last_emit, last_t, last_sw = 0.0, time.monotonic(), None
        while not self.stop_ev.wait(0.01):
            out = []
            with self.lk:
                now = time.monotonic()
                dt, last_t = now - last_t, now
                if not self.timed_out and now - self.last_rx > CMD_TIMEOUT:
                    self.timed_out = True        # no frame for 0.5 s: PureDamper, unless Off (:162-170)
                    if self.cmd["mode"] != 0 and self._state() != 0:
                        self.queue = []
                        self._start(self._entry({"mode": 7, "gait_id": 0, "duration": 0}, now), now)
                        self.emit_now = True
                if self.pending is not None and now >= self.trans_until:
                    self._enter(self.pending)
                    self.pending = None
                    self.emit_now = True
                r = self.run
                el = now - r["t0"]
                end = r["len"] if r["mode"] == 62 else r["dur"]
                self._integrate(dt, now)         # before the expiry, so the ending entry reaches its target
                if r is self.cmd and end > 0 and el >= end and (r["mode"] == 62 or r["dur"] > 0):
                    if not self.final_sent:      # the robot publishes every tick in 95..100
                        out.append(self._response(now, bar=100))
                        self.final_sent = True
                    self._expire(r, r["t0"] + end)   # the queue runs on ticks: no drift from the loop period
                    # The next entry's first response waits one tick (2 ms on the robot), so a manager
                    # waiting for bar > 95 sees the 100 before the filler overwrites it.
                    self.emit_now = True
                    resp = None
                else:
                    resp = self._response(now)
                if resp is not None and (self.emit_now or resp["switch_status"] != last_sw
                                         or now - last_emit >= 0.02
                                         or (resp["order_process_bar"] >= 95 and not self.final_sent)):
                    out.append(resp)
                    last_emit, last_sw, self.emit_now = now, resp["switch_status"], False
                    if resp["order_process_bar"] >= 95:
                        self.final_sent = True
            for o in out:
                for cb in self.cbs:
                    cb(dict(o))

    def _expire(self, r, now):
        """A timed entry ended: next queued entry, or an idle filler that repeats every 0.4 s
        (command_interface.cpp:206-250; Gazebo shows the filler's bar cycling 0 -> 100)."""
        if self.queue:
            self._next(now)
            return
        m, g = r["mode"], r["gait"]
        if m == 3:
            fill = (3, 1)
        elif m == 11:
            fill = (11, 1) if g <= 30 else (11, 31)
        else:
            fill = (21, 3)                       # also after a mode-62 list (motion_trigger = 0)
        e = self._entry({"mode": fill[0], "gait_id": fill[1], "duration": 0}, now)
        e["dur"] = FILLER * self.s
        self._start(e, now)

    # ---- kinematic pose ------------------------------------------------------------
    def _integrate(self, dt, now):
        p, r = self.pose, self.run
        mode, gait = r["mode"], r["gait"]
        el = max(0.0, now - r["t0"])
        span = r["dur"] or r["len"] or 0.3 * self.s
        k = min(1.0, el / span) if span > 0 else 1.0

        def lag(tau):
            return 1.0 - math.exp(-dt / max(tau * self.s, 1e-3))

        def toward(key, target, kk=k):
            p[key] = self.p0[key] + (target - self.p0[key]) * kk

        def track(key, target, tau=TRACK_TAU):
            p[key] += (target - p[key]) * lag(tau)

        tv = (0.0, 0.0, 0.0)                     # body-frame velocity target
        if mode in (7, 0):
            kk = min(1.0, el / (Z_SPAN.get((mode, gait if mode == 7 else 0), 2.0) * self.s))
            for key, t in (("z", LIE_Z), ("roll", 0.0), ("pitch", 0.0)):
                toward(key, t, kk)
        elif (mode, gait) == (12, 0) and r["dur"] == 0:   # 111: folds first, z at 95 % at 0.94 L (Gazebo 1.88 s)
            kk = _clamp((el / max(r["len"], 1e-6) - 0.6) / 0.37, 0.0, 1.0)
            for key, t in (("z", STAND_Z), ("roll", 0.0), ("pitch", 0.0)):
                toward(key, t, kk)
        elif mode in (12, 62, 64, 80, 81, 22, 9, 15):
            for key, t in (("z", STAND_Z), ("roll", 0.0), ("pitch", 0.0)):
                toward(key, t)
        elif mode == 11:
            for key, t in (("z", WALK_Z), ("roll", 0.0), ("pitch", 0.0)):
                track(key, t)
            if self.pending is None and not self.ban:
                lo, hi, vy, wz = VEL.get(gait, (-2.0, 2.0, 1.0, 2.5))
                tv = (_clamp(r["vel"][0], lo, hi) * VEL_GAIN[0], _clamp(r["vel"][1], -vy, vy) * VEL_GAIN[1],
                      _clamp(r["vel"][2], -wz, wz) * VEL_GAIN[2])
        elif mode == 21:
            if gait == 5:                        # absolute height, level body (fsm_state_posectrl.cpp:153-181)
                toward("z", _clamp(r["pos"][2], 0.10, 0.30) - POSE5_SAG)
                toward("roll", 0.0)
                toward("pitch", 0.0)
            elif r["dur"] > 0:                   # incremental, spread over the duration; 0 is 1800000 ticks
                toward("z", _clamp(self.p0["z"] + r["pos"][2], 0.10, 0.30))
                toward("roll", self.p0["roll"] + r["rpy"][0])
                toward("pitch", self.p0["pitch"] + PITCH21 * r["rpy"][1])
                toward("yaw", self.p0["yaw"] + r["rpy"][2])
        elif mode == 3:
            if gait == 3 and r["dur"] == 0:      # 202 untimed: never popped, rpy_des added again every 2 s
                P = 2.0 * self.s
                n = el // P + min(1.0, (el % P) / P)
                toward("roll", self.p0["roll"] + n * r["rpy"][0], 1.0)
                toward("pitch", self.p0["pitch"] + n * r["rpy"][1], 1.0)
            elif gait == 3:                      # 202 relative
                toward("roll", self.p0["roll"] + r["rpy"][0])
                toward("pitch", self.p0["pitch"] + r["rpy"][1])
            elif gait != 1:                      # 201 absolute; the height is clamped to 0.15-0.35 (:261)
                track("z", _clamp(r["pos"][2], 0.15, 0.35))
                roll, pitch, _ = _rpy_rescale(r["pos"][2], r["rpy"])
                track("roll", roll)
                track("pitch", pitch)
        elif mode == 16:
            toward("z", STAND_Z)
            if not r["jumped"] and el >= r["len"]:
                r["jumped"] = True
                dx, dy, dyaw = JUMP.get(gait, (0, 0, 0))
                c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
                p["x"] += dx * c - dy * s
                p["y"] += dx * s + dy * c
                p["yaw"] = math.atan2(math.sin(p["yaw"] + dyaw), math.cos(p["yaw"] + dyaw))
        a = lag(VEL_TAU)
        for i in range(3):
            self.bv[i] += (tv[i] - self.bv[i]) * a
        c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
        p["vx"], p["vy"], p["wz"] = self.bv[0] * c - self.bv[1] * s, self.bv[0] * s + self.bv[1] * c, self.bv[2]
        p["x"] += p["vx"] * dt
        p["y"] += p["vy"] * dt
        yaw = p["yaw"] + p["wz"] * dt
        p["yaw"] = math.atan2(math.sin(yaw), math.cos(yaw))
