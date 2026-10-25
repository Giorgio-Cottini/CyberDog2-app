"""Maps and navigation: 6002, 6003, 6004, 6009, 6010 (nav_map.md sections 2-4; grpc_fidelity.md 7.7).

The reverse channel is down, as on the user's dog: the final result of a 6004/6010 stream is never
written, and every live stream is cut at each 5 s heartbeat-failure event (scaled by time_scale).
Tasks keep running after the cut; 10001 shows task_status / task_sub_status.
"""
from __future__ import annotations

import collections
import math
import queue
import threading
import time

from emulator.common import Crash, MISSING, f32, get, is_double, jdump

ACCEPT_STATUS = {1: 1, 5: 5, 7: 7, 9: 9, 11: 11, 13: 13, 0: 0}      # MGRH:259-295 (indoor)
OUTDOOR_STATUS = {1: 21, 5: 15, 7: 17}
INDOOR_ONLY = {0, 9, 11, 13}                                         # Task.toml: no outdoor entry
STOP_ALLOWED = {1: (1, 21), 5: (5, 15), 7: (7, 17), 9: (9,), 11: (11,), 13: (13, 3)}   # MGR:190-250
SUCCESS_NEXT = {7: 8, 1: 8, 17: 18, 21: 18}                          # MGR:343-373, else 101
ABORT_NEXT = {1: 8, 21: 18, 7: 6, 17: 16}                            # MGR:403-436, else 101
CUT_PERIOD = 5.0                                                     # heartbeat-failure cycle (app.cpp:534-548)
AB_SPEED = 0.3                                                       # m/s for the fake goal approach [INF]


def _casa():
    """Saved map "casa": 80 x 60 cells of 5 cm, walls on the border and one obstacle."""
    w, h = 80, 60
    data = []
    for row in range(h):
        for col in range(w):
            wall = row in (0, h - 1) or col in (0, w - 1) or (40 <= col < 50 and 20 <= row < 26)
            data.append(100 if wall else 0)
    grid = {"resolution": f32(0.05), "width": w, "height": h, "px": -2.0, "py": -1.5, "pz": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "data": data}
    labels = collections.OrderedDict([("cucina", (f32(1.3), f32(0.7))), ("divano", (f32(-1.1), f32(-0.6)))])
    return {"grid": grid, "json": {"is_outdoor": False, "labels": labels}}


class Stream:
    def __init__(self):
        self.q = queue.Queue()


class Task:
    def __init__(self, nav, typ, status, outdoor, goal, obj):
        self.nav, self.typ, self.outdoor, self.goal, self.obj = nav, typ, outdoor, goal, obj
        self.lk = threading.Lock()
        self.stream = None
        self.buffering = True                # pre-accept feedback buffer (TASK:250-266)
        self.pre = []
        self.suppress = False
        self.latest = None
        self.last_code = None
        self.done = False
        self.goal_sent = False
        self.cancel = threading.Event()
        self.pose_from = self.pose_t = None
        nav.status = status

    def feedback(self, code, msg=""):
        self.nav.sub = code                  # global_feedback_ is set before the dedup (MGR:327)
        if code == self.last_code:           # MGR:331-336
            return
        self.last_code = code
        with self.lk:
            self.latest = (code, msg)
            if self.buffering:
                self.pre.append((code, msg))
            elif self.stream is not None and not self.suppress:
                self.stream.q.put(("feedback", code, msg))

    def where(self):
        """The fake approach: a straight line at AB_SPEED from where the goal was sent; None before that."""
        if self.pose_t is None:
            return None
        (x0, y0), (x1, y1) = self.pose_from, self.goal
        dist = math.hypot(x1 - x0, y1 - y0)
        k = 1.0 if dist == 0 else min(1.0, (time.monotonic() - self.pose_t) / self.nav.emu.sc(dist / AB_SPEED))
        return {"x": x0 + (x1 - x0) * k, "y": y0 + (y1 - y0) * k}

    def finish(self, ok):
        with self.nav.lk:
            if self.done:
                return
            self.done = True
            if self.pose_t is not None:          # the dog stays where the task left it
                self.nav.pose = {"x": self.goal[0], "y": self.goal[1]} if ok else self.where()
            st = self.nav.status
            self.nav.status = (SUCCESS_NEXT if ok else ABORT_NEXT).get(st, 101)
            if ok:
                self.nav.sub = 0                 # MGR:358
        with self.lk:
            if self.stream is not None:
                self.stream.q.put(("result", 0 if ok else 3, ""))

    def wait(self, t):
        """Sleep t scaled seconds; False when the task was stopped meanwhile."""
        return not self.cancel.wait(self.nav.emu.sc(t)) and not self.nav.emu.stopping.is_set()


class Nav:
    def __init__(self, emu):
        self.emu = emu
        self.lk = threading.RLock()
        self.task_init_lock = threading.Lock()  # serialises 6004/6010/5010/5011 accept phases
        self.epoch = 0
        self.reset()
        threading.Thread(target=self._cutter, name="nav-cut", daemon=True).start()

    def reset(self):
        with self.lk:
            for t in getattr(self, "tasks", {}).values():
                t.cancel.set()
            self.status, self.sub = 101, 0
            self.tasks = {}
            self.maps = collections.OrderedDict([("casa", _casa())])
            self.pose = None
            self.epoch += 1

    def _cutter(self):                          # RemoveRequest on every live stream (app.cpp:1638-1671)
        while not self.emu.stopping.wait(self.emu.sc(CUT_PERIOD)):
            self.epoch += 1

    # ---- 6003 -------------------------------------------------------------------------
    def h6003(self, doc, nc, raw):
        dep = self.emu.dep("get_label")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        req_name = get(doc, "mapName", str, "")
        if not dep.ready(10):
            yield self.emu.err(321, nc)
            yield self.emu.msg(nc, "")           # the timeout branch writes a second, empty message (app.cpp:1754-1758)
            return
        with self.lk:
            name = next(iter(self.maps), "") or req_name   # first *.pgm wins (LBL:93-96)
            m = self.maps.get(name)
            if m is None or m["json"] is None:
                out = {"mapName": "", "success": 1, "is_outdoor": False, "locationLabelInfo": [],
                       "map": {"resolution": 0.0, "width": 0, "height": 0, "px": 0.0, "py": 0.0, "pz": 0.0,
                               "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "data": []}}
            else:
                labels = [{"labelName": k, "physicX": x, "physicY": y} for k, (x, y) in m["json"]["labels"].items()]
                out = {"mapName": name, "success": 0, "is_outdoor": m["json"]["is_outdoor"],
                       "locationLabelInfo": labels, "map": m["grid"]}
        yield self.emu.msg(nc, jdump(out))

    # ---- 6002 -------------------------------------------------------------------------
    def h6002(self, doc, nc, raw):
        if not isinstance(doc, dict):
            raise Crash("6002: HasMember on a non-object (app.cpp:1774)")
        name = get(doc, "mapName", str, "")
        only_delete = get(doc, "only_delete", bool, False)
        labels = []
        info = doc.get("locationLabelInfo")
        if isinstance(info, list):
            for e in info:
                if not isinstance(e, dict):
                    raise Crash("6002: labels[i].HasMember on a non-object (app.cpp:1781)")
                if isinstance(e.get("labelName"), str) and is_double(e.get("physicX")) and is_double(e.get("physicY")):
                    labels.append((e["labelName"], f32(e["physicX"]), f32(e["physicY"])))
                else:
                    self.emu.client_bug("6002 label dropped: physicX/physicY must be JSON doubles", label=e)
        dep = self.emu.dep("set_label")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        if not dep.ready(10):
            yield self.emu.err(321, nc)
            return
        r = self._set_labels(name, only_delete, labels)
        if r:
            self.emu.refuse(nc, "set_label failed", mapName=name, only_delete=only_delete)
        yield self.emu.msg(nc, jdump({"success": r}))

    def _set_labels(self, name, only_delete, labels):   # LBL:159-244
        with self.lk:
            m = self.maps.get(name)
            if m is None:
                return 1
            if only_delete and not labels:
                self.emu.dangerous_event("6002 delete-all: every map removed", mapName=name)
                self.maps.clear()               # remove_all("/home/mi/mapping/")
                return 0
            if m["json"] is None:
                return 1
            stored = m["json"]["labels"]
            if only_delete:
                for n, _, _ in labels:          # not atomic: earlier names stay deleted
                    if n not in stored:
                        return 1
                    del stored[n]
                return 0
            names = [n for n, _, _ in labels]
            if len(set(names)) != len(names):
                return 1
            m["json"]["labels"] = collections.OrderedDict((n, (x, y)) for n, x, y in labels)
            return 0

    # ---- 6004 / 6010 --------------------------------------------------------------------
    def hnav(self, doc, nc, raw):
        create = nc == 6004
        dep = self.emu.dep("start_algo_task")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        if not isinstance(doc, dict):
            raise Crash("%d: HasMember on a non-object (app.cpp:1434)" % nc)
        typ = get(doc, "type", "uint")
        if typ is MISSING:
            self.emu.client_bug("%d type missing or not a uint: uninitialised on the dog (read as 0 here)" % nc)
            typ = 0
        typ &= 0xFF                              # nav_type is uint8
        stream = Stream()
        epoch0 = self.epoch
        with self.task_init_lock:
            if create:
                if get(doc, "keep_distance", "double") is MISSING:
                    self.emu.client_bug("6004 keep_distance missing: uninitialised on the dog")
                if not dep.ready(3):             # no goal handle, then no identifying feedback (TASK:383-402)
                    self.emu.sleep(3)
                    self.emu.refuse(nc, "accepted 2: action server did not answer", type=typ)
                    yield self.emu.msg(nc, jdump({"accepted": 2}))
                    return
                task = self._start(typ, doc)
                if task is None:
                    yield self.emu.msg(nc, jdump({"accepted": 2}))
                    return
                with task.lk:
                    task.stream = stream
                yield self.emu.msg(nc, jdump({"accepted": 1}))
                with task.lk:                   # replay the feedback buffered before acceptance
                    pre, task.pre, task.buffering = task.pre, [], False
                for code, m in pre:
                    yield self.emu.msg(nc, jdump({"feedback_code": code, "feedback_msg": m}))
            else:
                with self.lk:
                    task = self.tasks.get(typ)
                if task is None or task.done:
                    if task is None and typ == 11 and self.status == 11:
                        yield self.emu.msg(nc, jdump({"accepted": 1}))   # UWB started elsewhere (app.cpp:1546-1595)
                        yield self.emu.msg(nc, jdump({"result": 3}))     # loop skipped: connect_mark_ is false
                        return
                    self.emu.refuse(nc, "accepted 3: no live task of this type", type=typ)
                    yield self.emu.msg(nc, jdump({"accepted": 3}))
                    return
                with task.lk:                   # AccessTask steals the feedback callback (TASK:441-460)
                    task.stream = stream
                    latest = task.latest
                    task.suppress = latest is None   # never cleared if there is nothing to replay
                yield self.emu.msg(nc, jdump({"accepted": 1}))
                if latest is not None:
                    yield self.emu.msg(nc, jdump({"feedback_code": latest[0], "feedback_msg": latest[1]}))
        deadline = time.monotonic() + self.emu.sc(7200)
        while True:
            try:
                ev = stream.q.get(timeout=0.02)
            except queue.Empty:
                if self.epoch != epoch0 or self.emu.stopping.is_set():
                    return                       # fake result from RemoveRequest; nothing is written
                if time.monotonic() > deadline:
                    yield self.emu.err(321, nc)
                    return
                continue
            if ev[0] == "feedback":
                yield self.emu.msg(nc, jdump({"feedback_code": ev[1], "feedback_msg": ev[2]}))
            else:
                return                           # real result: skipped because connect_mark_ is false (app.cpp:1619-1622)

    def _start(self, typ, doc):
        outdoor = get(doc, "outdoor", bool, False)
        obj = get(doc, "object_tracking", bool, False)
        goal = None
        if "goalX" in doc and "goalY" in doc:    # both keys, raw HasMember (app.cpp:1434-1438)
            goal = (get(doc, "goalX", "double", 0.0), get(doc, "goalY", "double", 0.0))
        with self.lk:
            st = self.status
            if typ not in ACCEPT_STATUS or (outdoor and typ in INDOOR_ONLY):
                why = "type %d / outdoor %s matches no Task.toml entry" % (typ, outdoor)
            elif typ == 1 and st != (18 if outdoor else 8):
                why = "type 1 needs task_status %d, is %d" % (18 if outdoor else 8, st)
            elif typ != 1 and st not in (101, 6, 16):
                why = "type %d needs task_status 101/6/16, is %d" % (typ, st)
            else:
                why = None
            if why:
                self.emu.refuse(6004, "accepted 2: " + why, type=typ)
                return None
            if typ == 0:
                self.emu.dangerous_event("6004 type 0 (ResetNav) accepted: algorithm_manager wedged in status 0")
            status = OUTDOOR_STATUS.get(typ, typ) if outdoor else ACCEPT_STATUS[typ]
            if typ == 13:
                status = 3 if obj else 13
            old = self.tasks.get(typ)
            if old is not None:
                old.cancel.set()
            task = Task(self, typ, status, outdoor, goal, obj)
            self.tasks[typ] = task
            first = {7: 3100, 1: 309, 5: 6, 9: 500, 11: 10, 13: 500}.get(typ)
            if first is not None:
                task.feedback(first)
        threading.Thread(target=self._run, args=(task,), name="nav-task-%d" % typ, daemon=True).start()
        return task

    def _run(self, t):
        """Feedback sequences of the executors (nav_map.md 4.4); times in seconds before scaling."""
        if t.typ == 7:
            if not self._map_ok():
                if t.wait(0.3):
                    t.feedback(3102)
                    t.finish(False)              # 7 -> 6
                return
            for dt, code in ((0.3, 3101), (0.3, 1000), (0.9, 1001), (0.5, 3103), (0.5, 3104), (0.5, 3107),
                             (2.0, 3108)):
                if not t.wait(dt):
                    return
                t.feedback(code)
            t.finish(True)                       # 7 -> 8
        elif t.typ == 1:
            for dt, code in ((0.3, 310), (0.3, 1000), (0.6, 1001)):
                if not t.wait(dt):
                    return
                t.feedback(code)
            if t.goal is None:
                if t.wait(0.3):
                    t.feedback(305)              # goal empty
                    t.finish(False)
                return
            if not t.wait(0.3):
                return
            p = self.pose_now()
            t.pose_from = (p["x"], p["y"])
            t.pose_t = time.monotonic()
            t.goal_sent = True
            t.feedback(300)
            if not t.wait(0.3):
                return
            t.feedback(307)
            dist = math.hypot(t.goal[0] - t.pose_from[0], t.goal[1] - t.pose_from[1])
            if not t.wait(max(dist / AB_SPEED - 0.3, 0.0)):
                return
            t.feedback(308)
            t.finish(True)
        elif t.typ == 9:
            if t.wait(3.0):
                self.emu.device.wireless = True  # seatadjust waits for power_wp_charging
                t.finish(True)
        elif t.typ == 13:
            if t.wait(0.5):
                t.feedback(501)
        # 5 (mapping), 11 (UWB), 13 and 0 run until 6009

    def _map_ok(self):
        with self.lk:
            m = next(iter(self.maps.values()), None)
            return m is not None and m["json"] is not None

    # ---- 6009 ----------------------------------------------------------------------------
    def h6009(self, doc, nc, raw):
        dep = self.emu.dep("stop_algo_task")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        typ = get(doc, "type", "uint", 0) & 0xFF
        map_name = get(doc, "map_name", str, "")
        if not dep.ready(60):
            yield self.emu.msg(nc, jdump({"result": 10}))
            return
        r = self.stop(typ, map_name)
        if r:
            self.emu.refuse(nc, "stop failed", type=typ, task_status=self.status)
        yield self.emu.msg(nc, jdump({"result": r}))

    def stop(self, typ, map_name=""):
        with self.lk:
            st = self.status
            if typ == 0:                         # ResetNav (MGR:197-209)
                if st not in (1, 21, 7, 17, 8, 18):
                    return 1
                self._end_all()
                self.status, self.sub = 101, 0
                return 0
            if typ in STOP_ALLOWED and st in STOP_ALLOWED[typ]:
                t = self.tasks.get(typ)
                if typ == 1:
                    self.status = 18 if st == 21 else 8
                    if t is not None and not t.goal_sent:
                        return 0                 # Stop before feedback 300 does nothing; the goal goes out anyway
                    self._end(t)
                    return 0
                if typ == 5 and map_name:        # save: fresh <map_name>.json, labels gone (LBL:297-326)
                    self.maps[map_name] = {"grid": _casa()["grid"], "json": {"is_outdoor": False,
                                                                         "labels": collections.OrderedDict()}}
                self._end(t)
                self.status, self.sub = 101, 0
                return 0
            if typ == st:                        # ReParseStatus passes the raw value (MGRH:303-326)
                self._end_all()
                self.status, self.sub = 101, 0
                return 0
            return 0                             # mismatch: fake success, nothing done (MGR:226-233)

    def _end(self, t):
        if t is not None:
            t.cancel.set()
            t.done = True
            if t.pose_t is not None:
                self.pose = t.where()
            with t.lk:
                if t.stream is not None:
                    t.stream.q.put(("result", 5, ""))

    def _end_all(self):
        for t in self.tasks.values():
            if not t.done:
                self._end(t)

    # ---- inspection ------------------------------------------------------------------------
    def pose_now(self):
        """Map-frame pose: where the last AB task left the dog, else the controller's ground truth."""
        with self.lk:
            if self.pose is not None:
                return dict(self.pose)
        gt = self.emu.ctl.ground_truth()
        return {"x": gt["x"], "y": gt["y"]}

    def state(self):
        with self.lk:
            live = [t for t in self.tasks.values() if not t.done]
            task = None
            pose = None
            if live:
                t = live[-1]
                task = {"type": t.typ, "outdoor": t.outdoor, "goal": t.goal, "latest_feedback": t.latest}
                if t.typ == 1:
                    pose = t.where()
            if pose is None:
                pose = self.pose_now()
            return {"task": task, "status": self.status, "sub_status": self.sub, "pose": pose,
                    "maps": {n: sorted(m["json"]["labels"]) if m["json"] else None for n, m in self.maps.items()},
                    "cut_period_s": self.emu.sc(CUT_PERIOD)}

