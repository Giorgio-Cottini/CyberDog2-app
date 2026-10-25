"""Maps and navigation: 6002 / 6003 / 6004 / 6009 / 6010 and 10001 polling (nav_map.md).

6004/6010 are long streams: {"accepted":1|2|3}, then {"feedback_code","feedback_msg"}.
The final {"result"} is never written to us (it is gated on the reverse channel)
and the dog unbinds the stream after ~5-8 s, so progress comes from the
feedback seen, 6010 re-attach and 10001 task_status / task_sub_status.
"""
from __future__ import annotations

import json
import math
import threading
from typing import Callable, Iterable, Iterator, List, NamedTuple, Optional, Tuple

from .transport import NAV_TYPES, DogError, ForbiddenCommand, as_float, error_code

STREAM_DEADLINE = 7300.0      # 6004/6010: none or >= 7300 s (nav_map.md §7)
MAP_TIMEOUT = 15.0            # 6002 / 6003
STOP_TIMEOUT = 65.0           # 6009 waits up to 60 s on the dog
STOP_TYPES = (0, 1, 5, 9, 11, 13)

STATUS_NAMES = {
    100: "uninitialised", 101: "idle", 102: "launching", 103: "stopping",
    5: "laser mapping", 15: "vision mapping",
    7: "laser relocalization", 17: "vision relocalization",
    6: "laser localization failed", 16: "vision localization failed",
    8: "laser localized", 18: "vision localized",
    1: "laser AB navigation", 21: "vision AB navigation",
    9: "auto docking", 11: "UWB tracking", 13: "human tracking", 3: "object following",
    0: "wedged (6004 type 0); only LowPower or a reboot recovers",
}
# running task_status -> 6004 type, and -> 6009 type that stops it (nav_map.md §5 d)
RUNNING_TYPE = {1: 1, 21: 1, 5: 5, 15: 5, 7: 7, 17: 7, 9: 9, 11: 11, 13: 13, 3: 13}
STOP_TYPE = {1: 1, 21: 1, 5: 5, 15: 5, 7: 0, 17: 0, 8: 0, 18: 0, 9: 9, 11: 11, 13: 13, 3: 13}
# AB before feedback 300: a type 1 stop answers success, does nothing, and the goal still goes
# out; type 0 (ResetNav) sets is_exit_ in every phase (nav_map.md §5 b-d, executor_ab_navigation.cpp:188-219)
AB_GOAL_SENT = (300, 307)

FEEDBACK = {
    1000: "preparing dependencies", 1001: "dependencies ready", 1002: "dependencies failed",
    6: "mapping started", 7: "mapping failed",
    3100: "checking map", 3101: "map ok", 3102: "map unavailable", 3103: "starting relocalization",
    3104: "relocalization started", 3105: "relocalization start failed", 3106: "relocalization timeout",
    3107: "not localized yet: drive slowly", 3108: "localized", 3109: "relocalization failed",
    3110: "vision relocalization", 3111: "vision relocalization",
    300: "goal sent", 302: "nav2 connect failed", 303: "send goal failed", 304: "navigation aborted",
    305: "goal empty or illegal", 307: "navigating", 308: "arrived", 309: "checking map",
    310: "map ok", 311: "map missing",
    500: "started", 501: "waiting for target selection", 502: "target selected", 503: "following",
    504: "target lost, searching", 505: "target lost", 506: "tracking failure", 507: "selection failed",
    508: "tracking failure", 509: "tracking failure",
    10: "UWB normal", 11: "UWB target lost", 12: "UWB TF exception", 13: "UWB planner exception",
    14: "UWB controller exception", 15: "UWB empty target", 16: "UWB stair jump",
    17: "UWB autonomous behaviour", 18: "UWB behaviour abnormal",
}
_FEEDBACK_TYPES = {6: (5,), 7: (5,), 10: (11,), 11: (11,), 12: (11,), 13: (11,), 14: (11,),
                   15: (11,), 16: (11,), 17: (11,), 18: (11,)}


def feedback_name(code: int, task_type: Optional[int]) -> str:
    """Meaning of a feedback code in the context of a task type; "" when not meaningful.
    500 lingers after a dock (nav_map.md §4.4): ignored outside types 9 and 13."""
    if code == 0:
        return ""
    if 500 <= code <= 509 and task_type not in (9, 13):
        return ""
    allowed = _FEEDBACK_TYPES.get(code)
    if allowed is not None and task_type not in allowed:
        return ""
    return FEEDBACK.get(code, "")


class MapData(NamedTuple):
    name: str
    success: int                 # 0 ok, 1 missing, 2-5 / 255 see nav_map.md §2 6003
    is_outdoor: bool
    resolution: float
    width: int
    height: int
    origin: Tuple[float, float]  # (px, py) of the grid's origin in the map frame
    yaw: float                   # 2 * atan2(qz, qw)
    data: List[int]              # row-major occupancy, one int per cell
    labels: List[Tuple[str, float, float]]


class NavState(NamedTuple):
    status: int                  # raw task_status
    sub_status: int              # raw task_sub_status (last feedback code)
    name: str
    task_type: Optional[int]     # 6004 type of the running task, else None
    stop_type: Optional[int]     # 6009 type that stops it, else None
    localized: bool
    wedged: bool
    feedback: str                # meaning of sub_status for this task, "" if none


def decode_state(task_status: int, task_sub_status: int) -> NavState:
    running = RUNNING_TYPE.get(task_status)
    ctx = running if running is not None else (1 if task_status in (8, 18) else None)
    stop = STOP_TYPE.get(task_status)
    if running == 1 and task_sub_status not in AB_GOAL_SENT:
        stop = 0
    return NavState(task_status, task_sub_status, STATUS_NAMES.get(task_status, "unknown"),
                    running, stop, task_status in (8, 18),
                    task_status == 0, feedback_name(task_sub_status, ctx))


def label_dict(label) -> dict:
    """(name, x, y) or {"labelName","physicX","physicY"} -> a 6002 label with float coordinates."""
    if isinstance(label, dict):
        name, x, y = label.get("labelName"), label.get("physicX"), label.get("physicY")
    else:
        name, x, y = label
    if not isinstance(name, str) or not name:
        raise ValueError("label needs a non-empty name: %r" % (label,))
    return {"labelName": name, "physicX": as_float(x, "physicX"), "physicY": as_float(y, "physicY")}


class NavTask:
    """A 6004 / 6010 stream (or a 6009 save) consumed on a daemon thread."""

    def __init__(self, task_type: int, kind: str, source: Callable[[], Iterator[str]]):
        self.task_type = task_type
        self.kind = kind              # "start", "attach" or "save"
        self.accepted = None          # 1 accepted, 2 rejected, 3 not found (6010)
        self.feedback = []            # [(code, msg)] in arrival order
        self.result = None            # {"result"} when it arrives (6009 save; 6004 only via reverse)
        self.error = None             # error_code int, or an exception text
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(source,), daemon=True)
        self._thread.start()

    @property
    def last_feedback(self) -> Optional[int]:
        return self.feedback[-1][0] if self.feedback else None

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self.done.wait(timeout)

    def _run(self, source) -> None:
        try:
            for data in source():
                try:
                    item = json.loads(data)
                except ValueError:
                    continue
                if not isinstance(item, dict):
                    continue
                code = error_code(item)
                if code is not None:
                    self.error = code
                    return
                if "accepted" in item:
                    self.accepted = item["accepted"]
                if "feedback_code" in item:
                    fc = item["feedback_code"]
                    if fc == 500 and self.task_type not in (9, 13):
                        continue
                    self.feedback.append((fc, item.get("feedback_msg", "")))
                if "result" in item:
                    self.result = item["result"]
        except Exception as e:
            self.error = str(e)
        finally:
            self.done.set()


class Nav:
    def __init__(self, transport):
        self.t = transport

    # ---- maps -------------------------------------------------------------------
    def get_map(self, name: str = "") -> MapData:
        r = self.t.call_one(6003, {"mapName": name}, timeout=MAP_TIMEOUT)
        m = r.get("map") or {}
        qz, qw = float(m.get("qz", 0.0)), float(m.get("qw", 1.0))
        labels = [(lab.get("labelName", ""), float(lab.get("physicX", 0.0)), float(lab.get("physicY", 0.0)))
                  for lab in r.get("locationLabelInfo") or [] if isinstance(lab, dict)]
        return MapData(r.get("mapName", ""), int(r.get("success", 1)), bool(r.get("is_outdoor", False)),
                       float(m.get("resolution", 0.0)), int(m.get("width", 0)), int(m.get("height", 0)),
                       (float(m.get("px", 0.0)), float(m.get("py", 0.0))),
                       2.0 * math.atan2(qz, qw) if (qz or qw) else 0.0,
                       list(m.get("data") or []), labels)

    def set_labels(self, map_name: str, labels: Iterable, *, only_delete: bool = False,
                   confirm_delete_all: bool = False) -> bool:
        """6002 with the COMPLETE label list (the dog rewrites the file with exactly these).
        only_delete with no labels wipes /home/mi/mapping/ entirely: refused without
        confirm_delete_all."""
        if not isinstance(map_name, str):
            raise ValueError("map_name must be a str")
        labs = [label_dict(lab) for lab in labels]
        if only_delete and not labs and not confirm_delete_all:
            raise ForbiddenCommand("6002 only_delete with no labels deletes every map; "
                                   "pass confirm_delete_all=True")
        params = {"mapName": map_name, "is_outdoor": False, "only_delete": bool(only_delete),
                  "locationLabelInfo": labs}
        items = self.t.call_json(6002, params, timeout=MAP_TIMEOUT, force=confirm_delete_all)
        if not items:
            raise DogError("6002: no reply", None, 6002)
        code = error_code(items[0])
        if code is not None:
            raise DogError("6002: error_code %d" % code, code, 6002)
        return items[0].get("success") == 0

    # ---- tasks ------------------------------------------------------------------
    def _start(self, params: dict) -> NavTask:
        if params.get("type") not in NAV_TYPES:
            raise ForbiddenCommand("6004 type must be one of %s" % (NAV_TYPES,))
        p = {"type": params["type"], "outdoor": False, "keep_distance": 0.0}
        p.update(params)
        source = self.t.stream(6004, p, timeout=STREAM_DEADLINE)   # guards run here, not in the thread
        return NavTask(p["type"], "start", lambda: source)

    def start_mapping(self, outdoor: bool = False) -> NavTask:
        return self._start({"type": 5, "outdoor": bool(outdoor)})

    def relocalize(self, outdoor: bool = False) -> NavTask:
        return self._start({"type": 7, "outdoor": bool(outdoor)})

    def go_to(self, x: float, y: float, outdoor: bool = False) -> NavTask:
        return self._start({"type": 1, "outdoor": bool(outdoor), "goalX": as_float(x, "x"),
                            "goalY": as_float(y, "y"), "theta": 0.0})

    def dock(self) -> NavTask:
        return self._start({"type": 9})

    def save_map(self, name: str) -> NavTask:
        """Finish mapping and save as `name` (6009 type 5). Re-saving a name wipes its labels.
        The 6009 call can take up to 60 s, so it runs on the task thread; see NavTask.result."""
        if not isinstance(name, str) or not name:
            raise ValueError("map name must be a non-empty str (an empty name discards the map)")
        source = self.t.stream(6009, {"type": 5, "map_name": name}, timeout=STOP_TIMEOUT)
        return NavTask(5, "save", lambda: source)

    def stop(self, task_type: int) -> int:
        """6009 result: 0 success, 1 failed, 10 bridge timeout (keep polling state())."""
        if task_type not in STOP_TYPES:
            raise ValueError("stop type must be one of %s (see NavState.stop_type)" % (STOP_TYPES,))
        r = self.t.call_one(6009, {"type": task_type}, timeout=STOP_TIMEOUT)
        return int(r.get("result", -1))

    def attach(self, task_type: int) -> NavTask:
        """6010 re-attach to a running task's feedback."""
        if task_type not in NAV_TYPES:
            raise ValueError("attach type must be one of %s" % (NAV_TYPES,))
        source = self.t.stream(6010, {"type": task_type}, timeout=STREAM_DEADLINE)
        return NavTask(task_type, "attach", lambda: source)

    def state(self, timeout: float = 3.0) -> NavState:
        r = self.t.call_one(10001, {}, timeout=timeout)
        ts = r.get("task_status") or {}
        return decode_state(int(ts.get("task_status", -1)), int(ts.get("task_sub_status", 0)))
