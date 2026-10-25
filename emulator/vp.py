"""VP engine behind 2002, LED arbitration, and the fake `cyberdog` object task bodies run against
(vp_led_sensors.md sections 1, 2 and 4).

2002 itself writes zero messages; the engine's feedback goes to robotend, which the PC never sees, so
it is kept here for /state. Bodies run in-process in a restricted namespace: no real modules, open()
confined to <fs_root>/home/mi/Camera, time.sleep scaled. It is a test fake, not a security boundary.
"""
from __future__ import annotations

import ast
import builtins
import collections
import datetime
import json
import math
import os
import posixpath
import queue
import random
import re
import sys
import threading
import time

from emulator.common import ParseError, rj_parse

CLIENTS = ("lowpower", "bms", "connector", "vp", "tracking", "system")   # led_priority.toml: lower index wins
TARGETS = {1: "head", 2: "tail", 3: "mini"}


class LedConstraint:                                    # LedExecute.srv:11-74 (api/common.cpp:399+)
    target_head, target_tail, target_mini = 1, 2, 3
    effect_line_on, effect_line_blink, effect_line_blink_fast = 0x01, 0x02, 0x03
    effect_line_breath, effect_line_breath_fast = 0x04, 0x05
    effect_line_one_by_one, effect_line_one_by_one_fast = 0x06, 0x07
    effect_line_back_and_forth, effect_line_trailing_race = 0x08, 0x09
    system_effect_line_off = 0xA0
    effect_mini_circular_breath, effect_mini_circular_ring = 0x30, 0x31
    system_effect_mini_off, system_effect_mini_rectangle_color = 0x32, 0x33
    system_effect_mini_centre_color, system_effect_mini_three_circular = 0x34, 0x35
    system_effect_mini_one_by_one = 0x36


for _i, _c in enumerate(("red", "blue", "yellow")):
    for _j, _e in enumerate(("on", "blink", "blink_fast", "breath", "breath_fast", "one_by_one", "one_by_one_fast")):
        setattr(LedConstraint, "system_effect_line_%s_%s" % (_c, _e), 0xA1 + 7 * _i + _j)


class StateCode:                                        # common.hpp:443-465
    invalid, success, error_base, fail, uninitialized = -1, 0, 7900, 7921, 7922
    fsm_does_not_allow, module_status_error, network_error = 7923, 7924, 7925
    no_operation_authority, timeout, command_does_not_support = 7926, 7927, 7928
    self_test_failed, parameter_is_invalid, status_is_busy, hardware_error = 7929, 7930, 7931, 7932
    command_waiting_execute, spin_future_interrupted, spin_future_timeout = 7941, 7942, 7943
    no_data_update, service_client_interrupted, service_appear_timeout = 7944, 7945, 7946
    service_request_interrupted, service_request_rejected, service_request_timeout = 7947, 7948, 7949
    action_request_timeout, action_request_rejected, action_result_timeout = 7950, 7951, 7952
    motion_error = 7961


def _effect_ok(target, mode, effect):                   # led_plugin.cpp:480-520
    if target == 3:
        return 0x30 <= effect <= 0x31 if mode == 2 else 0x32 <= effect <= 0x36
    return 0x01 <= effect <= 0x09 if mode == 2 else 0xA0 <= effect <= 0xB5


class Leds:
    """LedCarpo::Play (led_plugin.cpp:418-575): one slot per client and target."""

    def __init__(self):
        self.lk = threading.Lock()
        self.reset()

    def reset(self):
        with self.lk:
            self.slots = {t: [None] * len(CLIENTS) for t in TARGETS}
            self.shown = {t: None for t in TARGETS}

    def execute(self, occupation, client, target, mode=0, effect=0, r=0, g=0, b=0):
        with self.lk:
            if target not in self.slots:
                return 1121
            if client not in CLIENTS:
                return 1122
            if occupation:
                if mode not in (1, 2):
                    return 1123
                if not _effect_ok(target, mode, effect):
                    return 1124
            i = CLIENTS.index(client)
            slots = self.slots[target]
            slots[i] = dict(owner=client, mode=mode, effect=effect, r=r, g=g, b=b) if occupation else None
            if any(slots[:i]):                          # stored, but a higher priority holds the LED
                return 1125
            cmd = next((s for s in slots[i:] if s), None)   # find_cmd: own slot, else next lower priority
            self.shown[target] = _shown(target, cmd)
            return 0

    def state(self):
        with self.lk:
            return {TARGETS[t]: self.shown[t] for t in TARGETS}


def _shown(target, cmd):
    if cmd is None:
        return None
    r, g, b = cmd["r"], cmd["g"], cmd["b"]
    if cmd["mode"] == 1 and target != 3:                # system colours
        e = cmd["effect"]
        r, g, b = (0, 0, 0) if e == 0xA0 else ((255, 0, 0), (0, 0, 255), (255, 255, 0))[(e - 0xA1) // 7]
    elif cmd["mode"] == 1 and cmd["effect"] == 0x32:
        r, g, b = 0, 0, 0
    return dict(r=r, g=g, b=b, effect=cmd["effect"], mode=cmd["mode"], owner=cmd["owner"])


# ---- request validation (frontend_message.cpp) ----------------------------------------------
_SINGLE = re.compile(r"(now|([0]{0,1}\d|1\d|2[0-3]):([0-5][0-9]{0,1})([\s]*([\d]+)\-([0]{0,1}[1-9]|1[0-2])\-"
                     r"([0]{0,1}[1-9]|[1-2]\d|3[0-1])){0,1})([\s]*\+[\s]*([\d]+)(minutes|hours|days|weeks|months|years)){0,1}",
                     re.ASCII)


_MIN, _HOUR = r"[0]{0,1}[0-5][0-9]{0,1}", r"[0]{0,1}\d|1\d|2[0-3]"
_DAY, _MON, _DOW = r"[0]{0,1}[1-9]|[1-2]\d|3[0-1]", r"[0]{0,1}[1-9]|1[0-2]", r"[0]{0,1}[1-7]"
_CYCLE = re.compile(r"[\s]".join((                      # frontend_message.cpp:161-206, read as written
    r"(\*|(\*|(%s))/([1-5][0-9]{0,1})|(%s)(,(%s)){0,58}|(%s)(-(%s)){0,1})" % ((_MIN,) * 5),
    r"(\*|(\*|(%s))/([1-9]|1\d|2[0-3])|(%s)(,(%s)){0,22}|(%s)(-([1-9]|1\d|2[0-3])){0,1})" % ((_HOUR,) * 4),
    r"(\*|(\*|(%s))/(%s)|(%s)(,(%s)){0,30}|(%s)(-(%s)){0,1})" % ((_DAY,) * 6),
    r"(\*|(\*|(%s))/(%s)|(%s)(,(%s)){0,11}|(%s)(-(%s)){0,1})" % ((_MON,) * 6),
    r"(\*|(\*|(%s))/(%s)|(%s)(,(%s)){0,6}|(%s)(-(%s)){0,1})" % ((_DOW,) * 6))), re.ASCII)
_UNITS = {"minutes": 60, "hours": 3600, "days": 86400, "weeks": 604800, "months": 2592000, "years": 31536000}


class Bad(Exception):
    def __init__(self, code, describe):
        Exception.__init__(self, describe)
        self.code, self.describe = code, describe


def parse_request(doc, tasks):
    """FrontendMessage (frontend_message.cpp:24-506). Returns the fields, or raises Bad.
    get_str/get_strs have no return statement in the source (undefined behaviour); this follows the
    intended first-error-wins order."""
    if doc is None:
        raise Bad(8021, "does not conform to json rules")
    f = {}

    def s(key, code, must):
        if not isinstance(doc, dict):
            raise Bad(8022, "input doc should be kObejectType")
        if key not in doc:
            if must:
                raise Bad(code, "no [%s] primary key" % key)
            f[key] = ""
            return
        if not isinstance(doc[key], str) or (must and not doc[key]):
            raise Bad(code, "[%s] is empty or not a string" % key)
        f[key] = doc[key]

    s("type", 8022, True)
    s("id", 8023, True)
    s("operate", 8027, True)
    op = f["operate"]
    tid = doc.get("target_id")
    if not isinstance(tid, list) or not all(isinstance(e, str) and e for e in tid):
        raise Bad(8024, "[target_id] missing, not an array, or with an empty/non-string element")
    if op != "inquiry" and not tid:
        raise Bad(8024, "[target_id] is empty")
    f["target_id"] = tid
    need = op in ("save", "debug")
    for key, code in (("describe", 8025), ("style", 8026), ("mode", 8028), ("condition", 8029), ("body", 8030)):
        s(key, code, need)
    if f["type"] != "task":
        if f["type"] in ("module", "AI", "SLAM"):
            return f                                    # judge_module/ai/slam: not emulated
        raise Bad(8021, "type does not meet the constraint rules")

    def cond(mode):
        if mode == "single":
            ok = _SINGLE.fullmatch(f["condition"])
        elif mode == "cycle":
            ok = _CYCLE.fullmatch(f["condition"])
        else:
            raise Bad(8028, "'mode' is invalid: " + mode)
        if not ok:
            raise Bad(8029, "'condition' is invalid: " + f["condition"])

    if op == "save":
        skip = ("id", "terminal_default", "visual_default", "debug", tid[0])
        if any(t["describe"] == f["describe"] for k, t in tasks.items() if k not in skip):
            raise Bad(8025, "'describe' is not unique: " + f["describe"])
        cond(f["mode"])
    elif op == "run":
        if f["mode"]:
            cond(f["mode"])
    elif op == "debug":                                 # three checks, the last failing one wins (:321-343)
        code = 0
        if tid[0] != "debug":
            code = 8024
        if f["mode"] != "single":
            code = 8028
        if f["condition"] != "now":
            code = 8029
        if code:
            raise Bad(code, "debug needs target_id ['debug'], mode single, condition now")
    elif op not in ("inquiry", "delete", "shutdown", "suspend", "recover"):
        raise Bad(8027, "'operate' is invalid: " + op)
    return f


_SINGLE_FLOW = {                                        # task.hpp:50-145
    "null": {"delete": "null", "debug": "run_wait", "save": "wait_run"},
    "error": {"debug": "run_wait", "save": "wait_run", "delete": "null", "inquiry": "error"},
    "wait_run": {"debug": "run_wait", "save": "wait_run", "delete": "null", "inquiry": "wait_run",
                 "shutdown": "shutdown", "run": "run_wait"},
    "run_wait": {"debug": "run_wait", "inquiry": "run_wait", "shutdown": "shutdown", "start": "run"},
    "run": {"run": "run", "start": "run", "recover": "run", "debug": "run_wait", "inquiry": "run",
            "suspend": "suspend", "shutdown": "shutdown", "stop": "shutdown"},
    "suspend": {"suspend": "suspend", "debug": "run_wait", "inquiry": "suspend", "recover": "run",
                "shutdown": "shutdown", "stop": "shutdown"},
    "shutdown": {"shutdown": "shutdown", "debug": "run_wait", "save": "wait_run", "inquiry": "shutdown",
                 "delete": "null", "run": "run_wait"},
}
FLOW = {"single": _SINGLE_FLOW,
        "cycle": dict(_SINGLE_FLOW, run=dict(_SINGLE_FLOW["run"], shutdown="run_wait", stop="run_wait"),
                      suspend=dict(_SINGLE_FLOW["suspend"], shutdown="run_wait", stop="run_wait"))}

# ---- build check (pyflakes on the generated file, task.cpp:194-203) ---------------------------
HEADER_NAMES = {"os", "sys", "time", "threading", "mi", "get_namespace", "get_argv", "Cyberdog", "now_task_id",
                "now_task_parameters", "task_id", "task_parameters", "cyberdog", "__name__", "__file__"}
AUTO = {"StateCode.": "StateCode", "LedConstraint.": "LedConstraint", "MotionSequence(": "MotionSequence",
        "MotionSequenceGait(": "MotionSequenceGait", "MotionSequencePace(": "MotionSequencePace"}   # utils.py:153-165
BAD_ATTRS = {"gi_frame", "gi_code", "gi_yieldfrom", "cr_frame", "cr_code", "cr_await", "ag_frame", "ag_code",
             "f_back", "f_globals", "f_locals", "f_builtins", "f_code", "tb_frame", "tb_next", "co_code"}


def _parse36(body):
    try:
        return ast.parse(body, feature_version=(3, 6))  # the dog runs Python 3.6
    except ValueError:                                  # feature_version floor raised on a newer Python
        return ast.parse(body)


def build_check(body):
    """pyflakes approximation: syntax (3.6 grammar), undefined names, unused imports, import *.
    Returns None when the build passes, else the message. Also refuses sandbox escapes."""
    try:
        tree = _parse36(body)
    except SyntaxError as e:
        return "pyflakes: syntax error line %s: %s" % (e.lineno, e.msg)
    defined = set(HEADER_NAMES) | set(dir(builtins)) | {v for k, v in AUTO.items() if k in body}
    imported = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Name) and not isinstance(n.ctx, ast.Load):
            defined.add(n.id)
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            defined.add(n.name)
        elif isinstance(n, ast.arg):
            defined.add(n.arg)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            defined.add(n.name)
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            defined.update(n.names)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            for a in n.names:
                if a.name == "*":
                    return "pyflakes: 'from %s import *' used" % getattr(n, "module", "")
                name = a.asname or a.name.split(".")[0]
                defined.add(name)
                imported[name] = n.lineno
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    undefined = sorted(used - defined)
    if undefined:
        return "pyflakes: undefined name %r" % undefined[0]
    unused = sorted(k for k in imported if k not in used)
    if unused:
        return "pyflakes: %r imported but unused" % unused[0]
    for n in ast.walk(tree):                            # emulator sandbox, not pyflakes
        if isinstance(n, ast.Attribute) and (n.attr.startswith("_") or n.attr in BAD_ATTRS):
            return "emulator sandbox: attribute %r refused" % n.attr
        if isinstance(n, ast.Name) and n.id.startswith("__") and n.id not in ("__name__", "__file__"):
            return "emulator sandbox: name %r refused" % n.id
    return None


# ---- the fake runtime ---------------------------------------------------------------------------
class _Stop(BaseException):
    """Raised inside a body when its task is shut down (kill -9 on the dog)."""


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __repr__(self):
        return "Msg(%s)" % ", ".join("%s=%r" % kv for kv in sorted(self.__dict__.items()))


def _ret(code=0, response=None):
    return Msg(state=Msg(code=code, describe=""), response=response)


def _module(name, **attrs):
    m = Msg(**attrs)
    m.__dict__["__name__"] = name
    return m


class _Run:
    """One execution of a task body; every fake API call checks for a pending kill."""

    def __init__(self, vp, tid, stop, log):
        self.vp, self.tid, self.stop, self.log = vp, tid, stop, log

    def check(self):
        if self.stop.is_set() or self.vp.emu.stopping.is_set():
            raise _Stop()

    def sleep(self, t):
        self.check()
        if self.stop.wait(self.vp.emu.sc(max(float(t), 0.0))):
            raise _Stop()

    def host(self, path):                               # confine to <fs_root>/home/mi/Camera
        p = posixpath.normpath(str(path))
        if not p.startswith("/home/mi/Camera/") and p != "/home/mi/Camera":
            self.vp.emu.refuse(2002, "VP body touched a path outside /home/mi/Camera", path=str(path), task=self.tid)
            raise PermissionError("emulator: only /home/mi/Camera is writable from a VP task: %s" % path)
        return os.path.join(self.vp.emu.fs_root, p.lstrip("/").replace("/", os.sep))

    def led(self, occupation, target, mode, effect, r, g, b):
        self.check()
        code = self.vp.leds.execute(occupation, "vp", int(target), int(mode), int(effect), int(r), int(g), int(b))
        self.vp.led_log.append(dict(task=self.tid, occupation=occupation, target=int(target), mode=mode,
                                    effect=int(effect), rgb=[r, g, b], code=code))
        return _ret(0, Msg(code=code))                  # check .response.code, not .state.code (sdk/led.cpp:116-130)


class _Led:
    def __init__(self, run):
        self._run = run

    def play(self, target, effect):
        return self._run.led(True, target, 1, effect, 0, 0, 0)

    def play_rgb(self, target, effect, r, g, b):
        return self._run.led(True, target, 2, effect, r, g, b)

    def freed(self, target):
        return self._run.led(False, target, 0, 0, 0, 0, 0)


class _Sensor:
    def __init__(self, run, make):
        self._run, self._make = run, make
        self.state = Msg(code=0, describe="")
        self.data = make()

    def get_data(self, timeout=5):
        self._run.check()
        self.data = self._make()
        return self.data


class _Skin:
    def __init__(self, run):
        self._run = run

    def discolored(self, model=0, duration_ms=1000):    # disable, enable, set_elec_skin (sdk/skin.cpp:279-329)
        self._run.check()
        d = self._run.vp.emu.device
        d.skin.update(enabled=True, mode=int(model), wave_cycle_time=int(duration_ms))
        return _ret(0)

    def electrochromic(self, *a, **kw):                 # raw CAN, bypasses skin_manager
        self._run.vp.unsupported.append(dict(task=self._run.tid, call="skin.electrochromic"))
        return _ret(StateCode.command_does_not_support)


class _Task:
    def __init__(self, run):
        self._run = run

    def start(self):
        self._run.check()
        return _ret(0)

    def stop(self):
        self._run.vp.transition(self._run.tid, "stop")
        return _ret(0)

    def block(self, _id="", **kw):
        self._run.check()
        return _ret(0)


class _Unsupported:
    def __init__(self, run, name):
        self._run, self._name = run, name

    def __getattr__(self, attr):
        if attr.startswith("_"):
            raise AttributeError(attr)

        def call(*a, **kw):
            self._run.check()
            self._run.vp.unsupported.append(dict(task=self._run.tid, call="%s.%s" % (self._name, attr)))
            return _ret(StateCode.command_does_not_support)
        return call


class _Cyberdog:
    def __init__(self, run):
        self._run = run
        emu = run.vp.emu
        self.led, self.skin, self.task = _Led(run), _Skin(run), _Task(run)
        for name in ("motion", "navigation", "audio", "network", "follow", "train", "personnel", "gesture",
                     "skeleton"):
            setattr(self, name, _Unsupported(run, name))
        hdr = lambda: Msg(stamp=Msg(sec=int(time.time()), nanosec=0), frame_id="")
        tof = lambda pos: Msg(header=hdr(), data_available=True, tof_position=pos, data=[0.66] * 64)

        def bms():
            d = emu.device
            return Msg(header=hdr(), batt_volt=25200, batt_curr=-1500, batt_soc=d.soc, batt_temp=30,
                       power_adapter_temp=0, wireless_charging_temp=0, batt_st=0, bms_state_one=0,
                       bms_state_two=0, batt_health=100, batt_loop_number=0, power_normal=True,
                       power_wired_charging=d.charging, power_finished_charging=False, power_motor_shutdown=False,
                       power_soft_shutdown=False, power_wp_place=d.wireless, power_wp_charging=d.wireless)

        def odom():
            gt = emu.ctl.ground_truth()
            v3 = lambda x, y, z: Msg(x=x, y=y, z=z)
            cy, sy = math.cos(gt["yaw"] / 2), math.sin(gt["yaw"] / 2)
            return Msg(header=hdr(), child_frame_id="base_link",
                       pose=Msg(pose=Msg(position=v3(gt["x"], gt["y"], gt["z"]), orientation=Msg(x=0.0, y=0.0, z=sy, w=cy))),
                       twist=Msg(twist=Msg(linear=v3(gt["vx"], gt["vy"], 0.0), angular=v3(0.0, 0.0, gt["wz"]))))

        self.bms = _Sensor(run, bms)
        self.odometer = _Sensor(run, odom)
        self.tof = _Sensor(run, lambda: Msg(head=Msg(left_head=tof(0), right_head=tof(1)),
                                            rear=Msg(left_rear=tof(2), right_rear=tof(3))))
        self.ultrasonic = _Sensor(run, lambda: Msg(header=hdr(), radiation_type=0, field_of_view=15.0,
                                                   min_range=0.0, max_range=3.0, range=1.2))
        self.imu = _Sensor(run, lambda: Msg(header=hdr(), orientation=Msg(x=0.0, y=0.0, z=0.0, w=1.0),
                                            angular_velocity=Msg(x=0.0, y=0.0, z=0.0),
                                            linear_acceleration=Msg(x=0.0, y=0.0, z=9.8)))
        self.gps = _Sensor(run, lambda: Msg(sec=int(time.time()), nanosec=0, itow=0, fix_type=0, num_sv=0,
                                            lon=0.0, lat=0.0))
        self.touch = _Sensor(run, lambda: Msg(header=hdr(), touch_state=0))
        self.lidar = _Sensor(run, lambda: Msg(header=hdr(), angle_min=-math.pi, angle_max=math.pi,
                                              angle_increment=math.pi / 180, range_min=0.1, range_max=12.0,
                                              ranges=[2.0] * 360, intensities=[]))
        self.state = Msg(code=0, describe="")

    def set_log(self, log=True):
        return _ret(0)

    def ready(self, timeout=30):
        self._run.check()
        return True

    def shutdown(self, exit=True):                      # frees head, tail and mini (sdk/cyberdog.cpp:229-260)
        for t in TARGETS:
            self._run.vp.leds.execute(False, "vp", t)
        return _ret(0)


def _namespace(run, task):
    vp = run.vp

    def _print(*a, **kw):
        line = kw.get("sep", " ").join(str(x) for x in a)
        run.log.append(line)
        del run.log[:-200]

    def _open(path, mode="r", *a, **kw):
        run.check()
        p = run.host(path)
        if any(c in mode for c in "wax+"):
            os.makedirs(os.path.dirname(p), exist_ok=True)
        return open(p, mode, *a, **kw)

    def _exists(path):
        try:
            return os.path.exists(run.host(path))
        except PermissionError:
            return False

    ftime = _module("time", sleep=run.sleep, time=time.time, monotonic=time.monotonic, perf_counter=time.perf_counter,
                    strftime=time.strftime, localtime=time.localtime, gmtime=time.gmtime, asctime=time.asctime,
                    ctime=time.ctime, mktime=time.mktime, time_ns=getattr(time, "time_ns", None))
    fos = _module("os", path=_module("os.path", join=posixpath.join, basename=posixpath.basename,
                                     dirname=posixpath.dirname, splitext=posixpath.splitext, exists=_exists,
                                     getsize=lambda p: os.path.getsize(run.host(p))),
                  listdir=lambda p="/home/mi/Camera": os.listdir(run.host(p)),
                  remove=lambda p: os.remove(run.host(p)), getpid=os.getpid, sep="/",
                  # the header's real `import os` (utils.py:175); POSIX rename replaces an existing target
                  rename=lambda a, b: os.replace(run.host(a), run.host(b)),
                  replace=lambda a, b: os.replace(run.host(a), run.host(b)))
    fsys = _module("sys", argv=["%s.py" % task["tid"], "cyberdog_vp_task:=%s" % task["tid"]],
                   version="3.6.9 (emulator)", path=[])
    abilityset = _module("mi.cyberdog_vp.abilityset", StateCode=StateCode, LedConstraint=LedConstraint,
                         Cyberdog=None)
    mods = {
        "time": ftime, "os": fos, "sys": fsys,
        "threading": _module("threading", Thread=threading.Thread, Lock=threading.Lock, RLock=threading.RLock,
                             Event=threading.Event, Timer=threading.Timer, get_ident=threading.get_ident),
        "json": _module("json", dumps=json.dumps, loads=json.loads, JSONDecodeError=json.JSONDecodeError),
        "math": _module("math", **{k: getattr(math, k) for k in dir(math) if not k.startswith("_")}),
        "random": _module("random", **{k: getattr(random, k) for k in ("random", "randint", "uniform", "choice",
                                                                       "shuffle", "sample", "seed", "gauss")}),
        "datetime": _module("datetime", datetime=datetime.datetime, date=datetime.date,
                            timedelta=datetime.timedelta),
        "collections": _module("collections", deque=collections.deque, OrderedDict=collections.OrderedDict,
                               defaultdict=collections.defaultdict, Counter=collections.Counter,
                               namedtuple=collections.namedtuple),
        "mi.cyberdog_vp.abilityset": abilityset,
    }

    def _import(name, globals=None, locals=None, fromlist=(), level=0):
        if name in mods and (fromlist or "." not in name):
            return mods[name]
        vp.emu.refuse(2002, "VP body imported a module the emulator does not provide", module=name, task=run.tid)
        raise ImportError("emulator: module %r is not available to VP tasks" % name)

    safe = {k: getattr(builtins, k) for k in (
        "abs", "all", "any", "bool", "bytes", "chr", "dict", "dir", "divmod", "enumerate", "filter", "float", "format",
        "frozenset", "hash", "hex", "int", "isinstance", "issubclass", "iter", "len", "list", "map", "max", "min",
        "next", "object", "oct", "ord", "pow", "range", "repr", "reversed", "round", "set", "slice", "sorted",
        "str", "sum", "super", "tuple", "zip", "property", "staticmethod", "classmethod", "Exception",
        "BaseException", "ValueError", "TypeError", "KeyError", "IndexError", "RuntimeError", "ZeroDivisionError",
        "StopIteration", "AttributeError", "NameError", "ArithmeticError", "LookupError", "OSError", "IOError",
        "FileNotFoundError", "PermissionError", "ImportError", "NotImplementedError", "AssertionError")}
    safe.update(print=_print, open=_open, __import__=_import, __build_class__=builtins.__build_class__)
    cyberdog = _Cyberdog(run)
    ns = {"__builtins__": safe, "__name__": "__main__", "__file__": "%s.py" % task["tid"],
          "os": fos, "sys": fsys, "time": ftime, "threading": mods["threading"], "cyberdog": cyberdog,
          "task_id": task["tid"], "task_parameters": "", "now_task_id": "", "now_task_parameters": ""}
    for k, v in AUTO.items():
        if k in task["body"] and hasattr(abilityset, v):
            ns[v] = getattr(abilityset, v)
    return ns, cyberdog


class VP:
    def __init__(self, emu):
        self.emu = emu
        self.leds = Leds()
        self.lk = threading.RLock()
        self.q = queue.Queue()
        self.gen = 0
        self.reset()
        threading.Thread(target=self._loop, name="vp-engine", daemon=True).start()

    def reset(self):
        with self.lk:
            for ev in getattr(self, "runs", {}).values():
                ev.set()
            self.gen += 1
            self.tasks = {}                             # target_id -> registry entry
            self.runs = {}                              # target_id -> kill event of the live execution
            self.feedback = collections.deque(maxlen=50)
            self.logs = {}
            self.led_log = collections.deque(maxlen=100)
            self.unsupported = collections.deque(maxlen=50)
        self.leds.reset()

    def frontend(self, raw):                            # frontend_message topic: the engine works serially
        self.q.put((self.gen, raw))

    def _loop(self):
        while not self.emu.stopping.is_set():
            try:
                gen, raw = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if gen == self.gen:
                try:
                    self._handle(raw)
                except Exception as e:                  # an engine bug must not kill the engine thread
                    self.feedback.append({"emulator_error": repr(e)})

    def _fb(self, f, state, describe, kind="parse"):
        self.feedback.append({"feedback": {"type": kind, "id": f.get("id", ""), "operate": f.get("operate", ""),
                                           "state": state, "describe": describe}})

    def _handle(self, raw):
        try:
            doc = rj_parse(raw)
        except ParseError:
            doc = None
        try:
            with self.lk:
                f = parse_request(doc, self.tasks)
        except Bad as e:
            fid = doc if isinstance(doc, dict) else {}
            self._fb({"id": fid.get("id", ""), "operate": fid.get("operate", "")}, e.code, e.describe)
            self.emu.refuse(2002, "VP parse %d: %s" % (e.code, e.describe))
            return
        if f["type"] != "task":
            self._fb(f, 0, "type %s is not emulated" % f["type"], "emulator")
            return
        op = f["operate"]
        if op == "inquiry":
            with self.lk:
                lst = [dict(target_id=k, describe=t["describe"], style=t["style"], mode=t["mode"],
                            condition=t["condition"], state=t["state"])
                       for k, t in self.tasks.items() if k != "debug" and t["state"] != "null"
                       and (not f["target_id"] or k in f["target_id"]) and (not f["mode"] or t["mode"] == f["mode"])]
            self.feedback.append({"response": {"type": "task", "id": f["id"], "operate": op, "list": lst}})
            return
        for tid in (f["target_id"] if op == "delete" else f["target_id"][:1]):   # only delete iterates
            self._operate(f, tid)

    def transition(self, tid, op):
        with self.lk:
            t = self.tasks.get(tid)
            state = t["state"] if t else "null"
            nxt = FLOW.get(t["mode"] if t else "single", _SINGLE_FLOW)[state].get(op)
            if nxt is not None and t is not None:
                t["state"] = nxt
            return state, nxt

    def _operate(self, f, tid):
        op = f["operate"]
        with self.lk:
            t = self.tasks.get(tid)
            if t is None and op in ("save", "debug"):
                t = self.tasks[tid] = dict(tid=tid, state="null", mode=f["mode"], condition=f["condition"],
                                           describe=f["describe"], style=f["style"], body=f["body"])
            state, nxt = self.transition(tid, op)
            if nxt is None:
                self._fb(f, -1, "%s not allowed from state %s" % (op, state), "task")
                self.emu.refuse(2002, "VP %s refused in state %s" % (op, state), target_id=tid)
                return
            if op == "delete":
                self.tasks.pop(tid, None)
            if op in ("shutdown", "delete", "debug", "run") and tid in self.runs:
                self.runs.pop(tid).set()                # kill the live process
            if op in ("save", "debug"):
                t.update(mode=f["mode"], condition=f["condition"], describe=f["describe"], style=f["style"],
                         body=f["body"])
                err = build_check(f["body"])
                if err:
                    t["state"] = "error"
                    self.logs[tid] = [err]
                    self._fb(f, -1, err, "task")
                    self.emu.refuse(2002, "VP build failed: " + err, target_id=tid)
                    return
            if op == "run" and f["mode"]:
                t.update(mode=f["mode"], condition=f["condition"])
        self._fb(f, 0, "%s -> %s" % (op, nxt), "task")
        if op == "debug":
            self._start(tid, 0.0)
        elif op == "run":
            m = re.fullmatch(r"now([\s]*\+[\s]*(\d+)(minutes|hours|days|weeks|months|years))?", t["condition"])
            if t["mode"] == "single" and m:
                self._start(tid, int(m.group(2) or 0) * _UNITS.get(m.group(3), 0))
            else:                                       # clock times and cron are scheduled, never fired here
                self.logs[tid] = ["emulator: scheduled for %r (%s); not executed" % (t["condition"], t["mode"])]

    def _start(self, tid, delay):
        stop = threading.Event()
        with self.lk:
            old = self.runs.pop(tid, None)
            if old is not None:
                old.set()
            self.runs[tid] = stop
        threading.Thread(target=self._exec, args=(tid, stop, delay), name="vp-task-" + tid, daemon=True).start()

    def _exec(self, tid, stop, delay):
        if stop.wait(self.emu.sc(1.0 + delay)):         # the build sleeps 1 s (task.cpp:197), then at/cron fires
            return
        with self.lk:
            t = self.tasks.get(tid)
            if t is None or self.runs.get(tid) is not stop:
                return
            self.transition(tid, "start")
            log = self.logs[tid] = []
            body = t["body"]
        run = _Run(self, tid, stop, log)
        ns, cyberdog = _namespace(run, dict(t, tid=tid))

        def tracer(frame, event, arg):                  # lets shutdown kill a body that never calls the API
            if stop.is_set() or self.emu.stopping.is_set():
                raise _Stop()
            return tracer
        sys.settrace(tracer)
        try:
            exec(compile(body, "<vp:%s>" % tid, "exec"), ns)
            sys.settrace(None)
            cyberdog.task.stop()                        # generated footer (task.cpp:166-170)
            cyberdog.shutdown()
            log.append("emulator: task finished")
        except _Stop:
            log.append("emulator: task killed (LEDs stay as they were)")
        except BaseException as e:                      # the python process dies: no footer, LEDs stay occupied
            log.append("Traceback: %s: %s" % (type(e).__name__, e))
        finally:
            sys.settrace(None)
            with self.lk:
                if self.runs.get(tid) is stop:
                    del self.runs[tid]

    def state(self):
        with self.lk:
            return {"tasks": {k: {x: t[x] for x in ("state", "mode", "condition", "describe")}
                              for k, t in self.tasks.items()},
                    "running": sorted(self.runs), "feedback": list(self.feedback), "logs": dict(self.logs),
                    "led_calls": list(self.led_log), "unsupported": list(self.unsupported)}
