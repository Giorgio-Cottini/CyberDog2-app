#!/usr/bin/env python3
# CyberDog 2 PC controller v2 - local app server.
#
# The bridge between app2/ui.html and the dog, built only on cyberdog_sdk
# (docs/design/contracts.md section 3) and the standard library. A
# ThreadingHTTPServer on 127.0.0.1:8794 serves the page, the JSON API of
# contracts.md 6.1, the SSE event stream and the camera MJPEG.
#
# SAFETY (contracts.md section 1, all enforced here and covered by tests/app):
#   - the movement gate is off at start and on every page load (GET /);
#     drive, action, motion, posture and nav start/save answer 403 without it;
#     disarming stops the drive and posture streams and cancels queued steps;
#   - ESTOP takes no lock and waits behind no queue: it goes straight to
#     CyberDog.estop(), which halts the streams and sends motion 0 on the
#     SDK's priority channel. It also disarms;
#   - drive dead-man: 400 ms without /api/drive stops the walk (END);
#   - only R0-R2 catalog motions are offered; R2 needs confirm ("pavimento
#     libero?"); force=True / allow_risky=True are never passed to the SDK;
#   - navigation has no 6004 type 0 path and no map-delete path at all;
#     during goto, dock and tracking, drive, posture, motion and action
#     answer 409 {navigating: true} (ESTOP and nav/stop stay open).
#
# CROSS-SITE: any web page can POST to 127.0.0.1, so every request must carry
# Host 127.0.0.1:<port> or localhost:<port> (403, blocks DNS rebinding), every
# POST needs Content-Type application/json (415) and an Origin, when present,
# must be http://127.0.0.1:<port> or http://localhost:<port> (403). No CORS.
#
# THREADS: one per HTTP request; the status poller (10001, 2 Hz); the info
# poller (1001: battery every 5 s, serial number once per connection); the
# dead-man (40 Hz); the state broadcaster (5 Hz, only on change); the camera
# manager; one worker thread per queued motion (at most one is current, that
# is state.motion.busy; after a 321 it stays current until the status poll
# shows the dog has left the motion). Locks: `lock` guards the state and is
# never held across a dog call; `dog_lock` orders stream control (drive, gait,
# posture, the stream stop before a motion) and client rebuilds. ESTOP, disarm
# and the dead-man never take `dog_lock` (see App.__init__).
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, os.path.join(ROOT, "pylibs") if os.name == "nt" else None):    # pylibs: Windows wheels
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyberdog_sdk import CyberDog, DogError, DogUnreachable, ForbiddenCommand, Gait, catalog  # noqa: E402
from cyberdog_sdk.client import NOT_SENT_ESTOP, SIT_IDS, trick_steps  # noqa: E402
from cyberdog_sdk.codes import describe  # noqa: E402
from cyberdog_sdk.nav import RUNNING_TYPE, decode_state  # noqa: E402
from cyberdog_sdk.servo import GAIT_LIMITS  # noqa: E402

# ---- config -----------------------------------------------------------------
UI_PATH = os.path.join(HERE, "ui.html")
DEFAULT_DOG = "192.168.137.115:50052"
DEFAULT_PORT = 8794
DEFAULT_BIND = "127.0.0.1"

DEADMAN_S = 0.4             # drive stops (END) this long after the last /api/drive
DEADMAN_TICK = 0.025
POLL_S = 0.5                # 10001 status poll period
STATUS_TIMEOUT = 1.5        # deadline of one 10001 (it reads a cache on the dog)
LOST_AFTER = 2              # consecutive failed polls before "disconnected"
REBUILD_AFTER_S = 3.0       # disconnected this long: rebuild the client (fresh channels,
REBUILD_EVERY_S = 5.0       # no gRPC reconnect backoff), at most this often
INFO_EVERY_S = 5.0          # 1001 battery poll period
WARM_EVERY_S = 30.0         # both gRPC channels are asked to connect: ESTOP never waits for TCP + mTLS
CONNECT_TIMEOUT = 5.0
INFO_TIMEOUT = 8.0          # 1001 deadline ("sn" can take 2 s + 3 s on the dog)
SN_TRIES = 3                # 1001 "sn" attempts per connection for the emulator test
QUICK_S = 1.0               # request/response calls answer within this, else "pending"
MAP_WAIT_S = 12.0           # a 6003 still running after this is reported as failed (the dog gives 321 at 10 s)
MAP_KEEP_S = 5.0            # a finished 6003 that no request collected yet is served for this long
MAP_LOG_EVERY_S = 60.0      # the same map read again logs at most once a minute
NAV_ACCEPT_S = 9.0          # 6004 accept reply: up to ~6 s, ~9 s with the service wait
NAV_GRACE_S = 10.0          # an accepted task that never shows as running is dropped
NAV_STOP_WAIT_S = 10.0      # a goto/dock accepted after a stop waits this long for that stop (ResetNav 2 s+)
SENSOR_TIMEOUT = 20.0       # VP sensor read-back (3-20 s)
BRIDGE_CAP_S = 20.0         # the bridge waits this long for a 1004 reply, then answers 321 (motion.md 2.2)
# After a 321 for a long motion (MotionResult.still_running) the motion goes on and later 1004s get 3011
# until it ends (motion.md, verifier additions). Busy holds until LEFT_POLLS consecutive 10001 polls show
# another motion id, at most LINGER_MARGIN x the catalog length (typical_s or min_exec_time, at least
# BRIDGE_CAP_S), counted from the 1004 send. Only motions of 20 s or more can give still_running, so 1.5x
# leaves at least 10 s beyond the nominal end: 30 s for 124/138, 45 s for 151, 90 s for 140, 180 s for
# the 120 s dances.
LINGER_MARGIN = 1.5
LEFT_POLLS = 2              # two polls (about 0.5-1 s): a single odd sample does not end the wait
STATE_HZ = 5                # at most 5 "state" events per second
KEEPALIVE_S = 10.0          # SSE comment period
SSE_QUEUE = 256             # events buffered per SSE client; overflow drops the client
WRITE_TIMEOUT = 10.0        # a stuck SSE / MJPEG client is dropped after this
MAX_BODY = 64 * 1024
DRAIN_MAX = 1024 * 1024     # an oversized body is read and dropped up to this size
BOUNDARY = "cyberdogframe"

DEFAULT_SPEED = 0.4         # fraction of the gait limit; with "slow": 0.2 m/s, 0.08 m/s, 0.4 rad/s
SPEED_RANGE = (0.1, 1.0)
# The app caps the slow gait below the SDK limit (0.6, 0.2, 1.3) so that the default
# level gives v1's 0.2 m/s forward without passing v1's 0.4 rad/s turn (contracts.md 1.11).
SLOW_CAP = (0.5, 0.2, 1.0)
GAIT_TABLE = (("slow", Gait.SLOW, "lenta"), ("medium", Gait.MEDIUM, "media"),
              ("fast", Gait.FAST, "veloce"), ("auto", Gait.AUTO, "automatica"))

APP_RISKS = ("R0", "R1", "R2")
EXCLUDED_NEEDS = frozenset(("servo", "pose", "never"))   # gaits, postures, registration ids
KEYMAP = {"1": "stand_sit", "2": 141, "3": 123, "4": 146, "5": 144, "6": 145, "7": 174, "8": 173,
          "9": 140}
ACTIONS = {"stand": 111, "sit": 143, "lie": 101, "stand_sit_toggle": None}
ACTION_IT = {"stand": "alzati", "sit": "seduto", "lie": "sdraiati", "stand_sit_toggle": "alzati / siediti"}
LYING_IDS = frozenset((0, 101, 102))       # the toggle stands up from these (0: after ESTOP)
LED_TARGETS = ("head", "tail", "mini")
LED_IT = {"head": "testa", "tail": "coda", "mini": "mini"}
POSE_KEYS = ("z", "roll", "pitch", "yaw")
NAV_TASKS = ("mapping", "relocalize", "goto", "dock")
NAV_TASK_IT = {"mapping": "creazione della mappa", "relocalize": "rilocalizzazione",
               "goto": "vai al punto", "dock": "ritorno alla base"}
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")
COLOR_RE = re.compile(r"^#([0-9a-fA-F]{2})([0-9a-fA-F]{2})([0-9a-fA-F]{2})$")

# 10001 task_status -> nav.state key and Italian task name (nav_map.md section 4)
NAV_KEYS = {101: "idle", 100: "unknown", 102: "launching", 103: "stopping", 5: "mapping", 15: "mapping",
            7: "relocalizing", 17: "relocalizing", 6: "failed", 16: "failed", 8: "localized",
            18: "localized", 1: "navigating", 21: "navigating", 9: "docking", 11: "tracking",
            13: "tracking", 3: "tracking", 0: "wedged"}
TASK_IT = {100: "navigazione non inizializzata", 101: "nessun compito", 102: "avvio del compito",
           103: "arresto del compito", 5: "creazione della mappa (laser)", 15: "creazione della mappa (visione)",
           7: "rilocalizzazione (laser)", 17: "rilocalizzazione (visione)",
           6: "posizione non trovata (laser)", 16: "posizione non trovata (visione)",
           8: "sa dove si trova (laser)", 18: "sa dove si trova (visione)",
           1: "in viaggio verso il punto", 21: "in viaggio verso il punto (visione)",
           9: "ritorno alla base", 11: "inseguimento UWB", 13: "inseguimento di una persona",
           3: "inseguimento di un oggetto", 0: "navigazione bloccata: serve un riavvio del cane"}
MAP_SUCCESS_IT = {1: "nessuna mappa salvata sul cane", 2: "mappa senza nome o ancora in costruzione",
                  3: "mappa visiva non disponibile", 4: "servizio mappe non raggiungibile",
                  5: "servizio mappe occupato", 255: "errore sconosciuto del servizio mappe"}
# present-state names for state.motion.name (the catalog names are commands)
MOTION_STATE_IT = {0: "arresto di emergenza", 101: "sdraiato", 102: "sdraiato", 111: "in piedi",
                   112: "in piedi", 143: "seduto", 214: "seduto", 201: "postura regolata",
                   303: "cammina", 304: "cammina", 305: "cammina", 308: "cammina"}

MSG_GATE = "movimento disabilitato: attiva prima l'interruttore del movimento"
MSG_BUSY = "un movimento è già in corso: attendi che finisca"
MSG_CONFIRM = ("pavimento libero? Questo movimento si sposta o dura a lungo: conferma che intorno "
               "al cane ci sia almeno 1,5 m di spazio libero")
MSG_UNREACHABLE = "il cane non risponde (connessione assente o scaduta)"
MSG_NAVIGATING = "navigazione autonoma in corso: fermala prima di guidare o regolare la postura"
MSG_NAV_MOTION = "navigazione autonoma in corso: premi prima «Ferma navigazione», poi riprova il movimento"
# Types 1/9/11/13: no 1002 stream (nav_map.md 4) and no 1004 either, because what the dog does with a
# result motion during autonomous navigation is unverified: the app never mixes the two.
STREAM_FREE_NAV = ("navigating", "docking", "tracking")

clock = time.perf_counter   # monotonic; time.monotonic() ticks in 15.6 ms steps on Windows
ZERO = (0.0, 0.0, 0.0)


def _gait_limits(key, gait):
    lo, hi = GAIT_LIMITS[int(gait)]
    if key == "slow":
        hi = tuple(min(h, c) for h, c in zip(hi, SLOW_CAP))
        lo = tuple(max(v, -c) for v, c in zip(lo, SLOW_CAP))
    return tuple(lo), tuple(hi)


GAITS = collections.OrderedDict()
for _key, _gait, _name in GAIT_TABLE:
    _lo, _hi = _gait_limits(_key, _gait)
    GAITS[_key] = {"gait": _gait, "name": _name, "lo": _lo, "hi": _hi}

APP_MOTIONS = collections.OrderedDict(
    (m.id, m) for m in catalog.all()
    if m.risk in APP_RISKS and m.id != 0 and not (set(m.needs) & EXCLUDED_NEEDS))
assert all(v in APP_MOTIONS for v in KEYMAP.values() if isinstance(v, int)), "keymap id outside the app catalog"


def build_catalog() -> dict:
    motions = [{"id": m.id, "name": m.name_it, "risk": m.risk, "needs_confirm": m.risk == "R2",
                "typical_s": m.typical_s} for m in APP_MOTIONS.values()]
    gaits = [{"key": k, "name": g["name"], "motion_id": int(g["gait"]), "max": list(g["hi"]),
              "min": list(g["lo"])} for k, g in GAITS.items()]
    return {"motions": motions, "keymap": dict(KEYMAP), "gaits": gaits,
            "speed": {"default": DEFAULT_SPEED, "min": SPEED_RANGE[0], "max": SPEED_RANGE[1]}}


CATALOG = build_catalog()


def linger_cap_s(motion_id: int, margin: float = LINGER_MARGIN) -> float:
    """How long busy may hold after a 321 for this motion, from its 1004 send (see LINGER_MARGIN)."""
    m = catalog.get(motion_id)
    length = BRIDGE_CAP_S
    if m is not None:
        length = max(length, m.typical_s or 0.0, max(m.min_ms, 0) / 1000.0)
    return margin * length

PLACEHOLDER = """<!doctype html>
<html lang="it"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CyberDog</title></head>
<body style="font-family:system-ui,sans-serif;max-width:40rem;margin:3rem auto;padding:0 1rem;line-height:1.5">
<h1>CyberDog</h1>
<p>Il server è acceso, ma manca la pagina di controllo (<code>app2/ui.html</code>).</p>
<p>Lo stato del cane è qui: <a href="/api/state">/api/state</a>.</p>
</body></html>
"""


# ---- helpers ----------------------------------------------------------------
class ApiError(Exception):
    """An answer in the error shape of contracts.md 6.1."""

    def __init__(self, status: int, error: str, code=None, **extra):
        super().__init__(error)
        self.status, self.error, self.code, self.extra = status, error, code, extra

    def body(self) -> dict:
        b = {"ok": False, "error": self.error, "code": self.code}
        b.update(self.extra)
        return b


_MISSING = object()


def _num(body: dict, key: str, default=_MISSING) -> float:
    v = body.get(key)
    if v is None:
        if default is _MISSING:
            raise ApiError(400, "manca il campo «%s»" % key)
        return default
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not _finite(v):
        raise ApiError(400, "il campo «%s» deve essere un numero" % key)
    return float(v)


def _finite(v) -> bool:
    try:
        return math.isfinite(v)
    except OverflowError:   # an int too large for a float, such as 10**400
        return False


def _int(body: dict, key: str, default=_MISSING, lo=None, hi=None) -> int:
    v = body.get(key)
    if v is None:
        if default is _MISSING:
            raise ApiError(400, "manca il campo «%s»" % key)
        return default
    if isinstance(v, float) and v.is_integer():
        v = int(v)
    if isinstance(v, bool) or not isinstance(v, int):
        raise ApiError(400, "il campo «%s» deve essere un numero intero" % key)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ApiError(400, "il campo «%s» deve essere tra %s e %s" % (key, lo, hi))
    return v


def _bool(body: dict, key: str, default=_MISSING) -> bool:
    v = body.get(key)
    if v is None and default is not _MISSING:
        return default
    if not isinstance(v, bool):
        raise ApiError(400, "il campo «%s» deve essere true o false" % key)
    return v


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _clean(obj):
    """JSON-safe copy: NaN/inf from the dog's sensors become null."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    return obj


def dog_error(e: BaseException):
    """(HTTP status, Italian message, code) for an exception from the SDK."""
    if isinstance(e, ApiError):
        return e.status, e.error, e.code
    if isinstance(e, DogUnreachable):
        return 502, MSG_UNREACHABLE, None
    if isinstance(e, DogError):
        if e.code is not None:
            return 502, "il cane ha rifiutato il comando: %s" % describe(e.code, "it"), e.code
        return 502, "il cane non ha dato una risposta valida", None
    if isinstance(e, ForbiddenCommand):
        return 400, "comando non consentito dalle regole di sicurezza", None
    if isinstance(e, (ValueError, TypeError)):
        return 400, "valore non valido", None
    return 502, "errore inatteso nel parlare con il cane", None


def sse_frame(event: str, data_json: str) -> bytes:
    return ("event: %s\ndata: %s\n\n" % (event, data_json)).encode("utf-8")


def _dumps(obj) -> str:
    return json.dumps(obj, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


# ---- SSE fan-out ------------------------------------------------------------
class _Sub:
    """One SSE client's queue. put() never blocks: an overflowing client is dropped."""

    def __init__(self, maxsize: int):
        self.maxsize = maxsize
        self.q = collections.deque()
        self.cv = threading.Condition()
        self.dead = False

    def put(self, item) -> None:
        with self.cv:
            if self.dead:
                return
            if len(self.q) >= self.maxsize:
                self.dead = True        # too slow: the handler closes it, EventSource reconnects
                self.q.clear()
            else:
                self.q.append(item)
            self.cv.notify()

    def get(self, timeout: float):
        with self.cv:
            if not self.q and not self.dead:
                self.cv.wait(timeout)
            return self.q.popleft() if self.q else None


class EventHub:
    def __init__(self, maxsize: int = SSE_QUEUE):
        self.maxsize = maxsize
        self.lock = threading.Lock()
        self.subs = set()

    def subscribe(self) -> _Sub:
        s = _Sub(self.maxsize)
        with self.lock:
            self.subs.add(s)
        return s

    def unsubscribe(self, s: _Sub) -> None:
        with self.lock:
            self.subs.discard(s)

    def count(self) -> int:
        with self.lock:
            return len(self.subs)

    def publish(self, event: str, data_json: str) -> None:
        item = (event, sse_frame(event, data_json))
        with self.lock:
            subs = list(self.subs)
        for s in subs:
            s.put(item)


# ---- bounded SDK call -------------------------------------------------------
class _Call:
    """Runs one SDK call on its own thread. wait() gives up after `timeout`; the
    call then goes on and its late outcome becomes a log event, or goes to late(value, error)."""

    def __init__(self, app, label: str, fn, *args, late=None, start=True, **kw):
        self.app, self.label, self.late = app, label, late
        self.lock = threading.Lock()
        self.ev = threading.Event()
        self.done = self.detached = False
        self.value = self.error = None
        self.started, self.finished = clock(), None
        self.read = False           # api_map: a request has returned this outcome
        self._fn, self._args, self._kw = fn, args, kw
        if start:
            self.start()

    def start(self) -> None:
        """start=False callers run this outside App.lock: Thread.start() waits for the new thread."""
        threading.Thread(target=self._run, name="call-" + self.label, daemon=True).start()

    def _run(self) -> None:
        try:
            v, e = self._fn(*self._args, **self._kw), None
        except Exception as ex:     # noqa: BLE001 (reported to the caller or the log)
            v, e = None, ex
        with self.lock:
            # finished before done: api_map reads finished, without the lock, once it sees done
            self.value, self.error, self.finished, self.done = v, e, clock(), True
            detached = self.detached
        self.ev.set()
        if detached:
            if self.late is not None:
                self.late(v, e)
            elif e is None:
                self.app.log("info", "%s: eseguito (in ritardo)" % self.label)
            else:
                self.app.log("error", "%s: %s" % (self.label, dog_error(e)[1]))

    def wait(self, timeout: float) -> bool:
        """True with value/error set; False when still running (now detached)."""
        self.ev.wait(timeout)
        with self.lock:
            if not self.done:
                self.detached = True
                return False
            return True

    def peek(self, timeout: float) -> bool:
        """True once finished. Unlike wait(), a call still running stays attached: no late log."""
        return self.ev.wait(timeout)

    def result(self, timeout: float):
        """(done, value); raises the call's exception when it finished with one."""
        if not self.wait(timeout):
            return False, None
        if self.error is not None:
            raise self.error
        return True, self.value


# ---- camera viewers ---------------------------------------------------------
class CameraHub:
    """/api/camera allows the camera; the SDK grabber runs only while it is allowed
    and at least one /stream.mjpg viewer is connected. All viewers share it."""

    def __init__(self, app):
        self.app = app
        self.lock = threading.Lock()
        self.on = False
        self.viewers = 0
        self.running_on = None      # the client whose camera grabber runs
        self.wake = threading.Event()
        self._thread = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()

    def set_on(self, on: bool) -> None:
        with self.lock:
            self.on = on
        self.wake.set()

    def join(self) -> None:
        with self.lock:
            self.viewers += 1
        self.wake.set()

    def leave(self) -> None:
        with self.lock:
            self.viewers = max(0, self.viewers - 1)
        self.wake.set()

    def state(self) -> dict:
        with self.lock:
            on, viewers, running = self.on, self.viewers, self.running_on
        fps = 0.0
        if running is not None:
            try:
                fps = round(float(running.camera.fps), 1)
            except Exception:
                fps = 0.0
        return {"on": on, "fps": fps, "viewers": viewers}

    def _loop(self) -> None:
        app = self.app
        while not app.stopping.is_set():
            self.wake.wait(0.5)
            self.wake.clear()
            self.sync()
        self._stop_running()

    def sync(self) -> None:
        with self.lock:
            want = self.on and self.viewers > 0
        dog = self.app.dog
        if want and self.running_on is not dog:
            self._stop_running()
            try:
                dog.camera.start()
                self.running_on = dog
                self.app.log("info", "fotocamera avviata")
            except Exception as e:
                self.app.log("error", "fotocamera: %s" % dog_error(e)[1], key="camera-err", every=5.0)
        elif not want and self.running_on is not None:
            self._stop_running()
            self.app.log("info", "fotocamera fermata")

    def _stop_running(self) -> None:
        dog, self.running_on = self.running_on, None
        if dog is not None:
            try:
                dog.camera.stop()
            except Exception:
                pass


# ---- motion jobs ------------------------------------------------------------
class _Job:
    def __init__(self, kind: str, target, label: str):
        self.kind, self.target, self.label = kind, target, label
        self.cancel = threading.Event()     # set by disarm and ESTOP: send no further step
        self.linger_id = None               # after a 321: the motion still running on the dog
        self.away = 0                       # consecutive polls since then showing another motion id


# ---- the app ----------------------------------------------------------------
class App:
    def __init__(self, target: str, dog_factory, ui_path: str = UI_PATH, echo: bool = False,
                 deadman_s: float = DEADMAN_S, poll_s: float = POLL_S, keepalive_s: float = KEEPALIVE_S,
                 quick_s: float = QUICK_S, nav_accept_s: float = NAV_ACCEPT_S,
                 sensor_timeout: float = SENSOR_TIMEOUT, info_every_s: float = INFO_EVERY_S,
                 rebuild_after_s: float = REBUILD_AFTER_S, rebuild_every_s: float = REBUILD_EVERY_S,
                 linger_margin: float = LINGER_MARGIN):
        self.target = target
        self.factory = dog_factory
        self.ui_path = ui_path
        self.echo = echo
        self.deadman_s, self.poll_s, self.keepalive_s = deadman_s, poll_s, keepalive_s
        self.quick_s, self.nav_accept_s, self.sensor_timeout = quick_s, nav_accept_s, sensor_timeout
        self.info_every_s = info_every_s
        self.rebuild_after_s, self.rebuild_every_s = rebuild_after_s, rebuild_every_s
        self.linger_margin = linger_margin
        self.dog = dog_factory()
        self.lock = threading.RLock()
        # dog_lock holders: /api/drive (move/stop), /api/gait, /api/posture and /reset, the stream
        # stop before a motion (worker thread) and before nav goto/dock, and _rebuild. A move()/set()
        # that restarts a stream holds it up to about 0.4 s (join of the closing stream thread plus
        # the Arbiter's 0.15 s SWITCH_PAUSE), a stream stop up to about 0.5 s (frame in flight + END,
        # 0.25 s deadline each). Only those same stream calls wait behind it, and they would wait on
        # the SDK's Arbiter lock anyway. ESTOP, disarm (/api/arm, page load), the dead-man,
        # /api/nav/stop, the queueing of /api/motion and /api/action, and the GET paths never take it,
        # so no safety or status path stalls (tests: test_app_safety *_dog_lock*).
        self.dog_lock = threading.Lock()
        self.stopping = threading.Event()
        self.info_due = threading.Event()
        self.warm_due = threading.Event()
        self.hub = EventHub()
        self.camera = CameraHub(self)
        self.logs = collections.deque(maxlen=200)
        self._log_last = {}
        self._threads = []
        # connection
        self.connected = False
        self.latency_ms = None
        self.emulator = None            # None until 1001 "sn" answered
        self.sn_tries = 0
        self.down_since = clock()
        self.last_rebuild = clock()
        self.rebuilds = 0
        # dog state from the pollers
        self.soc = None
        self.charging = False
        self.docked = False
        self.protected = False
        self.motion_id = None
        self.task_status = None
        self.task_sub = None
        # app state
        self.armed = False
        self.job = None
        self.last_result = None
        self.result_seq = 0             # last_result.seq: +1 on every new result, ESTOP included
        self.gait = "slow"
        self.speed = DEFAULT_SPEED
        self.driving = False
        self.last_drive = 0.0
        self.drive_vel = ZERO
        self.leds = {t: None for t in LED_TARGETS}
        self.skin_mode = None
        self.sensors = {"t": None, "pending": False, "error": None, "bms": {}, "tof": {}, "ultrasonic": {},
                        "odom": {}}
        self.nav_task = None
        self.nav_goal = None
        self.nav_started = 0.0
        self.nav_seen_running = False
        self.nav_starting = 0      # goto/dock requests waiting for the dog's accept
        self.map_call = None       # (dog, _Call) of the 6003 in flight: concurrent /api/map share it
        self.map_logged = None     # ((name, width, height), time) of the last "mappa letta" log line
        self.nav_stopping = False  # a 6009 stop is in flight: a second «Ferma» sends no second one
        self.estop_seq = 0         # +1 on every ESTOP: a goto/dock the dog accepts after one is stopped
        self.nav_stop_seq = 0      # +1 on every /api/nav/stop, the same for «Ferma navigazione»

    # ---- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        for name, fn in (("warm", self._warm_loop), ("poller", self._poll_loop), ("info", self._info_loop),
                         ("deadman", self._deadman_loop), ("broadcast", self._broadcast_loop)):
            th = threading.Thread(target=fn, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        self.camera.start()

    def close(self) -> None:
        self.stopping.set()
        self.info_due.set()
        self.warm_due.set()
        dog = self.dog
        for s in (dog.drive, dog.posture):
            try:
                s.stop()
            except Exception:
                pass
        self.camera.wake.set()
        for th in self._threads:
            th.join(2.0)
        try:
            dog.close()
        except Exception:
            pass

    # ---- logging and events -------------------------------------------------
    def log(self, level: str, msg: str, key=None, every: float = 0.0) -> None:
        """A "log" event. With `key`, at most one per `every` seconds for that key."""
        now = clock()
        if key is not None:
            with self.lock:
                last = self._log_last.get(key)
                if last is not None and now - last < every:
                    return
                self._log_last[key] = now
        item = {"t": round(time.time(), 3), "level": level, "msg": msg}
        self.logs.append(item)
        self.hub.publish("log", _dumps(item))
        if self.echo:
            line = "%s %-5s %s" % (time.strftime("%H:%M:%S"), level, msg)
            try:
                print(line, flush=True)
            except UnicodeEncodeError:
                print(line.encode("ascii", "replace").decode("ascii"), flush=True)

    # ---- state --------------------------------------------------------------
    def snapshot(self) -> dict:
        dog = self.dog
        try:
            active = bool(dog.drive.active)
        except Exception:
            active = False
        try:
            pose = {k: round(float(v), 4) for k, v in dict(dog.posture.target).items() if k in POSE_KEYS}
        except Exception:
            pose = {}
        now = time.time()
        with self.lock:
            for t, v in self.leds.items():
                if v is not None and v["until"] < now:
                    self.leds[t] = None
            vel = self.drive_vel if (active and self.driving) else ZERO
            ts = self.task_status
            s = {
                "connected": self.connected, "target": self.target, "emulator": bool(self.emulator),
                "latency_ms": self.latency_ms, "armed": self.armed,
                "battery": {"soc": self.soc, "charging": self.charging},
                "motion": {"id": self.motion_id, "name": self._motion_name(self.motion_id),
                           "busy": self.job is not None,
                           "last_result": dict(self.last_result) if self.last_result else None},
                "drive": {"active": active, "gait": self.gait, "speed": round(self.speed, 3),
                          "vx": round(vel[0], 4), "vy": round(vel[1], 4), "wz": round(vel[2], 4)},
                "posture": pose,
                "task": {"status": ts, "sub": self.task_sub, "name": TASK_IT.get(ts, "") if ts is not None
                         else ""},
                "leds": {t: (dict(v) if v else None) for t, v in self.leds.items()},
                "skin": {"mode": self.skin_mode},
                "sensors": dict(self.sensors),
                "nav": {"state": self._nav_key(), "task": self.nav_task,
                        "goal": list(self.nav_goal) if self.nav_goal else None},
            }
        s["camera"] = self.camera.state()
        return s

    def state_json(self) -> str:
        return _dumps(self.snapshot())

    @staticmethod
    def _motion_name(mid) -> str:
        if mid is None or mid < 0:
            return ""
        if mid in MOTION_STATE_IT:
            return MOTION_STATE_IT[mid]
        m = catalog.get(mid)
        return m.name_it if m is not None else "movimento %d" % mid

    def _nav_key(self) -> str:
        if not self.connected or self.task_status is None:
            return "unknown"
        key = NAV_KEYS.get(self.task_status, "unknown")
        if key == "idle" and self.docked:
            return "charging"
        return key

    # ---- background loops ---------------------------------------------------
    def _poll_loop(self) -> None:
        fails = 0
        while not self.stopping.is_set():
            t_next = clock() + self.poll_s
            dog = self.dog
            t0 = clock()
            try:
                st = dog.status(timeout=STATUS_TIMEOUT)
            except Exception as e:      # noqa: BLE001
                fails += 1
                self._on_poll_fail(fails, e)
            else:
                fails = 0
                self._on_status(st, int(round((clock() - t0) * 1000)))
            self.stopping.wait(max(0.02, t_next - clock()))

    def _on_status(self, st, latency_ms: int) -> None:
        msgs = []
        with self.lock:
            was = self.connected
            self.connected = True
            self.latency_ms = latency_ms
            self.motion_id = st.motion_id if st.motion_id >= 0 else None
            job = self.job
            if job is not None and job.linger_id is not None:     # waiting for a 321 motion to end
                away = self.motion_id is not None and self.motion_id != job.linger_id
                job.away = job.away + 1 if away else 0
            old_status = self.task_status
            self.task_status = st.task_status if st.task_status >= 0 else None
            self.task_sub = st.task_sub_status
            self.charging = bool(st.wired_charging or st.wireless_charging)
            self.docked = bool(st.wireless_charging)
            protected = st.state == 1
            if protected and not self.protected:
                msgs.append(("warn", "batteria scarica: il cane è in modalità protetta"))
            self.protected = protected
            if self.task_status != old_status and was and self.task_status is not None:
                msgs.append(("info", "navigazione: %s" % TASK_IT.get(self.task_status, "stato %d" % self.task_status)))
            if self.nav_task is not None:
                if RUNNING_TYPE.get(self.task_status) is not None:
                    self.nav_seen_running = True
                elif self.nav_seen_running or clock() - self.nav_started > NAV_GRACE_S:
                    self.nav_task = self.nav_goal = None
        if not was:
            self.sn_tries = 0
            self.info_due.set()
            msgs.insert(0, ("info", "connesso al cane %s (risposta in %d ms)" % (self.target, latency_ms)))
        for level, msg in msgs:
            self.log(level, msg)

    def _on_poll_fail(self, fails: int, e: BaseException) -> None:
        lost = False
        with self.lock:
            if fails >= LOST_AFTER and self.connected:
                self.connected = False
                self.latency_ms = None
                self.motion_id = self.task_status = self.task_sub = None
                self.soc = None
                self.emulator = None    # asked again on reconnect: the target may now be another device
                self.down_since = clock()
                lost = True
            idle = self.job is None or self.job.linger_id is not None     # no 1004 in flight
            rebuild = (not self.connected and idle
                       and clock() - self.down_since >= self.rebuild_after_s
                       and clock() - self.last_rebuild >= self.rebuild_every_s)
        if lost:
            self.log("warn", "connessione con il cane persa: %s" % dog_error(e)[1])
        if rebuild:
            self._rebuild()

    def _rebuild(self) -> None:
        """A fresh client (new channels): gRPC's reconnect backoff grows to 2 minutes,
        so a dog that comes back would otherwise wait that long."""
        try:
            new = self.factory()
        except Exception as e:      # noqa: BLE001
            self.log("error", "impossibile ricreare il client: %s" % e, key="rebuild", every=30.0)
            with self.lock:
                self.last_rebuild = clock()
            return
        with self.dog_lock:
            old = self.dog
            self.dog = new
            try:
                new.drive.set_gait(GAITS[self.gait]["gait"])
            except Exception:
                pass
            with self.lock:
                self.driving = False
                self.drive_vel = ZERO
                self.last_rebuild = clock()
                self.rebuilds += 1
        self.camera.wake.set()
        self.warm_due.set()
        threading.Thread(target=_close_quietly, args=(old,), name="close-old", daemon=True).start()

    def _warm_loop(self) -> None:
        """The SDK connects a channel only when it is first used, and the priority channel carries
        only ESTOPs: without this the first ESTOP of each client waits for TCP + mTLS (about
        0.1 s in the sim). Repeated in case the dog drops an idle connection."""
        while not self.stopping.is_set():
            try:
                self.dog.connect(timeout=CONNECT_TIMEOUT)
            except Exception:       # noqa: BLE001 (the status poller reports the connection)
                pass
            self.warm_due.wait(WARM_EVERY_S)
            self.warm_due.clear()

    def _info_loop(self) -> None:
        while not self.stopping.is_set():
            self.info_due.wait(self.info_every_s)
            self.info_due.clear()
            if self.stopping.is_set():
                return
            with self.lock:
                if not self.connected:
                    continue
                ask_sn = self.emulator is None and self.sn_tries < SN_TRIES
                if ask_sn:
                    self.sn_tries += 1
            flags = ("bat_info", "sn", "wifi") if ask_sn else ("bat_info",)
            try:
                info = self.dog.device_info(*flags, timeout=INFO_TIMEOUT)
            except Exception:       # noqa: BLE001 (the status poller reports the connection)
                continue
            bat = info.get("bat_info") if isinstance(info.get("bat_info"), dict) else {}
            soc = bat.get("power")
            emu = None
            if ask_sn:
                sn, wifi = info.get("sn"), info.get("wifi")
                if isinstance(sn, str) and sn:
                    emu = sn.startswith("EMU")
                elif isinstance(wifi, dict) and wifi.get("name"):
                    emu = wifi.get("name") == "emulator"
            with self.lock:
                if isinstance(soc, int) and not isinstance(soc, bool):
                    self.soc = soc
                if emu is not None:
                    self.emulator = emu
                elif ask_sn and self.sn_tries >= SN_TRIES:
                    self.emulator = False
            if emu:
                self.log("info", "il bersaglio è l'emulatore (numero di serie EMU...)")

    def _deadman_loop(self) -> None:
        while not self.stopping.wait(DEADMAN_TICK):
            with self.lock:
                due = self.driving and clock() - self.last_drive >= self.deadman_s
                if due:
                    self.driving = False
                    self.drive_vel = ZERO
            if due:
                try:
                    self.dog.drive.stop()      # one END; no dog_lock: never waits behind a slow call
                except Exception:
                    pass
                self.log("info", "fermo: nessun comando di guida da %d ms" % int(self.deadman_s * 1000),
                         key="drive-deadman", every=0.0)

    def _broadcast_loop(self) -> None:
        last = None
        period = 1.0 / STATE_HZ
        while not self.stopping.wait(period):
            if not self.hub.count():
                last = None
                continue
            try:
                snap = self.state_json()
            except Exception as e:      # noqa: BLE001
                self.log("error", "stato non serializzabile: %s" % e, key="snap", every=10.0)
                continue
            if snap != last:
                last = snap
                self.hub.publish("state", snap)

    # ---- gate, busy, safety paths -------------------------------------------
    def _gate(self) -> None:
        with self.lock:
            armed = self.armed
        if not armed:
            raise ApiError(403, MSG_GATE, armed=False)

    def _not_busy(self) -> None:
        with self.lock:
            busy = self.job is not None
        if busy:
            raise ApiError(409, MSG_BUSY, busy=True)

    def _navigating(self) -> bool:
        """Autonomous navigation (goto, dock, tracking) runs, or a goto/dock was accepted and the
        poller has not seen it run yet. Mapping and relocalization are driven by hand."""
        with self.lock:
            return (self._nav_key() in STREAM_FREE_NAV or self.nav_task in ("goto", "dock")
                    or self.nav_starting > 0)

    def _not_navigating(self, msg: str = MSG_NAVIGATING) -> None:
        """No 1002 stream (walk or posture, nav_map.md) and no 1004 during autonomous navigation."""
        if self._navigating():
            raise ApiError(409, msg, navigating=True)

    def disarm(self, why: str) -> bool:
        """Gate off; stop the drive and posture streams; no further queued step."""
        with self.lock:
            was = self.armed
            self.armed = False
            job = self.job
            self.driving = False
            self.drive_vel = ZERO
        if job is not None:
            job.cancel.set()     # the step already on the dog finishes; nothing else is sent
        dog = self.dog
        stopped = False
        for s in (dog.drive, dog.posture):
            try:
                if s.active:
                    stopped = True
                s.stop()
            except Exception:
                pass
        if was or stopped or job is not None:
            self.log("info", "movimento disabilitato (%s)" % why)
        return was

    def estop(self) -> dict:
        """ESTOP: gate off, cancel and detach the current motion, then the SDK estop path
        (halts the streams, motion 0 on the priority channel). Takes no lock across the call."""
        with self.lock:
            self.armed = False
            self.estop_seq += 1
            job, self.job = self.job, None
            self.driving = False
            self.drive_vel = ZERO
        if job is not None:
            job.cancel.set()
        dog = self.dog
        t0 = clock()
        try:
            r = dog.estop()
        except Exception as e:      # noqa: BLE001
            status, msg, code = dog_error(e)
            self.log("error", "ARRESTO DI EMERGENZA NON CONFERMATO: %s" % msg)
            raise ApiError(502, "arresto di emergenza non confermato: %s" % msg, code, armed=False)
        ms = int((clock() - t0) * 1000)
        res = self._set_result(self._result(0, r))
        if not r.result:
            self.log("error", "ARRESTO DI EMERGENZA NON CONFERMATO: %s" % res["message"])
            raise ApiError(502, "arresto di emergenza non confermato: %s" % res["message"], r.code,
                           armed=False)
        self.log("warn", "ARRESTO DI EMERGENZA eseguito in %d ms; movimento disabilitato. "
                         "Per rialzarlo usa «alzati»." % ms)
        return {"ok": True, "armed": False, "result": res}

    # ---- motion worker ------------------------------------------------------
    def _queue(self, job: _Job) -> dict:
        with self.lock:
            if not self.armed:
                raise ApiError(403, MSG_GATE, armed=False)
            if self.job is not None:
                raise ApiError(409, MSG_BUSY, busy=True)
            if self._navigating():
                raise ApiError(409, MSG_NAV_MOTION, navigating=True)
            self.job = job
        threading.Thread(target=self._run_job, args=(job,), name="motion-%s" % job.target,
                         daemon=True).start()
        self.log("info", "in esecuzione: %s" % job.label)
        return {"ok": True, "queued": True}

    def _plan(self, job: _Job, dog) -> list:
        """The 1004 steps, with the sit rules of the SDK trick() (motion.cpp:565-583)."""
        if job.kind == "action":
            if job.target != "stand_sit_toggle":
                return [ACTIONS[job.target]]
            mid = dog.status(timeout=STATUS_TIMEOUT).motion_id
            return [111] if (mid in SIT_IDS or mid in LYING_IDS or mid < 0) else [143]
        return trick_steps(job.target, dog.status(timeout=STATUS_TIMEOUT).motion_id, dog.last_motion)

    def _run_job(self, job: _Job) -> None:
        dog = self.dog
        target_id = ACTIONS.get(job.target) if job.kind == "action" else job.target
        result = None
        sent_all = True
        t_sent = clock()
        try:
            steps = self._plan(job, dog)
            for mid in steps:
                if not job.cancel.is_set():
                    self._stop_streams(dog)
                if job.cancel.is_set():
                    sent_all = False         # disarm or ESTOP: no further step goes out
                    break
                t_sent = clock()
                r = dog.motion(mid)          # catalog-checked by the SDK; never allow_risky
                result = self._result(mid, r)
                if not r.result:
                    break
            if not sent_all and (result is None or result["ok"]):
                result = {"id": steps[-1], "ok": False, "code": None,
                          "message": "annullato: movimento disabilitato", "still_running": False}
        except Exception as e:      # noqa: BLE001
            result = {"id": target_id, "ok": False, "code": dog_error(e)[2], "message": dog_error(e)[1],
                      "still_running": False}
        if result.get("still_running"):
            result = self._linger(job, result, t_sent)
            if result is None:
                return                       # ESTOP took over (its result stands), or the app closes
        with self.lock:
            current = self.job is job
            if current:
                self.job = None
                self._set_result(result)
        if not current and (not sent_all or result.get("code") == NOT_SENT_ESTOP):
            self.log("info", "%s: non inviato, arresto di emergenza in corso" % job.label)
        elif not current:
            self.log("info", "%s: risposta arrivata dopo l'arresto (%s)" % (job.label, result["message"]))
        elif result["ok"]:
            self.log("info", "%s: fatto" % job.label)
        elif job.linger_id is not None or result.get("still_running"):
            self.log("warn", "%s: %s" % (job.label, result["message"]))
        else:
            self.log("error", "%s: non riuscito (%s)" % (job.label, result["message"]))

    def _linger(self, job: _Job, result: dict, t_sent: float):
        """After a 321 the motion goes on and later 1004s get 3011 (motion.md, verifier additions):
        busy holds until LEFT_POLLS status polls show another motion id, or until linger_cap_s from
        the 1004 send. Returns the job's final result; None when ESTOP took over meanwhile. A reply
        that arrives after an ESTOP passes straight through."""
        mid = result["id"]
        with self.lock:
            if self.job is not job:
                return result
            job.linger_id, job.away = mid, 0
        cap = linger_cap_s(mid, self.linger_margin)
        self.log("warn", "%s: nessuna conferma dal cane dopo %d s, il movimento continua: attendo che "
                         "finisca (STOP per fermarlo subito)" % (job.label, BRIDGE_CAP_S))
        while not self.stopping.wait(0.05):
            with self.lock:
                if self.job is not job:
                    return None
                away = job.away
            el = clock() - t_sent
            if away >= LEFT_POLLS:
                return {"id": mid, "ok": False, "code": result["code"], "still_running": False,
                        "message": "terminato dopo circa %d s, senza conferma dal cane" % el}
            if el >= cap:
                return {"id": mid, "ok": False, "code": result["code"], "still_running": True,
                        "message": "nessuna notizia dal cane dopo %d s: considero il movimento finito, "
                                   "ma potrebbe essere ancora in corso" % el}
        return None

    def _set_result(self, result: dict) -> dict:
        """Stores a new state.motion.last_result with the next seq; returns a copy of it."""
        with self.lock:
            self.result_seq += 1
            self.last_result = dict(result, seq=self.result_seq)
            return dict(self.last_result)

    def _stop_streams(self, dog) -> None:
        """Drive and posture streams end (END) before a 1004 motion."""
        stopped = False
        with self.dog_lock:
            for s in (dog.drive, dog.posture):
                try:
                    if s.active:
                        s.stop()
                        stopped = True
                except Exception:
                    pass
            with self.lock:
                self.driving = False
                self.drive_vel = ZERO
        if stopped:
            time.sleep(0.15)    # motion.md section 7: about 100 ms after END before the next 1004

    @staticmethod
    def _result(mid: int, r) -> dict:
        if r.result:
            msg = "OK"
        elif r.still_running:
            msg = "il movimento continua oltre 20 s: esito non disponibile"
        elif r.code == -1:
            msg = "nessuna risposta dal cane"
        elif r.code == NOT_SENT_ESTOP:
            msg = "non inviato: arresto di emergenza in corso"
        elif r.code == 0:
            msg = "nessun esito: arresto durante l'alzata automatica"
        else:
            msg = describe(r.code, "it")
        return {"id": mid, "ok": bool(r.result), "code": r.code, "message": msg,
                "still_running": bool(r.still_running)}

    # ---- API: GET -----------------------------------------------------------
    def api_state(self) -> dict:
        return self.snapshot()

    def api_catalog(self) -> dict:
        return CATALOG

    def api_map(self) -> dict:
        """One 6003 shared by every request. A request waits at most quick_s, then answers pending
        while the read goes on: no request holds a browser connection for the whole 6003 (up to
        10 s on the dog), which with two tabs open left ESTOP without a connection."""
        dog, now, new = self.dog, clock(), False
        with self.lock:             # one 6003 at a time: a slow dog gets no pile of them
            mc = self.map_call
            if (mc is None or mc[0] is not dog
                    or (mc[1].done and (mc[1].read or now - mc[1].finished > MAP_KEEP_S))):
                mc = self.map_call = (dog, _Call(self, "mappa", dog.nav.get_map, start=False))
                new = True
        call = mc[1]
        if new:
            try:
                call.start()        # outside the lock: estop() takes it
            except Exception:       # noqa: BLE001 (no thread: the next request makes a new call)
                with self.lock:
                    if self.map_call is mc:
                        self.map_call = None
                raise ApiError(502, "mappa non disponibile: errore inatteso")
        if not call.peek(self.quick_s):
            if clock() - call.started > MAP_WAIT_S:
                raise ApiError(502, "mappa non disponibile: il cane non ha risposto in %d s" % MAP_WAIT_S)
            return {"ok": True, "pending": True}
        call.read = True
        if call.error is not None:
            status, msg, code = dog_error(call.error)
            if code == 321:
                msg = "il servizio mappe del cane non ha risposto in 10 s"
            raise ApiError(502 if status != 400 else 400, "mappa non disponibile: %s" % msg, code)
        m = call.value
        labels = [{"name": n, "x": x, "y": y} for n, x, y in m.labels]
        if m.success != 0:
            msg = MAP_SUCCESS_IT.get(m.success, "mappa non disponibile (codice %d)" % m.success)
            self.log("info", "mappa: %s" % msg, key=("map", m.success), every=MAP_LOG_EVERY_S)
            return {"ok": True, "name": m.name, "success": m.success, "message": msg, "resolution": 0.0,
                    "width": 0, "height": 0, "origin": [0.0, 0.0], "yaw": 0.0, "data": [], "labels": labels}
        key, now = (m.name, m.width, m.height), clock()
        with self.lock:             # every tab reads the map, and again after each pending answer
            fresh = self.map_logged is None or self.map_logged[0] != key or now - self.map_logged[1] > MAP_LOG_EVERY_S
            if fresh:
                self.map_logged = (key, now)
        if fresh:
            self.log("info", "mappa «%s» letta (%d x %d celle)" % (m.name, m.width, m.height))
        return {"ok": True, "name": m.name, "success": 0, "resolution": m.resolution, "width": m.width,
                "height": m.height, "origin": [m.origin[0], m.origin[1]], "yaw": m.yaw,
                "data": list(m.data), "labels": labels}

    # ---- API: POST, safety ----------------------------------------------------
    def api_arm(self, body: dict) -> dict:
        armed = _bool(body, "armed")
        if armed:
            with self.lock:
                was, self.armed = self.armed, True
            if not was:
                self.log("warn", "movimento ABILITATO: il cane può muoversi")
        else:
            self.disarm("interruttore")
        return {"ok": True, "armed": armed}

    def api_estop(self, body: dict) -> dict:
        return self.estop()

    # ---- API: POST, walking ---------------------------------------------------
    def api_drive(self, body: dict) -> dict:
        self._gate()
        x, y, z = (_clamp(_num(body, k, 0.0), -1.0, 1.0) for k in ("x", "y", "z"))
        if x or y or z:                 # an all-zero stop is accepted even during a motion or navigation
            self._not_busy()
            self._not_navigating()
        with self.lock:
            g = GAITS[self.gait]
            level = self.speed
        lo, hi = g["lo"], g["hi"]
        vel = tuple(v * level * (hi[i] if v >= 0 else -lo[i]) for i, v in enumerate((x, y, z)))
        moving = any(v != 0.0 for v in (x, y, z))
        with self.dog_lock:
            self._gate()
            dog = self.dog
            with self.lock:
                was = self.driving
            try:
                if moving:
                    dog.drive.move(*vel)
                else:
                    dog.drive.stop()
            except Exception as e:      # noqa: BLE001
                status, msg, code = dog_error(e)
                raise ApiError(status, "guida: %s" % msg, code)
            with self.lock:
                self.driving = moving
                self.drive_vel = vel if moving else ZERO
                self.last_drive = clock()
                armed = self.armed
        if not armed:
            # a disarm or ESTOP ran meanwhile (they take no dog_lock): end what move() restarted
            try:
                dog.drive.stop()
            except Exception:
                pass
            with self.lock:
                self.driving = False
                self.drive_vel = ZERO
            raise ApiError(403, MSG_GATE, armed=False)
        if moving and not was:
            self.log("info", "in marcia (andatura %s, velocità %d%%)" % (g["name"], round(level * 100)),
                     key="drive", every=0.0)
        elif was and not moving:
            self.log("info", "fermo", key="drive", every=0.0)
        return {"ok": True, "active": moving, "vx": round(vel[0], 4) if moving else 0.0,
                "vy": round(vel[1], 4) if moving else 0.0, "wz": round(vel[2], 4) if moving else 0.0}

    def api_gait(self, body: dict) -> dict:
        key = body.get("gait")
        if not isinstance(key, str) or key not in GAITS:
            raise ApiError(400, "andatura sconosciuta: usa slow, medium, fast o auto")
        g = GAITS[key]
        with self.dog_lock:
            try:
                self.dog.drive.set_gait(g["gait"])
            except Exception as e:      # noqa: BLE001
                status, msg, code = dog_error(e)
                raise ApiError(status, "andatura: %s" % msg, code)
            with self.lock:
                self.gait = key
        self.log("info", "andatura: %s" % g["name"])
        return {"ok": True, "gait": key, "max": list(g["hi"])}

    def api_speed(self, body: dict) -> dict:
        level = round(_clamp(_num(body, "level"), *SPEED_RANGE), 3)
        with self.lock:
            self.speed = level
            g = GAITS[self.gait]
        self.log("info", "velocità: %d%%" % round(level * 100), key="speed", every=1.0)
        return {"ok": True, "speed": level, "max": [round(h * level, 4) for h in g["hi"]]}

    # ---- API: POST, result motions -------------------------------------------
    def api_action(self, body: dict) -> dict:
        self._gate()
        name = body.get("name")
        if not isinstance(name, str) or name not in ACTIONS:
            raise ApiError(400, "azione sconosciuta: usa stand, sit, lie o stand_sit_toggle")
        return self._queue(_Job("action", name, ACTION_IT[name]))

    def api_motion(self, body: dict) -> dict:
        self._gate()
        mid = body.get("id")
        if isinstance(mid, bool) or not isinstance(mid, int):
            raise ApiError(400, "il campo «id» deve essere il numero di un movimento")
        m = APP_MOTIONS.get(mid)
        if m is None:
            raise ApiError(400, "il movimento %d non è disponibile nell'app" % mid)
        confirm = _bool(body, "confirm", False)
        self._not_busy()
        self._not_navigating(MSG_NAV_MOTION)     # before "pavimento libero?": no confirm for a refusal
        if m.risk == "R2" and not confirm:
            raise ApiError(409, MSG_CONFIRM, need_confirm=True, id=m.id, name=m.name_it)
        return self._queue(_Job("motion", mid, m.name_it))

    # ---- API: POST, posture -------------------------------------------------
    def api_posture(self, body: dict) -> dict:
        self._gate()
        vals = {k: _num(body, k) for k in POSE_KEYS if body.get(k) is not None}
        if not vals:
            raise ApiError(400, "indica almeno uno tra z, roll, pitch e yaw")
        return self._posture("postura", lambda dog: dog.posture.set(**vals))

    def api_posture_reset(self, body: dict) -> dict:
        self._gate()
        return self._posture("postura iniziale", lambda dog: dog.posture.reset())

    def _posture(self, label: str, fn) -> dict:
        self._not_busy()
        self._not_navigating()
        with self.dog_lock:
            self._gate()
            dog = self.dog
            try:
                target = fn(dog)            # radians and metres; the SDK clamps
            except Exception as e:      # noqa: BLE001
                status, msg, code = dog_error(e)
                raise ApiError(status, "%s: %s" % (label, msg), code)
            with self.lock:
                self.driving = False        # the Posture stream took over from the Driver
                self.drive_vel = ZERO
                armed = self.armed
        if not armed:
            try:
                dog.posture.stop()
            except Exception:
                pass
            raise ApiError(403, MSG_GATE, armed=False)
        pose = {k: round(float(v), 4) for k, v in dict(target).items() if k in POSE_KEYS}
        self.log("info", "%s: altezza %.3f m, rollio %.2f, beccheggio %.2f, imbardata %.2f rad"
                 % (label, pose.get("z", 0.0), pose.get("roll", 0.0), pose.get("pitch", 0.0),
                    pose.get("yaw", 0.0)), key="posture", every=1.0)
        return {"ok": True, "posture": pose}

    # ---- API: POST, LED, skin, volume, sensors, camera ----------------------------
    def _quick(self, label: str, fn, *args, late=None):
        """Request/response call bounded to quick_s: (done, value) or ApiError."""
        try:
            return _Call(self, label, fn, *args, late=late).result(self.quick_s)
        except ApiError:
            raise
        except Exception as e:      # noqa: BLE001
            status, msg, code = dog_error(e)
            raise ApiError(status, "%s: %s" % (label, msg), code)

    def api_led(self, body: dict) -> dict:
        target = body.get("target")
        if target not in LED_TARGETS:
            raise ApiError(400, "LED sconosciuto: usa head, tail o mini")
        color = body.get("color")
        rgb = None
        if color is not None:
            mt = COLOR_RE.match(color) if isinstance(color, str) else None
            if mt is None:
                raise ApiError(400, "colore non valido: usa #rrggbb oppure null")
            rgb = tuple(int(c, 16) for c in mt.groups())
        effect = body.get("effect", "on")
        if not isinstance(effect, str) or not effect or len(effect) > 40:
            raise ApiError(400, "effetto non valido")
        seconds = _num(body, "seconds", 3.0)
        if seconds <= 0:
            raise ApiError(400, "la durata deve essere maggiore di zero")
        label = "LED %s" % LED_IT[target]
        try:
            done, tid = self._quick(label, self.dog.led, target, rgb, effect, seconds)
        except ApiError as e:
            if e.status == 400:
                raise ApiError(400, "effetto «%s» non valido per il LED %s%s"
                               % (effect, LED_IT[target], "" if rgb is None else " con un colore"))
            raise
        secs = min(max(seconds, 0.1), 600.0)
        with self.lock:
            self.leds[target] = {"color": color.lower() if color else None, "effect": effect,
                                 "seconds": secs, "until": round(time.time() + secs, 3)}
        self.log("info", "%s: %s%s per %g s" % (label, effect, " " + color if color else "", secs))
        return {"ok": True, "task_id": tid} if done else {"ok": True, "pending": True}

    def api_led_off(self, body: dict) -> dict:
        target = body.get("target")
        if target not in LED_TARGETS:
            raise ApiError(400, "LED sconosciuto: usa head, tail o mini")
        label = "LED %s" % LED_IT[target]
        done, tid = self._quick(label, self.dog.led_off, target)
        with self.lock:
            self.leds[target] = None
        self.log("info", "%s: spento" % label)
        return {"ok": True, "task_id": tid} if done else {"ok": True, "pending": True}

    def api_skin(self, body: dict) -> dict:
        mode = _int(body, "mode", lo=0, hi=6)
        wave = _int(body, "wave_ms", 3000, lo=100, hi=600000)
        done, ok = self._quick("pelle elettronica", self.dog.skin, mode, wave)
        if done and not ok:
            raise ApiError(502, "la pelle elettronica non ha accettato il comando")
        with self.lock:
            self.skin_mode = mode
        self.log("info", "pelle elettronica: modo %d" % mode)
        return {"ok": True, "mode": mode} if done else {"ok": True, "mode": mode, "pending": True}

    def api_volume(self, body: dict) -> dict:
        level = _int(body, "level", lo=0, hi=100)
        done, ok = self._quick("volume", self.dog.set_volume, level)
        if done and not ok:
            raise ApiError(502, "il cane non ha accettato il volume")
        self.log("info", "volume: %d" % level, key="volume", every=1.0)
        return {"ok": True, "level": level} if done else {"ok": True, "level": level, "pending": True}

    def api_sensors_refresh(self, body: dict) -> dict:
        with self.lock:
            if self.sensors["pending"]:
                return {"ok": True, "pending": True}
            self.sensors["pending"] = True
            self.sensors["error"] = None
        dog = self.dog
        threading.Thread(target=self._read_sensors, args=(dog,), name="sensors", daemon=True).start()
        self.log("info", "lettura dei sensori avviata")
        return {"ok": True, "pending": True}

    def _read_sensors(self, dog) -> None:
        try:
            r = dog.read_sensors(timeout=self.sensor_timeout)
        except Exception as e:      # noqa: BLE001
            if isinstance(e, DogError) and not isinstance(e, DogUnreachable):
                msg = ("il cane non ha restituito i sensori entro %d s (il motore di programmazione "
                       "visiva è attivo?)" % self.sensor_timeout)
            else:
                msg = dog_error(e)[1]
            with self.lock:
                self.sensors["pending"] = False
                self.sensors["error"] = msg
            self.log("error", "sensori: %s" % msg)
            return
        r = _clean(r if isinstance(r, dict) else {})
        with self.lock:
            for k in ("bms", "tof", "ultrasonic", "odom"):
                self.sensors[k] = r.get(k) if isinstance(r.get(k), dict) else {}
            self.sensors["t"] = round(time.time(), 3)
            self.sensors["pending"] = False
            self.sensors["error"] = None
        self.log("info", "sensori aggiornati")

    def api_camera(self, body: dict) -> dict:
        on = _bool(body, "on")
        self.camera.set_on(on)
        self.log("info", "fotocamera %s" % ("consentita" if on else "spenta"))
        return {"ok": True, "on": on}

    # ---- API: POST, navigation ------------------------------------------------
    def api_nav_start(self, body: dict) -> dict:
        self._gate()
        task = body.get("task")
        if task not in NAV_TASKS:
            raise ApiError(400, "compito sconosciuto: usa mapping, relocalize, goto o dock")
        goal = None
        if task == "goto":
            if body.get("x") is None or body.get("y") is None:
                raise ApiError(400, "per «vai al punto» servono x e y in metri sulla mappa")
            goal = (_num(body, "x"), _num(body, "y"))
        moving = task in ("goto", "dock")
        with self.lock:
            if not self.armed:          # checked again here: an ESTOP after _gate() is in seq0 or refused
                raise ApiError(403, MSG_GATE, armed=False)
            if self.job is not None:    # a motion runs, or a 321 wait still holds busy
                raise ApiError(409, MSG_BUSY, busy=True)
            before = self._nav_key()
            seqs = (self.estop_seq, self.nav_stop_seq)
            if moving:
                self.nav_starting += 1  # _queue refuses a motion until the dog has answered
        try:
            return self._nav_start(task, goal, before, seqs)
        finally:
            if moving:
                with self.lock:
                    self.nav_starting -= 1

    def _nav_start(self, task: str, goal, before: str, seqs) -> dict:
        dog = self.dog
        seq0, stop0 = seqs
        if task in ("goto", "dock"):
            self._stop_streams(dog)     # no 1002 during autonomous navigation (nav_map.md 4)
        try:
            if task == "mapping":
                nt = dog.nav.start_mapping()
            elif task == "relocalize":
                nt = dog.nav.relocalize()
            elif task == "goto":
                nt = dog.nav.go_to(goal[0], goal[1])
            else:
                nt = dog.nav.dock()
        except Exception as e:      # noqa: BLE001
            status, msg, code = dog_error(e)
            raise ApiError(status, "%s: %s" % (NAV_TASK_IT[task], msg), code)
        end = clock() + self.nav_accept_s
        while nt.accepted is None and nt.error is None and not nt.done.is_set() and clock() < end:
            time.sleep(0.02)
        label = NAV_TASK_IT[task]
        if nt.accepted == 1 and task in ("goto", "dock"):
            with self.lock:
                why = ("STOP" if self.estop_seq != seq0 else
                       "«Ferma navigazione»" if self.nav_stop_seq != stop0 else None)
            if why is not None:
                # an ESTOP or a nav stop (from any tab) came while the dog decided: the accepted task
                # must not walk off, and nav_task stays unset so drive and motions are not refused
                self.log("warn", "%s: il cane l'ha accettato dopo %s, lo fermo" % (label, why))
                outcome = self._stop_late_accept(dog, task, label)
                raise ApiError(409, "«%s» annullato da %s: il cane l'aveva accettato e %s"
                               % (label, why, outcome), None, cancelled=True)
        if nt.accepted == 1:
            with self.lock:
                self.nav_task = task
                self.nav_goal = goal
                self.nav_started = clock()
                self.nav_seen_running = False
            self.log("info", "%s: accettato%s" % (label, "" if goal is None else " (x %.2f, y %.2f)" % goal))
            return {"ok": True, "accepted": 1}
        if nt.accepted is not None:
            hint = ""
            if task == "goto" and before not in ("localized", "navigating"):
                hint = ": prima rilocalizza il cane sulla mappa"
            elif task != "goto" and before not in ("idle", "failed", "charging", "unknown"):
                hint = ": un altro compito è in corso, fermalo prima"
            raise ApiError(502, "il cane ha rifiutato «%s»%s" % (label, hint), None, accepted=nt.accepted)
        if isinstance(nt.error, int):
            raise ApiError(502, "%s: %s" % (label, describe(nt.error, "it")), nt.error)
        if nt.error is not None:
            raise ApiError(502, "%s: %s" % (label, MSG_UNREACHABLE))
        if nt.done.is_set():
            raise ApiError(502, "%s: il cane ha chiuso la richiesta senza risposta" % label)
        raise ApiError(502, "%s: nessuna risposta dal cane entro %d s" % (label, self.nav_accept_s))

    def api_nav_save_map(self, body: dict) -> dict:
        self._gate()
        name = body.get("name")
        if not isinstance(name, str) or not MAP_NAME_RE.match(name.strip()):
            raise ApiError(400, "nome della mappa non valido: 1-32 caratteri tra lettere senza accenti, "
                                "cifre, _ e -")
        name = name.strip()
        with self.lock:
            status = self.task_status
        if status not in (5, 15):
            # 6009 type 5 outside mapping answers a fake success (nav_map.md 6009)
            raise ApiError(409, "nessuna mappa in costruzione: avvia prima la creazione della mappa")
        try:
            nt = self.dog.nav.save_map(name)
        except Exception as e:      # noqa: BLE001
            status, msg, code = dog_error(e)
            raise ApiError(status, "salvataggio della mappa: %s" % msg, code)
        if not nt.wait(self.nav_accept_s):
            threading.Thread(target=self._save_outcome, args=(nt, name), name="save-map", daemon=True).start()
            self.log("info", "salvataggio della mappa «%s» in corso" % name)
            return {"ok": True, "accepted": 1, "pending": True}
        self._save_check(nt, name)
        return {"ok": True, "accepted": 1, "result": 0}

    def _save_check(self, nt, name: str) -> None:
        if isinstance(nt.error, int):
            raise ApiError(502, "salvataggio della mappa: %s" % describe(nt.error, "it"), nt.error)
        if nt.error is not None:
            raise ApiError(502, "salvataggio della mappa: %s" % MSG_UNREACHABLE)
        r = nt.result
        if r != 0:
            msg = {1: "non riuscito (verifica la mappa: le etichette potrebbero mancare)",
                   10: "il cane non ha finito entro 60 s"}.get(r, "esito sconosciuto (%r)" % (r,))
            raise ApiError(502, "salvataggio della mappa: %s" % msg, r if isinstance(r, int) else None)
        self.log("info", "mappa «%s» salvata" % name)

    def _save_outcome(self, nt, name: str) -> None:
        nt.wait(70.0)
        try:
            self._save_check(nt, name)
        except ApiError as e:
            self.log("error", e.error)

    def _stop_late_accept(self, dog, task: str, label: str) -> str:
        """Stop a goto/dock the dog accepted after a stop; returns the end of the 409 text. A stop
        still running is waited for first: a 6009 that meets its ResetNav (status 103) answers FAILED.
        The stop is always sent, since 10001 reads a cache that can still show 101 for a goto that has
        just started. Its type comes from the state when the state shows that task, else 0 for a goto
        (no feedback 300 yet, nav.AB_GOAL_SENT) and 9 for a dock. A FAILED answer is checked against the state: the stop
        before may already have ended the task (no stop type), and then nothing is wrong."""
        failed = "non sono riuscito a fermarlo: premi «Ferma navigazione»"
        end = clock() + NAV_STOP_WAIT_S
        while True:
            with self.lock:
                if not self.nav_stopping:
                    self.nav_stopping = True
                    break
            if clock() >= end:
                self.log("error", "%s: non riesco a fermarlo, l'arresto precedente è ancora in corso: premi "
                                  "«Ferma navigazione»" % label)
                return failed
            time.sleep(0.05)
        stop_type = 0 if task == "goto" else 9
        ns = self._nav_state_or_none(dog)
        if ns is not None and ns.task_type == (1 if task == "goto" else 9) and ns.stop_type is not None:
            stop_type = ns.stop_type    # a goto past feedback 300 needs type 1
        try:
            done, r = self._send_nav_stop(dog, stop_type)
        except ApiError as e:
            self.log("error", "%s: non riesco a fermarlo: %s" % (label, e.error))
            return failed
        if done and r == 1:
            ns = self._nav_state_or_none(dog)
            if ns is not None and ns.stop_type is None and not ns.wedged:
                self.log("info", "%s: l'arresto precedente l'aveva già fermato" % label)
                return "l'arresto precedente l'ha fermato"
            self.log("error", "%s: il cane non è riuscito a fermarlo: premi «Ferma navigazione»" % label)
            return failed
        return "l'ho fermato" if done else "ho inviato l'arresto"

    def _nav_state_or_none(self, dog):
        try:
            return dog.nav.state(timeout=min(1.0, STATUS_TIMEOUT))
        except Exception:           # noqa: BLE001
            return None

    def api_nav_stop(self, body: dict) -> dict:
        with self.lock:
            self.nav_stop_seq += 1  # a goto/dock accepted from here on is stopped by _nav_start
            # a second «Ferma» while a stop runs sends no second 6009: a type 0 that meets the first
            # one's ResetNav (status 103) answers FAILED at once (algorithm_task_manager.cpp:197-208)
            if self.nav_stopping:
                return {"ok": True, "stopped": True, "pending": True}
            self.nav_stopping = True
        handed = False              # once the stop call runs, it clears nav_stopping when it ends
        try:
            dog = self.dog
            try:
                ns = dog.nav.state(timeout=min(1.0, STATUS_TIMEOUT))
            except Exception:       # noqa: BLE001 (fall back to the poller's view)
                with self.lock:
                    ts, sub = self.task_status, self.task_sub
                if ts is None:
                    raise ApiError(502, "navigazione: %s" % MSG_UNREACHABLE)
                ns = decode_state(ts, sub or 0)
            if ns.wedged:
                raise ApiError(502, TASK_IT[0])
            if ns.stop_type is None:
                self.log("info", "navigazione: nessun compito da fermare")
                with self.lock:
                    self.nav_task = self.nav_goal = None
                return {"ok": True, "stopped": False}
            if ns.status in (5, 15):
                self.log("warn", "creazione della mappa interrotta: la mappa non viene salvata")
            if ns.task_type == 1 and ns.stop_type == 0:
                self.log("warn", "viaggio fermato prima della partenza: il cane deve ritrovare la posizione")

            handed = True
            done, r = self._send_nav_stop(dog, ns.stop_type)
            if done and r == 1:
                raise ApiError(502, "il cane non è riuscito a fermare il compito", 1)
            with self.lock:
                self.nav_task = self.nav_goal = None
            self.log("info", "navigazione fermata" if done else "arresto della navigazione inviato")
            return {"ok": True, "stopped": True, "result": r} if done else {"ok": True, "stopped": True,
                                                                             "pending": True}
        finally:
            if not handed:
                with self.lock:
                    self.nav_stopping = False

    def _send_nav_stop(self, dog, stop_type: int):
        """One 6009 on its own thread, (done, result) within quick_s. The caller has set nav_stopping;
        the call clears it when it ends, and a result after quick_s goes to _nav_stop_late."""
        ran = threading.Event()

        def stop():
            ran.set()
            try:
                return dog.nav.stop(stop_type)
            finally:
                with self.lock:
                    self.nav_stopping = False
        try:
            return self._quick("ferma navigazione", stop, late=self._nav_stop_late)
        except ApiError:
            if not ran.is_set():    # the thread never started, so stop() cannot clear the flag
                with self.lock:
                    self.nav_stopping = False
            raise

    def _nav_stop_late(self, r, e) -> None:
        """The outcome of a nav stop that answered after quick_s: a type 0 runs ResetNav (2 s or more)."""
        if e is not None:
            self.log("error", "ferma navigazione: %s" % dog_error(e)[1])
        elif r == 0:
            self.log("info", "navigazione fermata")
        elif r == 1:
            ns = self._nav_state_or_none(self.dog)
            if ns is not None and ns.stop_type is None and not ns.wedged:
                self.log("info", "navigazione fermata: nessun compito in corso")
            else:
                self.log("error", "il cane non è riuscito a fermare il compito: controlla lo stato e premi di "
                                  "nuovo «Ferma navigazione»")
        elif r == 10:
            self.log("warn", "l'arresto della navigazione è ancora in corso: controlla lo stato del cane")
        else:
            self.log("warn", "ferma navigazione: esito sconosciuto (%r)" % (r,))


def _close_quietly(dog) -> None:
    try:
        dog.close()
    except Exception:
        pass


# ---- HTTP -------------------------------------------------------------------
GET_ROUTES = {"/": "ui", "/api/state": "state", "/api/events": "events", "/api/catalog": "catalog",
              "/api/map": "map", "/stream.mjpg": "mjpeg"}
POST_ROUTES = {"/api/arm": "api_arm", "/api/estop": "api_estop", "/api/drive": "api_drive",
               "/api/gait": "api_gait", "/api/speed": "api_speed", "/api/action": "api_action",
               "/api/motion": "api_motion", "/api/posture": "api_posture",
               "/api/posture/reset": "api_posture_reset", "/api/led": "api_led", "/api/led/off": "api_led_off",
               "/api/skin": "api_skin", "/api/sensors/refresh": "api_sensors_refresh",
               "/api/camera": "api_camera", "/api/volume": "api_volume", "/api/nav/start": "api_nav_start",
               "/api/nav/save_map": "api_nav_save_map", "/api/nav/stop": "api_nav_stop"}


class Handler(BaseHTTPRequestHandler):
    server_version = "CyberDogApp/2"
    sys_version = ""

    def log_message(self, fmt, *args):     # the app logs commands itself
        pass

    @property
    def app(self) -> App:
        return self.server.app

    # ---- checks -------------------------------------------------------------
    def _hosts(self):
        port = self.server.server_address[1]
        return ("127.0.0.1:%d" % port, "localhost:%d" % port)

    def _host_ok(self) -> bool:
        return (self.headers.get("Host") or "").strip().lower() in self._hosts()

    def _origin_ok(self) -> bool:
        origin = self.headers.get("Origin")
        return origin is None or origin.strip().lower() in tuple("http://" + h for h in self._hosts())

    # ---- replies ------------------------------------------------------------
    def _send(self, status: int, ctype: str, body: bytes, headers=()) -> None:
        try:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in headers:
                self.send_header(k, v)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)
        except OSError:
            pass        # the client went away

    def _json(self, status: int, obj, headers=()) -> None:
        self._send(status, "application/json; charset=utf-8", _dumps(obj).encode("utf-8"), headers)

    def _error(self, status: int, msg: str, code=None, headers=(), **extra) -> None:
        self._json(status, ApiError(status, msg, code, **extra).body(), headers)

    def _not_allowed(self, path: str) -> None:
        allow = "GET" if path in GET_ROUTES else "POST" if path in POST_ROUTES else None
        if allow is None:
            self._error(404, "percorso sconosciuto")
        else:
            self._error(405, "metodo non consentito: usa %s" % allow, headers=(("Allow", allow),))

    # ---- methods ------------------------------------------------------------
    def do_GET(self):
        path = urlsplit(self.path).path
        if not self._host_ok():
            return self._error(403, "host non consentito: apri http://127.0.0.1:%d/" % self.server.server_address[1])
        route = GET_ROUTES.get(path)
        if route is None:
            return self._not_allowed(path)
        try:
            if route == "ui":
                return self._ui()
            if route == "events":
                return self._events()
            if route == "mjpeg":
                return self._mjpeg()
            out = {"state": self.app.api_state, "catalog": self.app.api_catalog, "map": self.app.api_map}[route]()
            self._json(200, out)
        except ApiError as e:
            self.app.log("warn" if e.status < 500 else "error", "%s: %s" % (path, e.error),
                         key=(path, e.status), every=1.0)
            self._json(e.status, e.body())
        except Exception as e:      # noqa: BLE001
            self.app.log("error", "errore interno su %s: %s" % (path, e), key=(path, 500), every=1.0)
            self._error(500, "errore interno del server")

    def do_POST(self):
        path = urlsplit(self.path).path
        raw = self._read_body()      # always drained, so an early answer is not cut by a reset
        if not self._host_ok():
            return self._error(403, "host non consentito")
        if not self._origin_ok():
            return self._error(403, "origine non consentita: comandi accettati solo dalla pagina dell'app")
        ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
        if ctype != "application/json":
            return self._error(415, "il corpo deve essere JSON (Content-Type: application/json)")
        name = POST_ROUTES.get(path)
        if name is None:
            return self._not_allowed(path)
        app = self.app
        try:
            if name == "api_estop":
                body = {}               # ESTOP takes no fields: a bad or oversize body never stops it
            elif raw is None:
                raise ApiError(400, "corpo della richiesta troppo grande o illeggibile")
            else:
                body = self._parse(raw)
            out = getattr(app, name)(body)
            self._json(200, out)
        except ApiError as e:
            app.log("warn" if e.status < 500 else "error", "rifiutato %s: %s" % (path, e.error),
                    key=(path, e.status), every=1.0)
            self._json(e.status, e.body())
        except Exception as e:      # noqa: BLE001
            app.log("error", "errore interno su %s: %s" % (path, e), key=(path, 500), every=1.0)
            self._error(500, "errore interno del server")

    def _method_not_allowed(self):
        self._not_allowed(urlsplit(self.path).path)

    do_PUT = do_DELETE = do_PATCH = do_OPTIONS = do_HEAD = _method_not_allowed

    def _read_body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return None
        if n < 0:
            return None
        try:
            if n > MAX_BODY:
                # read and drop it (up to DRAIN_MAX), so the 400 is not lost to a connection reset
                left = min(n, DRAIN_MAX)
                while left > 0:
                    chunk = self.rfile.read(min(left, 65536))
                    if not chunk:
                        break
                    left -= len(chunk)
                return None
            return self.rfile.read(n) if n else b""
        except OSError:
            return None

    @staticmethod
    def _parse(raw: bytes) -> dict:
        if not raw.strip():
            return {}

        def no_constant(c):
            raise ValueError(c)
        try:
            body = json.loads(raw.decode("utf-8"), parse_constant=no_constant)
        except (ValueError, UnicodeDecodeError, RecursionError):    # RecursionError: nesting too deep
            raise ApiError(400, "JSON non valido") from None
        if not isinstance(body, dict):
            raise ApiError(400, "il corpo deve essere un oggetto JSON")
        return body

    # ---- GET bodies ---------------------------------------------------------
    def _ui(self):
        self.app.disarm("pagina caricata")      # contracts.md 1.10: gate forced off on every page load
        try:
            with open(self.app.ui_path, "rb") as f:
                body = f.read()
        except OSError:
            body = PLACEHOLDER.encode("utf-8")
        self._send(200, "text/html; charset=utf-8", body,
                   (("Content-Security-Policy", "frame-ancestors 'none'"), ("X-Frame-Options", "DENY")))

    def _events(self):
        app = self.app
        sub = app.hub.subscribe()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.connection.settimeout(WRITE_TIMEOUT)   # a stuck client fails its write and is dropped
            sent = sse_frame("state", app.state_json())
            self.wfile.write(b"retry: 2000\n\n" + sent)
            period = 1.0 / STATE_HZ
            last_state = clock()
            pending = None              # newest state frame held back by the 5-per-second limit
            next_ka = last_state + app.keepalive_s
            while not app.stopping.is_set() and not sub.dead:
                now = clock()
                wait = min(0.5, next_ka - now)
                if pending is not None:
                    wait = min(wait, last_state + period - now)
                item = sub.get(timeout=max(0.0, wait))
                if item is not None:
                    if item[0] == "state":
                        pending = item[1] if item[1] != sent else None    # only on change
                    else:
                        self.wfile.write(item[1])
                if pending is not None and clock() >= last_state + period:
                    self.wfile.write(pending)
                    sent, pending, last_state = pending, None, clock()
                if clock() >= next_ka:
                    self.wfile.write(b": keepalive\n\n")
                    next_ka = clock() + app.keepalive_s
        except OSError:
            pass
        finally:
            app.hub.unsubscribe(sub)

    def _client_gone(self) -> bool:
        try:
            r, _, _ = select.select([self.connection], [], [], 0)
            return bool(r) and self.connection.recv(1, socket.MSG_PEEK) == b""
        except (OSError, ValueError):
            return True

    def _mjpeg(self):
        app = self.app
        hub = app.camera
        if not hub.on:
            return self._error(409, "fotocamera spenta: accendila prima")
        hub.join()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.connection.settimeout(WRITE_TIMEOUT)
            cam_obj, last = None, 0
            while hub.on and not app.stopping.is_set():
                cam = app.dog.camera
                if cam is not cam_obj:
                    cam_obj, last = cam, 0
                fr = cam.wait(last, timeout=0.5)
                if self._client_gone():
                    break
                if fr is None or fr[0] == last:
                    continue
                last, jpeg = fr
                self.wfile.write(b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                 % (BOUNDARY.encode("ascii"), len(jpeg)) + jpeg + b"\r\n")
        except OSError:
            pass
        finally:
            hub.leave()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"     # on Windows SO_REUSEADDR lets two servers share a port
    request_queue_size = 32

    def __init__(self, addr, app: App):
        self.app = app
        super().__init__(addr, Handler)


def make_server(app: App, bind: str = DEFAULT_BIND, port: int = DEFAULT_PORT) -> Server:
    return Server((bind, port), app)


def parse_target(s: str):
    s = s.strip()
    if s.startswith("["):
        host, _, rest = s[1:].partition("]")
        port = rest[1:] if rest.startswith(":") else ""
    else:
        host, _, port = s.rpartition(":")
    if not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise argparse.ArgumentTypeError("usa host:porta, per esempio 127.0.0.1:50052")
    return host, int(port)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="app2/server.py", description="CyberDog 2: server dell'app (v2)")
    ap.add_argument("--dog", type=parse_target, default=parse_target(DEFAULT_DOG),
                    help="host:porta del cane o dell'emulatore (predefinito %s)" % DEFAULT_DOG)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="porta HTTP (predefinita %d)" % DEFAULT_PORT)
    ap.add_argument("--bind", default=DEFAULT_BIND, help="indirizzo di ascolto (predefinito %s)" % DEFAULT_BIND)
    ap.add_argument("--quiet", action="store_true", help="non stampare il registro dei comandi")
    a = ap.parse_args(argv)
    host, port = a.dog
    target = "%s:%d" % (host, port)
    try:
        app = App(target, lambda: CyberDog(host, port), echo=not a.quiet)
    except Exception as e:      # noqa: BLE001
        print("ERRORE: impossibile preparare il collegamento con il cane: %s" % e, file=sys.stderr)
        return 1
    try:
        httpd = make_server(app, a.bind, a.port)
    except OSError as e:
        print("ERRORE: impossibile aprire %s:%d (la porta è già in uso?): %s" % (a.bind, a.port, e),
              file=sys.stderr)
        app.close()
        return 1
    app.start()
    print("CyberDog app v2 su http://127.0.0.1:%d/ (cane: %s)" % (httpd.server_address[1], target), flush=True)
    try:
        httpd.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
