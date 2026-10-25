"""CyberDog: the high-level client (contracts.md §3)."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, NamedTuple, Optional

from . import motions as catalog
from . import vp
from .camera import Camera
from .codes import describe
from .nav import Nav
from .servo import ESTOP_LATE_S, WATCHDOG_S, Arbiter, Driver, Posture, clock
from .transport import (ARRAY_SIZES, DEFAULT_HOST, DEFAULT_PORT, DogUnreachable, ForbiddenCommand,
                        RiskyMotion, Transport, error_code)

MOTION_DEADLINE = 25.0      # 1004 waits 3 s for the service + 20 s for the reply (grpc_fidelity.md §7.3)
ESTOP_TIMEOUT = 3.0
LONG_MOTION_S = 20.0        # longer motions make 1004 return 321 while they keep running
SIT_IDS = frozenset((143, 214, 148, 149, 150))     # status read as sitting; 214 also follows every mode-62 trick
SIT_PREV = frozenset((140, 143, 148, 149, 150))    # motions that leave the dog sitting (VP SDK motion.cpp:566-571)
SIT_SWING = frozenset((148, 149, 150))
NO_STAND_FIRST = frozenset((148, 149, 150, 111, 101, 0))
# duration 0 is never popped and adds rpy_des/pos_des again every 2 s: the body keeps tilting
# (fsm_state_balancestand.cpp:338-396, command_interface.cpp:222)
ENDLESS_AT_0 = frozenset((202,))
# MotionResult.code of a 1004 the SDK did not send because estop() ran: a 111 sent then would
# clear the dog's ESTOP latch (motion.md §2.4)
NOT_SENT_ESTOP = -2
# 1001 flags; is_uid is never sent: the bridge reads it into is_version (app.cpp:2705)
DEVICE_FLAGS = ("sn", "version", "nick_name", "volume", "mic_state", "voice_control", "wifi",
                "bat_info", "motor_temper", "audio_state", "device_model", "stand", "lowpower_control")
DEFAULT_DEVICE_FLAGS = ("bat_info", "stand", "wifi")


class MotionResult(NamedTuple):
    motion_id: int
    result: bool
    code: int
    message: str
    still_running: bool


class Status(NamedTuple):
    motion_id: int
    task_status: int
    task_sub_status: int
    state: int
    state_code: int
    wired_charging: bool
    wireless_charging: bool
    self_check_code: int
    raw: Dict[str, Any]


def _int(v, default: int) -> int:
    return v if isinstance(v, int) and not isinstance(v, bool) else default


# estop() helpers: what cuts a step short goes to caught, for estop() to raise after both ESTOPs
SHIELD_TRIES = 3            # a step cut short by a Ctrl+C runs again, up to this many times in all


def _shielded(caught: list, fn):
    """fn(), run again when a Ctrl+C cuts it short; a failure gives None."""
    for _ in range(SHIELD_TRIES):
        try:
            return fn()
        except Exception as e:
            caught.append(e)
            return None
        except BaseException as e:
            caught.append(e)
    return None


def _sleep_until(t: float, caught: list) -> None:
    while clock() < t:
        try:
            time.sleep(max(0.0, t - clock()))
        except BaseException as e:
            caught.append(e)


def _join(th: threading.Thread, until: float, caught: list) -> None:
    while th.is_alive() and clock() < until:
        try:
            th.join(max(0.0, until - clock()))
        except BaseException as e:
            caught.append(e)


def check_risk(motion_id: int, allow_risky: bool = False) -> Optional[catalog.Motion]:
    """Catalog gate shared by motion() and trick(); returns the catalog entry (None if unknown)."""
    m = catalog.get(motion_id)
    if m is None:
        if not allow_risky:
            raise RiskyMotion("motion %r is not in the catalog; pass allow_risky=True" % (motion_id,))
        return None
    if "never" in m.needs:
        raise ForbiddenCommand("motion %d (%s) cannot be sent through the bridge" % (m.id, m.name_en))
    if m.risk in ("R3", "R4") and not allow_risky:
        raise RiskyMotion("motion %d (%s) is %s; pass allow_risky=True" % (m.id, m.name_en, m.risk))
    return m


def trick_steps(motion_id: int, status_id: int, last_motion: Optional[int] = None) -> List[int]:
    """The 1004 steps of trick(), from the VP SDK sit rules (motion.cpp:565-583) on the safe side.

    Sitting means status_id in SIT_IDS. The dog also reports 214 after every mode-62 trick (the
    21/3 filler, sim_bridge.md 1.2), so a trick after a non-sit trick gets an extra 111 (about
    2 s); that is kept. For 148-150 the same reading would skip the 143 they need, so at status
    214 they get 143 first when last_motion (the client's last motion with result true) is known
    and not in SIT_PREV. last_motion None (unknown) keeps the status-only rule.
    """
    sitting = status_id in SIT_IDS
    if motion_id in SIT_SWING:
        stood_up = status_id == 214 and last_motion is not None and last_motion not in SIT_PREV
        return [143, motion_id] if (not sitting or stood_up) else [motion_id]
    if sitting and motion_id not in NO_STAND_FIRST:
        return [111, motion_id]
    return [motion_id]


class CyberDog:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, cert_dir: Optional[str] = None,
                 transport: Optional[Transport] = None):
        self.transport = transport if transport is not None else Transport(host, port, cert_dir)
        arbiter = Arbiter()
        self.drive = Driver(self.transport, arbiter)
        self.posture = Posture(self.transport, arbiter)
        self.camera = Camera(self.transport)
        self.nav = Nav(self.transport)
        self._vp = vp.Slot(self.transport)     # the dog keeps one VP debug task: every 2002 goes through here
        self._estops = 0                        # ESTOPs sent by this client
        self._last = (0, None)                  # (_estops when it was sent, id) of the last motion with result true

    def connect(self, timeout: float = 5.0) -> None:
        self.transport.connect(timeout)

    def close(self) -> None:
        for ev in list(self.drive.arbiter.estopping):   # let a running estop() send its repeat
            ev.wait(2 * ESTOP_TIMEOUT + 1.5)
        for s in (self.drive, self.posture):
            try:
                s.close()       # END if streaming; a move()/set() racing close() never starts
            except Exception:
                pass
        self.camera.close()
        self.transport.close()

    # ---- status and info --------------------------------------------------------
    def status(self, timeout: float = 3.0) -> Status:
        r = self.transport.call_one(10001, {}, timeout=timeout)
        ms = r.get("motion_status") or {}
        ts = r.get("task_status") or {}
        sc = r.get("self_check_status") or {}
        ss = r.get("state_switch_status") or {}
        cs = r.get("charging_status") or {}
        return Status(_int(ms.get("motion_id"), -1), _int(ts.get("task_status"), -1),
                      _int(ts.get("task_sub_status"), 0), _int(ss.get("state"), -1),
                      _int(ss.get("code"), 0), bool(cs.get("wired_charging", False)),
                      bool(cs.get("wireless_charging", False)), _int(sc.get("code"), -1), r)

    def device_info(self, *flags: str, timeout: float = 15.0) -> dict:
        names = [f[3:] if f.startswith("is_") else f for f in (flags or DEFAULT_DEVICE_FLAGS)]
        bad = [f for f in names if f not in DEVICE_FLAGS]
        if bad:
            raise ValueError("unknown 1001 flags %s (uid is never sent)" % bad)
        if "version" in names:
            timeout = max(timeout, 30.0)   # version: up to 13 s more on the dog (grpc_fidelity.md §7.1)
        return self.transport.call_one(1001, {"is_" + f: True for f in names}, timeout=timeout)

    def set_volume(self, level: int) -> bool:
        level = max(0, min(100, int(level)))     # the dog truncates to uint8
        # 1010-1014 can block on the dog after their 3 s wait: keep a 10 s deadline
        return bool(self.transport.call_one(1012, {"volume": level}, timeout=10.0).get("success"))

    # ---- result motions (1004) --------------------------------------------------
    def _run_motion(self, params: dict, timeout: float, priority: bool = False) -> MotionResult:
        mid = params["motion_id"]
        if mid == 0:            # ESTOP: the previous motion no longer counts, whatever the reply
            self._estops += 1
            self._last = (self._estops, 0)
        gen = self._estops
        items = self.transport.call_json(1004, params, timeout=timeout, priority=priority)
        if not items:
            return MotionResult(mid, False, -1, "no reply", False)
        r = items[0]
        code = error_code(r)
        if code is not None:
            m = catalog.get(mid)
            long_motion = m is not None and ((m.typical_s or 0.0) > LONG_MOTION_S
                                             or m.min_ms >= LONG_MOTION_S * 1000)
            return MotionResult(mid, False, code, describe(code, "en"), code == 321 and long_motion)
        code = _int(r.get("code"), -1)
        ok = r.get("result") is True
        if ok and mid != 0:
            self._last = (gen, mid)
        # result false with code 0: the manager left the reply unfilled, after an ESTOP during the
        # 111 it inserts (motion.md §2.5)
        msg = "no result (ESTOP during the stand first)" if not ok and code == 0 else describe(code, "en")
        return MotionResult(_int(r.get("motion_id"), mid), ok, code, msg, False)

    def _not_sent(self, motion_id: int, since: int) -> Optional[MotionResult]:
        """A refusal when estop() is running or an ESTOP went out after the caller read since."""
        if self.drive.arbiter.estopping or self._estops != since:
            return MotionResult(motion_id, False, NOT_SENT_ESTOP, "not sent: estop() was called", False)
        return None

    @property
    def last_motion(self) -> Optional[int]:
        """The last motion id this client completed with result true: None on a new client,
        0 from the moment an ESTOP is sent (a motion that was running then does not count)."""
        gen, mid = self._last
        return mid if gen == self._estops else 0

    def motion(self, motion_id: int, *, duration: int = 0, timeout: float = 25.0,
               allow_risky: bool = False, **arrays) -> MotionResult:
        params = self._params(motion_id, duration, allow_risky, arrays)
        return self._motion(params, max(timeout, MOTION_DEADLINE), self._estops)

    @staticmethod
    def _params(motion_id: int, duration: int, allow_risky: bool, arrays: dict) -> dict:
        m = check_risk(motion_id, allow_risky)
        if m is not None:
            if "servo" in m.needs:
                raise ForbiddenCommand("motion %d is a gait: stream it with dog.drive" % m.id)
            if "pose" in m.needs and not ("pos_des" in arrays or "rpy_des" in arrays):
                raise ValueError("motion %d needs pos_des and/or rpy_des" % m.id)
            # 3010 duration rule (motion.md §2.5): min > 0 needs 0, min < 0 needs > 0
            if (m.min_ms > 0 and duration != 0) or (m.min_ms < 0 and duration <= 0):
                raise ValueError("motion %d (min_exec_time %d) does not accept duration %r"
                                 % (m.id, m.min_ms, duration))
            if m.id in ENDLESS_AT_0 and duration <= 0:
                raise ValueError("motion %d with duration 0 never ends on the dog (it repeats the move "
                                 "every 2 s): give a duration in ms" % m.id)
        params = {"motion_id": motion_id, "duration": duration}
        for key, v in arrays.items():
            if key not in ARRAY_SIZES and key not in ("value", "contact"):
                raise TypeError("motion() got an unexpected keyword %r" % key)
            params[key] = v
        return params

    def _motion(self, params: dict, timeout: float, since: int) -> MotionResult:
        mid = params["motion_id"]
        if mid != 0:
            refused = self._not_sent(mid, since)
            if refused is not None:
                return refused
        return self._run_motion(params, timeout, priority=mid == 0)

    def stand(self) -> MotionResult:
        return self.motion(111)

    def sit(self) -> MotionResult:
        return self.motion(143)

    def lie_down(self) -> MotionResult:
        return self.motion(101)

    def trick(self, motion_id: int, allow_risky: bool = False) -> MotionResult:
        """A catalog motion with the sit rules of the VP SDK (motion.cpp:565-583), see
        trick_steps(): after a sit, 111 first unless the target is 148-150, 111, 101 or 0;
        148-150 need 143 first. No step goes out once an ESTOP was sent or estop() began after
        trick() was called (code NOT_SENT_ESTOP)."""
        m = check_risk(motion_id, allow_risky)
        if m is not None and ("servo" in m.needs or "pose" in m.needs):
            raise ForbiddenCommand("motion %d is not a trick (%s)" % (m.id, ", ".join(m.needs)))
        since = self._estops
        refused = self._not_sent(motion_id, since) if motion_id != 0 else None
        if refused is not None:
            return refused
        steps = trick_steps(motion_id, self.status().motion_id, self.last_motion)
        for pre in steps[:-1]:
            r = self._motion(self._params(pre, 0, False, {}), MOTION_DEADLINE, since)
            if not r.result:
                return r
        return self._motion(self._params(motion_id, 0, allow_risky, {}), MOTION_DEADLINE, since)

    def estop(self) -> MotionResult:
        """ESTOP on the priority channel, always three times. Streams are halted first (no END, no locks).

        The servo watchdog sends WalkStand 112 about 200-250 ms after the last frame it got,
        bypassing motion_manager's ESTOP latch (motion_handler.cpp:234-300). The open-source
        controller refuses that move out of ESTOP (fsm_state_passive.cpp:65-93); the dog's
        firmware is not verified, so the SDK does not rely on it. A walk frame can reach the
        dog just before the first ESTOP: from the stream halted here, or from one that ended by
        itself moments ago with its END lost. The second ESTOP always follows, WATCHDOG_S after
        the later of the halts (just before the first ESTOP) and the halted Driver stream's
        end; it also covers a first ESTOP lost on the link. On the real dog the 112 came between
        0.61 and 0.72 s after the last frame (probe of 2026-10-25), later than the source says, so
        a third ESTOP follows ESTOP_LATE_S after the same point. Both go out from their own
        thread, so a slow reply to the first
        (ESTOP waits up to its 1000 ms min_exec_time) does not delay it, and that thread is not
        a daemon, so an interpreter exit waits for it. While estop() runs no stream starts or
        sends (a move()/set() meanwhile is dropped), and close() waits for it. Returns after
        all replies (or failures), so a stand sent after estop() returns cannot be knocked
        down by a repeat. A Ctrl+C during estop() is raised after all ESTOPs went out."""
        token = threading.Event()
        estopping = self.drive.arbiter.estopping
        try:
            estopping.append(token)     # list operations are atomic: no lock on this path
            return self._estop()
        finally:                        # after the repeat went out: close() waits until then
            if token in estopping:
                estopping.remove(token)
            token.set()

    def _estop(self) -> MotionResult:
        caught = []                 # what cut a step short (a Ctrl+C): raised once all ESTOPs went out
        th = _shielded(caught, self.drive.halt_thread)
        _shielded(caught, self.posture.halt)
        sent = clock()
        out = []
        claim, done = threading.Lock(), threading.Event()

        def repeat():
            if not claim.acquire(False):    # runs once: in its thread, or below if that never ran
                return
            try:
                if th is not None:          # exactly the halted stream, never one started after it
                    _join(th, clock() + 1.0, caught)
                for delay in (WATCHDOG_S, ESTOP_LATE_S):
                    _sleep_until(max(sent, self.drive.ended_at) + delay, caught)
                    out.append(self._estop_once(caught))
            finally:
                done.set()

        try:
            threading.Thread(target=repeat, name="estop-repeat", daemon=False).start()
        except BaseException as e:  # no thread left, or a Ctrl+C in start(): the thread may not run
            if not isinstance(e, RuntimeError):
                caught.append(e)
        results = [self._estop_once(caught)]
        repeat()                    # here, unless the thread already took it
        while not done.is_set():
            try:
                done.wait()
            except BaseException as e:
                caught.append(e)
        results.extend(out or [None])
        if caught:                  # Ctrl+C and the like first, then a failure of the halt
            raise ([e for e in caught if not isinstance(e, Exception)] or caught)[0]
        for r in results:
            if isinstance(r, MotionResult) and r.result:
                return r
        last = results[-1]
        if isinstance(last, BaseException):
            raise last
        return last

    def _estop_once(self, caught):
        """One ESTOP. A Ctrl+C that cuts the send short is kept in caught and the ESTOP sent again
        (SHIELD_TRIES sends at most); a failure is returned, so it never stops the other ESTOP.
        A priority channel still in reconnect backoff fails at once (UNAVAILABLE) while the main
        one may be up: the ESTOP then goes again at once on the main lane. A deadline is not
        retried there: the dog or the link did not answer in ESTOP_TIMEOUT."""
        for _ in range(SHIELD_TRIES):
            try:
                try:
                    return self._run_motion({"motion_id": 0}, ESTOP_TIMEOUT, priority=True)
                except DogUnreachable as e:
                    if getattr(e.status, "name", None) != "UNAVAILABLE":
                        raise
                return self._run_motion({"motion_id": 0}, ESTOP_TIMEOUT)
            except Exception as e:
                return e
            except BaseException as e:
                caught.append(e)
        return caught[-1]

    # ---- LED, skin, sensors -----------------------------------------------------
    # VP: every task carries the whole LED state, since a new debug task kills the running one
    # (vp.Slot). read_sensors keeps the slot for the whole read: an LED call made meanwhile waits.
    def led(self, target: str, color=None, effect: str = "on", seconds: float = 3.0) -> str:
        return self._vp.led(target, color, effect, seconds)

    def led_off(self, target: str) -> str:
        return self._vp.led_off(target)

    def skin(self, mode: int, wave_ms: int = 3000) -> bool:
        if not 0 <= int(mode) <= 6:
            raise ValueError("skin mode must be 0..6")
        self.transport.call_one(11001, {"data": True}, timeout=10.0)
        r = self.transport.call_one(11002, {"mode": int(mode), "wave_cycle_time": int(wave_ms)},
                                    timeout=10.0)
        return r.get("success") is True

    def read_sensors(self, kinds=vp.SENSORS, timeout: float = 20.0) -> dict:
        return self._vp.read_sensors(kinds, timeout)
