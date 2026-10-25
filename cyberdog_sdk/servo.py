"""Servo streaming over 1002: Driver (walk) and Posture (201 absolute pose).

Each frame is its own RPC with a short deadline, sent from a dedicated thread
on a monotonic schedule. The dog's servo watchdog stops the stream 200-250 ms
after the last frame (motion.md §3), so a frame must go out at least every
~200 ms. Driver and Posture share an Arbiter: starting one stops the other.
The Driver has a dead-man: without a new move() for DEADMAN_S it sends one END
and stops, so a hung caller cannot leave the dog walking.
Stream threads are daemons: if the interpreter exits mid-stream, no END goes out
and the dog's watchdog stops it with the same WalkStand 112 about 250 ms later.
"""
from __future__ import annotations

import enum
import math
import threading
import time
from typing import Optional

from .codes import SERVO_DATA, SERVO_END

DRIVE_HZ = 25
POSE_HZ = 15
FRAME_DEADLINE = 0.25
SWITCH_PAUSE = 0.15         # after END, before the next stream (gait change or Driver<->Posture)
POSE_IDLE_END = 0.6         # Posture sends END this long after the last change
WATCHDOG_S = 0.3            # servo watchdog fires 200-250 ms after the last frame (MH:234-249)
ESTOP_LATE_S = 1.0          # the real dog's watchdog fired 0.61-0.72 s after the last frame
DEADMAN_S = 0.5             # Driver default: END this long after the last move(); None turns it off
clock = time.perf_counter   # monotonic; time.monotonic() ticks in 15.6 ms steps on Windows


class Gait(enum.IntEnum):
    SLOW = 303
    MEDIUM = 308
    FAST = 305
    AUTO = 304


# (min [vx, vy, wz], max [vx, vy, wz]) per gait, motion.md §5.1 (304: the doc range)
GAIT_LIMITS = {
    303: ((-0.6, -0.2, -1.3), (0.6, 0.2, 1.3)),
    308: ((-1.2, -0.4, -2.25), (1.2, 0.4, 2.25)),
    305: ((-1.2, -0.55, -2.5), (1.6, 0.55, 2.5)),
    304: ((-1.2, -0.6, -2.5), (1.6, 0.6, 2.5)),
}
STEP_HEIGHT_RANGE = (0.0, 0.06)      # doc range, motion.md §5.3

POSE_LIMITS = {"z": (0.18, 0.30), "roll": (-0.35, 0.35), "pitch": (-0.2, 0.25), "yaw": (-0.5, 0.5)}
POSE_DEFAULT = {"z": 0.235, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _finite(v, name: str) -> float:
    if isinstance(v, (bool, str, bytes, bytearray)):     # float() would accept "0.3" and True
        raise ValueError("%s must be a number, got %r" % (name, v))
    try:
        f = float(v)
    except OverflowError:
        raise ValueError("%s must be finite" % name) from None
    if not math.isfinite(f):
        raise ValueError("%s must be finite" % name)
    return f


def frame(motion_id: int, cmd_type: int, vel=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0),
          z: float = 0.0, step_height: float = 0.0) -> dict:
    """A full 1002 frame as in motion.md §7: every array present, every element a float."""
    return {"motion_id": int(motion_id), "cmd_type": cmd_type,
            "vel_des": [float(x) for x in vel], "rpy_des": [float(x) for x in rpy],
            "pos_des": [0.0, 0.0, float(z)], "acc_des": [0.0] * 6, "ctrl_point": [0.0] * 3,
            "foot_pose": [0.0] * 6, "step_height": [float(step_height)] * 2, "value": 0}


class Arbiter:
    """Makes sure Driver and Posture never stream at the same time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.members = []
        self.estopping = []     # one Event per estop() in progress: no stream starts or sends meanwhile

    def claim(self, me) -> None:
        """Stop any other stream (END), then keep SWITCH_PAUSE after the last stream's end."""
        for other in self.members:
            if other is not me and other.active:
                other.stop()
        wait = max(m.ended_at for m in self.members) + SWITCH_PAUSE - clock()
        if wait > 0:
            time.sleep(wait)
        for other in self.members:
            if other is not me:
                other.on_other_started()


class _Run:
    """Per-stream exit flag and wake Event, tied to one stream thread."""
    __slots__ = ("exit", "wake")

    def __init__(self):
        self.exit = None
        self.wake = threading.Event()


class _Streamer:
    hz = DRIVE_HZ

    @property
    def _exit(self):
        return self._run.exit

    @_exit.setter
    def _exit(self, v):
        self._run.exit = v

    @property
    def _wake(self):
        return self._run.wake

    @_wake.setter
    def _wake(self, v):
        self._run.wake = v

    def __init__(self, transport, arbiter: Optional[Arbiter] = None):
        self.t = transport
        self.arbiter = arbiter or Arbiter()
        self.arbiter.members.append(self)
        self._thread = None
        self._run = _Run()          # exit: None, "end" (stop) or "halt" (ESTOP); wake
        self._closing = False       # the loop has decided to finish
        self._plock = threading.Lock()
        self._stats = {"frames": 0, "ends": 0, "errors": 0, "max_gap_ms": 0.0, "last_error": None}
        self._last_ok = None        # clock() time the last DATA frame that went through was sent
        self._last_cmd = None
        self._last_frame = None     # the last frame sent: the template for an END after a crash
        self._halts = 0             # halt() calls so far; a start that began before the latest one never streams
        self._gen = 0               # the _halts value the current stream was started under
        self._closed = False        # after close() nothing starts again
        self.started_at = 0.0       # clock() time the current stream thread began running
        self.ended_at = 0.0         # clock() time the last stream thread finished

    @property
    def active(self) -> bool:
        th = self._thread
        return th is not None and th.is_alive()

    @property
    def stats(self) -> dict:
        return dict(self._stats)

    def on_other_started(self) -> None:
        pass

    def _halted(self) -> bool:
        return self._exit == "halt" or self._halts != self._gen or bool(self.arbiter.estopping)

    def _cancelled(self, gen: int) -> bool:
        # a halt() since the caller read gen, an estop() in progress, or close()
        return self._halts != gen or bool(self.arbiter.estopping) or self._closed

    def _send(self, f: dict) -> None:
        # last check before the wire: no frame starts once halt() is seen. A frame already past
        # this check can still reach the dog after the first ESTOP; estop() repeats for that.
        if self._halted():
            return
        now = clock()
        end = f["cmd_type"] == SERVO_END
        self._last_cmd = f["cmd_type"]
        self._last_frame = f
        try:
            self.t.call(1002, f, timeout=FRAME_DEADLINE)
        except Exception as e:   # keep streaming; the watchdog covers a lost frame
            self._stats["errors"] += 1
            self._stats["last_error"] = str(e)
            return
        if end:
            self._last_ok = None
            self._stats["ends"] += 1
            return
        # max_gap_ms is the gap the dog sees: between frames that went through
        if self._last_ok is not None:
            gap = (now - self._last_ok) * 1000.0
            if gap > self._stats["max_gap_ms"]:
                self._stats["max_gap_ms"] = gap
        self._last_ok = now
        self._stats["frames"] += 1

    def _start(self, gen: int, write) -> None:
        """Start the stream thread unless one is running, and apply the caller's change with
        write() (under _plock) only once the start is known not to be cancelled. Caller holds
        arbiter.lock. gen is _halts as the caller read it on entry: a halt() since then, or an
        estop() in progress, cancels the start and the change, also while this waits for a
        finishing stream or for claim()'s END and pause."""
        if self._cancelled(gen):
            return
        if self.active:
            with self._plock:
                if self._exit is None and not self._closing and self._gen == self._halts:
                    if not self._cancelled(gen):
                        write()
                    return
            # finishing, or started before a halt() it has not seen yet (it exits on its
            # next check): let it end, then restart
            self._thread.join()
        if self._cancelled(gen):
            return
        self.arbiter.claim(self)
        with self._plock:
            if self._cancelled(gen):
                return
            write()
        self._gen = gen
        self._closing = False
        run = _Run()
        self._run = run
        self._last_ok = self._last_cmd = None
        th = threading.Thread(target=self._loop, name=type(self).__name__, daemon=True)
        th._cd_run = run
        self._thread = th
        th.start()

    def _loop(self) -> None:
        period = 1.0 / self.hz
        self.started_at = next_t = clock()
        try:
            while True:
                if self._halted():
                    return
                with self._plock:
                    go = self._exit is None and self._want()
                    self._closing = not go
                if not go:
                    # one END per stream that sent a frame, none for a stream that sent nothing
                    # (END alone makes the dog run WalkStand 112), and none after halt() (ESTOP)
                    if not self._halted() and self._last_cmd == SERVO_DATA:
                        self._send(self._end_frame())
                    return
                self._step()
                next_t += period
                delay = next_t - clock()
                # more than one period behind (slow RPC): resync. Less than that, the next frame
                # goes out at once to keep the average rate (Event.wait is coarse on Windows),
                # so at most one frame follows a slow one closely
                if delay < -period:
                    next_t = clock()
                elif delay > 0:
                    self._wake.wait(delay)
        except BaseException as e:  # a bug or a bad attribute: end with an END, never die silently
            self._closing = True    # a move() now waits for this thread and restarts
            self._stats["errors"] += 1
            try:
                if not self._halted() and self._last_cmd == SERVO_DATA:
                    # the END of what the dog last got, not of the state that just failed
                    self._send(dict(self._last_frame, cmd_type=SERVO_END, vel_des=[0.0, 0.0, 0.0]))
            finally:                # recorded last, so a failing END does not overwrite it
                self._stats["last_error"] = "stream stopped: %r" % (e,)
        finally:
            self.ended_at = clock()

    def stop(self) -> None:
        """Send one END (only if streaming) and wait for the thread to finish."""
        th = self._thread
        if th is None or not th.is_alive():
            return
        run = th._cd_run
        if run.exit is None:
            run.exit = "end"
        run.wake.set()
        th.join()

    def halt(self) -> bool:
        """ESTOP path: stop streaming at once, without END and without waiting.
        Also cancels a move()/set() that is restarting the stream right now; a call that
        begins after halt() starts a new stream as usual.
        Returns True if a stream was running."""
        return self.halt_thread() is not None

    def halt_thread(self) -> Optional[threading.Thread]:
        """halt(), returning the stream thread it stopped (None if none was running), so the
        caller can wait for exactly that thread and not for a stream started after it."""
        self._halts += 1
        th = self._thread
        if th is None or not th.is_alive():
            return None
        run = th._cd_run
        run.exit = "halt"
        run.wake.set()
        return th

    def join(self, timeout: Optional[float] = None) -> None:
        """Wait for the current stream thread, if any."""
        th = self._thread
        if th is not None:
            try:
                th.join(timeout)
            except RuntimeError:     # a move()/set() is starting it right now: nothing to wait for
                pass

    def close(self) -> None:
        """Send one END if streaming, and never start again. A move()/set() that is starting a
        stream right now finishes first (it holds the arbiter lock), then sees the flag."""
        self._closed = True
        with self.arbiter.lock:
            self.stop()

    # subclasses
    def _want(self) -> bool:
        return True

    def _step(self) -> None:
        raise NotImplementedError

    def _end_frame(self) -> dict:
        raise NotImplementedError


class Driver(_Streamer):
    """Walking by 1002 stream: move() starts or updates it, stop() sends one END.

    Dead-man: when move() is not called again within deadman_s (default 0.5 s),
    the stream sends one END and stops, exactly as stop() does. The check runs
    at every frame boundary, so on a slow link the END can come up to
    FRAME_DEADLINE later. To walk for a fixed time, call move() in a loop faster
    than deadman_s, or set deadman_s = None and call stop() yourself."""
    hz = DRIVE_HZ

    def __init__(self, transport, arbiter: Optional[Arbiter] = None,
                 deadman_s: Optional[float] = DEADMAN_S):
        super().__init__(transport, arbiter)
        self.gait = Gait.SLOW
        self.step_height = 0.05
        self.deadman_s = deadman_s
        self._vel = (0.0, 0.0, 0.0)
        self._last_move = 0.0
        self._next_gait = None
        self._stats["deadman_stops"] = 0

    @property
    def deadman_s(self) -> Optional[float]:
        return self._deadman_s

    @deadman_s.setter
    def deadman_s(self, v) -> None:
        if v is not None:
            v = _finite(v, "deadman_s")
            if v <= 0:
                raise ValueError("deadman_s must be > 0, or None to turn the dead-man off")
        self._deadman_s = v

    @property
    def step_height(self) -> float:
        return self._step_height

    @step_height.setter
    def step_height(self, v) -> None:
        self._step_height = _finite(v, "step_height")   # clamped to STEP_HEIGHT_RANGE when sent

    def set_gait(self, gait) -> None:
        g = Gait(int(gait))       # ValueError for anything but the four walking gaits
        # _step and move() swap gait and _next_gait under the same lock. Under the CPython GIL
        # these read-then-write sequences happen not to be preempted; the lock keeps them
        # correct without relying on that (free-threaded builds, or a property on gait).
        with self._plock:
            if self.active:
                self._next_gait = g if g != self.gait else None
            else:
                self.gait, self._next_gait = g, None

    def move(self, vx: float, vy: float, wz: float) -> None:
        vel = (_finite(vx, "vx"), _finite(vy, "vy"), _finite(wz, "wz"))
        gen = self._halts           # an estop() from here on cancels this (re)start
        with self._plock:
            if self._cancelled(gen):  # cancelled while waiting for the lock
                return

        def write():                # under _plock, once the start is known not cancelled
            self._vel = vel
            self._last_move = clock()   # the loop reads it under the same lock (_want)
        with self.arbiter.lock:
            with self._plock:       # a gait change left over from a stopped stream; one read
                g = self._next_gait
                if g is not None and not self.active:
                    self.gait, self._next_gait = g, None
            self._start(gen, write)

    def velocity(self) -> tuple:
        """The velocity actually sent: the last move() clamped to the current gait."""
        lo, hi = GAIT_LIMITS[int(self.gait)]
        return tuple(clamp(v, lo[i], hi[i]) for i, v in enumerate(self._vel))

    def _h(self) -> float:
        return clamp(float(self.step_height), *STEP_HEIGHT_RANGE)

    def _want(self) -> bool:
        # counted from the later of the last move() and the moment the stream thread
        # began, so neither the handover from Posture (END + pause) nor a slow thread
        # start eats the first move()'s time
        limit = self._deadman_s
        if limit is None or clock() - max(self._last_move, self.started_at) < limit:
            return True
        self._stats["deadman_stops"] += 1     # _loop calls this once per decision to end
        return False

    def _step(self) -> None:
        with self._plock:
            if self._exit is not None:   # stop() or halt() came while we waited for the lock
                return
            g, old = self._next_gait, self.gait
            if g is not None:
                self.gait, self._next_gait = g, None
        if g is not None and self._last_cmd == SERVO_DATA:
            # gait change while streaming: END the old gait and pause. The loop then checks
            # stop, halt and the dead-man again before the first frame of the new gait.
            # A stream with no frame yet needs no END: claim() already kept the pause.
            self._send(frame(old, SERVO_END, step_height=self._h()))
            self._wake.wait(SWITCH_PAUSE)
            return
        self._send(frame(self.gait, SERVO_DATA, vel=self.velocity(), step_height=self._h()))

    def _end_frame(self) -> dict:
        return frame(self.gait, SERVO_END, step_height=self._h())


class Posture(_Streamer):
    """Absolute body pose by 201 stream; END after POSE_IDLE_END keeps the pose (motion.md §3)."""
    hz = POSE_HZ

    def __init__(self, transport, arbiter: Optional[Arbiter] = None):
        super().__init__(transport, arbiter)
        self.target = dict(POSE_DEFAULT)
        self._changed = 0.0

    def set(self, z=None, roll=None, pitch=None, yaw=None) -> dict:
        changes = {key: clamp(_finite(v, key), *POSE_LIMITS[key])
                   for key, v in (("z", z), ("roll", roll), ("pitch", pitch), ("yaw", yaw))
                   if v is not None}
        gen = self._halts           # an estop() from here on cancels this (re)start
        out = {}

        def write():                # under _plock, after claim()'s reset: read-modify-write
            new = dict(self.target)
            new.update(changes)
            self.target = new
            self._changed = clock()
            out["new"] = new
        with self._plock:
            cancelled = self._cancelled(gen)
        if not cancelled:
            with self.arbiter.lock:
                self._start(gen, write)
        if "new" in out:
            return dict(out["new"])
        new = dict(self.target)     # cancelled: the target is unchanged; report what was asked
        new.update(changes)
        return new

    def reset(self) -> dict:
        return self.set(**POSE_DEFAULT)

    def on_other_started(self) -> None:
        with self._plock:
            self.target = dict(POSE_DEFAULT)     # walking ends the held pose

    def _want(self) -> bool:
        # like the Driver's dead-man: a slow handover (Driver END + pause) or thread start
        # must not use up the idle window before the first frame
        return clock() - max(self._changed, self.started_at) < POSE_IDLE_END

    def _step(self) -> None:
        self._send(self._frame(SERVO_DATA))

    def _frame(self, cmd_type: int) -> dict:
        p = self.target
        return frame(201, cmd_type, rpy=(p["roll"], p["pitch"], p["yaw"]), z=p["z"])

    def _end_frame(self) -> dict:
        return self._frame(SERVO_END)
