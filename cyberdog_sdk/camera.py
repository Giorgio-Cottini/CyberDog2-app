"""Still-frame camera grabber over 4002 / getFile, ported from app/server.py (Grabber).

N workers, each on its own channel, loop TAKE_PICTURE over getFile and publish
the newest JPEG to one slot. The dog allows up to 5 concurrent transfers
(error 10 above that) and 4 pullers roughly double the frame rate of one.
Frames are full resolution: the 4002 handler ignores width/height.
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Optional, Tuple

from .codes import CAM_START_LIVE_STREAM, CAM_STOP_LIVE_STREAM, CAM_TAKE_PICTURE

PICTURE_TIMEOUT = 30.0
INVALID_STATE = 5        # camera dropped streaming: re-arm START_LIVE_STREAM
STALE_S = 2.0            # fps reads 0 when the newest frame is older than this
STOP_TRIES = 3           # STOP_LIVE_STREAM sends while the dog refuses it (7 service slow, 10, 4 busy)
STOP_DONE = (0, INVALID_STATE)   # 5: no live stream to stop


class Camera:
    def __init__(self, transport, workers: int = 4):
        self.t = transport
        self.n = max(1, workers)
        self.last_error = None
        self._want = False
        self._fps = 0.0
        self._lock = threading.Lock()
        self._cond = threading.Condition()
        self._latest = None          # (seq, jpeg) of this run, None until its first frame
        self._seq = 0
        self._run = 0                # +1 per start(): a pull from an earlier run publishes nothing
        self._stamps = collections.deque(maxlen=12)
        self._channels = []
        self._workers = {}           # id(channel) -> thread

    @property
    def running(self) -> bool:
        return self._want

    @property
    def fps(self) -> float:
        """Frames per second over the last 12 frames; 0 when none came for STALE_S."""
        with self._cond:
            if not self._stamps or time.monotonic() - self._stamps[-1] > STALE_S:
                return 0.0
            return self._fps

    def start(self) -> None:
        """A failure (cert files unreadable, no thread left) raises with running False again,
        after a STOP, so the next start() tries again."""
        with self._lock:
            if self._want:
                return
            if not self._channels:
                self._channels = [self.t.new_channel() for _ in range(self.n)]
            channels = list(self._channels)
            self._want = True
            with self._cond:            # this run's frames only: fps, and no frame from before the stop
                self._stamps.clear()
                self._fps = 0.0
                self._latest = None
                self._run += 1
        try:
            self._command(CAM_START_LIVE_STREAM, if_wanted=True)
            for i, ch in enumerate(channels):
                th = self._workers.get(id(ch))
                if th is None or not th.is_alive():   # a worker from an earlier start() may still run
                    th = threading.Thread(target=self._work, args=(ch,), name="camera-%d" % i, daemon=True)
                    th.start()
                    self._workers[id(ch)] = th
        except BaseException:
            self.stop()
            raise

    def stop(self) -> None:
        with self._lock:
            was, self._want = self._want, False
        if was:
            self._stop_stream()

    def close(self) -> None:
        self.stop()
        with self._lock:
            channels, self._channels = self._channels, []
            self._workers = {}
        for ch in channels:            # cancels in-flight pulls; the workers then exit
            try:
                ch.close()
            except Exception:
                pass

    def latest(self) -> Optional[Tuple[int, bytes]]:
        with self._cond:
            return self._latest

    def wait(self, after_seq: int = 0, timeout: float = 5.0) -> Optional[Tuple[int, bytes]]:
        """The newest frame with seq > after_seq, waiting up to timeout; else the latest (or None)."""
        with self._cond:
            if self._latest is None or self._latest[0] <= after_seq:
                self._cond.wait(timeout)
            return self._latest

    def _command(self, command: int, if_wanted: bool = False) -> None:
        """One 4002 command. With if_wanted (a START): skipped once stop() ran, and followed by
        another STOP when stop() ran while it was in flight. So after stop() the last command is
        STOP, and stop() never waits behind a START (up to 10 s on the dog)."""
        if if_wanted and not self._want:
            return
        self._call(command)
        if if_wanted and not self._want:
            self._stop_stream()

    def _stop_stream(self) -> None:
        """STOP_LIVE_STREAM, sent again 0.5 s later while the dog refuses it and no start() came.
        _want is read before each send: a STOP after a new start() would end that stream."""
        for i in range(STOP_TRIES):
            if self._want or self._call(CAM_STOP_LIVE_STREAM) in STOP_DONE:
                return
            if i + 1 < STOP_TRIES:
                time.sleep(0.5)

    def _call(self, command: int) -> Optional[int]:
        """The dog's result (0 success), or None with no reply. A non-zero result sets last_error."""
        try:
            items = self.t.call_json(4002, {"command": command}, timeout=10.0)
        except Exception as e:
            self.last_error = "4002 %d: %s" % (command, e)
            return None
        r = items[0].get("result") if items else None
        if r != 0:
            self.last_error = "4002 %d: result %r" % (command, r)
        return r

    def _work(self, channel) -> None:
        while self._want and channel in self._channels:
            run = self._run
            try:
                res = self.t.get_file(4002, {"command": CAM_TAKE_PICTURE},
                                      timeout=PICTURE_TIMEOUT, channel=channel)
            except Exception as e:
                self.last_error = str(e)
                time.sleep(0.2)
                continue
            if res.error_code == INVALID_STATE:
                self._command(CAM_START_LIVE_STREAM, if_wanted=True)
                time.sleep(0.2)
            elif res.error_code == 0 and res.data:
                with self._cond:
                    if run != self._run or not self._want:  # pulled before a stop() or a new start()
                        continue
                    self._seq += 1
                    self._latest = (self._seq, res.data)
                    now = time.monotonic()
                    self._stamps.append(now)
                    if len(self._stamps) >= 2:
                        self._fps = (len(self._stamps) - 1) / max(1e-6, self._stamps[-1] - self._stamps[0])
                    self._cond.notify_all()
            else:
                self.last_error = "getFile 4002: error_code %d" % res.error_code
                time.sleep(0.1)
