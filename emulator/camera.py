"""4002 camera_service and the getFile transfers of 4002 / 4004 (grpc_fidelity.md 7.6).

The camera_service server is not public: its replies here are [INF]. Files live under
<fs_root>/home/mi/Camera and are deleted after a successful download, as on the dog.
"""
from __future__ import annotations

import os
import posixpath
import struct
import threading
import time

from emulator.common import get, jdump, wire

CH = 4_194_304                                          # transmit_files.hpp:30
PREFIX = "/home/mi/Camera/"
PHOTO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "photo.jpeg")
STATES = {0: "idle", 1: "recording"}


class Camera:
    def __init__(self, emu):
        self.emu = emu
        self.lk = threading.Lock()
        with open(PHOTO, "rb") as f:
            self.jpeg = f.read()
        self.dir = os.path.join(emu.fs_root, "home", "mi", "Camera")
        self.reset()

    def reset(self):
        with self.lk:
            self.recording = self.live = self.publishing = False
            self.thread_counts = 0
            self.uncomplete = set()
            self.n = 0
        os.makedirs(self.dir, exist_ok=True)
        for name in os.listdir(self.dir):
            p = os.path.join(self.dir, name)
            if os.path.isfile(p):
                os.remove(p)

    def _write(self, name, data):
        with open(os.path.join(self.dir, name), "wb") as f:
            f.write(data)
        return "%s,%d" % (name, len(data))

    def call(self, cmd):
        """camera_service: (ok, result, msg). ok False -> the bridge's 7 (5 s service wait, 5 s reply)."""
        dep = self.emu.dep("camera_service")
        if not dep.up(5) or not dep.ready(5):
            return False, 0, ""
        with self.lk:
            self.n += 1
            stamp = time.strftime("%Y%m%d_%H%M%S")
            if cmd in (0, 5):                           # args never filled by the bridge (app.cpp:839-840)
                return True, 1, ""
            if cmd == 1:                                # TAKE_PICTURE: a COM segment makes each file unique
                note = ("emulator shot %d" % self.n).encode()
                data = self.jpeg[:2] + b"\xff\xfe" + struct.pack(">H", len(note) + 2) + note + self.jpeg[2:]
                return True, 0, self._write("IMG_%s_%d.jpg" % (stamp, self.n), data)
            if cmd == 2:
                if self.recording:
                    return True, 4, ""
                self.recording = True
                return True, 0, ""
            if cmd == 3:
                if not self.recording:
                    return True, 5, ""
                self.recording = False
                return True, 0, self._write("VID_%s_%d.mp4" % (stamp, self.n), b"\x00\x00\x00\x18ftypmp42emulator")
            if cmd == 4:
                return True, 0, STATES[int(self.recording)]
            if cmd == 6:
                return True, 0, ""
            if cmd in (7, 8):
                self.live = cmd == 7
                return True, 0, ""
            if cmd in (9, 10):
                self.publishing = cmd == 9
                return True, 0, ""
            return True, 255, ""

    def send_msg(self, doc, nc, raw):                   # processCameraMsg (app.cpp:3950-3963)
        cmd = get(doc, "command", "uint", 2) & 0xFF     # default 2 = START_RECORDING
        if self.thread_counts > 5:
            res = 10
        else:
            ok, res, _ = self.call(cmd)
            res = res if ok else 7
        if res:
            self.emu.refuse(nc, "camera result %d" % res, command=cmd)
        yield self.emu.msg(nc, jdump({"result": res}))

    def get_file(self, doc, nc, ctx):
        if nc == 4002:                                  # app.cpp:3965-3978
            cmd = get(doc, "command", "uint", 255) & 0xFF
            if self.thread_counts > 5:
                yield wire.FileChunk(10, "", 0, b"")
                return
            ok, res, m = self.call(cmd)
            if not ok or res:
                yield wire.FileChunk(res if ok else 7, "", 0, b"")
                return
            if "," not in m:
                yield wire.FileChunk(8, "", 0, b"")
                return
            name, rest = m.split(",", 1)
            digits = ""
            for c in rest.strip():
                if not c.isdigit():
                    break
                digits += c
            yield from self.send_file(ctx, name, int(digits or 0), nc)
        else:                                           # 4004: file_name, not sanitised on the dog
            yield from self.send_file(ctx, get(doc, "file_name", str, ""), 0, nc)

    def _host(self, name):
        """PREFIX + name resolved like the dog's open(), but kept inside fs_root."""
        if "\\" in name or ":" in name or "\x00" in name:
            return None
        p = posixpath.normpath(PREFIX + name)
        if p.startswith("//"):
            p = p[1:]
        return os.path.join(self.emu.fs_root, p.lstrip("/").replace("/", os.sep))

    def send_file(self, ctx, name, size, nc):               # transmit_files.hpp:111-206
        if "/" in name or ".." in name:
            self.emu.client_bug("4004 file_name with '/' or '..': the dog reads and then deletes that path",
                                file_name=name)
        with self.lk:
            self.thread_counts += 1
        try:
            path = self._host(name)
            try:
                f = open(path, "rb") if path and os.path.isfile(path) else None
            except OSError:
                f = None
            if f is None:
                self.emu.refuse(nc, "getFile: open failed (9)", file_name=name)
                yield wire.FileChunk(9, "", 0, b"")
                return
            with f:
                size = size or os.path.getsize(path)
                while True:
                    buf = f.read(CH)
                    if ctx is not None and not ctx.is_active():   # Write() fails four times
                        with self.lk:
                            self.uncomplete.add(name)
                        return
                    yield wire.FileChunk(0, name, size & 0xFFFFFFFF, buf)
                    if len(buf) < CH:                   # an exact multiple of 4 MiB ends with an empty chunk
                        break
            os.remove(path)                             # deleted on the dog after a successful send (:195)
            with self.lk:
                self.uncomplete.discard(name)
        except GeneratorExit:                           # the client went away mid-stream
            with self.lk:
                self.uncomplete.add(name)
            raise
        finally:
            with self.lk:
                self.thread_counts -= 1

    def state(self):
        with self.lk:
            files = sorted(os.listdir(self.dir)) if os.path.isdir(self.dir) else []
            return {"state": STATES[int(self.recording)], "live": self.live, "image_publish": self.publishing,
                    "files": files, "transfers": self.thread_counts, "uncomplete": sorted(self.uncomplete)}
