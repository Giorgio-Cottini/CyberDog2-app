"""Integration fixtures: the real SDK against an in-process emulator over mTLS (contracts.md 7).

One emulator per session (fake controller, time scale 0.05, free ports on 127.0.0.1). Every test gets
a fresh, connected CyberDog and a freshly reset emulator. After every test the emulator must show no
dangerous call and no client bug: the SDK never sends a forbidden code and never mistypes a field.
Nothing here can reach the real dog: the host is always 127.0.0.1.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request

import pytest

from cyberdog_sdk import CyberDog
from emulator.server import Emulator

SCALE = 0.05
HOST = "127.0.0.1"


class Control:
    """The emulator's HTTP control endpoint: what the dog actually received."""

    def __init__(self, port):
        self.base = "http://%s:%d" % (HOST, port)

    def state(self):
        with urllib.request.urlopen(self.base + "/state", timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request(self.base + path, method="POST", data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def wait(self, pred, timeout=5.0, step=0.02):
        """Poll /state until pred(state) is truthy; returns that state."""
        end = time.monotonic() + timeout
        while True:
            s = self.state()
            if pred(s):
                return s
            if time.monotonic() > end:
                raise AssertionError("emulator state condition not met in %.1f s: motion %s, servo %s, nav %s"
                                     % (timeout, s["motion"], s["servo"], s["nav"]["status"]))
            time.sleep(step)

    def calls(self):
        return dict(self.state()["calls"])


class Recorder:
    """Wraps a Transport instance to log what the SDK asked to send, in order: (t, name_code, params)."""

    def __init__(self, transport):
        self.sent = []
        self._lk = threading.Lock()
        for name in ("call", "get_file", "stream"):
            self._wrap(transport, name)

    def _wrap(self, t, name):
        orig = getattr(t, name)

        def wrapper(name_code, params=None, *a, **kw):
            with self._lk:
                self.sent.append((time.perf_counter(), name_code, params))
            return orig(name_code, params, *a, **kw)
        setattr(t, name, wrapper)

    def of(self, name_code):
        with self._lk:
            return [(t, p) for t, nc, p in self.sent if nc == name_code]

    def motions(self):
        """motion_id of every 1004 sent, in order."""
        return [p["motion_id"] for _, p in self.of(1004)]

    def frames(self):
        """(t, motion_id, cmd_type) of every 1002 sent."""
        return [(t, p["motion_id"], p["cmd_type"]) for t, p in self.of(1002)]


@pytest.fixture(scope="session")
def emu():
    e = Emulator(port=0, host=HOST, controller="fake", time_scale=SCALE, control_port=0)
    e.start()
    yield e
    e.stop()


@pytest.fixture(scope="session")
def ctl(emu):
    return Control(emu.control_port)


@pytest.fixture
def dog(emu, ctl):
    assert ctl.post("/reset")[0] == 200
    ctl.wait(lambda s: s["motion"]["motion_id"] == 101 and not s["motion"]["busy"])
    d = CyberDog(host=HOST, port=emu.port)
    assert d.transport.host == HOST
    d.connect(timeout=5.0)
    d.rec = Recorder(d.transport)
    yield d
    d.close()
    s = ctl.state()
    assert s["dangerous_calls"] == [], s["dangerous_calls"]
    assert s["client_bugs"] == [], s["client_bugs"]
    assert s["crashed"] is None


@pytest.fixture
def standing(dog, ctl):
    """The same dog, brought to standing (111) with the emulator idle."""
    r = dog.stand()
    assert r.result and r.code == 0, r
    ctl.wait(lambda s: s["motion"]["motion_id"] == 111 and not s["motion"]["busy"])
    return dog
