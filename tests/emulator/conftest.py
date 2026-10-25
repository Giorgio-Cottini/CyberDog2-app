"""Emulator fixtures: one in-process emulator (time scale 0.05, free ports), raw gRPC through wire.py,
and the HTTP control endpoint. The emulator is reset before every test."""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request

import grpc
import pytest

from emulator.common import wire
from emulator.server import Emulator

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCALE = 0.05


def _read(name):
    with open(os.path.join(ROOT, "certs", name), "rb") as f:
        return f.read()


class Dog:
    def __init__(self, port, cport, emu):
        self.port, self.cport, self.emu = port, cport, emu
        creds = grpc.ssl_channel_credentials(_read("ca-cert.pem"), _read("client-key.pem"), _read("client-cert.pem"))
        self.ch = grpc.secure_channel("127.0.0.1:%d" % port, creds, options=[
            ("grpc.ssl_target_name_override", "cyberdog2.server"), ("grpc.max_receive_message_length", 32 << 20)])
        self._send = self.ch.unary_stream(wire.SEND_MSG, request_serializer=wire.encode_send_request,
                                          response_deserializer=wire.decode_rec_response)
        self._file = self.ch.unary_stream(wire.GET_FILE, request_serializer=wire.encode_send_request,
                                          response_deserializer=wire.decode_file_chunk)

    @staticmethod
    def _params(p):
        return p if isinstance(p, str) else json.dumps(p)

    def call(self, nc, params="{}", timeout=15):
        """All messages of one sendMsg stream as (nameCode, data) pairs."""
        return [(m.name_code, m.data) for m in self._send(wire.SendRequest(nc, self._params(params)), timeout=timeout)]

    def stream(self, nc, params="{}", timeout=15):
        return self._send(wire.SendRequest(nc, self._params(params)), timeout=timeout)

    def j(self, nc, params="{}", timeout=15):
        """The single JSON reply of a one-message call."""
        msgs = self.call(nc, params, timeout)
        assert len(msgs) == 1, msgs
        assert msgs[0][0] == nc, msgs
        return json.loads(msgs[0][1])

    def files(self, nc, params="{}", timeout=15):
        return list(self._file(wire.SendRequest(nc, self._params(params)), timeout=timeout))

    def state(self):
        with urllib.request.urlopen("http://127.0.0.1:%d/state" % self.cport, timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.cport, path), method="POST",
                                     data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def wait(self, pred, timeout=5.0, step=0.05):
        """Poll /state until pred(state) is truthy; returns the last state."""
        end = time.monotonic() + timeout
        while True:
            s = self.state()
            if pred(s):
                return s
            if time.monotonic() > end:
                raise AssertionError("condition not met in %.1f s" % timeout)
            time.sleep(step)


@pytest.fixture(scope="session")
def emu():
    e = Emulator(port=0, host="127.0.0.1", controller="fake", time_scale=SCALE, control_port=0)
    e.start()
    yield e
    e.stop()


@pytest.fixture
def dog(emu):
    d = Dog(emu.port, emu.control_port, emu)
    assert d.post("/reset")[0] == 200
    d.wait(lambda s: s["motion"]["motion_id"] == 101 and not s["motion"]["busy"])
    yield d
    d.ch.close()
