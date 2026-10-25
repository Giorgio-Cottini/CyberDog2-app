"""Fixtures for the SDK unit tests: a fake transport, no network at all.

FakeTransport keeps the real Transport guards (prepare, call, call_json,
get_file, stream) and replaces only the RPC core: every request is recorded and
answered from scripted replies, encoded and decoded through cyberdog_sdk.wire.
"""
from __future__ import annotations

import json
import threading
import time
from typing import NamedTuple

import pytest

from cyberdog_sdk import wire
from cyberdog_sdk.client import CyberDog
from cyberdog_sdk.transport import Transport


class Sent(NamedTuple):
    name_code: int
    raw: str            # the params string exactly as it went on the wire
    params: dict
    lane: object        # "main", "priority" or a channel from new_channel()
    method: str
    timeout: float
    t: float            # time.perf_counter() when sent (servo.clock)


def _default_reply(name_code: int, params: dict):
    if name_code == 1004:
        return [{"motion_id": params.get("motion_id"), "result": True, "code": 0}]
    if name_code == 1002:
        return [""]
    if name_code == 10001:
        return [{"motion_status": {"motion_id": 111},
                 "task_status": {"task_status": 101, "task_sub_status": 0},
                 "self_check_status": {"code": 0, "description": ""},
                 "state_switch_status": {"state": 0, "code": 0},
                 "charging_status": {"wired_charging": False, "wireless_charging": False}}]
    if name_code == 4002:
        return [{"result": 0}]
    if name_code in (1012, 11001, 11002):
        return [{"success": True}]
    return []


class FakeTransport(Transport):
    def __init__(self):
        super().__init__(host="fake", port=0, cert_dir="unused")
        self.sent = []
        self.replies = {}          # name_code -> list of messages, or callable(Sent) -> list
        self.file_replies = {}     # name_code -> list of FileChunk, or callable(Sent) -> list
        self.closed = False
        self._lock = threading.Lock()

    def new_channel(self):
        self.channels_made = getattr(self, "channels_made", 0) + 1
        return "channel-%d" % self.channels_made

    def connect(self, timeout=5.0):
        pass

    def close(self):
        self.closed = True

    def _rpc(self, method, request, timeout, lane="main"):
        # re-decode what would go on the wire, so the codec is on the path
        req = wire.decode_send_request(wire.encode_send_request(request))
        rec = Sent(req.name_code, req.params, json.loads(req.params), lane, method, timeout,
                   time.perf_counter())
        with self._lock:
            self.sent.append(rec)
        if method == wire.GET_FILE:
            r = self.file_replies.get(req.name_code, [wire.FileChunk(error_code=9)])
            for chunk in (r(rec) if callable(r) else r):
                yield wire.decode_file_chunk(wire.encode_file_chunk(chunk))
            return
        r = self.replies.get(req.name_code)
        out = _default_reply(req.name_code, rec.params) if r is None else (r(rec) if callable(r) else r)
        for item in out:
            data = item if isinstance(item, str) else json.dumps(item)
            yield wire.decode_rec_response(wire.encode_rec_response(wire.RecResponse(req.name_code, data)))

    # helpers for the tests
    def of(self, name_code):
        with self._lock:
            return [s for s in self.sent if s.name_code == name_code]

    def codes(self):
        with self._lock:
            return [s.name_code for s in self.sent]


@pytest.fixture
def fake():
    return FakeTransport()


@pytest.fixture
def dog(fake):
    d = CyberDog(transport=fake)
    yield d
    d.drive.halt()
    d.posture.halt()
    d.drive.join(2)
    d.posture.join(2)
