"""The fake dog: GrpcApp over mTLS with the dog's own certificate (grpc_fidelity.md sections 2-6).

sendMsg and getFile are served through a generic handler; heartbeat is not registered, so it answers
UNIMPLEMENTED like the dog's :50052. Every stream ends with status OK, zero messages included.
"""
from __future__ import annotations

import collections
import os
import shutil
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import grpc

from emulator import ROOT
from emulator.camera import Camera
from emulator.common import Crash, Dep, Hung, MISSING, ParseError, get, jdump, rj_parse, wire
from emulator.control import Control
from emulator.device import Device
from emulator.manager import ARRAYS, CAP, Manager
from emulator.nav import Nav
from emulator.vp import VP

FLOOR = 0.2                     # a scaled wait never drops below min(t, FLOOR) seconds
OPTIONS = [("grpc.max_send_message_length", 5_242_880), ("grpc.max_receive_message_length", 5_242_880),
           ("grpc.keepalive_time_ms", 1000), ("grpc.keepalive_timeout_ms", 1000),
           ("grpc.http2.min_ping_interval_without_data_ms", 500),
           ("grpc.http2.min_time_between_pings_ms", 1000)]          # app.cpp:577-597
DANGEROUS = {9030, 9031, 10006, 12001, 55001}                       # contracts.md 1, rule 1
LOG = 200


class Emulator:
    def __init__(self, port=0, host="127.0.0.1", controller="fake", time_scale=1.0, control_port=0,
                 cert_dir=None):
        self.host, self.port, self.control_port = host, port, control_port
        self.cert_dir = cert_dir or os.path.join(ROOT, "certs")
        self.controller = controller
        self.time_scale = float(time_scale) if controller == "fake" else 1.0   # Gazebo runs in real time
        self.stopping = threading.Event()
        self.lk = threading.Lock()
        self.deps = {}
        if controller == "lcm":
            from emulator.controllers.lcm_link import LcmController      # part C; ImportError off the VM
            self.ctl = LcmController()
        else:
            from emulator.controllers.fake import FakeController
            self.ctl = FakeController(self.time_scale)
        self.fs_root = tempfile.mkdtemp(prefix="cyberdog_emu_")
        self._logs()
        self.device = Device(self)
        self.mgr = Manager(self.ctl, self)
        self.nav = Nav(self)
        self.cam = Camera(self)
        self.vp = VP(self)
        d = self.device
        self.handlers = {
            1001: d.h1001, 1002: self.h1002, 1004: self.h1004, 1012: d.h1012, 2002: self.h2002,
            4002: self.cam.send_msg, 6002: self.nav.h6002, 6003: self.nav.h6003, 6004: self.nav.hnav,
            6009: self.nav.h6009, 6010: self.nav.hnav, 10001: d.h10001, 11001: d.h11001, 11002: d.h11002,
            9030: d.h9030, 9031: d.h9031, 10006: d.h10006, 12001: d.h12001, 55001: d.h55001,
            5010: d.hota_action, 5011: d.hota_action}
        for nc in (5001, 5002, 5003, 5004, 5005, 5006):
            self.handlers[nc] = d.hota
        self.server = self.control = None

    def _logs(self):
        self.calls = collections.Counter()
        self.refused = collections.deque(maxlen=LOG)
        self.bugs = collections.deque(maxlen=LOG)
        self.dangerous_calls = collections.deque(maxlen=LOG)
        self.crashed = None

    # ---- services used by the modules --------------------------------------------------
    def sc(self, t):
        """Scale a dog-side duration (contracts.md 4.1), never below min(t, FLOOR)."""
        return max(t * self.time_scale, min(t, FLOOR))

    def sleep(self, t):
        self.stopping.wait(self.sc(t))

    def dep(self, name):
        with self.lk:
            if name not in self.deps:
                self.deps[name] = Dep(self, name)
            return self.deps[name]

    def faults(self):
        with self.lk:
            return {n: d.mode for n, d in self.deps.items() if d.mode != "ok"}

    def protected(self):
        return self.device.protected()

    def msg(self, nc, data):
        return wire.RecResponse(nc, data)

    def err(self, code, nc):
        return self.msg(nc, jdump({"error_code": code}))

    def refuse(self, nc, reason, **kw):
        self.refused.append(dict(t=round(time.time(), 3), nameCode=nc, reason=reason, **kw))

    def client_bug(self, msg, **kw):
        self.bugs.append(dict(t=round(time.time(), 3), bug=msg, **kw))

    def dangerous_event(self, msg, **kw):
        self.dangerous_calls.append(dict(t=round(time.time(), 3), event=msg, **kw))

    # ---- handlers kept here -------------------------------------------------------------
    def h1002(self, doc, nc, raw):                       # app.cpp:2900-2922: publish once, answer ""
        f = {k: get(doc, k, "int32", 0) for k in ("motion_id", "cmd_type", "value")}
        for name, n in ARRAYS:
            f[name] = self._floats(doc, name, n, nc)
        self.mgr.servo_frame(f)
        yield self.msg(nc, "")

    def h1004(self, doc, nc, raw):                       # app.cpp:2924-2966, callMotionServoCmd :753-780
        dep = self.dep("motion_result_cmd")
        if not dep.up(3):
            yield self.err(321, nc)
            return
        req = {}
        for k in ("motion_id", "contact", "duration", "value"):
            req[k] = get(doc, k, "int32", 0)
            if isinstance(doc, dict) and k in doc and get(doc, k, "int32") is MISSING:
                self.client_bug("1004 %s=%r is not an int32 literal: read as 0" % (k, doc[k]), field=k)
        if not isinstance(doc, dict) or "motion_id" not in doc:
            self.client_bug("1004 without motion_id: the request's 0 is ESTOP")
        for name, n in ARRAYS:
            req[name] = self._floats(doc, name, n, nc)
        if not dep.ready(CAP):
            yield self.err(321, nc)
            return
        r = self.mgr.motion_cmd_grpc(req)
        if r is None:
            self.refuse(nc, "321: no reply within the 20 s cap (the motion goes on)", motion_id=req["motion_id"])
            yield self.err(321, nc)
            return
        if not r["result"]:
            self.refuse(nc, "code %d" % r["code"], motion_id=req["motion_id"])
        yield self.msg(nc, jdump({"motion_id": r["motion_id"], "result": r["result"], "code": r["code"]}))

    def _floats(self, doc, name, n, nc):
        if not isinstance(doc, dict) or name not in doc:
            return []
        v = get(doc, name, "floats")                     # raises Crash on a scalar
        if len(v) != len(doc[name]):
            self.client_bug("%d %s: an int literal stops the float copy (IsFloat)" % (nc, name), value=doc[name])
        if len(v) != n:                                  # GET_VALUE in motion_manager (motion_macros.hpp:111-121)
            self.client_bug("%d %s: %d floats, %d expected: sent as zeros" % (nc, name, len(v), n), value=doc[name])
        return v

    def h2002(self, doc, nc, raw):                       # published on frontend_message; zero messages
        self.vp.frontend(raw)
        return
        yield

    # ---- gRPC ---------------------------------------------------------------------------
    def _send_msg(self, req, ctx):
        nc = req.name_code
        if self.crashed:
            ctx.abort(grpc.StatusCode.UNAVAILABLE, "emulated cyberdog_grpc abort: " + self.crashed)
        self.calls[str(nc)] += 1
        try:
            doc = rj_parse(req.params)
        except ParseError:
            self.refuse(nc, "params are not valid JSON for rapidjson: 323 with nameCode 0", params=req.params[:200])
            yield self.err(323, 0)
            return
        if nc in DANGEROUS or 5000 <= nc <= 5999:
            self.dangerous_calls.append(dict(t=round(time.time(), 3), nameCode=nc, params=req.params[:500]))
        h = self.handlers.get(nc)
        if h is None:
            return                                       # default: break -> zero messages, OK
        try:
            for m in h(doc, nc, req.params):
                yield m
        except Crash as e:
            self.crashed = "%d: %s" % (nc, e)
            self.refuse(nc, "CRASH: " + str(e))
            ctx.abort(grpc.StatusCode.UNAVAILABLE, "emulated cyberdog_grpc abort: " + self.crashed)
        except Hung:
            return

    def _get_file(self, req, ctx):
        nc = req.name_code
        if self.crashed:
            ctx.abort(grpc.StatusCode.UNAVAILABLE, "emulated cyberdog_grpc abort: " + self.crashed)
        self.calls["getFile:%d" % nc] += 1
        try:
            doc = rj_parse(req.params)
        except ParseError:
            doc = {}                                     # parse errors are ignored on getFile
        if nc in (4002, 4004):
            for c in self.cam.get_file(doc, nc, ctx):
                yield c
        else:
            yield wire.FileChunk(255, "", 0, b"")

    def start(self):
        rd = lambda n: open(os.path.join(self.cert_dir, n), "rb").read()
        creds = grpc.ssl_server_credentials([(rd("client-key.pem"), rd("client-cert.pem"))],
                                            root_certificates=rd("ca-cert.pem"), require_client_auth=True)
        handler = grpc.method_handlers_generic_handler(wire.SERVICE, {
            "sendMsg": grpc.unary_stream_rpc_method_handler(
                self._send_msg, request_deserializer=wire.decode_send_request,
                response_serializer=wire.encode_rec_response),
            "getFile": grpc.unary_stream_rpc_method_handler(
                self._get_file, request_deserializer=wire.decode_send_request,
                response_serializer=wire.encode_file_chunk)})
        opts = OPTIONS
        if self.host in ("0.0.0.0", "::", "[::]"):
            # grpc listens on one dual-stack tcp6 socket for a wildcard, and WSL forwards a tcp6 listener
            # to Windows [::1] only. One socket per interface gives real IPv4 listeners, so 127.0.0.1 works.
            opts = OPTIONS + [("grpc.expand_wildcard_addrs", 1)]
        self.server = grpc.server(ThreadPoolExecutor(max_workers=100, thread_name_prefix="grpc"), options=opts)
        self.server.add_generic_rpc_handlers((handler,))
        self.port = self.server.add_secure_port("%s:%d" % (self.host, self.port), creds)
        if not self.port:
            raise RuntimeError("cannot bind %s" % self.host)
        self.server.start()
        self.control = Control(self, self.host, self.control_port)
        self.control_port = self.control.port
        return self.port, self.control_port

    def stop(self):
        self.stopping.set()
        with self.lk:
            for d in self.deps.values():
                d.set("ok")                              # releases hung handlers
        if self.server is not None:
            self.server.stop(0.5).wait(2)
        if self.control is not None:
            self.control.close()
        self.mgr.close()
        self.ctl.close()
        shutil.rmtree(self.fs_root, ignore_errors=True)

    # ---- control endpoint -----------------------------------------------------------------
    def reset(self):
        with self.lk:
            for d in self.deps.values():
                d.set("ok")
        self._logs()
        self.device.reset()
        self.mgr.reset()
        self.nav.reset()
        self.cam.reset()
        self.vp.reset()

    def state(self):
        d = self.device
        return {"motion": self.mgr.state(), "ground_truth": self.ctl.ground_truth(), "leds": self.vp.leds.state(),
                "skin": dict(d.skin), "servo": self.mgr.servo_state(), "nav": self.nav.state(),
                "camera": self.cam.state(), "calls": dict(self.calls), "refused": list(self.refused),
                "dangerous_calls": list(self.dangerous_calls), "client_bugs": list(self.bugs),
                "battery": {"soc": d.soc, "charging": d.charging, "wireless": d.wireless,
                            "protected": d.protected()},
                "vp": self.vp.state(), "faults": self.faults(), "crashed": self.crashed,
                "provider_ip": d.provider_ip, "time_scale": self.time_scale, "controller": self.controller,
                "fs_root": self.fs_root}
