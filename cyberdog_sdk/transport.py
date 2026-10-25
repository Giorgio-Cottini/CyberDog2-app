"""mTLS transport to the dog's GrpcApp service (contracts.md §1, §3).

Two channels with separate TCP connections: "main" for everything and
"priority" for ESTOP, so an ESTOP never waits behind another RPC. No locks:
gRPC multiplexes concurrent calls on a channel.

Every request goes through _prepare(), which enforces the safety rules that
apply at the wire level: forbidden name codes (rule 1), the 6004 type and the
6002 delete-all guards (rules 2-3, also enforced in nav.py), the motion value
mask (rule 4), the 4004 file-name guard (rule 5), never-empty params (rule 6)
and exact JSON typing (rule 7).
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Iterator, List, NamedTuple, Optional

import grpc

from . import wire

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_HOST = "192.168.137.115"
DEFAULT_PORT = 50052
TARGET_NAME = "cyberdog2.server"

FORBIDDEN = frozenset((9030, 9031, 10006, 12001, 55001))   # plus 5000-5999 (OTA)
NAV_TYPES = (1, 5, 7, 9, 11, 13)
VALUE_CALIBRATION_BIT = 0x04     # factory speed-offset calibration (motion.md §5.4)

ARRAY_SIZES = {"vel_des": 3, "rpy_des": 3, "pos_des": 3, "ctrl_point": 3,
               "acc_des": 6, "foot_pose": 6, "step_height": 2}
INT_FIELDS = {
    1002: ("motion_id", "cmd_type", "value"),
    1004: ("motion_id", "duration", "value", "contact"),
    1012: ("volume",),
    4002: ("command",),
    6004: ("type", "relative_pos"),
    6009: ("type",),
    6010: ("type",),
    11002: ("mode", "wave_cycle_time"),
}
FLOAT_FIELDS = {6004: ("goalX", "goalY", "theta", "keep_distance")}
INT32 = (-2 ** 31, 2 ** 31 - 1)


class DogError(Exception):
    """The dog answered with an error, or did not answer in the expected form."""

    def __init__(self, message: str, code: Optional[int] = None, name_code: Optional[int] = None):
        super().__init__(message)
        self.code = code
        self.name_code = name_code


class DogUnreachable(DogError):
    """The RPC itself failed (connection, TLS, deadline); `status` is the grpc.StatusCode."""

    def __init__(self, message: str, status=None, name_code: Optional[int] = None):
        super().__init__(message, None, name_code)
        self.status = status


class ForbiddenCommand(Exception):
    """Refused by an SDK safety rule before anything was sent."""


class RiskyMotion(ForbiddenCommand):
    """An R3/R4 (or unknown) motion without allow_risky=True."""


class FileResult(NamedTuple):
    error_code: int     # last chunk's error_code; -1 when the stream carried no chunk
    file_name: str
    data: bytes


# ---- typing helpers -----------------------------------------------------------
def as_int(v, key: str) -> int:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError("%s must be an int, got %r" % (key, v))
    if isinstance(v, float):
        if not v.is_integer():
            raise ValueError("%s must be an int, got %r" % (key, v))
        v = int(v)
    if not INT32[0] <= v <= INT32[1]:
        raise ValueError("%s out of int32 range: %r" % (key, v))
    return v


def as_float(v, key: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError("%s must be a number, got %r" % (key, v))
    f = float(v)
    if not math.isfinite(f):
        raise ValueError("%s must be finite, got %r" % (key, v))
    return f


def as_floats(v, key: str, size: int) -> List[float]:
    if not isinstance(v, (list, tuple)) or len(v) != size:
        raise ValueError("%s must be a list of %d numbers, got %r" % (key, size, v))
    return [as_float(x, key) for x in v]


def _check_file_name(name) -> None:
    # 4004 opens /home/mi/Camera/<name> unsanitised and deletes it after sending
    # (transmit_files.hpp:195). "", "." or a NUL-led name would target the folder itself.
    if (not isinstance(name, str) or not name or name == "." or "\x00" in name
            or "/" in name or "\\" in name or ".." in name):
        raise ForbiddenCommand("4004 file_name refused: %r" % (name,))


def prepare(name_code: int, params=None, force: bool = False) -> str:
    """Validate, coerce and encode params for name_code. Returns the JSON string to send."""
    if isinstance(name_code, bool) or not isinstance(name_code, int):
        raise ValueError("name_code must be an int, got %r" % (name_code,))
    if not 0 <= name_code <= 0xFFFFFFFF:     # the wire keeps the low 32 bits: 9031 + 2**32 is 9031
        raise ValueError("name_code must fit in 32 bits, got %r" % (name_code,))
    if (name_code in FORBIDDEN or 5000 <= name_code <= 5999) and not force:
        raise ForbiddenCommand("name code %d is never sent without force=True" % name_code)
    if params is None:
        p = {}
    elif isinstance(params, (str, bytes)):
        text = params.decode("utf-8") if isinstance(params, bytes) else params
        p = json.loads(text) if text.strip() else {}
    else:
        p = params
    if not isinstance(p, dict):
        # a non-object hits raw rapidjson asserts in 6002/6004/6010 (grpc_fidelity.md §3)
        raise ValueError("params must be a JSON object, got %r" % (p,))
    p = dict(p)

    for key in INT_FIELDS.get(name_code, ()):
        if key in p:
            p[key] = as_int(p[key], key)
    for key in FLOAT_FIELDS.get(name_code, ()):
        if key in p:
            p[key] = as_float(p[key], key)

    if name_code in (1002, 1004):
        if "motion_id" not in p:
            raise ValueError("%d needs motion_id" % name_code)
        for key, size in ARRAY_SIZES.items():
            if key in p:
                p[key] = as_floats(p[key], key, size)
        if "value" in p:
            p["value"] &= ~VALUE_CALIBRATION_BIT
    elif name_code == 4002:
        # without a valid uint command, sendMsg 4002 defaults to START_RECORDING (2)
        if not isinstance(p.get("command"), int) or not 0 <= p["command"] <= 10:
            raise ValueError("4002 needs an int command 0..10, got %r" % (p.get("command"),))
    elif name_code == 4004:
        _check_file_name(p.get("file_name"))
    elif name_code == 6004:
        _check_nav_start(p)
    elif name_code == 6002:
        _check_labels(p, force)

    return json.dumps(p, separators=(",", ":"), allow_nan=False)


def _check_nav_start(p: dict) -> None:
    if p.get("type") not in NAV_TYPES:
        raise ForbiddenCommand("6004 type must be one of %s, got %r" % (NAV_TYPES, p.get("type")))
    # type and keep_distance are read uninitialised when missing (grpc_fidelity.md §4)
    if not isinstance(p.get("outdoor"), bool):
        raise ValueError("6004 needs outdoor as a bool")
    if "keep_distance" not in p:
        raise ValueError("6004 needs keep_distance")
    if p["type"] == 1 and ("goalX" not in p or "goalY" not in p):
        raise ValueError("6004 type 1 needs goalX and goalY")


def _check_labels(p: dict, force: bool) -> None:
    only_delete = p.get("only_delete", False)
    if not isinstance(only_delete, bool):
        raise ValueError("6002 only_delete must be a bool")
    labels = p.get("locationLabelInfo")
    if labels is None:
        labels = []
    if not isinstance(labels, list):
        raise ValueError("6002 locationLabelInfo must be a list")
    out = []
    for lab in labels:
        if not isinstance(lab, dict) or not isinstance(lab.get("labelName"), str) or not lab["labelName"]:
            raise ValueError("6002 label needs a non-empty labelName: %r" % (lab,))
        lab = dict(lab)
        # int literals make the bridge drop the label (nav_map.md §2)
        lab["physicX"] = as_float(lab.get("physicX"), "physicX")
        lab["physicY"] = as_float(lab.get("physicY"), "physicY")
        out.append(lab)
    if only_delete and not out and not force:
        raise ForbiddenCommand("6002 only_delete with no labels deletes every map; "
                               "pass confirm_delete_all=True")
    if "locationLabelInfo" in p or only_delete:
        p["locationLabelInfo"] = out


def error_code(item) -> Optional[int]:
    """The bridge error code of a reply item ({"error_code":N}), else None."""
    if isinstance(item, dict) and "error_code" in item:
        code = item["error_code"]
        return code if isinstance(code, int) else -1
    return None


class Transport:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, cert_dir: Optional[str] = None):
        self.host = host
        self.port = port
        self.cert_dir = cert_dir or os.path.join(ROOT, "certs")
        self._channels = {"main": self.new_channel(), "priority": self.new_channel()}

    # ---- channels ---------------------------------------------------------------
    def new_channel(self):
        """A fresh mTLS channel with its own TCP connection (no lazy connect is forced)."""
        def read(name):
            with open(os.path.join(self.cert_dir, name), "rb") as f:
                return f.read()
        creds = grpc.ssl_channel_credentials(
            root_certificates=read("ca-cert.pem"),
            private_key=read("client-key.pem"),
            certificate_chain=read("client-cert.pem"))
        opts = (("grpc.ssl_target_name_override", TARGET_NAME),
                ("grpc.default_authority", TARGET_NAME),
                ("grpc.max_receive_message_length", 32 * 1024 * 1024),
                # the priority channel carries only ESTOPs: never let it fall idle (default 30 min),
                # or the next ESTOP first redoes TCP + mTLS (about 0.1 s in the sim, more over Wi-Fi)
                ("grpc.client_idle_timeout_ms", 2 ** 31 - 1),
                # after a link loss, try again every 0.5-1 s (default 1 s growing to 120 s): a
                # channel in backoff fails every RPC at once, ESTOP included
                ("grpc.initial_reconnect_backoff_ms", 500),
                ("grpc.max_reconnect_backoff_ms", 1000),
                # identical args would otherwise share one connection via the global pool
                ("grpc.use_local_subchannel_pool", 1))
        return grpc.secure_channel("%s:%s" % (self.host, self.port), creds, options=opts)

    def connect(self, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        for lane, ch in self._channels.items():
            try:
                grpc.channel_ready_future(ch).result(timeout=max(0.0, deadline - time.monotonic()))
            except grpc.FutureTimeoutError:
                raise DogUnreachable("%s channel to %s:%s not ready within %.1f s"
                                     % (lane, self.host, self.port, timeout)) from None

    def close(self) -> None:
        for ch in self._channels.values():
            try:
                ch.close()
            except Exception:
                pass

    # ---- RPC core ---------------------------------------------------------------
    def _rpc(self, method: str, request: wire.SendRequest, timeout: float, lane="main") -> Iterator:
        """Yield decoded response messages. `lane` is "main", "priority" or a channel."""
        ch = self._channels[lane] if isinstance(lane, str) else lane
        decode = wire.decode_rec_response if method == wire.SEND_MSG else wire.decode_file_chunk
        fn = ch.unary_stream(method, request_serializer=wire.encode_send_request,
                             response_deserializer=decode)
        call = fn(request, timeout=timeout)
        try:
            for msg in call:
                yield msg
        except grpc.RpcError as e:
            code = e.code() if hasattr(e, "code") else None
            details = e.details() if hasattr(e, "details") else str(e)
            raise DogUnreachable("%s %d: %s %s" % (method, request.name_code, code, details),
                                 status=code, name_code=request.name_code) from None
        finally:
            call.cancel()

    # ---- public API -------------------------------------------------------------
    def call(self, name_code: int, params=None, timeout: float = 10.0, force: bool = False,
             priority: bool = False) -> List[str]:
        req = wire.SendRequest(name_code, prepare(name_code, params, force))
        lane = "priority" if priority else "main"
        return [m.data for m in self._rpc(wire.SEND_MSG, req, timeout, lane)]

    def call_json(self, name_code: int, params=None, timeout: float = 10.0, force: bool = False,
                  priority: bool = False) -> List[dict]:
        out = []
        for data in self.call(name_code, params, timeout, force, priority):
            try:
                item = json.loads(data)
            except ValueError:
                continue
            if isinstance(item, dict):
                out.append(item)
        return out

    def call_one(self, name_code: int, params=None, timeout: float = 10.0) -> dict:
        """First JSON reply; DogError on an error_code reply or on no reply at all."""
        items = self.call_json(name_code, params, timeout)
        if not items:
            raise DogError("%d: no reply" % name_code, None, name_code)
        code = error_code(items[0])
        if code is not None:
            raise DogError("%d: error_code %d" % (name_code, code), code, name_code)
        return items[0]

    def stream(self, name_code: int, params=None, timeout: float = 10.0) -> Iterator[str]:
        """Guards run now; the returned iterator yields each reply's data as it arrives."""
        req = wire.SendRequest(name_code, prepare(name_code, params))
        return (m.data for m in self._rpc(wire.SEND_MSG, req, timeout, "main"))

    def get_file(self, name_code: int, params, timeout: float = 30.0, channel=None) -> FileResult:
        req = wire.SendRequest(name_code, prepare(name_code, params))
        buf = bytearray()
        err, name = -1, ""
        for chunk in self._rpc(wire.GET_FILE, req, timeout, channel or "main"):
            err = chunk.error_code
            name = chunk.file_name or name
            buf += chunk.buffer
        return FileResult(err, name, bytes(buf))
