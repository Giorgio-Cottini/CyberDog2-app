"""Pieces shared by the emulator services: rapidjson-faithful parsing, CyberdogJson::Get typing,
ROS dependencies with fault injection (grpc_fidelity.md sections 3, 4, 9 and 10)."""
from __future__ import annotations

import importlib.util
import json
import math
import os
import struct
import threading

from emulator import ROOT


def _load_wire():
    """cyberdog_sdk/wire.py loaded by path: `import cyberdog_sdk.wire` would run cyberdog_sdk/__init__.py,
    which imports the whole SDK (built in parallel, and not needed here)."""
    spec = importlib.util.spec_from_file_location("cyberdog_wire", os.path.join(ROOT, "cyberdog_sdk", "wire.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


wire = _load_wire()
FLT_MAX = 3.4028234663852886e38
MISSING = object()


class ParseError(Exception):
    """rapidjson Parse<0> failed: the bridge answers {"error_code":323} with nameCode 0."""


class Crash(Exception):
    """RAPIDJSON_ASSERT or an uncaught C++ exception: the whole cyberdog_grpc process dies."""


class Hung(Exception):
    """A blocking future.get() was released by /reset, /fault or stop: the RPC ends with no message."""


def rj_parse(s: str):
    s = s.split("\x00", 1)[0]                      # params().c_str() stops at the first NUL
    if not s.strip():
        raise ParseError("empty")

    def no_const(c):                               # NaN / Infinity
        raise ParseError(c)

    def first_wins(pairs):                         # rapidjson FindMember returns the first duplicate
        d = {}
        for k, v in pairs:
            d.setdefault(k, v)
        return d

    def fl(x):
        f = float(x)
        if math.isinf(f):                          # 1e400: kParseErrorNumberTooBig
            raise ParseError(x)
        return f

    def num(x):                                    # past the 64-bit range ParseNumber goes on as a double
        i = int(x)
        return i if -2 ** 63 <= i < 2 ** 64 else fl(x)
    try:
        doc = json.loads(s, parse_constant=no_const, object_pairs_hook=first_wins, parse_float=fl, parse_int=num)
    except (ValueError, RecursionError):
        raise ParseError(s[:40])
    if _lone_high(doc):                            # kParseErrorStringUnicodeSurrogateInvalid
        raise ParseError("surrogate")
    return doc


def _lone_high(v) -> bool:
    """Python json keeps an unpaired high surrogate (U+D800-U+DBFF); rapidjson 1.1.0 rejects it.
    A lone low half passes both (reader.h ParseStringToStream checks only the high half)."""
    if isinstance(v, str):
        return any("\ud800" <= c <= "\udbff" for c in v)
    if isinstance(v, dict):
        return any(_lone_high(k) or _lone_high(x) for k, x in v.items())
    if isinstance(v, list):
        return any(_lone_high(x) for x in v)
    return False


def int32(v) -> bool:
    return type(v) is int and -2 ** 31 <= v < 2 ** 31


def get(doc, key, typ, default=MISSING):
    """CyberdogJson::Get (json.hpp:476-635). A failed Get leaves the default."""
    if not isinstance(doc, dict) or key not in doc:
        return default
    v = doc[key]
    if typ is bool:
        return v if isinstance(v, bool) else default
    if typ == "int32":
        return v if int32(v) else default
    if typ == "uint":
        return v if type(v) is int and 0 <= v < 2 ** 32 else default
    if typ == "uint64":
        return v if type(v) is int and 0 <= v < 2 ** 64 else default
    if typ == "float":
        if type(v) is float and abs(v) <= FLT_MAX:
            return v
        return float(v) if int32(v) else default
    if typ == "double":
        if type(v) is float:
            return v
        return float(v) if int32(v) else default
    if typ is str:
        return v if isinstance(v, str) else default
    if typ == "floats":
        if not isinstance(v, list):                # GetArray() with no IsArray check
            raise Crash("RAPIDJSON_ASSERT: %r is not an array" % key)
        out = []
        for e in v:                                # IsFloat: an int literal stops the copy
            if type(e) is not float or abs(e) > FLT_MAX:
                break
            out.append(e)
        return out
    raise ValueError(typ)


def is_double(v) -> bool:
    """rapidjson IsDouble: a number written with a decimal point or exponent."""
    return type(v) is float


def f32(x: float) -> float:
    """float32 widened to double, as ROS float32 fields come back through rapidjson."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def jdump(o) -> str:
    return json.dumps(o, separators=(",", ":"), ensure_ascii=False)


class Dep:
    """One ROS service or action the bridge calls, with fault injection (grpc_fidelity.md section 9 item 11).
    ok: answers at once. absent: wait_for_service fails after its wait. timeout: answers after the
    bridge's timeout. hang: never answers; a blocking get() waits until the fault is released."""

    def __init__(self, emu, name):
        self.emu, self.name, self.mode = emu, name, "ok"
        self.release = threading.Event()

    def set(self, mode):
        self.mode = mode
        self.release.set()                         # wake anything blocked on the old mode
        self.release = threading.Event()

    def up(self, wait=3.0) -> bool:                # wait_for_service(wait)
        if self.mode == "absent":
            self.emu.sleep(wait)
            return False
        return True

    def ready(self, timeout) -> bool:              # future.wait_for(timeout)
        if self.mode in ("timeout", "hang"):
            self.emu.sleep(timeout)
            return False
        return True

    def block(self):                               # future.get() after a timed-out wait_for
        if self.mode == "hang":
            ev = self.release
            while not ev.wait(0.2):
                if self.emu.stopping.is_set():
                    break
            raise Hung(self.name)
