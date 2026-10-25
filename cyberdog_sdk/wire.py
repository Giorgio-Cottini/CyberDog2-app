"""Protobuf wire codec for the CyberDog 2 gRPC service (proto/cyberdog_app.proto).

Hand-written so that the SDK and the emulator need only grpcio, on any Python
from 3.8 up. The generated stubs in proto/ pin protobuf >= 7.35 and grpcio >=
1.84, which do not exist for the Python 3.8 inside the simulator VM.

Only the three messages that cross the forward channel are modelled:
SendRequest, RecResponse and FileChunk. Proto3 rules apply: a field at its
default value is not written, a missing field decodes to its default, and an
unknown field is skipped.
"""
from __future__ import annotations

import struct
from typing import Dict, NamedTuple, Tuple

SERVICE = "grpcapi.GrpcApp"
SEND_MSG = "/grpcapi.GrpcApp/sendMsg"      # SendRequest -> stream RecResponse
GET_FILE = "/grpcapi.GrpcApp/getFile"      # SendRequest -> stream FileChunk
HEARTBEAT = "/grpcapi.GrpcApp/heartbeat"   # Ticks -> Result (not served on :50052)

_VARINT, _FIXED64, _LEN, _FIXED32 = 0, 1, 2, 5


class SendRequest(NamedTuple):
    name_code: int = 0      # fixed32 nameCode = 1
    params: str = ""        # string params = 2


class RecResponse(NamedTuple):
    name_code: int = 0      # fixed32 nameCode = 1
    data: str = ""          # string data = 2


class FileChunk(NamedTuple):
    error_code: int = 0     # fixed32 error_code = 1
    file_name: str = ""     # string file_name = 2
    file_size: int = 0      # fixed32 file_size = 3
    buffer: bytes = b""     # bytes buffer = 4


# ---- primitives ---------------------------------------------------------------
def _varint(n: int) -> bytes:
    n &= 0xFFFFFFFFFFFFFFFF
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _fixed32(field: int, value: int) -> bytes:
    if not value:
        return b""
    return _varint(field << 3 | _FIXED32) + struct.pack("<I", value & 0xFFFFFFFF)


def _bytes(field: int, value: bytes) -> bytes:
    if not value:
        return b""
    return _varint(field << 3 | _LEN) + _varint(len(value)) + value


def _read_varint(buf: bytes, pos: int) -> Tuple[int, int]:
    shift = result = 0
    while True:
        if pos >= len(buf):
            raise ValueError("truncated varint")
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, pos
        shift += 7
        if shift > 63:
            raise ValueError("varint too long")


def _fields(buf: bytes) -> Dict[int, object]:
    """Parse one message into {field_number: value}. A repeated scalar keeps the last value."""
    out: Dict[int, object] = {}
    pos = 0
    while pos < len(buf):
        key, pos = _read_varint(buf, pos)
        field, wt = key >> 3, key & 7
        if wt == _VARINT:
            out[field], pos = _read_varint(buf, pos)
        elif wt == _FIXED32:
            if pos + 4 > len(buf):
                raise ValueError("truncated fixed32")
            out[field] = struct.unpack_from("<I", buf, pos)[0]
            pos += 4
        elif wt == _FIXED64:
            if pos + 8 > len(buf):
                raise ValueError("truncated fixed64")
            out[field] = struct.unpack_from("<Q", buf, pos)[0]
            pos += 8
        elif wt == _LEN:
            n, pos = _read_varint(buf, pos)
            if pos + n > len(buf):
                raise ValueError("truncated length-delimited field")
            out[field] = bytes(buf[pos:pos + n])
            pos += n
        else:
            raise ValueError("unsupported wire type %d" % wt)
    return out


def _str(v: object) -> str:
    return v.decode("utf-8", errors="replace") if isinstance(v, bytes) else ""


def _int(v: object) -> int:
    return v if isinstance(v, int) else 0


# ---- messages -----------------------------------------------------------------
def encode_send_request(m: SendRequest) -> bytes:
    return _fixed32(1, m.name_code) + _bytes(2, m.params.encode("utf-8"))


def decode_send_request(buf: bytes) -> SendRequest:
    f = _fields(buf)
    return SendRequest(_int(f.get(1)), _str(f.get(2)))


def encode_rec_response(m: RecResponse) -> bytes:
    return _fixed32(1, m.name_code) + _bytes(2, m.data.encode("utf-8"))


def decode_rec_response(buf: bytes) -> RecResponse:
    f = _fields(buf)
    return RecResponse(_int(f.get(1)), _str(f.get(2)))


def encode_file_chunk(m: FileChunk) -> bytes:
    return (_fixed32(1, m.error_code) + _bytes(2, m.file_name.encode("utf-8"))
            + _fixed32(3, m.file_size) + _bytes(4, m.buffer))


def decode_file_chunk(buf: bytes) -> FileChunk:
    f = _fields(buf)
    raw = f.get(4)
    return FileChunk(_int(f.get(1)), _str(f.get(2)), _int(f.get(3)),
                     raw if isinstance(raw, bytes) else b"")
