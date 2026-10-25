"""Codec round trip of cyberdog_sdk.wire (the hand-written protobuf codec)."""
from __future__ import annotations

import pytest

from cyberdog_sdk import wire


@pytest.mark.parametrize("msg", [
    wire.SendRequest(1004, '{"motion_id":111,"duration":0}'),
    wire.SendRequest(0, ""),
    wire.SendRequest(0xFFFFFFFF, "{}"),
    wire.SendRequest(2002, '{"body":"print(\'caffè ✓\')"}'),
])
def test_send_request_round_trip(msg):
    assert wire.decode_send_request(wire.encode_send_request(msg)) == msg


@pytest.mark.parametrize("msg", [
    wire.RecResponse(1004, '{"motion_id":111,"result":true,"code":0}'),
    wire.RecResponse(0, '{"error_code":323}'),
    wire.RecResponse(1002, ""),
])
def test_rec_response_round_trip(msg):
    assert wire.decode_rec_response(wire.encode_rec_response(msg)) == msg


@pytest.mark.parametrize("msg", [
    wire.FileChunk(0, "pc_1.json", 12, b'{"a": 1.0}\x00\xff'),
    wire.FileChunk(9, "", 0, b""),
    wire.FileChunk(0, "big.jpg", 5 * 1024 * 1024, bytes(range(256)) * 4096),
])
def test_file_chunk_round_trip(msg):
    assert wire.decode_file_chunk(wire.encode_file_chunk(msg)) == msg


def test_defaults_are_not_written():
    assert wire.encode_send_request(wire.SendRequest(0, "")) == b""
    # nameCode is fixed32 (field 1, wire type 5)
    assert wire.encode_send_request(wire.SendRequest(1, "")) == b"\x0d\x01\x00\x00\x00"


def test_unknown_field_is_skipped():
    buf = wire.encode_rec_response(wire.RecResponse(7, "x")) + b"\x18\x96\x01"   # field 3 varint 150
    assert wire.decode_rec_response(buf) == wire.RecResponse(7, "x")


def test_truncated_input_raises():
    buf = wire.encode_send_request(wire.SendRequest(1004, "{}"))
    with pytest.raises(ValueError):
        wire.decode_send_request(buf[:-1])
