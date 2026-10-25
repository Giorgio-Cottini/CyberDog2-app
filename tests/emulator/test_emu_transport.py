"""Transport and parsing fidelity: mTLS, UNIMPLEMENTED heartbeat, zero-message codes, rapidjson traps, crash model."""
from __future__ import annotations

import os

import grpc
import pytest

from emulator.common import wire


def test_tls_refuses_a_client_without_certificate(dog):
    with open(os.path.join(dog.emu.cert_dir, "ca-cert.pem"), "rb") as f:
        creds = grpc.ssl_channel_credentials(f.read())                   # no client key pair
    ch = grpc.secure_channel("127.0.0.1:%d" % dog.port, creds,
                             options=[("grpc.ssl_target_name_override", "cyberdog2.server")])
    send = ch.unary_stream(wire.SEND_MSG, request_serializer=wire.encode_send_request,
                           response_deserializer=wire.decode_rec_response)
    with pytest.raises(grpc.RpcError) as e:
        list(send(wire.SendRequest(10001, "{}"), timeout=5))
    assert e.value.code() == grpc.StatusCode.UNAVAILABLE
    ch.close()
    assert dog.call(10001)[0][0] == 10001                                # the certified client still works


def test_heartbeat_is_unimplemented(dog):
    hb = dog.ch.unary_unary(wire.HEARTBEAT, request_serializer=lambda b: b, response_deserializer=lambda b: b)
    with pytest.raises(grpc.RpcError) as e:
        hb(b"", timeout=5)
    assert e.value.code() == grpc.StatusCode.UNIMPLEMENTED


def test_unknown_and_push_only_codes_give_zero_messages(dog):
    assert dog.call(7777) == []
    assert dog.call(1003) == []
    assert dog.call(2002, '{"type":"task"}') == []                       # fire and forget
    chunks = dog.files(1234)
    assert [c.error_code for c in chunks] == [255]


@pytest.mark.parametrize("params", ["", "   ", '{"a":NaN}', '{"a":1,}', '{"a":1e400}', "{} x", "{/*c*/}",
                                    '{"a":1%s}' % ("0" * 400), '{"a":"\\ud800"}', '{"\\udbffx":1}'])
def test_rapidjson_parse_errors_reply_323_with_namecode_0(dog, params):
    assert dog.call(10001, params) == [(0, '{"error_code":323}')]


@pytest.mark.parametrize("params", ['{"a":"\\ud83d\\ude00"}', '{"a":"\\udc00"}', '{"a":18446744073709551616}'])
def test_inputs_rapidjson_accepts(dog, params):
    assert dog.call(10001, params)[0][0] == 10001     # a pair, a lone low half, an integer read as a double


def test_parse_stops_at_nul_and_first_duplicate_key_wins(dog):
    assert dog.call(10001, "{}\x00garbage")[0][0] == 10001
    r = dog.j(1012, '{"volume":7,"volume":9}')
    assert r == {"success": True}
    assert dog.j(1001, {"is_volume": True})["volume"] == 7


def test_volume_truncates_to_uint8(dog):
    dog.j(1012, {"volume": 300})
    assert dog.j(1001, {"is_volume": True})["volume"] == 300 & 0xFF


def test_non_object_on_a_raw_rapidjson_path_crashes_until_reset(dog):
    with pytest.raises(grpc.RpcError) as e:
        dog.call(6002, "[]")
    assert e.value.code() == grpc.StatusCode.UNAVAILABLE
    with pytest.raises(grpc.RpcError) as e:
        dog.call(10001)
    assert e.value.code() == grpc.StatusCode.UNAVAILABLE
    assert dog.state()["crashed"].startswith("6002")
    dog.post("/reset")
    assert dog.call(10001)[0][0] == 10001


def test_scalar_where_an_array_is_expected_crashes(dog):
    with pytest.raises(grpc.RpcError):
        dog.call(1004, {"motion_id": 111, "vel_des": 0.3})
    assert "vel_des" in dog.state()["crashed"]
