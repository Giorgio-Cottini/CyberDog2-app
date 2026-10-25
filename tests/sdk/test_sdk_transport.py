"""Transport guards (contracts.md §1 rules 1-7), JSON typing and the RPC core."""
from __future__ import annotations

import json

import grpc
import pytest

from cyberdog_sdk import transport as tr
from cyberdog_sdk import wire
from cyberdog_sdk.transport import DogError, DogUnreachable, ForbiddenCommand, Transport


# ---- rule 1: forbidden name codes ---------------------------------------------
@pytest.mark.parametrize("code", [9030, 9031, 10006, 12001, 55001, 5000, 5001, 5555, 5999])
def test_forbidden_codes_refused_without_force(fake, code):
    with pytest.raises(ForbiddenCommand):
        fake.call(code)
    with pytest.raises(ForbiddenCommand):
        fake.call_json(code, {})
    assert fake.sent == []


@pytest.mark.parametrize("code", [9031 + 2 ** 32, 9031 - 2 ** 32, 6004 + 2 ** 32, -1, 2 ** 32])
def test_name_codes_outside_32_bits_are_refused(fake, code):
    # the wire keeps the low 32 bits: 9031 + 2**32 would reach the dog as 9031
    with pytest.raises(ValueError, match="32 bits"):
        fake.call(code, {}, force=True)
    assert fake.sent == []


@pytest.mark.parametrize("code", [9030, 10006, 5010])
def test_forbidden_codes_pass_with_force(fake, code):
    fake.call(code, {}, force=True)
    assert fake.codes() == [code]


@pytest.mark.parametrize("code", [4999, 6000, 1001, 10001])
def test_neighbouring_codes_are_not_forbidden(fake, code):
    fake.call(code)
    assert fake.codes() == [code]


# ---- rule 6: never empty params -------------------------------------------------
@pytest.mark.parametrize("params", [None, "", "   ", b"", {}])
def test_empty_params_become_empty_object(fake, params):
    fake.call(10001, params)
    assert fake.sent[0].raw == "{}"


@pytest.mark.parametrize("params", ["[]", "3", '"x"', [1], 5])
def test_non_object_params_refused(fake, params):
    with pytest.raises(ValueError):
        fake.call(10001, params)
    assert fake.sent == []


def test_string_params_keep_their_typing(fake):
    fake.call(1004, '{"motion_id":111,"vel_des":[0.3,0.0,0.0]}')
    assert fake.sent[0].raw == '{"motion_id":111,"vel_des":[0.3,0.0,0.0]}'


def test_nan_refused(fake):
    with pytest.raises(ValueError):
        fake.call(6004, {"type": 1, "outdoor": False, "keep_distance": 0.0,
                         "goalX": float("nan"), "goalY": 0.0})


# ---- rule 5: 4004 file name -----------------------------------------------------
@pytest.mark.parametrize("name", ["../etc/passwd", "a/b.json", "a\\b.json", "..", "x..y", "",
                                  ".", "\x00", "a\x00b", None, 5])
def test_4004_bad_file_name_refused(fake, name):
    with pytest.raises(ForbiddenCommand):
        fake.get_file(4004, {"file_name": name})
    with pytest.raises(ForbiddenCommand):
        fake.call(4004, {"file_name": name})
    assert fake.sent == []


def test_4004_missing_file_name_refused(fake):
    with pytest.raises(ForbiddenCommand):
        fake.get_file(4004, {})


def test_4004_plain_name_passes(fake):
    fake.get_file(4004, {"file_name": "pc_1_abcd.json"})
    s = fake.sent[0]
    assert s.method == wire.GET_FILE and s.raw == '{"file_name":"pc_1_abcd.json"}'


# ---- rule 4: value bit 0x04 -------------------------------------------------------
@pytest.mark.parametrize("value,expected", [(0x04, 0), (0x07, 0x03), (0x0C, 0x08), (2, 2)])
def test_value_calibration_bit_masked(fake, value, expected):
    fake.call(1004, {"motion_id": 111, "value": value})
    fake.call(1002, {"motion_id": 303, "cmd_type": 1, "value": value})
    assert [s.params["value"] for s in fake.sent] == [expected, expected]


# ---- rule 7: JSON typing ----------------------------------------------------------
def test_motion_arrays_are_doubles_and_ints_are_ints(fake):
    fake.call(1002, {"motion_id": 303.0, "cmd_type": 1, "vel_des": [0, 1, -2],
                     "rpy_des": (0, 0, 0), "pos_des": [0, 0, 0], "acc_des": [0] * 6,
                     "ctrl_point": [0, 0, 0], "foot_pose": [0] * 6, "step_height": [0, 0], "value": 0})
    raw = fake.sent[0].raw
    assert raw == ('{"motion_id":303,"cmd_type":1,"vel_des":[0.0,1.0,-2.0],"rpy_des":[0.0,0.0,0.0],'
                   '"pos_des":[0.0,0.0,0.0],"acc_des":[0.0,0.0,0.0,0.0,0.0,0.0],'
                   '"ctrl_point":[0.0,0.0,0.0],"foot_pose":[0.0,0.0,0.0,0.0,0.0,0.0],'
                   '"step_height":[0.0,0.0],"value":0}')


@pytest.mark.parametrize("key,size", sorted(tr.ARRAY_SIZES.items()))
def test_motion_array_sizes_are_exact(fake, key, size):
    for bad in ([0.0] * (size - 1), [0.0] * (size + 1), 0.0, None, [True] * size, [0.0] * (size - 1) + ["x"]):
        with pytest.raises(ValueError):
            fake.call(1004, {"motion_id": 111, key: bad})
    fake.call(1004, {"motion_id": 111, key: [1] * size})
    assert fake.sent[-1].params[key] == [1.0] * size
    assert ("[" + ",".join(["1.0"] * size) + "]") in fake.sent[-1].raw


@pytest.mark.parametrize("bad", [143.5, "143", True, None, 2 ** 31])
def test_int_fields_refuse_non_ints(fake, bad):
    with pytest.raises(ValueError):
        fake.call(1004, {"motion_id": bad})


def test_motion_requires_motion_id(fake):
    for code in (1002, 1004):
        with pytest.raises(ValueError):
            fake.call(code, {"duration": 0})


@pytest.mark.parametrize("code,params,raw", [
    (1004, {"motion_id": 143.0, "duration": 0.0, "contact": 1.0}, '{"motion_id":143,"duration":0,"contact":1}'),
    (1012, {"volume": 50.0}, '{"volume":50}'),
    (11002, {"mode": 2.0, "wave_cycle_time": 3000.0}, '{"mode":2,"wave_cycle_time":3000}'),
    (6009, {"type": 5.0, "map_name": "casa"}, '{"type":5,"map_name":"casa"}'),
    (6010, {"type": 9.0}, '{"type":9}'),
])
def test_int_fields_written_as_ints(fake, code, params, raw):
    fake.call(code, params)
    assert fake.sent[0].raw == raw


def test_4002_needs_int_command(fake):
    for bad in ({}, {"command": "1"}, {"command": 1.5}, {"command": 11}, {"command": -1}):
        with pytest.raises(ValueError):
            fake.call(4002, bad)
    fake.call(4002, {"command": 7})
    assert fake.sent[0].raw == '{"command":7}'


# ---- rule 2: 6004 -------------------------------------------------------------------
@pytest.mark.parametrize("t", [0, 2, 3, 4, 6, 8, 10, 12, 14, 256, -1, None, "5"])
def test_6004_type_whitelist(fake, t):
    with pytest.raises((ForbiddenCommand, ValueError)):
        fake.call(6004, {"type": t, "outdoor": False, "keep_distance": 0.0})
    assert fake.sent == []


def test_6004_type_0_even_with_force(fake):
    with pytest.raises(ForbiddenCommand):
        fake.call(6004, {"type": 0, "outdoor": False, "keep_distance": 0.0}, force=True)


@pytest.mark.parametrize("params", [
    {"type": 5, "keep_distance": 0.0},
    {"type": 5, "outdoor": 0, "keep_distance": 0.0},
    {"type": 5, "outdoor": False},
    {"type": 1, "outdoor": False, "keep_distance": 0.0, "goalX": 1.0},
    {"type": 1, "outdoor": False, "keep_distance": 0.0, "goalY": 1.0},
])
def test_6004_required_fields(fake, params):
    with pytest.raises(ValueError):
        fake.call(6004, params)
    assert fake.sent == []


def test_6004_floats_and_ints(fake):
    fake.call(6004, {"type": 1, "outdoor": False, "keep_distance": 0, "goalX": 1, "goalY": -2,
                     "theta": 0})
    assert fake.sent[0].raw == '{"type":1,"outdoor":false,"keep_distance":0.0,"goalX":1.0,"goalY":-2.0,"theta":0.0}'


# ---- rule 3: 6002 -------------------------------------------------------------------
@pytest.mark.parametrize("labels", [[], None])
def test_6002_delete_all_refused(fake, labels):
    p = {"mapName": "casa", "only_delete": True}
    if labels is not None:
        p["locationLabelInfo"] = labels
    with pytest.raises(ForbiddenCommand):
        fake.call(6002, p)
    assert fake.sent == []


def test_6002_delete_all_with_force(fake):
    fake.call(6002, {"mapName": "casa", "only_delete": True, "locationLabelInfo": []}, force=True)
    assert fake.sent[0].params == {"mapName": "casa", "only_delete": True, "locationLabelInfo": []}


@pytest.mark.parametrize("label", [
    {"labelName": "", "physicX": 1.0, "physicY": 1.0},
    {"physicX": 1.0, "physicY": 1.0},
    {"labelName": "k", "physicX": "1", "physicY": 1.0},
    {"labelName": "k", "physicX": 1.0},
    "kitchen",
])
def test_6002_invalid_labels_refused(fake, label):
    with pytest.raises(ValueError):
        fake.call(6002, {"mapName": "casa", "only_delete": True, "locationLabelInfo": [label]})
    assert fake.sent == []


def test_6002_coordinates_are_floats(fake):
    fake.call(6002, {"mapName": "casa", "is_outdoor": False, "only_delete": False,
                     "locationLabelInfo": [{"labelName": "door", "physicX": 3, "physicY": 0}]})
    assert '"physicX":3.0,"physicY":0.0' in fake.sent[0].raw


def test_6002_only_delete_must_be_bool(fake):
    with pytest.raises(ValueError):
        fake.call(6002, {"mapName": "casa", "only_delete": 1, "locationLabelInfo": []})


# ---- replies --------------------------------------------------------------------------
def test_call_returns_every_data_string(fake):
    fake.replies[6003] = [{"error_code": 321}, ""]
    assert fake.call(6003) == ['{"error_code": 321}', ""]


def test_call_json_skips_non_json_and_non_objects(fake):
    fake.replies[1001] = ["", "not json", "[1,2]", {"stand": True}, "3"]
    assert fake.call_json(1001) == [{"stand": True}]


def test_call_one(fake):
    fake.replies[1012] = [{"success": True}]
    assert fake.call_one(1012, {"volume": 3}) == {"success": True}
    fake.replies[1012] = [{"error_code": 322}]
    with pytest.raises(DogError) as e:
        fake.call_one(1012, {"volume": 3})
    assert e.value.code == 322 and e.value.name_code == 1012
    fake.replies[1012] = []
    with pytest.raises(DogError) as e:
        fake.call_one(1012, {"volume": 3})
    assert e.value.code is None


def test_get_file_assembles_chunks(fake):
    fake.file_replies[4004] = [wire.FileChunk(0, "a.jpg", 6, b"abc"), wire.FileChunk(0, "a.jpg", 6, b"def"),
                               wire.FileChunk(0, "a.jpg", 6, b"")]
    assert fake.get_file(4004, {"file_name": "a.jpg"}) == tr.FileResult(0, "a.jpg", b"abcdef")
    fake.file_replies[4004] = []
    assert fake.get_file(4004, {"file_name": "a.jpg"}) == tr.FileResult(-1, "", b"")
    fake.file_replies[4004] = [wire.FileChunk(error_code=9)]
    assert fake.get_file(4004, {"file_name": "a.jpg"}).error_code == 9


def test_priority_lane(fake):
    fake.call(1004, {"motion_id": 0}, timeout=3.0, priority=True)
    fake.call(10001)
    assert [s.lane for s in fake.sent] == ["priority", "main"]
    assert fake.sent[0].timeout == 3.0


def test_stream_runs_guards_before_iterating(fake):
    with pytest.raises(ForbiddenCommand):
        fake.stream(6004, {"type": 0, "outdoor": False, "keep_distance": 0.0})
    it = fake.stream(6004, {"type": 5, "outdoor": False, "keep_distance": 0.0}, timeout=7300.0)
    assert fake.sent == []
    fake.replies[6004] = [{"accepted": 1}]
    assert list(it) == ['{"accepted": 1}'] and fake.sent[0].timeout == 7300.0


# ---- the real RPC core, over a fake grpc channel ---------------------------------------
class FakeRpcError(grpc.RpcError):
    def code(self):
        return grpc.StatusCode.UNAVAILABLE

    def details(self):
        return "connection refused"


class FakeCall:
    def __init__(self, msgs, fail=False):
        self.msgs, self.fail, self.cancelled = msgs, fail, False

    def __iter__(self):
        for m in self.msgs:
            yield m
        if self.fail:
            raise FakeRpcError()

    def cancel(self):
        self.cancelled = True


class FakeChannel:
    def __init__(self):
        self.seen, self.calls, self.fail, self.closed = [], [], False, False

    def unary_stream(self, method, request_serializer=None, response_deserializer=None):
        def invoke(request, timeout=None):
            req = wire.decode_send_request(request_serializer(request))
            self.seen.append((method, req, timeout))
            if method == wire.GET_FILE:
                raw = [wire.encode_file_chunk(wire.FileChunk(0, "x.jpg", 2, b"hi"))]
            else:
                raw = [wire.encode_rec_response(wire.RecResponse(req.name_code, '{"ok":1}'))]
            call = FakeCall([response_deserializer(b) for b in raw], self.fail)
            self.calls.append(call)
            return call
        return invoke

    def close(self):
        self.closed = True


class ChannelTransport(Transport):
    def new_channel(self):
        return FakeChannel()


def test_real_rpc_core_encodes_and_decodes():
    t = ChannelTransport(cert_dir="unused")
    assert t.call_json(10001, None, timeout=2.5) == [{"ok": 1}]
    method, req, timeout = t._channels["main"].seen[0]
    assert method == "/grpcapi.GrpcApp/sendMsg" and req == wire.SendRequest(10001, "{}") and timeout == 2.5
    assert t._channels["main"].calls[0].cancelled
    t.call(1004, {"motion_id": 0}, timeout=3.0, priority=True)
    assert t._channels["priority"].seen[0][1].name_code == 1004
    res = t.get_file(4004, {"file_name": "x.jpg"})
    assert res == tr.FileResult(0, "x.jpg", b"hi")
    assert t._channels["main"].seen[-1][0] == "/grpcapi.GrpcApp/getFile"
    other = FakeChannel()
    t.get_file(4002, {"command": 1}, channel=other)
    assert other.seen[0][1] == wire.SendRequest(4002, '{"command":1}')
    t.close()
    assert t._channels["main"].closed and t._channels["priority"].closed


def test_rpc_error_becomes_dog_unreachable():
    t = ChannelTransport(cert_dir="unused")
    t._channels["main"].fail = True
    with pytest.raises(DogUnreachable) as e:
        t.call(10001)
    assert e.value.status == grpc.StatusCode.UNAVAILABLE and e.value.name_code == 10001
    assert isinstance(e.value, DogError)
    assert t._channels["main"].calls[0].cancelled


def test_new_channel_options(tmp_path, monkeypatch):
    for name in ("ca-cert.pem", "client-cert.pem", "client-key.pem"):
        (tmp_path / name).write_bytes(name.encode())
    seen = []
    monkeypatch.setattr(grpc, "ssl_channel_credentials", lambda **kw: ("creds", kw))
    monkeypatch.setattr(grpc, "secure_channel",
                        lambda target, creds, options=None: seen.append((target, creds, dict(options))) or FakeChannel())
    t = Transport(host="127.0.0.1", port=50052, cert_dir=str(tmp_path))
    assert len(seen) == 2 and t._channels["main"] is not t._channels["priority"]
    target, creds, opts = seen[0]
    assert target == "127.0.0.1:50052"
    assert creds[1] == {"root_certificates": b"ca-cert.pem", "private_key": b"client-key.pem",
                        "certificate_chain": b"client-cert.pem"}
    assert opts["grpc.ssl_target_name_override"] == "cyberdog2.server"
    assert opts["grpc.default_authority"] == "cyberdog2.server"
    assert opts["grpc.max_receive_message_length"] >= 6 * 1024 * 1024
    assert opts["grpc.use_local_subchannel_pool"] == 1
    for _, _, o in seen:        # the priority channel carries only ESTOPs: it must never go idle
        assert o["grpc.client_idle_timeout_ms"] == 2 ** 31 - 1


def test_default_cert_dir_is_repo_certs():
    t = ChannelTransport()
    assert t.cert_dir.replace("\\", "/").endswith("/certs")
    assert t.host == "192.168.137.115" and t.port == 50052


def test_connect_timeout_raises_unreachable(monkeypatch):
    class NeverReady:
        def result(self, timeout=None):
            raise grpc.FutureTimeoutError()
    monkeypatch.setattr(grpc, "channel_ready_future", lambda ch: NeverReady())
    t = ChannelTransport(cert_dir="unused")
    with pytest.raises(DogUnreachable):
        t.connect(timeout=0.01)


def test_prepare_output_is_compact_json():
    raw = tr.prepare(10001, {"a": [1.5, "x"], "b": None})
    assert raw == '{"a":[1.5,"x"],"b":null}' and json.loads(raw) == {"a": [1.5, "x"], "b": None}
