"""Safety rules of contracts.md 1 over the real wire: refused requests never reach the emulator, and what does
reach it is typed exactly (the per-test teardown also checks dangerous_calls and client_bugs are empty)."""
from __future__ import annotations

import time

import pytest

from cyberdog_sdk import ForbiddenCommand, RiskyMotion
from cyberdog_sdk import catalog

NAV = {"outdoor": False, "keep_distance": 0.0}


def test_forbidden_codes_raise_before_any_traffic(dog, ctl):
    before = ctl.calls()
    for nc in (9030, 9031, 10006, 12001, 55001, 5000, 5001, 5010, 5999):          # rule 1
        with pytest.raises(ForbiddenCommand):
            dog.transport.call(nc, {})
        with pytest.raises(ForbiddenCommand):
            dog.transport.call_json(nc)
    for params in ({"type": 0, **NAV}, {"type": 2, **NAV}, {"type": 0.0, **NAV}, {"outdoor": False}):   # rule 2
        with pytest.raises(ForbiddenCommand):
            dog.transport.call(6004, params)
    with pytest.raises(ValueError):
        dog.transport.call(6004, {"type": 1, **NAV, "goalX": 1.0})              # type 1 needs both goals
    with pytest.raises(ValueError):
        dog.transport.call(6004, {"type": 5, "outdoor": False})                 # keep_distance always sent
    with pytest.raises(ForbiddenCommand):                                        # rule 3
        dog.nav.set_labels("casa", [], only_delete=True)
    with pytest.raises(ForbiddenCommand):
        dog.transport.call(6002, {"mapName": "casa", "only_delete": True, "locationLabelInfo": []})
    with pytest.raises(ForbiddenCommand):
        dog.transport.call(6002, {"mapName": "casa", "only_delete": True})
    for name in ("../etc/passwd", "a/b.json", "a\\b.json", "..", "", "."):      # rule 5
        with pytest.raises(ForbiddenCommand):
            dog.transport.get_file(4004, {"file_name": name})
    for mid in (130, 136, 121, 139, 172):                                        # rule 8: R3 / R4
        with pytest.raises(RiskyMotion):
            dog.motion(mid)
        with pytest.raises(RiskyMotion):
            dog.trick(mid)
    with pytest.raises(ForbiddenCommand):
        dog.motion(400, allow_risky=True)                                        # never sendable
    assert ctl.calls() == before
    s = ctl.state()
    assert s["nav"]["maps"]["casa"] == ["cucina", "divano"] and s["nav"]["status"] == 101


def test_risk_classes_offered_to_the_app(dog):
    keys = (141, 123, 146, 144, 145, 174, 173, 140)
    assert all(catalog.get(m).risk in ("R0", "R1", "R2") for m in keys)
    assert catalog.get(140).risk == "R2"                                         # needs "floor clear" in the app
    assert all(m.risk in ("R3", "R4") for m in catalog.all() if m.id in (130, 121, 139))


def test_value_bit_4_is_masked_on_the_wire(dog, ctl):
    r = dog.motion(111, value=0x05)                                              # rule 4
    assert r.result
    assert dog.rec.of(1004)[-1][1]["value"] == 0x05                              # what the caller asked
    assert not any("0x04" in b["bug"] for b in ctl.state()["client_bugs"])       # what the dog received
    dog.drive.move(0.1, 0.0, 0.0)
    ctl.wait(lambda s: s["servo"]["frames"] >= 2, timeout=3)
    assert ctl.state()["servo"]["last_frame"]["value"] == 0
    dog.drive.stop()


def test_params_never_empty_and_typed(dog, ctl):
    assert dog.transport.call_json(10001)[0]["motion_status"]["motion_id"] == 101   # rule 6: None -> "{}"
    assert dog.transport.call_json(10001, "")[0]["task_status"]["task_status"] == 101
    assert not any(r["reason"].startswith("params are not valid JSON") for r in ctl.state()["refused"])
    # rule 7: integer-valued floats become JSON doubles in arrays, the emulator reads all three
    r = dog.motion(111, rpy_des=[0, 0, 0], pos_des=[0, 0, 0])
    assert r.result
    dog.drive.move(1, 0, 0)                                                      # ints in, doubles out
    ctl.wait(lambda s: s["servo"]["frames"] >= 2, timeout=3)
    f = ctl.state()["servo"]["last_frame"]
    assert f["vel_des"] == [0.6, 0.0, 0.0]                                       # clamped to the slow gait
    assert len(f["acc_des"]) == 6 and len(f["foot_pose"]) == 6 and len(f["step_height"]) == 2
    dog.drive.stop()
    with pytest.raises(ValueError):
        dog.transport.call(1004, {"motion_id": 111.5})
    with pytest.raises(ValueError):
        dog.transport.call(1004, {"motion_id": 111, "vel_des": [0.1, 0.0]})      # exact sizes


def test_estop_has_its_own_channel(dog, ctl):
    t = dog.transport
    assert set(t._channels) == {"main", "priority"} and t._channels["main"] is not t._channels["priority"]
    dog.stand()
    r = dog.estop()
    assert r.result and ctl.state()["motion"]["estop_latched"]
    assert dog.stand().result


def test_connect_leaves_the_estop_channel_ready(dog):
    # rule 9 in practice: an idle priority channel adds TCP + mTLS (80-130 ms here) to the first ESTOP
    seen = []
    ch = dog.transport._channels["priority"]
    ch.subscribe(seen.append, try_to_connect=False)
    time.sleep(0.1)
    ch.unsubscribe(seen.append)
    assert seen and seen[-1].name == "READY", seen
