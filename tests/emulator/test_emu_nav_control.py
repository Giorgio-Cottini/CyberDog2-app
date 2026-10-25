"""Maps and navigation (6002-6010) and the control endpoint (contracts.md 4.2)."""
from __future__ import annotations

import json


def _stream(dog, nc, params):
    return [json.loads(m.data) for m in dog.stream(nc, params, timeout=10)]


def test_6003_boot_map(dog):
    r = dog.j(6003)
    assert list(r) == ["mapName", "success", "is_outdoor", "locationLabelInfo", "map"]
    assert r["mapName"] == "casa" and r["success"] == 0
    assert [x["labelName"] for x in r["locationLabelInfo"]] == ["cucina", "divano"]
    assert all(isinstance(x["physicX"], float) for x in r["locationLabelInfo"])
    m = r["map"]
    assert list(m) == ["resolution", "width", "height", "px", "py", "pz", "qx", "qy", "qz", "qw", "data"]
    assert len(m["data"]) == m["width"] * m["height"]


def test_6003_timeout_writes_two_messages(dog):
    dog.post("/fault", {"dep": "get_label", "mode": "timeout"})
    assert dog.call(6003) == [(6003, '{"error_code":321}'), (6003, "")]


def test_6002_rewrites_labels_and_drops_integer_coordinates(dog):
    r = dog.j(6002, {"mapName": "casa", "locationLabelInfo": [
        {"labelName": "porta", "physicX": 0.5, "physicY": -0.25},
        {"labelName": "intero", "physicX": 1, "physicY": 2.0}]})
    assert r == {"success": 0}
    labels = dog.j(6003)["locationLabelInfo"]
    assert labels == [{"labelName": "porta", "physicX": 0.5, "physicY": -0.25}]
    assert any("label dropped" in b["bug"] for b in dog.state()["client_bugs"])


def test_6002_delete_all_wipes_every_map(dog):
    assert dog.j(6002, {"mapName": "casa", "only_delete": True, "locationLabelInfo": []}) == {"success": 0}
    r = dog.j(6003)
    assert r["success"] == 1 and r["mapName"] == "" and r["locationLabelInfo"] == []
    s = dog.state()
    assert s["nav"]["maps"] == {}
    assert any("delete-all" in d.get("event", "") for d in s["dangerous_calls"])
    assert dog.j(6002, {"mapName": "casa", "locationLabelInfo": []}) == {"success": 1}


def test_nav_refuses_type_1_without_localisation(dog):
    assert _stream(dog, 6004, {"type": 1, "outdoor": False, "keep_distance": 0.0, "goalX": 1.0, "goalY": 0.0}) \
        == [{"accepted": 2}]
    assert any("type 1 needs" in r["reason"] for r in dog.state()["refused"])


def test_nav_localise_navigate_stop(dog):
    msgs = _stream(dog, 6004, {"type": 7, "outdoor": False, "keep_distance": 0.0})
    assert msgs[0] == {"accepted": 1} and msgs[1] == {"feedback_code": 3100, "feedback_msg": ""}
    assert all("result" not in m for m in msgs)                          # the result is never written
    dog.wait(lambda s: s["nav"]["status"] == 8, timeout=5)
    assert dog.j(10001)["task_status"] == {"task_status": 8, "task_sub_status": 0}

    msgs = _stream(dog, 6004, {"type": 1, "outdoor": False, "keep_distance": 0.0, "goalX": 1.0, "goalY": 0.5})
    assert msgs[0] == {"accepted": 1} and msgs[1]["feedback_code"] == 309
    assert all("result" not in m for m in msgs)
    att = _stream(dog, 6010, {"type": 1})
    assert att[0] == {"accepted": 1}
    s = dog.wait(lambda s: s["nav"]["task"] is None or s["nav"]["task"]["latest_feedback"][0] == 308, timeout=5)
    assert s["nav"]["status"] in (1, 8)
    s = dog.wait(lambda s: s["nav"]["status"] == 8 and s["nav"]["task"] is None, timeout=5)
    assert s["nav"]["sub_status"] == 0

    assert dog.j(6009, {"type": 7}) == {"result": 0}                      # mismatch: fake success, unchanged
    assert dog.j(10001)["task_status"]["task_status"] == 8
    assert dog.j(6009, {"type": 0}) == {"result": 0}                      # ResetNav
    assert dog.j(10001)["task_status"]["task_status"] == 101
    assert dog.j(6009, {"type": 0}) == {"result": 1}


def test_nav_stop_type_1_mid_route_and_6010_without_task(dog):
    dog.emu.nav.status = 8                                                # localised
    msgs = _stream(dog, 6004, {"type": 1, "outdoor": False, "keep_distance": 0.0, "goalX": 30.0, "goalY": 0.0})
    assert msgs[0] == {"accepted": 1}
    dog.wait(lambda s: s["nav"]["task"] and s["nav"]["task"]["latest_feedback"][0] == 307, timeout=5)
    assert dog.j(6009, {"type": 1}) == {"result": 0}
    assert dog.j(10001)["task_status"]["task_status"] == 8
    assert _stream(dog, 6010, {"type": 1}) == [{"accepted": 3}]
    assert _stream(dog, 6010, {"type": 11}) == [{"accepted": 3}]


def test_nav_missing_type_reads_as_reset_nav(dog):
    msgs = _stream(dog, 6004, {"outdoor": False, "keep_distance": 0.0})   # uninitialised type, 0 here
    assert msgs == [{"accepted": 1}]
    s = dog.state()
    assert any("type missing" in b["bug"] for b in s["client_bugs"])
    assert any("ResetNav" in d.get("event", "") for d in s["dangerous_calls"])
    assert s["nav"]["status"] == 0


def test_control_state_keys_and_reset(dog):
    s = dog.state()
    for k in ("motion", "ground_truth", "leds", "skin", "servo", "nav", "camera", "calls", "refused"):
        assert k in s
    assert set(s["motion"]) >= {"motion_id", "switch_status", "bar", "busy", "estop_latched"}
    assert set(s["ground_truth"]) >= {"x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "wz", "t"}
    assert s["leds"] == {"head": None, "tail": None, "mini": None}
    assert s["battery"]["soc"] == 80 and s["nav"]["status"] == 101 and s["camera"]["state"] == "idle"
    dog.j(1004, {"motion_id": 111})
    dog.j(10001)
    s = dog.state()
    assert s["calls"]["1004"] == 1 and s["calls"]["10001"] == 1
    assert dog.post("/reset") == (200, {"ok": True})
    s = dog.wait(lambda s: s["motion"]["motion_id"] == 101)
    assert s["calls"] == {} and s["ground_truth"]["z"] < 0.15


def test_control_fault_injection(dog):
    assert dog.post("/fault", {"dep": "x", "mode": "broken"})[0] == 400
    assert dog.post("/fault", {"dep": "query_divice_info", "mode": "absent"})[0] == 200
    assert dog.call(1001, {"is_stand": True}) == []                       # 0 messages when absent
    dog.post("/fault", {"dep": "set_elec_skin", "mode": "absent"})
    assert dog.call(11002, {"mode": 1, "wave_cycle_time": 1}) == [(11002, '{"error_code":322}')]
    dog.post("/fault", {"dep": "motion_result_cmd", "mode": "timeout"})
    assert dog.call(1004, {"motion_id": 111}) == [(1004, '{"error_code":321}')]
    assert dog.state()["faults"] == {"query_divice_info": "absent", "set_elec_skin": "absent",
                                     "motion_result_cmd": "timeout"}


def test_control_hang_is_released_by_reset(dog):
    import threading
    dog.post("/fault", {"dep": "query_divice_info", "mode": "hang"})
    out = []
    t = threading.Thread(target=lambda: out.append(dog.call(1001, {"is_stand": True})), daemon=True)
    t.start()
    t.join(1.0)
    assert t.is_alive()                                                   # future.get() blocks
    dog.post("/reset")
    t.join(3)
    assert out == [[]]


def test_dangerous_codes_are_recorded_not_acted_on(dog):
    assert dog.j(9031) == {"reboot_result": 0}
    assert dog.j(10006) == {"success": True}
    assert dog.call(55001) == [(55001, "")]
    assert dog.call(5005) == [(5004, "0")]                                # 5005 replies with nameCode 5004
    assert dog.call(5999) == []
    s = dog.state()
    assert [d["nameCode"] for d in s["dangerous_calls"]] == [9031, 10006, 55001, 5005, 5999]
    assert s["provider_ip"] == "" and s["motion"]["motion_id"] == 101


def test_battery_endpoint(dog):
    assert dog.post("/battery", {"soc": 42, "charging": True})[1]["soc"] == 42
    r = dog.j(1001, {"is_bat_info": True})["bat_info"]
    assert r["power"] == 42 and r["is_charging"] is True
