"""2002 VP tasks (LED, sensors, file drop), 4002 camera and the 4002/4004 getFile transfers."""
from __future__ import annotations

import json

DEBUG = {"type": "task", "id": "pc-0001", "target_id": ["debug"], "operate": "debug", "describe": "pc test",
         "style": "pc", "mode": "single", "condition": "now"}


def _debug(dog, body, **kw):
    req = dict(DEBUG, body=body, **kw)
    assert dog.call(2002, req) == []           # fire and forget: zero messages


def test_vp_debug_led_task_plays_then_frees(dog):
    _debug(dog, "cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 0, 0, 255)\n"
                "time.sleep(20)\n"
                "cyberdog.led.freed(LedConstraint.target_head)")
    s = dog.wait(lambda s: s["leds"]["head"] is not None, timeout=3)
    assert s["leds"]["head"] == {"r": 0, "g": 0, "b": 255, "effect": 1, "mode": 2, "owner": "vp"}
    assert s["vp"]["tasks"]["debug"]["state"] == "run"
    s = dog.wait(lambda s: s["leds"]["head"] is None and s["vp"]["tasks"]["debug"]["state"] == "shutdown", timeout=4)
    assert [c["code"] for c in s["vp"]["led_calls"]] == [0, 0]


def test_vp_system_effect_and_priority(dog):
    assert dog.post("/led", {"client": "bms", "target": 2, "mode": 1, "effect": 0xA4})[1]["code"] == 0
    _debug(dog, "r = cyberdog.led.play(LedConstraint.target_tail, LedConstraint.system_effect_line_blue_on)\n"
                "print(r.response.code)")
    s = dog.wait(lambda s: s["vp"]["tasks"]["debug"]["state"] == "shutdown", timeout=4)
    assert s["vp"]["logs"]["debug"][0] == "1125"                          # LOW_PRIORITY: bms holds the tail
    assert s["leds"]["tail"]["owner"] == "bms" and s["leds"]["tail"]["r"] == 255


def test_vp_parse_errors_are_engine_feedback_only(dog):
    req = dict(DEBUG, body="print(1)")
    del req["describe"]
    assert dog.call(2002, req) == []
    s = dog.wait(lambda s: s["vp"]["feedback"], timeout=3)
    fb = s["vp"]["feedback"][-1]["feedback"]
    assert fb["type"] == "parse" and fb["state"] == 8025
    assert dog.call(2002, dict(DEBUG, body="print(1)", condition="now + 1minutes")) == []
    s = dog.wait(lambda s: len(s["vp"]["feedback"]) == 2, timeout=3)
    assert s["vp"]["feedback"][-1]["feedback"]["state"] == 8029


def test_vp_build_failure_puts_the_task_in_error(dog):
    _debug(dog, "import json\nprint(undefined_name)")
    s = dog.wait(lambda s: s["vp"]["tasks"].get("debug", {}).get("state") == "error", timeout=3)
    assert "pyflakes" in s["vp"]["logs"]["debug"][0]


def test_vp_file_drop_fetched_with_4004_then_deleted(dog):
    _debug(dog, "import json\n"
                "b = cyberdog.bms.get_data(5)\n"
                "open('/home/mi/Camera/vp_bms.json', 'w').write(json.dumps({'soc': b.batt_soc, 'mv': b.batt_volt}))")
    s = dog.wait(lambda s: s["vp"]["tasks"].get("debug", {}).get("state") == "shutdown", timeout=4)
    assert "vp_bms.json" in s["camera"]["files"]
    chunks = dog.files(4004, {"file_name": "vp_bms.json"})
    assert [c.error_code for c in chunks] == [0]
    assert chunks[0].file_name == "vp_bms.json" and chunks[0].file_size == len(chunks[0].buffer)
    assert json.loads(chunks[0].buffer) == {"soc": 80, "mv": 25200}
    assert "vp_bms.json" not in dog.state()["camera"]["files"]           # deleted after download
    assert [c.error_code for c in dog.files(4004, {"file_name": "vp_bms.json"})] == [9]


def test_vp_open_outside_camera_dir_is_refused(dog):
    _debug(dog, "open('/etc/passwd').read()")
    s = dog.wait(lambda s: any("Traceback" in x for x in s["vp"]["logs"].get("debug", [])), timeout=4)
    assert any("outside /home/mi/Camera" in r["reason"] for r in s["refused"])


def test_vp_shutdown_kills_a_running_body(dog):
    _debug(dog, "while True:\n    pass")
    dog.wait(lambda s: s["vp"]["running"] == ["debug"], timeout=3)
    dog.call(2002, {"type": "task", "id": "pc-2", "target_id": ["debug"], "operate": "shutdown"})
    s = dog.wait(lambda s: s["vp"]["running"] == [], timeout=3)
    assert s["vp"]["tasks"]["debug"]["state"] == "shutdown"


def test_camera_recording_state_machine(dog):
    assert dog.j(4002, {"command": 2}) == {"result": 0}
    assert dog.j(4002, {}) == {"result": 4}                               # default command 2 while recording
    assert dog.state()["camera"]["state"] == "recording"
    assert dog.j(4002, {"command": 3}) == {"result": 0}
    assert dog.j(4002, {"command": 3}) == {"result": 5}
    assert any(f.endswith(".mp4") for f in dog.state()["camera"]["files"])
    assert dog.j(4002, {"command": 0}) == {"result": 1}
    assert dog.j(4002, {"command": 11}) == {"result": 255}


def test_camera_take_picture_via_getfile(dog):
    chunks = dog.files(4002, {"command": 1})
    assert len(chunks) == 1 and chunks[0].error_code == 0
    c = chunks[0]
    assert c.file_name.startswith("IMG_") and c.file_name.endswith(".jpg")
    assert c.buffer[:2] == b"\xff\xd8" and c.buffer[-2:] == b"\xff\xd9" and c.file_size == len(c.buffer)
    assert dog.state()["camera"]["files"] == []                          # deleted after download


def test_camera_getfile_error_codes(dog):
    assert [c.error_code for c in dog.files(4002, {})] == [255]           # default command 255
    assert [c.error_code for c in dog.files(4002, {"command": 4})] == [8]  # "idle" has no comma
    assert [c.error_code for c in dog.files(4002, "not json")] == [255]  # parse errors are ignored
    dog.post("/fault", {"dep": "camera_service", "mode": "absent"})
    assert [c.error_code for c in dog.files(4002, {"command": 1})] == [7]
    assert dog.j(4002, {"command": 1}) == {"result": 7}


def test_4004_traversal_is_sandboxed_and_flagged(dog):
    assert [c.error_code for c in dog.files(4004, {"file_name": "../../etc/passwd"})] == [9]
    assert any("'..'" in b["bug"] for b in dog.state()["client_bugs"])


def test_skin_needs_enable_first(dog):
    assert dog.call(11002, {"mode": 2}) == [(11002, '{"error_code":323}')]
    assert dog.j(11002, {"mode": 2, "wave_cycle_time": 3000}) == {"success": False}
    assert dog.j(11001, {"data": True}) == {"success": True, "message": ""}
    assert dog.j(11002, {"mode": 2, "wave_cycle_time": 3000}) == {"success": True}
    assert dog.state()["skin"] == {"enabled": True, "mode": 2, "wave_cycle_time": 3000}
