"""Layer (a): routes, bodies and status codes of contracts.md 6.1, stub SDK, no network."""
from __future__ import annotations

import time

from apphelp import server

STATE_KEYS = {"connected", "target", "emulator", "latency_ms", "armed", "battery", "motion", "drive", "posture",
              "task", "leds", "skin", "sensors", "camera", "nav"}


def is_error_shape(body, code=None):
    return (isinstance(body, dict) and body.get("ok") is False and isinstance(body.get("error"), str)
            and body["error"] and "code" in body and (code is None or body["code"] == code))


# ---- GET / and static files -------------------------------------------------
def test_get_root_serves_ui_and_forces_the_gate_off(h):
    h.arm()
    s, hdrs, body = h.request("GET", "/")
    assert s == 200 and b"stub ui" in body
    assert hdrs["Content-Type"].startswith("text/html")
    assert "frame-ancestors 'none'" in hdrs["Content-Security-Policy"]
    assert h.state()["armed"] is False


def test_get_root_placeholder_in_italian_when_ui_missing(make_harness):
    h = make_harness(ui_text=None)
    s, hdrs, body = h.request("GET", "/")
    assert s == 200 and "manca la pagina di controllo".encode() in body


def test_no_other_file_is_served(h):
    for path in ("/qr.html", "/ui.html", "/app2/ui.html", "/index.html", "/../qr.html", "/server.py",
                 "/static/x.js", "/certs/client-key.pem", "/%2e%2e/qr.html"):
        s, body = h.get(path)
        assert s == 404 and is_error_shape(body), path


# ---- state, catalog, map ----------------------------------------------------
def test_state_object_shape(h):
    h.wait(lambda: h.app.soc == 80 and h.app.emulator is not None, what="info poll")
    st = h.state()
    assert set(st) >= STATE_KEYS
    assert st["connected"] is True and st["target"] == "stub:50052" and st["emulator"] is True
    assert isinstance(st["latency_ms"], int) and st["armed"] is False
    assert st["battery"] == {"soc": 80, "charging": False}
    assert set(st["motion"]) >= {"id", "name", "busy", "last_result"}
    assert st["motion"]["id"] == 101 and st["motion"]["name"] == "sdraiato" and st["motion"]["busy"] is False
    assert st["drive"] == {"active": False, "gait": "slow", "speed": 0.4, "vx": 0.0, "vy": 0.0, "wz": 0.0}
    assert set(st["posture"]) == {"z", "roll", "pitch", "yaw"}
    assert st["task"] == {"status": 101, "sub": 0, "name": "nessun compito"}
    assert st["leds"] == {"head": None, "tail": None, "mini": None}
    assert st["skin"] == {"mode": None}
    assert st["sensors"]["pending"] is False and st["sensors"]["t"] is None
    assert set(st["sensors"]) >= {"t", "pending", "bms", "tof", "ultrasonic", "odom"}
    assert st["camera"]["on"] is False and st["camera"]["fps"] == 0.0
    assert st["nav"] == {"state": "idle", "task": None, "goal": None}


def test_catalog_lists_r0_r2_only(h):
    s, cat = h.get("/api/catalog")
    assert s == 200
    ids = {m["id"] for m in cat["motions"]}
    assert {m["risk"] for m in cat["motions"]} <= {"R0", "R1", "R2"}
    for m in cat["motions"]:
        assert set(m) == {"id", "name", "risk", "needs_confirm", "typical_s"}
        assert m["needs_confirm"] is (m["risk"] == "R2")
    assert not ids & {0, 130, 136, 153, 121, 122, 139, 172, 179, 180, 513}     # ESTOP, R3, R4
    assert not ids & {303, 304, 305, 308, 201, 202, 211, 113, 400}             # gaits, postures, never
    assert {141, 123, 146, 144, 145, 174, 173, 140, 111, 143, 101} <= ids
    assert {m["id"]: m for m in cat["motions"]}[140]["needs_confirm"] is True
    assert cat["keymap"] == {"1": "stand_sit", "2": 141, "3": 123, "4": 146, "5": 144, "6": 145, "7": 174,
                             "8": 173, "9": 140}
    gaits = {g["key"]: g for g in cat["gaits"]}
    assert list(gaits) == ["slow", "medium", "fast", "auto"]
    assert gaits["slow"]["motion_id"] == 303 and gaits["slow"]["max"] == [0.5, 0.2, 1.0]
    assert {g["motion_id"] for g in cat["gaits"]} == {303, 308, 305, 304}


def test_map(h):
    s, m = h.get("/api/map")
    assert s == 200 and m["ok"] is True and m["name"] == "casa"
    assert (m["resolution"], m["width"], m["height"], m["origin"]) == (0.05, 4, 3, [-2.0, -1.5])
    assert m["data"] == [-1, 0, 100, 0] * 3
    assert m["labels"] == [{"name": "cucina", "x": 1.3, "y": 0.7}, {"name": "divano", "x": -1.1, "y": -0.6}]


def test_map_missing_and_unreachable(h):
    h.dog.nav.map_success = 1
    s, m = h.get("/api/map")
    assert s == 200 and m["ok"] is True and m["width"] == 0 and m["data"] == [] and m["success"] == 1
    assert m["message"] == "nessuna mappa salvata sul cane"
    h.dog.reachable = False
    s, m = h.get("/api/map")
    assert s == 502 and is_error_shape(m)


# ---- error shape, methods, bodies -------------------------------------------
def test_error_shape_and_methods(h):
    s, b = h.get("/api/nope")
    assert s == 404 and is_error_shape(b, None)
    s, hdrs, b = h.request("GET", "/api/arm")
    assert s == 405 and hdrs["Allow"] == "POST" and is_error_shape(b)
    s, hdrs, b = h.request("POST", "/api/state", body={})
    assert s == 405 and hdrs["Allow"] == "GET"
    s, _, b = h.request("POST", "/api/arm", raw=b"{not json", headers={"Content-Type": "application/json"})
    assert s == 400 and is_error_shape(b)
    s, _, b = h.request("POST", "/api/arm", raw=b"[1, 2]", headers={"Content-Type": "application/json"})
    assert s == 400 and is_error_shape(b)
    s, _, b = h.request("POST", "/api/speed", raw=b'{"level": NaN}', headers={"Content-Type": "application/json"})
    assert s == 400 and is_error_shape(b)
    s, _, b = h.request("POST", "/api/arm", raw=b"x" * (server.MAX_BODY + 1),
                        headers={"Content-Type": "application/json"})
    assert s == 400 and is_error_shape(b)
    s, b = h.post("/api/arm", {"armed": "yes"})
    assert s == 400 and is_error_shape(b)


def test_ungated_endpoints_answer_while_disarmed(h):
    cases = [("/api/gait", {"gait": "medium"}), ("/api/speed", {"level": 0.5}),
             ("/api/led", {"target": "head", "color": "#FF0000", "effect": "on", "seconds": 2}),
             ("/api/led/off", {"target": "head"}), ("/api/skin", {"mode": 2, "wave_ms": 3000}),
             ("/api/sensors/refresh", {}), ("/api/camera", {"on": False}), ("/api/volume", {"level": 40}),
             ("/api/nav/stop", {}), ("/api/estop", {})]
    for path, body in cases:
        s, b = h.post(path, body)
        assert s == 200 and b["ok"] is True, (path, s, b)


def test_gait_speed_bodies(h):
    s, b = h.post("/api/gait", {"gait": "fast"})
    assert (s, b) == (200, {"ok": True, "gait": "fast", "max": [1.6, 0.55, 2.5]})
    assert h.dog.drive.gaits[-1] == 305
    assert h.post("/api/gait", {"gait": "bound"})[0] == 400
    s, b = h.post("/api/speed", {"level": 0.5})
    assert s == 200 and b["speed"] == 0.5
    assert h.post("/api/speed", {"level": 7})[1]["speed"] == 1.0          # clamped to 0.1..1.0
    assert h.post("/api/speed", {"level": 0})[1]["speed"] == 0.1
    assert h.post("/api/speed", {})[0] == 400
    assert h.state()["drive"]["gait"] == "fast"


def test_led_skin_volume(h):
    s, b = h.post("/api/led", {"target": "tail", "color": "#00ff80", "effect": "breath", "seconds": 5})
    assert (s, b) == (200, {"ok": True, "task_id": "pc_stub_led"})
    assert h.dog.calls[-1] == ("led", ("tail", (0, 255, 128), "breath", 5.0), {})
    led = h.state()["leds"]["tail"]
    assert led["color"] == "#00ff80" and led["effect"] == "breath" and led["seconds"] == 5.0
    s, b = h.post("/api/led", {"target": "mini", "color": None, "effect": "three_circular", "seconds": 1})
    assert s == 200
    for bad in ({"target": "nose"}, {"target": "head", "color": "red"}, {"target": "head", "effect": "disco"},
                {"target": "head", "color": None, "effect": "blink", "seconds": 0}):
        assert h.post("/api/led", bad)[0] == 400, bad
    s, b = h.post("/api/led", {"target": "head", "color": None, "effect": "red_bounce"})
    assert s == 400 and "non valido" in b["error"]
    assert h.post("/api/led/off", {"target": "tail"}) == (200, {"ok": True, "task_id": "pc_stub_off"})
    assert h.state()["leds"]["tail"] is None
    assert h.post("/api/skin", {"mode": 3, "wave_ms": 2000}) == (200, {"ok": True, "mode": 3})
    assert h.state()["skin"] == {"mode": 3}
    assert h.post("/api/skin", {"mode": 7})[0] == 400
    assert h.post("/api/volume", {"level": 55}) == (200, {"ok": True, "level": 55})
    assert h.post("/api/volume", {"level": 101})[0] == 400
    assert h.post("/api/volume", {"level": "50"})[0] == 400


def test_led_state_expires(h):
    h.post("/api/led", {"target": "head", "color": "#ffffff", "effect": "on", "seconds": 0.2})
    assert h.state()["leds"]["head"] is not None
    time.sleep(0.3)
    assert h.state()["leds"]["head"] is None


def test_unreachable_dog_gives_502(h):
    h.dog.reachable = False
    for path, body in (("/api/led", {"target": "head", "color": "#ff0000"}), ("/api/skin", {"mode": 1}),
                       ("/api/volume", {"level": 10})):
        s, b = h.post(path, body)
        assert s == 502 and is_error_shape(b, None) and "non risponde" in b["error"], (path, b)


def test_odd_json_values_are_400(h):
    h.arm()
    for path, body in (("/api/action", {"name": ["stand"]}), ("/api/action", {"name": {"a": 1}}),
                       ("/api/gait", {"gait": ["slow"]}), ("/api/speed", {"level": 10 ** 400}),
                       ("/api/drive", {"x": 10 ** 400, "y": 0, "z": 0}), ("/api/posture", {"roll": 10 ** 400}),
                       ("/api/led", {"target": "head", "color": None, "effect": "on", "seconds": 10 ** 400})):
        s, b = h.post(path, body)
        assert s == 400 and is_error_shape(b), (path, s, b)


def test_concurrent_map_reads_share_one_6003(h):
    import threading
    gate = h.dog.block("get_map")
    out = []
    ths = [threading.Thread(target=lambda: out.append(h.get("/api/map"))) for _ in range(5)]
    for t in ths:
        t.start()
    for t in ths:
        t.join(5)
    # the dog has not answered: each request gets pending after quick_s, so none holds its connection
    assert out == [(200, {"ok": True, "pending": True})] * 5 and h.dog.nav.map_reads == 1
    gate.set()
    s, m = h.get("/api/map")
    assert s == 200 and m["name"] == "casa" and h.dog.nav.map_reads == 1     # the shared read's outcome
    h.get("/api/map")
    assert h.dog.nav.map_reads == 2                         # a new read once that outcome was given out


def test_an_uncollected_map_outcome_expires(h, monkeypatch):
    monkeypatch.setattr(server, "MAP_KEEP_S", 0.2)
    gate = h.dog.block("get_map")
    assert h.get("/api/map") == (200, {"ok": True, "pending": True})
    gate.set()
    time.sleep(0.4)                                         # done, nobody collected it, older than MAP_KEEP_S
    s, m = h.get("/api/map")
    assert s == 200 and m["name"] == "casa" and h.dog.nav.map_reads == 2


def test_a_6003_that_never_answers_is_502(h, monkeypatch):
    monkeypatch.setattr(server, "MAP_WAIT_S", 0.5)
    gate = h.dog.block("get_map")
    assert h.get("/api/map") == (200, {"ok": True, "pending": True})
    time.sleep(0.3)
    s, b = h.get("/api/map")                                # 0.3 s + quick_s 0.3 s > MAP_WAIT_S
    assert s == 502 and is_error_shape(b) and "non ha risposto" in b["error"] and h.dog.nav.map_reads == 1
    gate.set()


def test_map_321_says_the_map_service_did_not_answer(h):
    from cyberdog_sdk import DogError

    def no_answer(name=""):
        raise DogError("6003 timed out", code=321)
    h.dog.nav.get_map = no_answer
    s, b = h.get("/api/map")
    assert s == 502 and b["code"] == 321
    assert b["error"] == "mappa non disponibile: il servizio mappe del cane non ha risposto in 10 s"


def test_repeated_map_reads_log_once(h):
    for _ in range(5):
        assert h.get("/api/map")[0] == 200
    assert sum("letta" in e["msg"] for e in h.app.logs) == 1


def test_repeated_no_map_answers_log_once(h):
    h.dog.nav.map_success = 1
    for _ in range(5):
        assert h.get("/api/map")[1]["success"] == 1
    assert sum("nessuna mappa salvata" in e["msg"] for e in h.app.logs) == 1


def test_deeply_nested_json_is_400(h):
    raw = b'{"a":' * 5000 + b"1" + b"}" * 5000               # 30 KB, under MAX_BODY, deeper than the parser
    s, _, b = h.request("POST", "/api/arm", raw=raw, headers={"Content-Type": "application/json"})
    assert s == 400 and is_error_shape(b)


def test_estop_runs_whatever_its_body(h):
    for raw in (b"{not json", b"[1]", b'{"a":' * 5000 + b"1" + b"}" * 5000, b"x" * (server.MAX_BODY + 1)):
        n = len(h.dog.estops)
        s, _, b = h.request("POST", "/api/estop", raw=raw, headers={"Content-Type": "application/json"})
        assert s == 200 and b["ok"] is True and len(h.dog.estops) == n + 1, raw[:20]
