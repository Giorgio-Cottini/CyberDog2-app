"""VP LEDs and sensor read-back (2002 + 4004 file drop), skin (11001/11002) and camera (4002 / getFile)
(vp_led_sensors.md 1-4, grpc_fidelity.md 7.6)."""
from __future__ import annotations

import threading
import time

import pytest


def test_led_colour_effects_and_off(dog, ctl):
    tid = dog.led("head", (255, 0, 0), "on", seconds=600)
    s = ctl.wait(lambda s: s["leds"]["head"] is not None, timeout=3)
    assert s["leds"]["head"] == {"r": 255, "g": 0, "b": 0, "effect": 0x01, "mode": 2, "owner": "vp"}
    assert s["vp"]["tasks"]["debug"]["describe"] == "pc led head"
    assert tid.startswith("pc_")
    dog.led_off("head")
    s = ctl.wait(lambda s: s["leds"]["head"] is None, timeout=3)
    assert [c["code"] for c in s["vp"]["led_calls"]] == [0, 0]      # play_rgb, then freed

    dog.led("tail", None, "blue_blink", seconds=600)                 # a system effect carries its own colour
    s = ctl.wait(lambda s: s["leds"]["tail"] is not None, timeout=3)
    assert s["leds"]["tail"] == {"r": 0, "g": 0, "b": 255, "effect": 0xA1 + 7 + 1, "mode": 1, "owner": "vp"}
    dog.led("mini", (0, 128, 255), "circular_ring", seconds=600)     # a new debug task replaces the old one
    s = ctl.wait(lambda s: s["leds"]["mini"] is not None, timeout=3)
    assert s["leds"]["mini"] == {"r": 0, "g": 128, "b": 255, "effect": 0x31, "mode": 2, "owner": "vp"}
    assert s["leds"]["tail"]["effect"] == 0xA9                       # and replays the tail hold
    dog.led_off("tail")
    dog.led_off("mini")
    ctl.wait(lambda s: s["leds"]["tail"] is None or s["leds"]["mini"] is None, timeout=3)


def _lit(s, target):
    return s["leds"][target] is not None


@pytest.mark.parametrize("long,short", [("head", "tail"), ("tail", "head")])
def test_led_holds_survive_each_other(dog, ctl, long, short):
    # the second task kills the first (one debug task, task.cpp:301-313) and carries its hold;
    # without that, the short task's footer shutdown() would free the long hold too
    dog.led(long, (255, 0, 0), "on", seconds=600)
    ctl.wait(lambda s: _lit(s, long), timeout=3)
    dog.led(short, (0, 0, 255), "on", seconds=10.0)                  # 0.5 s at time scale 0.05
    ctl.wait(lambda s: _lit(s, short), timeout=3)
    s = ctl.wait(lambda s: not _lit(s, short), timeout=3)
    assert s["leds"][long] == {"r": 255, "g": 0, "b": 0, "effect": 0x01, "mode": 2, "owner": "vp"}
    assert s["vp"]["running"] == ["debug"]                           # still holding the long one
    time.sleep(0.3)
    assert _lit(ctl.state(), long)
    dog.led_off(long)
    s = ctl.wait(lambda s: not _lit(s, long), timeout=3)
    assert not any("Traceback" in line for line in s["vp"]["logs"]["debug"])


def test_read_sensors_during_a_hold_keeps_it(dog, ctl):
    dog.led("head", (0, 255, 0), "on", seconds=600)
    ctl.wait(lambda s: _lit(s, "head"), timeout=3)
    r = dog.read_sensors(kinds=("bms", "odom"))
    assert r["bms"]["batt_soc"] == 80 and len(r["odom"]["position"]) == 3
    time.sleep(0.3)                                                   # the sensor task is long past its read
    s = ctl.state()
    assert s["leds"]["head"] == {"r": 0, "g": 255, "b": 0, "effect": 0x01, "mode": 2, "owner": "vp"}
    assert s["vp"]["running"] == ["debug"]                           # the sensor task now holds the head
    assert not any("Traceback" in line for line in s["vp"]["logs"]["debug"])
    dog.led_off("head")
    ctl.wait(lambda s: not _lit(s, "head"), timeout=3)


def test_led_call_during_a_read_waits_and_the_read_succeeds(dog, ctl):
    out = {}

    def read():
        try:
            out["r"] = dog.read_sensors(kinds=("odom",), timeout=5.0)
        except Exception as e:      # noqa: BLE001
            out["error"] = e
    th = threading.Thread(target=read, daemon=True)
    t0 = time.monotonic()
    th.start()
    end = time.monotonic() + 3
    while not dog.rec.of(2002) and time.monotonic() < end:
        time.sleep(0.002)
    assert len(dog.rec.of(2002)) == 1
    dog.led("tail", None, "yellow_on", seconds=600)                  # would kill the sensor task
    th.join(10)
    assert "error" not in out, out.get("error")
    assert set(out["r"]) == {"task", "time", "odom"} and time.monotonic() - t0 < 5.0
    sends = dog.rec.of(2002)
    assert len(sends) == 2 and sends[1][0] > dog.rec.of(4004)[-1][0]   # sent after the file came back
    s = ctl.wait(lambda s: _lit(s, "tail"), timeout=3)
    assert s["leds"]["tail"]["effect"] == 0xAF
    dog.led_off("tail")
    ctl.wait(lambda s: not _lit(s, "tail"), timeout=3)


def test_led_off_twice_in_a_row_frees_both(dog, ctl):
    dog.led("head", (255, 0, 0), "on", seconds=600)
    dog.led("mini", (0, 128, 255), "circular_ring", seconds=600)
    dog.led("tail", None, "blue_on", seconds=600)
    ctl.wait(lambda s: all(_lit(s, t) for t in ("head", "tail", "mini")), timeout=3)
    dog.led_off("mini")
    dog.led_off("tail")                                              # kills the mini-off task before it runs
    s = ctl.wait(lambda s: not _lit(s, "mini") and not _lit(s, "tail"), timeout=3)
    assert _lit(s, "head") and s["vp"]["running"] == ["debug"]       # the head hold survives both
    time.sleep(0.3)
    s = ctl.state()
    assert _lit(s, "head") and not _lit(s, "mini") and not _lit(s, "tail")
    dog.led_off("head")
    ctl.wait(lambda s: not any(_lit(s, t) for t in ("head", "tail", "mini")), timeout=3)


def test_led_hold_ends_by_itself(dog, ctl):
    dog.led("head", None, "breath", seconds=2.0)                     # no colour on a USER effect: white
    s = ctl.wait(lambda s: s["leds"]["head"] is not None, timeout=3)
    assert (s["leds"]["head"]["r"], s["leds"]["head"]["g"], s["leds"]["head"]["b"]) == (255, 255, 255)
    s = ctl.wait(lambda s: s["leds"]["head"] is None and s["vp"]["running"] == [], timeout=3)
    assert s["vp"]["tasks"]["debug"]["state"] == "shutdown"
    assert not any("Traceback" in line for line in s["vp"]["logs"]["debug"])


def test_read_sensors_through_the_file_drop(dog, ctl):
    ctl.post("/battery", {"soc": 64})
    t0 = time.monotonic()
    r = dog.read_sensors()
    assert time.monotonic() - t0 < 5.0
    assert set(r) == {"task", "time", "bms", "tof", "ultrasonic", "odom"}
    assert r["task"].startswith("pc_")
    assert r["bms"]["batt_soc"] == 64 and r["bms"]["code"] == 0 and r["bms"]["wired_charging"] is False
    assert r["tof"]["left_head"]["available"] is True and len(r["tof"]["right_rear"]["data"]) == 64
    assert r["ultrasonic"]["range"] == 1.2
    assert len(r["odom"]["position"]) == 3 and len(r["odom"]["orientation"]) == 4
    s = ctl.state()
    assert s["camera"]["files"] == []                                # 4004 deleted it after sending
    assert not any("Traceback" in line for line in s["vp"]["logs"]["debug"])

    r = dog.read_sensors(kinds=("odom",))
    assert set(r) == {"task", "time", "odom"}


def test_skin(dog, ctl):
    assert dog.skin(3, wave_ms=2000) is True
    assert ctl.state()["skin"] == {"enabled": True, "mode": 3, "wave_cycle_time": 2000}
    assert dog.skin(0) is True
    assert ctl.state()["skin"]["mode"] == 0


def test_camera_frames(dog, ctl):
    dog.camera.start()
    try:
        first = dog.camera.wait(0, timeout=5)
        assert first is not None
        seq, jpeg = first
        assert jpeg[:2] == b"\xff\xd8" and len(jpeg) > 1000
        assert ctl.state()["camera"]["live"] is True
        end = time.monotonic() + 5
        while dog.camera.latest()[0] < seq + 8 and time.monotonic() < end:
            time.sleep(0.02)
        seq2, jpeg2 = dog.camera.latest()
        assert seq2 >= seq + 8 and jpeg2 != jpeg                     # every shot is a new file
        assert dog.camera.fps > 1.0
        assert dog.camera.last_error is None
    finally:
        dog.camera.stop()
    s = ctl.wait(lambda s: s["camera"]["live"] is False, timeout=3)
    calls = s["calls"]
    assert calls["getFile:4002"] >= 9 and calls["4002"] == 2         # START_LIVE_STREAM, STOP_LIVE_STREAM
    time.sleep(0.3)
    assert ctl.state()["camera"]["transfers"] == 0


def test_read_sensors_while_the_camera_runs(dog, ctl):
    dog.camera.start()                                               # 4 pullers + 1 sensor poll: 5 transfers, the limit
    try:
        assert dog.camera.wait(0, timeout=5) is not None
        r = dog.read_sensors(kinds=("bms",))
        assert r["bms"]["batt_soc"] == 80
        seq = dog.camera.latest()[0]
        assert dog.camera.wait(seq, timeout=3)[0] > seq              # the camera kept going
    finally:
        dog.camera.stop()
    assert not any("result 10" in x["reason"] for x in ctl.state()["refused"])
