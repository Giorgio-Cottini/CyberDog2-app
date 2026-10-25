"""tools/probe_dog.py end to end against an emulator subprocess.

The emulator runs as `python -m emulator` (fake controller, time scale 0.2, ports chosen by the OS),
and the probe as its own process with scripted answers on stdin and its --time-scale test knob.
Every connection goes to 127.0.0.1. After every run the emulator must show no dangerous call,
no client bug and no crash, and the probe must never send a forbidden nameCode.
"""
from __future__ import annotations

import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PROBE = os.path.join(ROOT, "tools", "probe_dog.py")
HOST = "127.0.0.1"
SCALE = "0.2"          # at 0.2 or more the emulator's VP sleeps scale exactly, like the probe's waits
STILL = ["status", "device_info", "parallel_status", "map", "camera_delete", "vp", "vp_v4", "skin"]
MOTION = ["stand", "sit", "trick", "posture", "walk_watchdog", "lie_down"]
# Part 1: V2 blue light seen, V4 what the light did, skin seen.
STILL_ANSWERS = ["s", "b", "s"]
# Part 2: floor clear (its own word), stood, sat, stood again, paw, pitch direction, pose held,
# head back level, trotted, lay down.
MOTION_ANSWERS = ["libero", "s", "s", "s", "s", "giu", "s", "s", "s", "s"]
FORBIDDEN = {9030, 9031, 10006, 12001, 55001} | set(range(5000, 6000))

# Runs the probe in this process's main thread and simulates Ctrl+C (KeyboardInterrupt in the
# main thread) 0.5 s after the emulator shows the trot in place (motion_id 303).
CTRL_C_DRIVER = r'''
import _thread, json, runpy, sys, threading, time, urllib.request
cport, probe = int(sys.argv[1]), sys.argv[2]
def trigger():
    while True:
        try:
            s = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/state" % cport, timeout=5).read())
            if s["motion"]["motion_id"] == 303:
                time.sleep(0.5)
                _thread.interrupt_main()
                return
        except Exception:
            pass
        time.sleep(0.03)
threading.Thread(target=trigger, daemon=True).start()
sys.argv = [probe] + sys.argv[3:]
runpy.run_path(probe, run_name="__main__")
'''


class Emu:
    """`python -m emulator` on OS-chosen ports, and its HTTP control endpoint."""

    def __init__(self, tmp):
        self.log = os.path.join(tmp, "emulator.log")
        with open(self.log, "w") as f:
            self.proc = subprocess.Popen([sys.executable, "-m", "emulator", "--host", HOST, "--port", "0",
                                          "--control-port", "0", "--time-scale", SCALE],
                                         cwd=ROOT, stdout=f, stderr=subprocess.STDOUT)
        end = time.monotonic() + 30
        while True:
            with open(self.log) as f:
                m = re.search(r"127\.0\.0\.1:(\d+), control http://127\.0\.0\.1:(\d+)", f.read())
            if m:
                self.port, self.cport = int(m.group(1)), int(m.group(2))
                break
            if self.proc.poll() is not None or time.monotonic() > end:
                self.proc.kill()
                with open(self.log) as f:
                    raise RuntimeError("emulator did not start: " + f.read()[-2000:])
            time.sleep(0.05)
        self.fs_root = self.state()["fs_root"]

    def state(self):
        with urllib.request.urlopen("http://%s:%d/state" % (HOST, self.cport), timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request("http://%s:%d%s" % (HOST, self.cport, path), method="POST",
                                     data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def reset(self):
        assert self.post("/reset") == 200
        end = time.monotonic() + 10
        while True:
            m = self.state()["motion"]
            if m["motion_id"] == 101 and not m["busy"]:
                return
            assert time.monotonic() < end, m
            time.sleep(0.05)

    def close(self):
        self.proc.terminate()             # TerminateProcess on Windows: the emulator cannot clean up
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        shutil.rmtree(self.fs_root, ignore_errors=True)


@pytest.fixture(scope="module")
def emu(tmp_path_factory):
    e = Emu(str(tmp_path_factory.mktemp("probe_emu")))
    yield e
    e.close()


def probe_cmd(emu, out):
    return [PROBE, "--host", HOST, "--port", str(emu.port), "--time-scale", SCALE, "--out", out]


def run_probe(emu, tmp_path, answers, driver=None):
    """Run the probe as its own process with the answers on stdin; returns (process, results)."""
    out = str(tmp_path / "results.json")
    cmd = [sys.executable] + ([driver, str(emu.cport)] if driver else []) + probe_cmd(emu, out)
    p = subprocess.run(cmd, input="\n".join(answers) + "\n", cwd=ROOT, capture_output=True,
                       universal_newlines=True, encoding="utf-8", timeout=240)
    with open(out, encoding="utf-8") as f:
        return p, json.load(f)


def exceptions(obj, path="results"):
    """Every place in the results where a step stored an exception."""
    if isinstance(obj, dict):
        found = [path] if "exception" in obj else []
        for k, v in obj.items():
            found += exceptions(v, "%s.%s" % (path, k))
        return found
    if isinstance(obj, list):
        return [p for i, v in enumerate(obj) for p in exceptions(v, "%s[%d]" % (path, i))]
    return []


def assert_emulator_clean(st):
    assert st["dangerous_calls"] == [], st["dangerous_calls"]
    assert st["client_bugs"] == [], st["client_bugs"]
    assert st["crashed"] is None
    sent = {int(k.split(":")[-1]) for k in st["calls"]}
    assert not sent & FORBIDDEN, sent
    # the only refusals are the probe's own polling for drop files that are not there yet
    others = [r for r in st["refused"] if not (r["nameCode"] == 4004 and "open failed (9)" in r["reason"])]
    assert others == [], others


def reply(rec):
    return json.loads(rec["data"][0])


def ok(step):
    return reply(step["reply"])["result"] is True and step["reply"]["error"] is None


def trail_ids(step):
    return [mid for _, mid in step["motion_id_trail"]]


def test_full_run_every_step_clean(emu, tmp_path):
    emu.reset()
    t0 = time.monotonic()
    p, r = run_probe(emu, tmp_path, STILL_ANSWERS + MOTION_ANSWERS)
    elapsed = time.monotonic() - t0
    assert p.returncode == 0, p.stdout[-3000:] + p.stderr[-3000:]
    assert list(r["steps"]) == STILL + MOTION
    assert exceptions(r) == []
    assert r["floor_confirmed"] == "si" and r["host"] == HOST
    assert not {"interrupted", "motion_stopped", "motion_skipped", "estop"} & set(r)
    s = r["steps"]

    # Part 1: the data each step exists to capture
    assert s["status"]["motion_id_at_rest"] == 101 and len(s["status"]["latency_s"]) == 5
    assert all(x["error"] is None for x in s["status"]["replies"])
    info = reply(s["device_info"])
    assert info["bat_info"]["power"] == 80 and "motor_temper" in info and "wifi" in info
    assert s["parallel_status"]["errors"] == [None, None, None]
    m = s["map"]
    assert m["mapName"] == "casa" and m["cells"] == m["width"] * m["height"] > 0 and len(m["labels"]) == 2
    c = s["camera_delete"]
    assert c["take"]["bytes"] == c["take"]["file_size"] > 0 and c["take"]["error_codes"] == [0]
    assert c["deleted_after_send"] is True
    assert reply(c["start"]) == {"result": 0} and reply(c["stop"]) == {"result": 0}
    vp = s["vp"]
    assert vp["v1_file"]["found_after_s"] is not None and vp["v1_file"]["bytes"] > 0
    assert vp["v2_file"]["found_after_s"] is not None and vp["v2_led_seen"] == "si"
    assert "batt_soc=80" in vp["v3_file"]["text"]
    v4 = s["vp_v4"]
    assert v4["a_on"]["data"]["play"] == [0, 0]
    assert v4["b_file"]["data"]["play"] == [0, 0] and v4["b_file"]["data"]["freed"] == [0, 0]
    assert v4["a_end"]["found_after_s"] is None                  # the emulator kills A, like task.cpp
    assert v4["verdicts"]["new_task_kills_running"] is True
    assert v4["verdicts"]["kill_evidence"] == "a_unfinished"     # the emulator's fake os has no kill()
    assert v4["verdicts"]["new_task_drives_led"] is True
    assert v4["seen"] == "blu_verde_normale"
    assert all(v4[k]["error"] is None for k in ("a_send", "b_send", "cleanup_send"))
    sk = s["skin"]
    assert all(reply(sk[k])["success"] is True for k in ("enable", "set", "disable")) and sk["seen"] == "si"

    # Part 2
    assert ok(s["stand"]) and trail_ids(s["stand"])[-1] == 111
    assert ok(s["sit"]) and s["sit"]["motion_id_3s_later"] in (143, 214) and ok(s["sit"]["stand_again"])
    assert s["sit"]["seen_standing"] == "si" and not any("stopped" in v for v in s.values())
    assert ok(s["trick"]) and 141 in trail_ids(s["trick"])
    po = s["posture"]
    assert po["frames"] == 23 and po["end"]["error"] is None and po["frame_latency_max_s"] < 0.25
    assert ok(po["pre"]) and po["pitch_direction"] == "giu" and ok(po["restore"]) and po["seen_restored"] == "si"
    w = s["walk_watchdog"]
    assert w["frames"] >= 50 and w["frames_timed_out"] == 0 and 112 in [mid for _, mid in w["after_stop_trail"]]
    assert ok(s["lie_down"]) and trail_ids(s["lie_down"])[-1] == 101

    # the Italian summary and the results path
    assert "== Riepilogo ==" in p.stdout and str(tmp_path / "results.json") in p.stdout
    assert "Problemi: nessuno." in p.stdout and "- in piedi" in p.stdout and "- mappa" in p.stdout
    assert not os.path.exists(str(tmp_path / "results.json") + ".tmp")

    st = emu.state()
    assert_emulator_clean(st)
    assert st["calls"]["1004"] == 7                              # 111, 143, 111, 141, 111, 111, 101: no ESTOP
    assert st["motion"]["motion_id"] == 101 and not st["motion"]["estop_latched"]
    leds = [(c["occupation"], c["rgb"]) for c in st["vp"]["led_calls"]]
    blue, green = leds.index((True, [0, 0, 255]), 2), leds.index((True, [0, 255, 0]))
    assert blue < green and leds[green + 1:] == [(False, [0, 0, 0])] * 2   # B frees, then the cleanup
    assert st["leds"]["head"] is None
    print("full probe run: %.1f s" % elapsed)


@pytest.mark.slow
def test_floor_not_confirmed_never_moves(emu, tmp_path):
    emu.reset()
    p, r = run_probe(emu, tmp_path, STILL_ANSWERS + ["n"])
    assert p.returncode == 0, p.stderr[-3000:]
    assert list(r["steps"]) == STILL and exceptions(r) == []
    assert r["floor_confirmed"] == "no"
    st = emu.state()
    assert_emulator_clean(st)
    assert "1004" not in st["calls"] and "1002" not in st["calls"]


@pytest.mark.slow
def test_ctrl_c_during_trot_sends_estop_three_times(emu, tmp_path):
    emu.reset()
    driver = tmp_path / "ctrl_c_driver.py"
    driver.write_text(CTRL_C_DRIVER)
    p, r = run_probe(emu, tmp_path, STILL_ANSWERS + MOTION_ANSWERS, driver=str(driver))
    assert p.returncode == 2, p.stdout[-3000:] + p.stderr[-3000:]
    assert r["interrupted"] == "ctrl+c in part 2: ESTOP sent three times"
    assert list(r["steps"]) == STILL + MOTION[:4] and exceptions(r) == []    # cut during walk_watchdog
    assert [e["error"] for e in r["estop"]] == [None, None, None]
    assert all(reply(e) == {"motion_id": 0, "result": True, "code": 0} for e in r["estop"])
    time.sleep(1.0)                                              # past the servo watchdog window
    st = emu.state()
    assert_emulator_clean(st)
    assert st["motion"]["motion_id"] == 0 and st["motion"]["estop_latched"] is True
    assert "ESTOP inviato" in p.stdout and "== Riepilogo ==" in p.stdout


# ---- pieces of the probe that need no emulator ------------------------------------------
@pytest.fixture(scope="module")
def probe():
    spec = importlib.util.spec_from_file_location("probe_dog", PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_battery_reads_the_power_field(probe):
    dev = lambda d: {"steps": {"device_info": {"data": [json.dumps(d)]}}}
    assert probe.battery(dev({"bat_info": {"power": 20, "voltage": 25200}})) == 20   # query_info.hpp:352
    assert probe.battery(dev({"bat_info": {}})) is None
    assert probe.battery({"steps": {"device_info": {"exception": "x"}}}) is None


def test_v4_verdicts(probe):
    def v(alive=None, a_end=None, play=(0, 0), b=True):
        res = {"a_end": {"found_after_s": a_end}}
        if b:
            res["b_file"] = {"data": {"a_alive_after_5s": alive, "play": list(play)}}
        return probe.v4_verdicts(res)
    assert v(alive="no")["new_task_kills_running"] is True
    assert v(alive="yes")["new_task_kills_running"] is False
    assert v(alive="AttributeError(...)", a_end=21.0)["new_task_kills_running"] is False
    assert v(alive="AttributeError(...)")["new_task_kills_running"] is True
    assert v(b=False)["new_task_kills_running"] is None and v(b=False)["new_task_drives_led"] is None
    assert v(play=(0, 1125))["new_task_drives_led"] is False                 # LOW_PRIORITY
    assert v(play=(7946, 0))["new_task_drives_led"] is None                  # service never appeared


def test_time_scale_knob_refused_off_localhost(probe):
    # 192.0.2.1 is TEST-NET-1 (never routed): argparse stops before any channel is made
    with pytest.raises(SystemExit) as e:
        probe.main(["--host", "192.0.2.1", "--time-scale", "0.5"])
    assert e.value.code == 2
    with pytest.raises(SystemExit):
        probe.main(["--host", HOST, "--time-scale", "0.1"])
