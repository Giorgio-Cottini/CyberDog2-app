"""VP 2002 debug tasks: JSON shape, LED bodies, sensor read-back body and the 4004 fetch."""
from __future__ import annotations

import ast
import builtins
import json
import re
import sys
import threading
import time
import types

import pytest

from cyberdog_sdk import DogError, DogUnreachable, ForbiddenCommand, vp, wire

ID_RE = re.compile(r"^[A-Za-z0-9_]+$")
HEADER_NAMES = {"os", "sys", "time", "threading", "mi", "get_namespace", "get_argv", "Cyberdog",
                "now_task_id", "now_task_parameters", "task_id", "task_parameters", "cyberdog"}


def parse36(src):
    try:
        return ast.parse(src, feature_version=(3, 6))
    except (TypeError, ValueError):       # feature_version below the interpreter's floor
        return ast.parse(src)


def assigned_names(tree):
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            out.add(node.id)
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            out.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            out.update((a.asname or a.name).split(".")[0] for a in node.names)
    return out


def test_debug_task_shape():
    t = vp.debug_task("time.sleep(1)\n", "pc test")
    assert set(t) == {"type", "id", "target_id", "operate", "describe", "style", "mode", "condition", "body"}
    assert (t["type"], t["target_id"], t["operate"], t["mode"], t["condition"]) == \
        ("task", ["debug"], "debug", "single", "now")
    assert t["describe"] and t["style"] and ID_RE.match(t["id"])
    assert vp.debug_task("x", "")["describe"]          # never empty (8025)


def test_task_ids_are_safe_and_unique():
    ids = {vp.new_task_id() for _ in range(200)}
    assert len(ids) == 200 and all(ID_RE.match(i) for i in ids)


def test_led_task_sent_on_2002(dog, fake):
    tid = dog.led("head", (0, 0, 255), "on", 3)
    s = fake.of(2002)[0]
    task = json.loads(s.raw)
    assert task["id"] == tid and task["operate"] == "debug" and task["target_id"] == ["debug"]
    assert task["body"] == ("pc_t0 = time.monotonic()\n"
                            "cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 0, 0, 255)\n"
                            "time.sleep(max(0.0, pc_t0 + 3.000 - time.monotonic()))\n"
                            "cyberdog.led.freed(LedConstraint.target_head)\n")


@pytest.mark.parametrize("args,play", [
    (("tail", None, "red_blink"), "cyberdog.led.play(LedConstraint.target_tail, LedConstraint.system_effect_line_red_blink)"),
    (("head", None, "off"), "cyberdog.led.play(LedConstraint.target_head, LedConstraint.system_effect_line_off)"),
    (("head", None, "breath"), "cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_breath, 255, 255, 255)"),
    (("tail", (1, 2, 3), "trailing_race"), "cyberdog.led.play_rgb(LedConstraint.target_tail, LedConstraint.effect_line_trailing_race, 1, 2, 3)"),
    (("mini", (9, 8, 7), "on"), "cyberdog.led.play_rgb(LedConstraint.target_mini, LedConstraint.effect_mini_circular_breath, 9, 8, 7)"),
    (("mini", (9, 8, 7), "circular_ring"), "cyberdog.led.play_rgb(LedConstraint.target_mini, LedConstraint.effect_mini_circular_ring, 9, 8, 7)"),
    (("mini", None, "three_circular"), "cyberdog.led.play(LedConstraint.target_mini, LedConstraint.system_effect_mini_three_circular)"),
])
def test_led_bodies(args, play):
    body = vp.led_body(*args, seconds=2)
    assert body.splitlines()[0] == play
    assert body.splitlines()[-1].startswith("cyberdog.led.freed(LedConstraint.target_")
    parse36(body)


@pytest.mark.parametrize("args", [
    ("back", None, "on"), ("head", (0, 0, 256), "on"), ("head", (0, 0, 255), "red_on"),
    ("head", None, "green_on"), ("mini", None, "red_on"), ("mini", (1, 1, 1), "rectangle_color"),
    ("head", None, "rainbow"),
])
def test_led_invalid(args):
    with pytest.raises(ValueError):
        vp.led_body(*args)


def test_led_seconds_clamped():
    assert "time.sleep(600.000)" in vp.led_body("head", None, "red_on", seconds=1e9)
    assert "time.sleep(0.100)" in vp.led_body("head", None, "red_on", seconds=-5)


def test_led_off(dog, fake):
    dog.led_off("tail")
    assert json.loads(fake.of(2002)[0].raw)["body"] == "cyberdog.led.freed(LedConstraint.target_tail)\n"


# ---- sensor read-back ------------------------------------------------------------------------
def test_sensor_body_is_valid_python36_and_header_safe():
    body = vp.sensor_body("pc_1_ab", vp.SENSORS)
    tree = parse36(body)
    names = assigned_names(tree)
    assert not (names & (HEADER_NAMES - {"json"})), names & HEADER_NAMES
    assert "json" in names                       # imported by the body, not by the header
    for call in ("cyberdog.bms.get_data(", "cyberdog.tof.get_data(", "cyberdog.ultrasonic.get_data(",
                 "cyberdog.odometer.get_data("):
        assert call in body
    assert "'/home/mi/Camera/pc_1_ab.json.tmp'" in body and \
        "os.rename('/home/mi/Camera/pc_1_ab.json.tmp', '/home/mi/Camera/pc_1_ab.json')" in body
    # every "except ... as e" uses e (pyflakes would reject the file otherwise)
    for node in ast.walk(tree):
        if isinstance(node, ast.ExceptHandler) and node.name:
            used = [n for n in ast.walk(node) if isinstance(n, ast.Name) and n.id == node.name
                    and isinstance(n.ctx, ast.Load)]
            assert used


def test_sensor_body_subset_and_errors():
    body = vp.sensor_body("pc_2", ["bms"])
    assert "cyberdog.bms" in body and "cyberdog.tof" not in body and "odometer" not in body
    for bad in ([], ["gps"], ["bms", "imu"]):
        with pytest.raises(ValueError):
            vp.sensor_body("pc_2", bad)


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def fake_cyberdog():
    stamp = _ns(sec=10, nanosec=5)
    hdr = _ns(stamp=stamp)
    state = _ns(code=0)
    tof = lambda pos: _ns(data_available=True, tof_position=pos, data=[0.2, 0.3])   # noqa: E731
    bms = _ns(header=hdr, batt_volt=25000, batt_curr=-1200, batt_soc=80, batt_temp=30,
              power_wired_charging=False, power_wp_charging=True)
    tofp = _ns(head=_ns(left_head=tof(0), right_head=tof(1)), rear=_ns(left_rear=tof(2), right_rear=tof(3)))
    rng = _ns(header=hdr, range=0.5, min_range=0.03, max_range=4.0, field_of_view=15.0)
    vec = lambda x, y, z: _ns(x=x, y=y, z=z)   # noqa: E731
    odom = _ns(header=hdr, pose=_ns(pose=_ns(position=vec(1.0, 2.0, 0.0), orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0))),
               twist=_ns(twist=_ns(linear=vec(0.1, 0.0, 0.0), angular=vec(0.0, 0.0, 0.2))))

    def sensor(msg):
        return _ns(get_data=lambda timeout=5: msg, state=state)
    return _ns(bms=sensor(bms), tof=sensor(tofp), ultrasonic=sensor(rng), odometer=sensor(odom))


def test_sensor_body_runs_against_the_vp_names():
    files, renames = {}, []

    class F:
        def __init__(self, path):
            self.path = path

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def write(self, s):
            files[self.path] = s

    fake_os = _ns(rename=lambda a, b: renames.append((a, b)))
    g = {"__builtins__": builtins, "cyberdog": fake_cyberdog(), "os": fake_os,
         "time": __import__("time"), "open": lambda p, m="r": F(p)}
    exec(compile(vp.sensor_body("pc_3", vp.SENSORS), "<task>", "exec"), g)
    out = json.loads(files["/home/mi/Camera/pc_3.json.tmp"])
    assert renames == [("/home/mi/Camera/pc_3.json.tmp", "/home/mi/Camera/pc_3.json")]
    assert out["bms"] == {"code": 0, "stamp": [10, 5], "batt_volt": 25000, "batt_curr": -1200, "batt_soc": 80,
                          "batt_temp": 30, "wired_charging": False, "wireless_charging": True}
    assert out["tof"]["right_rear"] == {"available": True, "position": 3, "data": [0.2, 0.3]}
    assert out["ultrasonic"]["range"] == 0.5 and out["ultrasonic"]["field_of_view"] == 15.0
    assert out["odom"]["position"] == [1.0, 2.0, 0.0] and out["odom"]["angular"] == [0.0, 0.0, 0.2]
    assert "error" not in json.dumps(out)


def test_sensor_body_survives_a_failing_sensor():
    files = {}
    cd = fake_cyberdog()
    cd.tof = _ns(get_data=lambda timeout=5: (_ for _ in ()).throw(RuntimeError("no tof")), state=_ns(code=7944))

    class F:
        def __init__(self, p):
            self.p = p

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def write(self, s):
            files[self.p] = s
    g = {"__builtins__": builtins, "cyberdog": cd, "os": _ns(rename=lambda a, b: None),
         "time": __import__("time"), "open": lambda p, m="r": F(p)}
    exec(vp.sensor_body("pc_4", vp.SENSORS), g)
    out = json.loads(files["/home/mi/Camera/pc_4.json.tmp"])
    assert out["tof"] == {"error": "no tof"} and out["bms"]["code"] == 0


def test_read_sensors_polls_4004(dog, fake, monkeypatch):
    monkeypatch.setattr(vp.time, "sleep", lambda s: None)
    calls = []

    def files(rec):
        calls.append(rec.params["file_name"])
        if len(calls) < 3:
            return [wire.FileChunk(error_code=9)]
        return [wire.FileChunk(0, rec.params["file_name"], 20, b'{"bms": {"code": 0}}')]
    fake.file_replies[4004] = files
    out = dog.read_sensors(("bms",), timeout=20)
    assert out == {"bms": {"code": 0}}
    task = json.loads(fake.of(2002)[0].raw)
    assert calls == [task["id"] + ".json"] * 3
    assert "cyberdog.bms.get_data(2)" in task["body"] and task["target_id"] == ["debug"]
    assert fake.codes()[0] == 2002


def test_read_sensors_timeout(dog, fake):
    with pytest.raises(DogError) as e:
        dog.read_sensors(("bms",), timeout=0.3)
    assert e.value.code == 9 and fake.of(4004)


def test_read_sensors_file_name_is_guarded(fake, monkeypatch):
    monkeypatch.setattr(vp, "new_task_id", lambda prefix="pc": "../x")
    with pytest.raises(ValueError):
        vp.read_sensors(fake, ("bms",), timeout=0.1)
    with pytest.raises(ValueError):
        vp.debug_task("x", "d", "a b")
    with pytest.raises(ForbiddenCommand):
        fake.get_file(4004, {"file_name": "../x.json"})
    assert fake.sent == []


def test_led_seconds_nan_refused():
    with pytest.raises(ValueError):
        vp.led_body("head", None, "red_on", seconds=float("nan"))


@pytest.mark.skipif(sys.version_info < (3, 8), reason="ast feature_version")
def test_generated_file_compiles_with_vp_header():
    # the engine's header (utils.py:171-198) + body + footer (task.cpp:164-170)
    header = ("import os\nimport sys\nimport time\nimport threading\n"
              "from mi.cyberdog_vp.abilityset import LedConstraint\n"
              "task_id = 'x'\ncyberdog = None\ncyberdog.task.start()\n")
    for body in (vp.sensor_body("pc_5", vp.SENSORS), vp.led_body("head", (1, 2, 3), "blink")):
        compile(header + body + "cyberdog.task.stop()\ncyberdog.shutdown()\n", "<task>", "exec")


# ---- the VP slot: the dog keeps one debug task, a new one kills it (task.cpp:301-313) ---------
HEAD_RED = "cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 255, 0, 0)"
TAIL_BLUE = "cyberdog.led.play(LedConstraint.target_tail, LedConstraint.system_effect_line_blue_on)"


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def bodies(fake):
    return [json.loads(s.raw)["body"] for s in fake.of(2002)]


def test_slot_carries_every_hold(fake):
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 10)
    clk.t += 2
    s.led("tail", None, "blue_on", 5)
    assert bodies(fake)[1] == ("pc_t0 = time.monotonic()\n"
                               + TAIL_BLUE + "\n"
                               "if time.monotonic() < pc_t0 + 8.000:\n"
                               "    " + HEAD_RED + "\n"
                               "time.sleep(max(0.0, pc_t0 + 5.000 - time.monotonic()))\n"
                               "cyberdog.led.freed(LedConstraint.target_tail)\n"
                               "time.sleep(max(0.0, pc_t0 + 8.000 - time.monotonic()))\n"
                               "cyberdog.led.freed(LedConstraint.target_head)\n")
    assert s.holds() == {"head": 8.0, "tail": 5.0}
    s.led("head", None, "yellow_on", 1)                      # a new hold on a target replaces its old one
    b = bodies(fake)[2]
    assert "play_rgb" not in b and b.count("system_effect_line_yellow_on") == 1
    assert s.holds() == {"head": 1.0, "tail": 5.0}


def test_slot_drops_expired_holds_and_frees_them_for_a_while(fake):
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 3)
    clk.t += 5                                               # head ended 2 s ago: freed again, not replayed
    s.led("tail", None, "blue_on", 2)
    assert bodies(fake)[1].splitlines()[1:3] == [TAIL_BLUE, "cyberdog.led.freed(LedConstraint.target_head)"]
    assert HEAD_RED not in bodies(fake)[1] and s.holds() == {"tail": 2.0}
    clk.t += 2 + vp.FREE_GRACE                               # both released more than FREE_GRACE ago
    s.led_off("mini")
    assert bodies(fake)[2] == "cyberdog.led.freed(LedConstraint.target_mini)\n"


def test_two_led_off_in_a_row_free_both(fake):
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 600)
    s.led("mini", (0, 128, 255), "circular_ring", 600)
    s.led("tail", None, "blue_on", 600)
    clk.t += 1
    s.led_off("mini")
    s.led_off("tail")                                        # may kill the mini task before its freed()
    assert bodies(fake)[-1] == ("pc_t0 = time.monotonic()\n"
                                "cyberdog.led.freed(LedConstraint.target_tail)\n"
                                "cyberdog.led.freed(LedConstraint.target_mini)\n"
                                "if time.monotonic() < pc_t0 + 599.000:\n"
                                "    " + HEAD_RED + "\n"
                                "time.sleep(max(0.0, pc_t0 + 599.000 - time.monotonic()))\n"
                                "cyberdog.led.freed(LedConstraint.target_head)\n")
    assert s.holds() == {"head": 599.0}


def test_read_sensors_carries_the_holds(dog, fake, monkeypatch):
    monkeypatch.setattr(vp.time, "sleep", lambda s: None)
    polls = []

    def files(rec):
        polls.append(rec)
        if len(polls) < 2:
            return [wire.FileChunk(error_code=9)]
        return [wire.FileChunk(0, rec.params["file_name"], 2, b'{"bms": {"code": 0}}')]
    fake.file_replies[4004] = files
    dog.led("head", (255, 0, 0), "on", 600)
    assert dog.read_sensors(("bms",)) == {"bms": {"code": 0}}
    task = json.loads(fake.of(2002)[1].raw)
    head = "pc_t0 = time.monotonic()\n" + vp.sensor_body(task["id"], ("bms",))
    assert task["body"].startswith(head)                     # the read first: the file drop never waits
    rest = task["body"][len(head):].splitlines()
    assert re.match(r"if time\.monotonic\(\) < pc_t0 \+ (59\d\.\d{3}|600\.000):$", rest[0])
    assert rest[1] == "    " + HEAD_RED
    assert rest[-1] == "cyberdog.led.freed(LedConstraint.target_head)" and len(rest) == 4
    assert set(dog._vp.holds()) == {"head"}
    assert fake.codes() == [2002, 2002, 4004, 4004]


def test_read_sensors_without_holds_sends_only_the_read(dog, fake, monkeypatch):
    monkeypatch.setattr(vp.time, "sleep", lambda s: None)
    fake.file_replies[4004] = lambda rec: [wire.FileChunk(0, rec.params["file_name"], 2, b"{}")]
    dog.read_sensors(("odom",))
    task = json.loads(fake.of(2002)[0].raw)
    assert task["body"] == vp.sensor_body(task["id"], ("odom",))


def test_led_during_a_read_waits_for_it(dog, fake):
    polled, release = threading.Event(), threading.Event()

    def files(rec):
        polled.set()
        release.wait(5)
        return [wire.FileChunk(0, rec.params["file_name"], 2, b'{"bms": {}}')]
    fake.file_replies[4004] = files
    out = {}
    reader = threading.Thread(target=lambda: out.update(r=dog.read_sensors(("bms",))))
    reader.start()
    assert polled.wait(2)
    led = threading.Thread(target=lambda: out.update(led=dog.led("tail", None, "red_on", 5)))
    led.start()
    time.sleep(0.2)
    assert "led" not in out and len(fake.of(2002)) == 1      # the LED task would kill the read
    release.set()
    reader.join(5)
    led.join(5)
    assert out["r"] == {"bms": {}} and out["led"]
    assert fake.codes() == [2002, 4004, 2002]


def test_read_during_a_read_waits_too(dog, fake):
    gate = threading.Event()
    first = []

    def files(rec):
        if not first:
            first.append(rec.params["file_name"])
            gate.wait(5)
        return [wire.FileChunk(0, rec.params["file_name"], 2, b"{}")]
    fake.file_replies[4004] = files
    ths = [threading.Thread(target=dog.read_sensors, args=(("bms",),)) for _ in range(2)]
    for th in ths:
        th.start()
    time.sleep(0.2)
    assert len(fake.of(2002)) == 1
    gate.set()
    for th in ths:
        th.join(5)
    assert fake.codes() == [2002, 4004, 2002, 4004]


def test_slot_serialises_every_2002(fake):
    lk = threading.Lock()
    active, peak = [0], [0]

    def slow(rec):
        with lk:
            active[0] += 1
            peak[0] = max(peak[0], active[0])
        time.sleep(0.02)
        with lk:
            active[0] -= 1
        return []
    fake.replies[2002] = slow
    s = vp.Slot(fake)
    ths = [threading.Thread(target=s.led, args=(t, None, "red_on", 30)) for t in ("head", "tail") * 4]
    ths += [threading.Thread(target=s.led_off, args=("mini",)) for _ in range(2)]
    for th in ths:
        th.start()
    for th in ths:
        th.join(5)
    assert peak[0] == 1 and len(fake.of(2002)) == 10
    assert set(s.holds()) == {"head", "tail"}
    last = bodies(fake)[-1]
    assert last.count("freed(LedConstraint.target_head)") == 1 and last.count("freed(LedConstraint.target_tail)") == 1


def test_failed_sends(fake):
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 100)

    def down(rec):
        raise DogUnreachable("down")
    fake.replies[2002] = down
    with pytest.raises(DogError):
        s.led("tail", None, "blue_on", 100)                  # may still have reached the dog
    with pytest.raises(DogError):
        s.led_off("head")                                    # off stays wanted
    assert s.holds() == {}
    del fake.replies[2002]
    s.led("mini", (1, 2, 3), "circular_ring", 5)
    b = bodies(fake)[-1]
    assert "freed(LedConstraint.target_tail)" in b and "freed(LedConstraint.target_head)" in b
    assert HEAD_RED not in b and TAIL_BLUE not in b
    with pytest.raises(ValueError):
        s.led("head", None, "green_on")                      # refused before anything is sent
    with pytest.raises(ValueError):
        s.led_off("back")
    assert len(fake.of(2002)) == 4


class _Rt:
    """time and cyberdog.led for exec()ing a composed body: logs LED calls and sleeps."""

    def __init__(self, on_play=0.0):
        self.now, self.on_play, self.log = 50.0, on_play, []

    def monotonic(self):
        return self.now

    def sleep(self, d):
        self.log.append(("sleep", round(d, 3)))
        self.now += d

    def play(self, target, effect):
        self.log.append(("play", target))
        self.now += self.on_play

    def play_rgb(self, target, effect, r, g, b):
        self.play(target, effect)

    def freed(self, target):
        self.log.append(("freed", target))


def _run_body(body, rt):
    lc = types.SimpleNamespace(target_head=1, target_tail=2, target_mini=3, effect_line_on=1,
                               system_effect_line_yellow_breath=0xB2)
    exec(body, {"__builtins__": builtins, "time": rt, "cyberdog": types.SimpleNamespace(led=rt),
                "LedConstraint": lc})
    return rt.log


def test_composed_body_frees_each_hold_at_its_end(fake):
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 10)
    clk.t += 4
    s.led_off("mini")
    s.led("tail", None, "yellow_breath", 2)
    body = bodies(fake)[-1]
    assert _run_body(body, _Rt()) == [("play", 2), ("freed", 3), ("play", 1), ("sleep", 2.0), ("freed", 2),
                                      ("sleep", 4.0), ("freed", 1)]
    # the own play took 7 s on the dog: head ended meanwhile, so it is not replayed, only freed
    assert _run_body(body, _Rt(on_play=7.0)) == [("play", 2), ("freed", 3), ("sleep", 0.0), ("freed", 2),
                                                 ("sleep", 0.0), ("freed", 1)]


@pytest.mark.skipif(sys.version_info < (3, 8), reason="ast feature_version")
def test_composed_bodies_pass_the_build_check(fake, monkeypatch):
    from emulator.vp import build_check                     # the emulator's pyflakes approximation
    monkeypatch.setattr(vp.time, "sleep", lambda s: None)
    fake.file_replies[4004] = lambda rec: [wire.FileChunk(0, rec.params["file_name"], 2, b"{}")]
    clk = Clock()
    s = vp.Slot(fake, clock=clk)
    s.led("head", (255, 0, 0), "on", 10)
    s.led("mini", None, "three_circular", 20)
    s.read_sensors(vp.SENSORS)
    clk.t += 15
    s.led_off("tail")
    s.read_sensors(("bms",))
    header = ("import os\nimport sys\nimport time\nimport threading\n"
              "from mi.cyberdog_vp.abilityset import LedConstraint\n"
              "task_id = 'x'\ncyberdog = None\ncyberdog.task.start()\n")
    for body in bodies(fake):
        assert build_check(body) is None, body
        names = assigned_names(parse36(body))
        assert not (names & (HEADER_NAMES - {"json"})), names & HEADER_NAMES
        compile(header + body + "cyberdog.task.stop()\ncyberdog.shutdown()\n", "<task>", "exec")
