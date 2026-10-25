"""VP (visual programming) debug tasks over 2002: LEDs and sensor read-back.

2002 is fire-and-forget: the engine's feedback goes to the reverse channel we
cannot use (vp_led_sensors.md §0). The engine writes the body verbatim into a
generated Python 3.6 file after a header that imports os, sys, time, threading
and creates `cyberdog`, and runs pyflakes on it: any warning deletes the file
(§1.6). Bodies therefore use only header names, import json themselves, use
every `except ... as e` name and never rebind cyberdog/task_id/os/time.
Sensor results come back as a file under /home/mi/Camera/ fetched with 4004,
which deletes it after sending (§4.3).

The dog runs one debug task at a time (target_id "debug"). Once a new one passes
pyflakes, the engine kills the running one (task.cpp:301-313: atrm, then kill -9
on 'cyberdog_vp_task:=debug', linux_at.cpp:135-200), which then never reaches its
freed() calls or its footer. Slot keeps a client's LED state across that.
"""
from __future__ import annotations

import itertools
import json
import random
import re
import threading
import time
from typing import Dict, Iterable, Optional, Sequence, Tuple

from .transport import DogError

CAMERA_DIR = "/home/mi/Camera/"
TARGETS = {"head": "target_head", "tail": "target_tail", "mini": "target_mini"}
LINE_USER = ("on", "blink", "blink_fast", "breath", "breath_fast", "one_by_one",
             "one_by_one_fast", "back_and_forth", "trailing_race")
LINE_SYSTEM_STYLES = ("on", "blink", "blink_fast", "breath", "breath_fast", "one_by_one", "one_by_one_fast")
SYSTEM_COLORS = ("red", "blue", "yellow")
MINI_USER = ("circular_breath", "circular_ring")
MINI_SYSTEM = ("off", "rectangle_color", "centre_color", "three_circular", "one_by_one")
SENSORS = ("bms", "tof", "ultrasonic", "odom")
MAX_LED_SECONDS = 600.0
FREE_GRACE = 60.0       # s after a target is released during which every new task frees it again
_SEQ = itertools.count()
_ID_RE = re.compile(r"^[A-Za-z0-9_]+$")


def new_task_id(prefix: str = "pc") -> str:
    # target_id/id end up unsanitised in paths and shell commands: [A-Za-z0-9_] only (§1.6)
    return "%s_%d_%d_%04x" % (prefix, int(time.time() * 1000), next(_SEQ), random.getrandbits(16))


def _check_id(task_id: str) -> str:
    if not isinstance(task_id, str) or not _ID_RE.match(task_id):
        raise ValueError("task id must match [A-Za-z0-9_]+, got %r" % (task_id,))
    return task_id


def debug_task(body: str, describe: str, task_id: Optional[str] = None) -> dict:
    """A one-shot debug task (§1.1): target_id ["debug"], mode single, condition now."""
    return {"type": "task", "id": _check_id(task_id or new_task_id()), "target_id": ["debug"],
            "operate": "debug", "describe": describe or "pc task", "style": "pc",
            "mode": "single", "condition": "now", "body": body}


def _target(target: str) -> str:
    if target not in TARGETS:
        raise ValueError("LED target must be one of %s, got %r" % (sorted(TARGETS), target))
    return "LedConstraint." + TARGETS[target]


def play_line(target: str, color: Optional[Sequence[int]] = None, effect: str = "on") -> str:
    """play_rgb (USER effect) with a color, play (SYSTEM effect) without.

    head/tail: USER styles (LINE_USER) take a color, white when none is given; SYSTEM effects
    are "off" or "<red|blue|yellow>_<style>" and take no color. mini: USER "circular_breath"
    (also "on") / "circular_ring" take a color; SYSTEM "off", "rectangle_color",
    "centre_color", "three_circular", "one_by_one" take none.
    """
    tgt = _target(target)
    mini = target == "mini"
    if color is None and effect in (MINI_USER + ("on",) if mini else LINE_USER):
        color = (255, 255, 255)          # a USER effect needs RGB: default white
    if color is not None:
        r, g, b = (int(c) for c in color)
        if not all(0 <= c <= 255 for c in (r, g, b)):
            raise ValueError("color components must be 0..255")
        if mini:
            name = "circular_breath" if effect == "on" else effect
            if name not in MINI_USER:
                raise ValueError("mini effect with a color must be one of %s" % (MINI_USER,))
            eff = "LedConstraint.effect_mini_" + name
        else:
            if effect not in LINE_USER:
                raise ValueError("head/tail effect with a color must be one of %s" % (LINE_USER,))
            eff = "LedConstraint.effect_line_" + effect
        play = "cyberdog.led.play_rgb(%s, %s, %d, %d, %d)" % (tgt, eff, r, g, b)
    else:
        if mini:
            if effect not in MINI_SYSTEM:
                raise ValueError("mini effect without a color must be one of %s" % (MINI_SYSTEM,))
            eff = "LedConstraint.system_effect_mini_" + effect
        else:
            ok = effect == "off" or any(effect == "%s_%s" % (c, s)
                                        for c in SYSTEM_COLORS for s in LINE_SYSTEM_STYLES)
            if not ok:
                raise ValueError("head/tail effect without a color: 'off' or '<red|blue|yellow>_<style>'")
            eff = "LedConstraint.system_effect_line_" + effect
        play = "cyberdog.led.play(%s, %s)" % (tgt, eff)
    return play


def led_seconds(seconds: float) -> float:
    secs = float(seconds)
    if secs != secs:
        raise ValueError("seconds must be a number")
    return min(max(secs, 0.1), MAX_LED_SECONDS)


def led_body(target: str, color: Optional[Sequence[int]] = None, effect: str = "on",
             seconds: float = 3.0) -> str:
    """One LED call on its own: play, hold, free (Slot adds the other targets' state)."""
    play = play_line(target, color, effect)
    return "%s\ntime.sleep(%.3f)\ncyberdog.led.freed(%s)\n" % (play, led_seconds(seconds), _target(target))


def led_off_body(target: str) -> str:
    return "cyberdog.led.freed(%s)\n" % _target(target)


# Each sensor block reads one VP sensor (api/cyberdog.cpp:57-64; get_data(timeout) returns the
# cached message even on timeout, so .state.code is recorded: 0 fresh, 7944 no update).
_SENSOR_CODE = {
    "bms": """try:
    pc_b = cyberdog.bms.get_data(2)
    pc_out['bms'] = {'code': int(cyberdog.bms.state.code),
                     'stamp': [pc_b.header.stamp.sec, pc_b.header.stamp.nanosec],
                     'batt_volt': pc_b.batt_volt, 'batt_curr': pc_b.batt_curr,
                     'batt_soc': pc_b.batt_soc, 'batt_temp': pc_b.batt_temp,
                     'wired_charging': bool(pc_b.power_wired_charging),
                     'wireless_charging': bool(pc_b.power_wp_charging)}
except Exception as e:
    pc_out['bms'] = {'error': str(e)}
""",
    "tof": """def pc_tof(s):
    return {'available': bool(s.data_available), 'position': int(s.tof_position),
            'data': [float(v) for v in s.data]}
try:
    pc_t = cyberdog.tof.get_data(2)
    pc_out['tof'] = {'code': int(cyberdog.tof.state.code),
                     'left_head': pc_tof(pc_t.head.left_head), 'right_head': pc_tof(pc_t.head.right_head),
                     'left_rear': pc_tof(pc_t.rear.left_rear), 'right_rear': pc_tof(pc_t.rear.right_rear)}
except Exception as e:
    pc_out['tof'] = {'error': str(e)}
""",
    "ultrasonic": """try:
    pc_u = cyberdog.ultrasonic.get_data(2)
    pc_out['ultrasonic'] = {'code': int(cyberdog.ultrasonic.state.code),
                            'stamp': [pc_u.header.stamp.sec, pc_u.header.stamp.nanosec],
                            'range': float(pc_u.range), 'min_range': float(pc_u.min_range),
                            'max_range': float(pc_u.max_range),
                            'field_of_view': float(pc_u.field_of_view)}
except Exception as e:
    pc_out['ultrasonic'] = {'error': str(e)}
""",
    "odom": """try:
    pc_o = cyberdog.odometer.get_data(2)
    pc_p = pc_o.pose.pose
    pc_v = pc_o.twist.twist
    pc_out['odom'] = {'code': int(cyberdog.odometer.state.code),
                      'stamp': [pc_o.header.stamp.sec, pc_o.header.stamp.nanosec],
                      'position': [pc_p.position.x, pc_p.position.y, pc_p.position.z],
                      'orientation': [pc_p.orientation.x, pc_p.orientation.y,
                                      pc_p.orientation.z, pc_p.orientation.w],
                      'linear': [pc_v.linear.x, pc_v.linear.y, pc_v.linear.z],
                      'angular': [pc_v.angular.x, pc_v.angular.y, pc_v.angular.z]}
except Exception as e:
    pc_out['odom'] = {'error': str(e)}
""",
}


def sensor_body(task_id: str, kinds: Iterable[str] = SENSORS) -> str:
    """Body that reads the sensors and drops /home/mi/Camera/<task_id>.json (written as .tmp
    then renamed, so 4004 never sees a half-written file)."""
    kinds = list(kinds)
    bad = [k for k in kinds if k not in SENSORS]
    if bad or not kinds:
        raise ValueError("sensor kinds must be a non-empty subset of %s, got %r" % (SENSORS, kinds))
    path = CAMERA_DIR + _check_id(task_id) + ".json"
    parts = ["import json\n", "pc_out = {'task': %r, 'time': time.time()}\n" % task_id]
    parts += [_SENSOR_CODE[k] for k in SENSORS if k in kinds]
    parts.append("with open(%r, 'w') as pc_f:\n    pc_f.write(json.dumps(pc_out))\n" % (path + ".tmp"))
    parts.append("os.rename(%r, %r)\n" % (path + ".tmp", path))
    return "".join(parts)


def send_task(transport, task: dict) -> str:
    transport.call(2002, task, timeout=10.0)
    return task["id"]


class Slot:
    """The VP debug slot of one client: every 2002 the client sends goes through it.

    LED occupancy is kept per (target, client), and every VP process asks as client "vp"
    (sdk/led.cpp:52-64; led_plugin.cpp:384-416), so a new task can replay or free a target
    that a killed task left lit. The slot keeps the desired state (per target: its play line
    and the monotonic end of its hold) and every task it sends carries all of it:
      1. the call's own work: the sensor read, the new play, or freed() for led_off;
      2. freed() for every other target released (led_off, or its hold ended) in the last
         FREE_GRACE s: the task that should have freed it may have been killed first;
      3. a replay of every other hold still running (skipped if it ended in the meantime);
      4. per hold, in end order, a sleep to its end (timed from the body start), then freed().
    A new task thus replaces the running one without losing the other holds. The task lives
    until its last hold ends; the footer's shutdown() then frees all three LEDs (§2.3).
    One lock serialises every 2002. read_sensors keeps it until the file is back, so an LED
    call made during a read waits for the read (at most its timeout plus one 4004 deadline).
    """

    def __init__(self, transport, clock=time.monotonic):
        self.t = transport
        self.lock = threading.Lock()
        self._clock = clock
        self._leds: Dict[str, Tuple[Optional[str], float]] = {}     # target -> (play line or None, end)

    def holds(self) -> dict:
        """{target: seconds left} for every hold still running."""
        now = self._clock()
        return {t: e - now for t, (p, e) in dict(self._leds).items() if p is not None and e > now}

    def led(self, target: str, color=None, effect: str = "on", seconds: float = 3.0) -> str:
        play = play_line(target, color, effect)        # bad arguments raise before anything is sent
        secs = led_seconds(seconds)
        with self.lock:
            now = self._clock()
            leds = self._live(now)
            leds[target] = (play, now + secs)
            task = debug_task(self._body(play + "\n", target, now, leds), "pc led %s" % target)
            try:
                send_task(self.t, task)
            except Exception:
                # it may still have reached the dog: hold nothing, keep freeing the target for a while
                leds[target] = (None, now + secs)
                self._leds = leds
                raise
            self._leds = leds
            return task["id"]

    def led_off(self, target: str) -> str:
        work = "cyberdog.led.freed(%s)\n" % _target(target)
        with self.lock:
            now = self._clock()
            leds = self._live(now)
            leds[target] = (None, now)
            self._leds = leds                           # off stays wanted even if the send fails
            return send_task(self.t, debug_task(self._body(work, target, now, leds),
                                                "pc led off %s" % target))

    def read_sensors(self, kinds: Iterable[str] = SENSORS, timeout: float = 20.0,
                     poll: float = 0.75) -> dict:
        tid = new_task_id()
        work = sensor_body(tid, kinds)                  # bad kinds or id raise before anything is sent
        with self.lock:
            now = self._clock()
            leds = self._live(now)
            send_task(self.t, debug_task(self._body(work, None, now, leds), "pc sensors", tid))
            self._leds = leds
            return _fetch_json(self.t, tid + ".json", timeout, poll)

    def _live(self, now: float) -> dict:
        """The desired state, without targets released more than FREE_GRACE s ago."""
        return {t: (p, e) for t, (p, e) in self._leds.items() if now < e + FREE_GRACE}

    @staticmethod
    def _body(work: str, own: Optional[str], now: float, leds: dict) -> str:
        held = sorted((e - now, t, p) for t, (p, e) in leds.items() if p is not None and e > now)
        names = set(t for _, t, _ in held)
        parts = ["pc_t0 = time.monotonic()\n"] if held else []
        parts.append(work)
        parts += ["cyberdog.led.freed(%s)\n" % _target(t) for t in sorted(leds) if t != own and t not in names]
        parts += ["if time.monotonic() < pc_t0 + %.3f:\n    %s\n" % (left, play)
                  for left, t, play in held if t != own]
        for left, t, _ in held:
            parts.append("time.sleep(max(0.0, pc_t0 + %.3f - time.monotonic()))\n"
                         "cyberdog.led.freed(%s)\n" % (left, _target(t)))
        return "".join(parts)


# One-shot helpers with no shared state: one task each, which kills any running debug task.
# CyberDog.led/led_off/read_sensors go through the client's Slot instead.
def led(transport, target: str, color=None, effect: str = "on", seconds: float = 3.0) -> str:
    return Slot(transport).led(target, color, effect, seconds)


def led_off(transport, target: str) -> str:
    return Slot(transport).led_off(target)


def read_sensors(transport, kinds: Iterable[str] = SENSORS, timeout: float = 20.0,
                 poll: float = 0.75) -> dict:
    return Slot(transport).read_sensors(kinds, timeout, poll)


def _fetch_json(transport, name: str, timeout: float, poll: float) -> dict:
    """Poll 4004 for /home/mi/Camera/<name> until it arrives (4004 then deletes it) or timeout."""
    deadline = time.monotonic() + timeout
    last = None
    while True:
        res = transport.get_file(4004, {"file_name": name}, timeout=min(10.0, max(1.0, timeout)))
        if res.error_code == 0 and res.data:
            try:
                return json.loads(res.data.decode("utf-8"))
            except ValueError as e:
                raise DogError("sensor file %s is not JSON: %s" % (name, e), 0, 4004) from None
        last = res.error_code
        if time.monotonic() + poll > deadline:
            raise DogError("sensor file %s not found within %.0f s (last getFile error_code %s); "
                           "is the VP engine running?" % (name, timeout, last), last, 4004)
        time.sleep(poll)
