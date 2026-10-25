"""Mid-work probe of the real CyberDog 2, for the v2 SDK.

It answers the questions that the protocol specs could not settle from the source
code alone (the "Unknowns that need the real dog" sections of docs/protocol/*.md).
It talks to the dog directly with grpcio and cyberdog_sdk/wire.py, not through the
SDK, so that an SDK bug cannot hide how the dog behaves.

    python tools\\probe_dog.py               every step; asks before anything moves
    python tools\\probe_dog.py --no-motion   only the steps where the dog stays still

Part 1 (the dog stays still, about 3 minutes): status, device info, map, one photo, VP tasks
V1-V4 (the head light), electronic skin. Part 2 (about 3 minutes) runs only after the person
at the dog confirms the floor is clear by typing "libero": stand, sit, paw, head tilt, trot in
place, lie down. Each move is announced first. Part 2 stops, with no further move, when the dog
refuses a move or the person answers that a move did not happen.

The results go to tools/probe_results-<date>-<time>.json (or --out); a short summary in
Italian is printed at the end.

During Part 2, Ctrl+C (or the end of the input) sends ESTOP (motion 0), and a second ESTOP
0.35 s after the first one is sent: the motors switch off and the dog drops to the floor. When
the second gets no reply, ESTOP is sent again for up to 6 s. The script then ends.
In Part 1 nothing moves, and Ctrl+C only stops the script.
Run it with the output on the screen, not piped into tee or more: the ESTOP goes out first
either way, but the messages after it are lost.

Test knob, for the emulator only (refused unless --host is 127.0.0.1 or localhost):
    --time-scale S   multiplies the probe's own waits by S: settle pauses and VP polling
                     deadlines. Start the emulator with the same --time-scale. Servo streams,
                     the watchdog watch and RPC timeouts stay in real time. Default 1 (off).
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import signal
import statistics
import sys
import threading
import time
import traceback
import unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# pylibs/ holds Windows builds (grpcio's cygrpc is a .pyd); on Linux it would shadow pip3's grpcio.
sys.path[:0] = ([os.path.join(ROOT, "pylibs")] if os.name == "nt" else []) + [ROOT]

import grpc  # noqa: E402
from cyberdog_sdk import wire  # noqa: E402

CERTS = os.path.join(ROOT, "certs")
ZERO3, ZERO6 = [0.0, 0.0, 0.0], [0.0] * 6
LOCAL_HOSTS = ("127.0.0.1", "localhost")
ESTOP_REPEAT_S = 0.35       # the servo watchdog's WalkStand 112 lands 200-250 ms after the last frame
                            # and can overwrite the first ESTOP (cyberdog_sdk/servo.py WATCHDOG_S)
ESTOP_LATE_S = 1.0          # on the real dog the 112 came 0.61-0.72 s after the last frame
ESTOP_RETRY_S = 6.0         # how long ESTOP is sent again when the second one gets no reply
SCALE = 1.0                # --time-scale test knob; 1 on the dog


def pause(seconds):
    """A settle wait of the probe itself (shortened only by the --time-scale test knob)."""
    time.sleep(seconds * SCALE)


def span(seconds):
    return seconds * SCALE


def _read(name):
    with open(os.path.join(CERTS, name), "rb") as f:
        return f.read()


class Dog:
    """Two mTLS channels: one for commands, one kept free for ESTOP."""

    def __init__(self, host, port):
        creds = grpc.ssl_channel_credentials(_read("ca-cert.pem"), _read("client-key.pem"),
                                             _read("client-cert.pem"))
        opts = (("grpc.ssl_target_name_override", "cyberdog2.server"),
                ("grpc.max_receive_message_length", 32 * 1024 * 1024))
        target = "%s:%d" % (host, port)
        self.ch = grpc.secure_channel(target, creds, opts)
        self.estop_ch = grpc.secure_channel(target, creds, opts + (("grpc.use_local_subchannel_pool", 1),))
        self._send = self.ch.unary_stream(wire.SEND_MSG, wire.encode_send_request, wire.decode_rec_response)
        self._estop = self.estop_ch.unary_stream(wire.SEND_MSG, wire.encode_send_request,
                                                 wire.decode_rec_response)
        self._file = self.ch.unary_stream(wire.GET_FILE, wire.encode_send_request, wire.decode_file_chunk)

    def wait_ready(self, timeout=8.0):
        """Both channels connected within timeout, or grpc.FutureTimeoutError. Waits in 0.2 s slices:
        on Windows a Ctrl+C lands only when a wait ends."""
        end = time.monotonic() + timeout
        for ch in (self.ch, self.estop_ch):
            fut = grpc.channel_ready_future(ch)
            try:
                while True:
                    try:
                        fut.result(timeout=min(0.2, max(0.0, end - time.monotonic())))
                        break
                    except grpc.FutureTimeoutError:
                        if time.monotonic() >= end:
                            raise
            finally:
                fut.cancel()

    def estop_ready(self, timeout=2.0):
        """The ESTOP channel is connected (a dropped one reconnects here, not on the first ESTOP)."""
        try:
            grpc.channel_ready_future(self.estop_ch).result(timeout=timeout)
            return True
        except grpc.FutureTimeoutError:
            return False

    def call(self, name_code, params, timeout=10.0):
        """One sendMsg. Returns a record: data strings, elapsed seconds, and the gRPC error if any."""
        t0 = time.time()
        rec = {"name_code": name_code, "params": params, "data": [], "error": None}
        try:
            for r in self._send(wire.SendRequest(name_code, json.dumps(params)), timeout=timeout):
                rec["data"].append(r.data)
        except grpc.RpcError as e:
            rec["error"] = "%s %s" % (e.code().name, e.details())
        rec["elapsed"] = round(time.time() - t0, 3)
        return rec

    def get_file(self, name_code, params, timeout=40.0):
        t0 = time.time()
        rec = {"name_code": name_code, "params": params, "error_codes": [], "file_name": None,
               "file_size": None, "bytes": 0, "error": None}
        try:
            for c in self._file(wire.SendRequest(name_code, json.dumps(params)), timeout=timeout):
                rec["error_codes"].append(c.error_code)
                rec["file_name"] = c.file_name or rec["file_name"]
                rec["file_size"] = c.file_size or rec["file_size"]
                rec["bytes"] += len(c.buffer)
        except grpc.RpcError as e:
            rec["error"] = "%s %s" % (e.code().name, e.details())
        rec["elapsed"] = round(time.time() - t0, 3)
        return rec

    def read_file(self, name, timeout=10.0):
        """getFile 4004 of a VP drop file -> (found, content). found means the dog opened and sent
        it (error code 0), which also deletes it on the dog, even when it was empty."""
        buf, codes = b"", []
        try:
            for c in self._file(wire.SendRequest(4004, json.dumps({"file_name": name})), timeout=timeout):
                codes.append(c.error_code)
                buf += c.buffer
        except grpc.RpcError:
            pass
        return 0 in codes, buf

    def estop(self):
        """ESTOP (motion 0) on the channel kept free for it: the reply, elapsed, and the gRPC error."""
        t0 = time.time()
        rec = {"data": [], "error": None}
        try:
            for r in self._estop(wire.SendRequest(1004, json.dumps({"motion_id": 0})), timeout=3):
                rec["data"].append(r.data)
        except grpc.RpcError as e:
            rec["error"] = "%s %s" % (e.code().name, e.details())
        rec["elapsed"] = round(time.time() - t0, 3)
        return rec


def ignore_ctrl_c():
    """Ignore further Ctrl+C presses. Returns the old handler, or None off the main thread."""
    while True:
        try:
            return signal.signal(signal.SIGINT, signal.SIG_IGN)
        except KeyboardInterrupt:
            continue
        except ValueError:                              # not the main thread
            return None


def restore_ctrl_c(old):
    if old is not None:
        signal.signal(signal.SIGINT, old)


def say(text):
    """print for the stop path: a stdout whose reader died with the same Ctrl+C (| tee, | more)
    raises here, and that must not skip an ESTOP, its retries or the save."""
    try:
        print(text, flush=True)
    except (OSError, ValueError):
        pass


def sleep_until(t):
    """Sleep until time.monotonic() reaches t; a Ctrl+C that still lands does not cut it short."""
    while time.monotonic() < t:
        try:
            time.sleep(max(0.0, t - time.monotonic()))
        except KeyboardInterrupt:
            pass


def send_estop(dog):
    """One ESTOP; a Ctrl+C that still lands during the send repeats it instead of cutting it short."""
    while True:
        try:
            return dog.estop()
        except KeyboardInterrupt:
            continue


def delivered(rec):
    return rec["error"] is None and bool(rec["data"])


def estop_burst(dog):
    """ESTOP now, again ESTOP_REPEAT_S and ESTOP_LATE_S after the first one is sent, after the
    servo watchdog window. The first goes from its own thread, so a slow reply cannot delay the
    others.
    The caller ignores Ctrl+C first (ignore_ctrl_c)."""
    first = [None]
    th = threading.Thread(target=lambda: first.__setitem__(0, send_estop(dog)), daemon=True)
    t_sent = time.monotonic()
    th.start()
    sleep_until(t_sent + ESTOP_REPEAT_S)
    second = send_estop(dog)
    sleep_until(t_sent + ESTOP_LATE_S)
    third = send_estop(dog)
    end = time.monotonic() + 4.0                        # Dog.estop gives up after 3 s
    while th.is_alive() and time.monotonic() < end:
        try:
            th.join(0.05)
        except KeyboardInterrupt:
            pass
    return [first[0] or {"data": [], "error": "no reply within 4 s", "elapsed": None}, second, third]


def emergency_stop(dog):
    """estop_burst, then ESTOP every 0.3 s until one sent after the watchdog window gets a reply,
    for up to ESTOP_RETRY_S. Returns (records, delivered)."""
    recs = estop_burst(dog)
    if not any(map(delivered, recs[1:])):
        say("Il cane non conferma ancora l'arresto: riprovo per %d s. Stai lontano dal cane."
            % ESTOP_RETRY_S)
    end = time.monotonic() + ESTOP_RETRY_S
    while not any(map(delivered, recs[1:])) and time.monotonic() < end:
        sleep_until(time.monotonic() + 0.3)
        recs.append(send_estop(dog))
    return recs, any(map(delivered, recs[1:]))


def status_of(rec):
    """10001 reply -> (motion_id, task_status) or (None, None)."""
    try:
        d = json.loads(rec["data"][0])
        return d["motion_status"]["motion_id"], d["task_status"]["task_status"]
    except (IndexError, KeyError, TypeError, ValueError):
        return None, None


def first_json(rec):
    """The first JSON reply of a call record, or {}."""
    try:
        d = json.loads(rec["data"][0])
        return d if isinstance(d, dict) else {}
    except (IndexError, KeyError, TypeError, ValueError):
        return {}


def servo_frame(motion_id, cmd_type, vel=ZERO3, rpy=ZERO3, pos=ZERO3):
    # Every float is a JSON double and every array has its exact size (motion.md §8).
    return {"motion_id": motion_id, "cmd_type": cmd_type, "value": 0,
            "vel_des": [float(v) for v in vel], "rpy_des": [float(v) for v in rpy],
            "pos_des": [float(v) for v in pos], "acc_des": list(ZERO6), "ctrl_point": list(ZERO3),
            "foot_pose": list(ZERO6), "step_height": [0.05, 0.05]}


def vp_debug(task_id, body):
    # VP 2002 debug task (vp_led_sensors.md §1.1, §5 A). The body runs at top level on the dog
    # after a header that already imports os, sys, time and threading; it must pass pyflakes.
    return {"type": "task", "id": task_id, "target_id": ["debug"], "operate": "debug",
            "describe": "pc probe %s" % task_id, "style": "pc", "mode": "single",
            "condition": "now", "body": body}


class Poller:
    """Polls 10001 in the background and keeps (t, motion_id, task_status, elapsed)."""

    def __init__(self, dog, period):
        self.dog, self.period, self.samples = dog, period, []
        self._stop = threading.Event()
        self._t0 = time.time()
        self._th = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._th.start()
        return self

    def __exit__(self, exc_type, *a):
        self._stop.set()
        if exc_type is None:            # on Ctrl+C do not wait for a poll in flight: ESTOP goes first
            end = time.monotonic() + 3  # short joins: on Windows a Ctrl+C lands only when a wait ends
            while self._th.is_alive() and time.monotonic() < end:
                self._th.join(0.05)

    def _run(self):
        while not self._stop.is_set():
            rec = self.dog.call(10001, {}, timeout=3)
            mid, task = status_of(rec)
            self.samples.append((round(time.time() - self._t0, 2), mid, task, rec["elapsed"], rec["error"]))
            self._stop.wait(self.period)

    def motion_ids(self):
        out = []
        for s in self.samples:
            if not out or out[-1][1] != s[1]:
                out.append((s[0], s[1]))
        return out


def flush_input():
    """Drop what was typed before the question showed, so a key pressed early cannot answer it."""
    try:
        if not sys.stdin.isatty():
            return
        if os.name == "nt":
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getwch()
        else:
            import termios
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass


def plain(text):
    """Lower case without accents: ' Giù ' -> 'giu'."""
    return "".join(c for c in unicodedata.normalize("NFKD", text.strip().lower())
                   if not unicodedata.combining(c))


def ask(question, options):
    """Ask the person at the dog. options maps the typed key to the recorded answer.
    No answer (end of input, or Ctrl+C, which some Windows consoles report as end of input)
    raises KeyboardInterrupt: in Part 2 that sends ESTOP."""
    keys = "/".join(options)
    while True:
        flush_input()
        try:
            a = plain(input("  %s [%s] " % (question, keys)))
        except EOFError:
            print()
            raise KeyboardInterrupt("no answer (end of input)")
        if a in options:
            return options[a]
        print("  Scrivi una di queste risposte: %s" % keys)


YES_NO = {"s": "si", "n": "no"}
FLOOR = {"libero": "si", "n": "no"}      # a word of its own: a habitual "s" cannot confirm the floor


class Ctx:
    """What a step may need besides the dog: this run's stamp and the results so far."""

    def __init__(self, stamp, results):
        self.stamp, self.results = stamp, results


# ---- steps where the dog stays still --------------------------------------------
def step_status(dog, ctx):
    recs = [dog.call(10001, {}, timeout=3) for _ in range(5)]
    return {"replies": recs[:2], "latency_s": [r["elapsed"] for r in recs],
            "motion_id_at_rest": status_of(recs[-1])[0], "task_status_at_rest": status_of(recs[-1])[1]}


def step_device_info(dog, ctx):
    flags = {"is_bat_info": True, "is_stand": True, "is_motor_temper": True, "is_wifi": True,
             "is_volume": True, "is_sn": True, "is_device_model": True, "is_lowpower_control": True}
    return dog.call(1001, flags, timeout=15)


def step_parallel_status(dog, ctx):
    out = []
    ths = [threading.Thread(target=lambda: out.append(dog.call(10001, {}, timeout=5))) for _ in range(3)]
    t0 = time.time()
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    return {"wall_s": round(time.time() - t0, 3), "each_s": [r["elapsed"] for r in out],
            "errors": [r["error"] for r in out]}


def step_map(dog, ctx):
    rec = dog.call(6003, {}, timeout=15)
    summary = {"elapsed": rec["elapsed"], "error": rec["error"], "messages": len(rec["data"])}
    for d in rec["data"]:
        try:
            j = json.loads(d)
        except ValueError:
            summary.setdefault("non_json", []).append(d[:200])
            continue
        m = j.get("map") or {}
        summary.update({"success": j.get("success"), "mapName": j.get("mapName"),
                        "is_outdoor": j.get("is_outdoor"), "labels": j.get("locationLabelInfo"),
                        "width": m.get("width"), "height": m.get("height"),
                        "resolution": m.get("resolution"), "cells": len(m.get("data") or []),
                        "error_code": j.get("error_code")})
    return summary


def step_camera_delete(dog, ctx):
    # app/README.md says stills pile up in /home/mi/Camera; transmit_files.hpp says they are
    # deleted after a successful send. A second fetch of the same name settles it. Only the
    # photo taken here is fetched again (4004 deletes what it sends).
    res = {}
    try:
        res["start"] = dog.call(4002, {"command": 7}, timeout=12)     # 7 START_LIVE_STREAM
        pause(1.0)
        res["take"] = dog.get_file(4002, {"command": 1})              # 1 TAKE_PICTURE
        name = res["take"]["file_name"]
        if name and not any(c in name for c in ("/", "\\", "..")):   # contracts.md 1, rule 5
            res["refetch"] = dog.get_file(4004, {"file_name": name})
            res["deleted_after_send"] = 9 in res["refetch"]["error_codes"]
    finally:
        res["stop"] = dog.call(4002, {"command": 8}, timeout=12)      # 8 STOP_LIVE_STREAM
    return res


def fetch_drop(dog, name, wait_s):
    """Poll getFile 4004 until the VP task has written the file, or give up."""
    t0 = time.time()
    tries = 0
    while time.time() - t0 < span(wait_s):
        tries += 1
        rec = dog.get_file(4004, {"file_name": name}, timeout=10)
        if 0 in rec["error_codes"]:                     # sent (and so deleted), even when empty
            return {"found_after_s": round(time.time() - t0, 1), "tries": tries, "bytes": rec["bytes"],
                    "refetch_error_codes": dog.get_file(4004, {"file_name": name})["error_codes"]}
        pause(1.0)
    return {"found_after_s": None, "tries": tries}


def fetch_text(dog, name, wait_s):
    """Same as fetch_drop, but returns the file content (for the BMS read-back)."""
    t0 = time.time()
    while time.time() - t0 < span(wait_s):
        found, buf = dog.read_file(name)
        if found:
            return {"found_after_s": round(time.time() - t0, 1), "text": buf.decode("utf-8", "replace")[:4000]}
        pause(1.0)
    return {"found_after_s": None}


def step_vp(dog, ctx):
    res = {}
    # V1: the smallest possible task. No import, no sleep: is the engine alive and can it write here?
    body1 = "open('/home/mi/Camera/probe_v1.json', 'w').write('{\"ok\": 1}')"
    res["v1_send"] = dog.call(2002, vp_debug("probe_v1", body1), timeout=10)
    res["v1_file"] = fetch_drop(dog, "probe_v1.json", 25)
    # V2: the spec's LED example (blue head light for 4 s), then a file to show the task ran to the end.
    print("  Guarda la luce sulla testa del cane per i prossimi 10 secondi.")
    body2 = ("cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 0, 0, 255)\n"
             "time.sleep(4)\n"
             "cyberdog.led.freed(LedConstraint.target_head)\n"
             "open('/home/mi/Camera/probe_v2.json', 'w').write('{\"ok\": 2}')")
    res["v2_send"] = dog.call(2002, vp_debug("probe_v2", body2), timeout=10)
    res["v2_file"] = fetch_drop(dog, "probe_v2.json", 30)
    res["v2_led_seen"] = ask("La luce della testa e' diventata blu per qualche secondo?",
                             {"s": "si", "n": "no", "x": "non so"})
    # V3: battery read-back, written as repr() so the field names of the VP object show up.
    # dir() is guarded only because the emulator's task sandbox lacks it; the dog has it.
    body3 = ("b = cyberdog.bms.get_data(5)\n"
             "try:\n"
             "    names = dir(b)\n"
             "except NameError:\n"
             "    names = []\n"
             "open('/home/mi/Camera/probe_v3.txt', 'w').write(repr(b) + '\\n' + repr(names))")
    res["v3_send"] = dog.call(2002, vp_debug("probe_v3", body3), timeout=10)
    res["v3_file"] = fetch_text(dog, "probe_v3.txt", 30)
    return res


# V4 task bodies (Python 3.6, pyflakes-clean). LED calls as in V2; names checked in the
# abilityset sources: cyberdog.led.play_rgb(target, effect, r, g, b) and freed(target)
# (src/api/led.cpp:26-44), LedConstraint.target_head / effect_line_on (src/api/common.cpp:402, 406),
# replies with .state.code and .response.code (common.cpp:528-533, ros_protocol.cpp:828).
# Drop files are written under a .tmp name and renamed, so a poll never fetches (and so deletes)
# a half-written file. @RUN@ is this run's stamp: a file left by an earlier run is ignored.
V4_HELPERS = (
    "def code(r):\n"
    "    try:\n"
    "        return [int(r.state.code), int(r.response.code)]\n"
    "    except Exception as e:\n"
    "        return repr(e)[:80]\n"
    "\n"
    "\n"
    "def drop(name, text):\n"
    "    with open('/home/mi/Camera/' + name + '.tmp', 'w') as fh:\n"
    "        fh.write(text)\n"
    "    os.rename('/home/mi/Camera/' + name + '.tmp', '/home/mi/Camera/' + name)\n"
    "\n"
    "\n")
# A: blue for 20 s, played again every second, so a task that survives B shows as blue coming back.
V4_A = V4_HELPERS + (
    "with open('/home/mi/Camera/probe_v4a.pid', 'w') as fh:\n"
    "    fh.write(str(os.getpid()))\n"
    "plays = []\n"
    "for i in range(20):\n"
    "    plays.append(code(cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on,"
    " 0, 0, 255)))\n"
    "    if i == 0:\n"
    "        drop('probe_v4a_on.txt', repr({'run': '@RUN@', 'play': plays[0]}))\n"
    "    time.sleep(1)\n"
    "freed = code(cyberdog.led.freed(LedConstraint.target_head))\n"
    "drop('probe_v4a_end.txt', repr({'run': '@RUN@', 'plays': plays, 'freed': freed}))\n")
# B: green for 5 s, then free, then report the LED codes and whether A's process still exists.
V4_B = V4_HELPERS + (
    "def alive(pid):\n"
    "    # signal 0 sends nothing: the kernel only says whether the process exists\n"
    "    try:\n"
    "        os.kill(pid, 0)\n"
    "    except OSError as e:\n"
    "        return 'yes' if e.errno == 1 else 'no' if e.errno == 3 else repr(e)[:80]\n"
    "    except Exception as e:\n"
    "        return repr(e)[:80]\n"
    "    return 'yes'\n"
    "\n"
    "\n"
    "pid = 0\n"
    "try:\n"
    "    with open('/home/mi/Camera/probe_v4a.pid') as fh:\n"
    "        pid = int(fh.read())\n"
    "    os.remove('/home/mi/Camera/probe_v4a.pid')\n"
    "except Exception:\n"
    "    pass\n"
    "before = alive(pid) if pid > 0 else None\n"
    "play = code(cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_on, 0, 255, 0))\n"
    "time.sleep(5)\n"
    "freed = code(cyberdog.led.freed(LedConstraint.target_head))\n"
    "after = alive(pid) if pid > 0 else None\n"
    "drop('probe_v4b.txt', repr({'run': '@RUN@', 'a_alive_at_start': before, 'a_alive_after_5s': after,"
    " 'play': play, 'freed': freed}))\n")
V4_CLEANUP = "cyberdog.led.freed(LedConstraint.target_head)\n"
V4_SEEN = {"b": "blu_verde_normale", "r": "rimasta_blu", "v": "rimasta_verde", "a": "altro",
           "x": "non_so"}
V4_SEEN_IT = {"blu_verde_normale": "blu, poi verde, poi normale", "rimasta_blu": "rimasta blu",
              "rimasta_verde": "rimasta verde", "altro": "altro", "non_so": "non visto bene"}
V4_EVIDENCE_IT = {"no_b_report": "il secondo task non ha risposto",
                  "a_pid_gone": "il processo del primo task non c'era piu'",
                  "a_pid_alive": "il processo del primo task era ancora vivo",
                  "a_finished": "il primo task e' arrivato alla fine",
                  "a_unfinished": "il primo task non e' arrivato alla fine: fermato, o andato in errore"}


def poll_drop(dog, name, run, deadline):
    """Poll for a V4 drop file (repr() of a dict) whose 'run' is this run's stamp, until the
    time.time() deadline. A file from an earlier run is fetched, so deleted, and skipped."""
    t0 = time.time()
    out = {"found_after_s": None, "tries": 0, "stale": 0}
    while True:
        out["tries"] += 1
        found, buf = dog.read_file(name)
        if found:
            text = buf.decode("utf-8", "replace")
            try:
                data = ast.literal_eval(text)
            except (ValueError, SyntaxError):
                data = None
            if isinstance(data, dict) and data.get("run") == run:
                out.update(found_after_s=round(time.time() - t0, 1), data=data)
                return out
            out["stale"] += 1
            out["stale_text"] = text[:300]
        if time.time() >= deadline:
            return out
        time.sleep(max(0.1, span(0.5)))


def v4_verdicts(res):
    """Machine answers of V4, from B's report and A's end file; None when the run cannot say."""
    b = (res.get("b_file") or {}).get("data")
    a_end = (res.get("a_end") or {}).get("found_after_s")
    out = {"new_task_kills_running": None, "kill_evidence": None, "new_task_drives_led": None}
    if b is None:
        out["kill_evidence"] = "no_b_report"
    elif b.get("a_alive_after_5s") == "no":            # os.kill(pid, 0) said ESRCH while B ran
        out.update(new_task_kills_running=True, kill_evidence="a_pid_gone")
    elif b.get("a_alive_after_5s") == "yes":
        out.update(new_task_kills_running=False, kill_evidence="a_pid_alive")
    elif a_end is not None:
        out.update(new_task_kills_running=False, kill_evidence="a_finished")
    else:                                               # A killed, or A crashed: the person's answer tells
        out.update(new_task_kills_running=True, kill_evidence="a_unfinished")
    play = b.get("play") if b else None
    if isinstance(play, list) and len(play) == 2 and play[0] == 0:
        out["new_task_drives_led"] = play[1] == 0       # LedExecute 0 SUCCEED, 1125 LOW_PRIORITY
    return out


def step_vp_v4(dog, ctx):
    """V4: does a new debug task kill the running one, and can the new task drive the head LED
    that the killed task still occupies? On the dog a debug request builds the new task, then
    calls at_.Cancellation("debug"): atrm plus kill -9 of the old process (cyberdog_vp_engine
    task.cpp:301-313, linux_at.cpp:135-200). kill -9 skips the generated footer, so the old task's
    cyberdog.shutdown() never frees its LED. Both tasks are LED client "vp".
    A plays blue for 20 s; 3 s after A's first blue, B plays green for 5 s, frees the LED and
    writes its report; then the person says what they saw. About 30 s on the dog."""
    run = ctx.stamp
    vp = ctx.results["steps"].get("vp") or {}
    for k in ("v1_file", "v2_file"):
        if (vp.get(k) or {}).get("found_after_s") is None:
            return {"skipped": "%s gave no drop file: VP is not usable" % k[:2].upper()}
    res = {"run": run}
    print("  Guarda la luce sulla testa del cane per circa mezzo minuto.")
    try:
        res["a_send"] = dog.call(2002, vp_debug("probe_v4a", V4_A.replace("@RUN@", run)), timeout=10)
        res["a_on"] = poll_drop(dog, "probe_v4a_on.txt", run, time.time() + span(25))
        if res["a_on"]["found_after_s"] is None:
            res["verdicts"] = v4_verdicts(res)
            return res
        t_on = time.time()
        pause(3.0)
        res["b_send"] = dog.call(2002, vp_debug("probe_v4b", V4_B.replace("@RUN@", run)), timeout=10)
        res["b_file"] = poll_drop(dog, "probe_v4b.txt", run, time.time() + span(25))
        if ((res["b_file"].get("data") or {}).get("a_alive_after_5s")) != "no":
            # A alive, or the dog could not say: A's own end file settles it (20 s of blue, plus margin)
            res["a_end"] = poll_drop(dog, "probe_v4a_end.txt", run, t_on + span(28))
        res["verdicts"] = v4_verdicts(res)
        res["seen"] = ask("Cosa ha fatto la luce della testa? b = blu, poi verde, poi normale; "
                          "r = e' rimasta blu (anche con lampi verdi); v = e' rimasta verde; "
                          "a = altro; x = non ho visto bene", V4_SEEN)
    finally:
        # A killed by kill -9 never frees the LED: free it, so the head light goes back to normal.
        res["cleanup_send"] = dog.call(2002, vp_debug("probe_v4c", V4_CLEANUP), timeout=10)
    return res


def step_skin(dog, ctx):
    res = {}
    try:
        res["enable"] = dog.call(11001, {"data": True}, timeout=8)
        res["set"] = dog.call(11002, {"mode": 2, "wave_cycle_time": 3000}, timeout=8)
        pause(3)
        res["seen"] = ask("Il corpo del cane ha cambiato colore (pelle elettronica)?",
                          {"s": "si", "n": "no", "x": "non ha la pelle elettronica"})
    finally:
        res["disable"] = dog.call(11001, {"data": False}, timeout=8)
    return res


# ---- steps that move the dog ------------------------------------------------------
def motion(dog, motion_id, period=0.25):
    """One 1004 with a 10001 poller beside it: the result, the timing, and the motion_id trail."""
    with Poller(dog, period) as p:
        rec = dog.call(1004, {"motion_id": motion_id}, timeout=25)
        pause(1.0)
    ok_polls = [s for s in p.samples if s[1] is not None and s[0] < rec["elapsed"]]
    return {"reply": rec, "motion_id_trail": p.motion_ids(),
            "polls_answered_during_1004": len(ok_polls), "poll_latency_max_s":
            max([s[3] for s in p.samples] or [None])}


def motion_ok(res):
    """True when the step's 1004 answered result true."""
    return first_json((res or {}).get("reply") or {}).get("result") is True


def announce(text):
    """Tell the person what the dog does next, a moment before it starts."""
    print("  " + text)
    pause(1.5)


def check(res, rec, motion_id, seen=None):
    """Mark res so that Part 2 stops after this step: rec's 1004 did not answer result true, or the
    person answered no. A result true only says the dog accepted the command, so both count."""
    if "stopped" not in res:
        if rec is not None and not motion_ok(rec):
            res["stopped"] = "%d not accepted" % motion_id
            res["stopped_it"] = "il cane non ha accettato il comando %d" % motion_id
        elif seen == "no":
            res["stopped"] = "the person answered no after %d" % motion_id
            res["stopped_it"] = "hai risposto no dopo il comando %d" % motion_id
    return res


def step_stand(dog, ctx):
    announce("Il cane si alzera' in piedi (111).")
    res = motion(dog, 111)
    res["seen"] = ask("Il cane si e' alzato in piedi?", YES_NO)
    return check(res, res, 111, res["seen"])


def step_sit(dog, ctx):
    announce("Il cane si siedera' (143).")
    res = motion(dog, 143)
    pause(3)
    res["motion_id_3s_later"] = status_of(dog.call(10001, {}, timeout=3))[0]
    res["seen"] = ask("Il cane si e' seduto ed e' rimasto seduto?", YES_NO)
    if "stopped" in check(res, res, 143, res["seen"]):
        return res
    announce("Il cane si rialzera' in piedi (111).")
    res["stand_again"] = motion(dog, 111)
    res["seen_standing"] = ask("Il cane e' di nuovo in piedi?", YES_NO)
    return check(res, res["stand_again"], 111, res["seen_standing"])


def step_trick(dog, ctx):
    announce("Il cane dara' la zampa sinistra (141).")
    res = motion(dog, 141)
    res["seen"] = ask("Ha dato la zampa?", YES_NO)
    return check(res, res, 141, res["seen"])


def step_posture(dog, ctx):
    """111 first: after the paw the dog rests in 214, and a 201 stream from there makes the dog
    stand up by itself first, which takes the tilt. Then the 201 stream at 15 Hz: pitch ramps to
    +0.15 rad by 0.02 per frame (as pose_teleop.py), holds 1 s, then END. Answers the pitch sign
    and whether END keeps the pose; 111 again puts the head back."""
    announce("Il cane si rimettera' dritto in piedi (111), poi la testa si inclinera' un poco.")
    res = {"pre": motion(dog, 111)}
    if "stopped" in check(res, res["pre"], 111):
        return res
    lat, pitch = [], 0.0
    frames = [min(0.15, 0.02 * (i + 1)) for i in range(8)] + [0.15] * 15
    for pitch in frames:
        t0 = time.time()
        rec = dog.call(1002, servo_frame(201, 1, rpy=[0.0, pitch, 0.0], pos=[0.0, 0.0, 0.235]), timeout=0.25)
        lat.append(rec["elapsed"])
        time.sleep(max(0.0, 1 / 15 - (time.time() - t0)))
    end = dog.call(1002, servo_frame(201, 2, rpy=[0.0, pitch, 0.0], pos=[0.0, 0.0, 0.235]), timeout=1)
    pause(2.0)
    res.update({"frames": len(frames), "frame_latency_max_s": max(lat), "end": end,
                "motion_id_after_end": status_of(dog.call(10001, {}, timeout=3))[0]})
    # the probe sent pitch +0.15 rad: the answer gives the sign on the real dog
    res["pitch_direction"] = ask("La testa del cane si e' alzata (su), si e' abbassata (giu) o non si e' mossa (n)?",
                                 {"su": "su", "giu": "giu", "n": "non si e' mossa"})
    res["held_after_end"] = ask("Dopo lo stop la testa e' rimasta inclinata?", YES_NO)
    announce("Il cane rimettera' la testa dritta (111).")
    res["restore"] = motion(dog, 111)
    res["seen_restored"] = ask("La testa e' tornata dritta, con il cane in piedi?", YES_NO)
    return check(res, res["restore"], 111, res["seen_restored"])


def step_walk_watchdog(dog, ctx):
    """Trot in place (303, zero velocity) at 25 Hz for 3 s, then stop sending without END and watch
    how long the dog takes to switch to WalkStand 112 (the servo watchdog, motion.md §3)."""
    announce("Il cane trottera' sul posto per circa 3 secondi, senza spostarsi.")
    lat, gaps, last = [], [], None
    t_end = time.time() + 3.0
    while time.time() < t_end:
        t0 = time.time()
        if last is not None:
            gaps.append(t0 - last)
        last = t0
        rec = dog.call(1002, servo_frame(303, 1), timeout=0.25)
        lat.append(rec["elapsed"])
        time.sleep(max(0.0, 0.04 - (time.time() - t0)))
    stopped = time.time()
    trail = []
    while time.time() - stopped < 2.0:
        mid = status_of(dog.call(10001, {}, timeout=1))[0]
        trail.append((round(time.time() - stopped, 2), mid))
        time.sleep(0.1)
    dog.call(1002, servo_frame(303, 2), timeout=1)   # tidy END; the dog republishes 112 at most
    res = {"frames": len(lat), "latency_p50_s": round(statistics.median(lat), 3),
           "latency_p95_s": round(sorted(lat)[int(0.95 * (len(lat) - 1))], 3),
           "latency_max_s": round(max(lat), 3), "gap_max_ms": round(1000 * max(gaps or [0]), 1),
           "frames_timed_out": sum(1 for x in lat if x >= 0.25), "after_stop_trail": trail}
    res["seen"] = ask("Ha trottato sul posto e poi si e' fermato in piedi?", YES_NO)
    return check(res, None, 303, res["seen"])


def step_lie_down(dog, ctx):
    announce("Il cane si sdraiera' (101).")
    res = motion(dog, 101)
    res["seen"] = ask("Il cane si e' sdraiato?", YES_NO)
    return check(res, res, 101, res["seen"])


STILL_STEPS = [("status", step_status), ("device_info", step_device_info),
               ("parallel_status", step_parallel_status), ("map", step_map),
               ("camera_delete", step_camera_delete), ("vp", step_vp), ("vp_v4", step_vp_v4),
               ("skin", step_skin)]
MOTION_STEPS = [("stand", step_stand), ("sit", step_sit), ("trick", step_trick),
                ("posture", step_posture), ("walk_watchdog", step_walk_watchdog),
                ("lie_down", step_lie_down)]
STEP_TITLES = {"status": "stato", "device_info": "informazioni del cane",
               "parallel_status": "tre richieste insieme", "map": "mappa", "camera_delete": "foto",
               "vp": "task VP e luce della testa", "vp_v4": "due task VP di seguito",
               "skin": "pelle elettronica", "stand": "in piedi", "sit": "seduto", "trick": "zampa",
               "posture": "testa inclinata", "walk_watchdog": "trotto sul posto", "lie_down": "sdraiato"}


def battery(results):
    """State of charge from 1001 bat_info: the dog calls it "power" (query_info.hpp:352)."""
    try:
        b = json.loads(results["steps"]["device_info"]["data"][0]).get("bat_info") or {}
    except (KeyError, IndexError, TypeError, ValueError, AttributeError):
        return None
    v = b.get("power", b.get("soc"))
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


# ---- the Italian summary ------------------------------------------------------------
def _yn(v):
    return {True: "si", False: "no", None: "non chiaro"}.get(v, str(v))


def summary_lines(results, out_path):
    st = results.get("steps") or {}
    lines = ["", "== Riepilogo =="]

    def section(fn):
        try:
            text = fn()
        except Exception:
            text = "(riepilogo di %s non disponibile)" % fn.__name__
        if text:
            lines.append("- " + text)

    def link():
        s = st.get("status") or {}
        lat = sorted(x for x in s.get("latency_s") or [] if x is not None)
        if not lat:
            return "Collegamento: nessuna risposta allo stato (10001)."
        return ("Collegamento: lo stato risponde in %d ms (mediana); a riposo motion_id %s."
                % (round(1000 * lat[len(lat) // 2]), s.get("motion_id_at_rest")))

    def power():
        soc = battery(results)
        return "Batteria: %s%%." % soc if soc is not None else "Batteria: non letta."

    def mapping():
        m = st.get("map") or {}
        if not m.get("mapName") and not m.get("cells"):
            return "Mappa: nessuna mappa letta (errore %s)." % (m.get("error") or m.get("error_code"))
        return ("Mappa: '%s', %sx%s celle, %d etichette."
                % (m.get("mapName"), m.get("width"), m.get("height"), len(m.get("labels") or [])))

    def camera():
        c = st.get("camera_delete") or {}
        t = c.get("take") or {}
        if not t.get("bytes"):
            return "Fotocamera: nessuna foto (codici %s)." % t.get("error_codes")
        return ("Fotocamera: foto di %d byte; cancellata dal cane dopo l'invio: %s."
                % (t["bytes"], _yn(c.get("deleted_after_send"))))

    def vp():
        v = st.get("vp") or {}
        if "exception" in v:
            return "VP: errore nel passo."
        f1 = (v.get("v1_file") or {}).get("found_after_s")
        f2 = (v.get("v2_file") or {}).get("found_after_s")
        f3 = (v.get("v3_file") or {}).get("found_after_s")
        return ("VP: task minimo %s; task con LED %s (luce blu vista: %s); batteria letta via VP: %s."
                % ("ok dopo %s s" % f1 if f1 is not None else "senza risposta",
                   "ok" if f2 is not None else "senza risposta", v.get("v2_led_seen"),
                   _yn(f3 is not None)))

    def v4():
        v = st.get("vp_v4") or {}
        if "skipped" in v:
            return "VP V4: saltato (%s)." % v["skipped"]
        if "exception" in v:
            return "VP V4: errore nel passo."
        d = v.get("verdicts") or {}
        return ("VP V4: un nuovo task debug ferma quello in corso: %s (%s); il nuovo task comanda "
                "la luce occupata: %s; luce vista: %s."
                % (_yn(d.get("new_task_kills_running")),
                   V4_EVIDENCE_IT.get(d.get("kill_evidence"), d.get("kill_evidence")),
                   _yn(d.get("new_task_drives_led")), V4_SEEN_IT.get(v.get("seen"), v.get("seen"))))

    def skin():
        return "Pelle elettronica: %s." % (st.get("skin") or {}).get("seen")

    def moves():
        if "floor_confirmed" not in results:
            return "Movimenti: non eseguiti."
        if results["floor_confirmed"] != "si":
            return "Movimenti: non eseguiti (pavimento non confermato)."
        parts = []
        for name, label in (("stand", "in piedi"), ("sit", "seduto"), ("trick", "zampa"),
                            ("lie_down", "sdraiato")):
            if name in st:
                parts.append("%s %s/%s" % (label, "ok" if motion_ok(st[name]) else "fallito",
                                           st[name].get("seen")))
        p = st.get("posture") or {}
        if p.get("pitch_direction"):
            parts.append("pitch +0.15 = testa %s, posa tenuta dopo lo stop: %s"
                         % (p["pitch_direction"], p.get("held_after_end")))
        w = st.get("walk_watchdog") or {}
        if "after_stop_trail" in w:
            t112 = next((t for t, mid in w["after_stop_trail"] if mid == 112), None)
            parts.append("watchdog: 112 dopo %s s" % t112 if t112 is not None else "watchdog: 112 mai visto")
        return "Movimenti (risposta del cane/risposta tua): " + "; ".join(parts) + "."

    def problems():
        bad = [STEP_TITLES.get(k, k) for k, v in st.items() if isinstance(v, dict) and "exception" in v]
        out = []
        if bad:
            out.append("errori nei passi %s" % ", ".join(bad))
        for key, label in (("interrupted", "interrotto"), ("motion_stopped", "movimenti fermati"),
                           ("motion_skipped", "movimenti saltati")):
            if results.get(key):
                out.append("%s (%s)" % (label, results[key]))
        return "Problemi: " + ("; ".join(out) if out else "nessuno") + "."

    for fn in (link, power, mapping, camera, vp, v4, skin, moves, problems):
        section(fn)
    lines.append("Risultati completi: %s" % out_path)
    return lines


def main(argv=None):
    global SCALE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("DOG_IP", "192.168.137.115"))
    ap.add_argument("--port", type=int, default=50052)
    ap.add_argument("--no-motion", action="store_true", help="only Part 1: the dog stays still")
    ap.add_argument("--out", help="results file (default tools/probe_results-<date>-<time>.json)")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="TEST ONLY, emulator only (needs --host 127.0.0.1): multiply the probe's own "
                         "waits by this, 0.2-1; start the emulator with the same --time-scale. Default 1")
    args = ap.parse_args(argv)
    if args.time_scale != 1.0:
        if args.host not in LOCAL_HOSTS:
            ap.error("--time-scale is a test knob for the emulator: use it only with --host 127.0.0.1")
        if not 0.2 <= args.time_scale <= 1.0:
            # below 0.2 the emulator floors short waits at 0.2 s and VP task timing no longer scales
            ap.error("--time-scale must be between 0.2 and 1")
    SCALE = args.time_scale

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_path = os.path.abspath(args.out or os.path.join(ROOT, "tools", "probe_results-%s.json" % stamp))
    results = {"started": stamp, "host": args.host, "port": args.port, "steps": {}}
    if SCALE != 1.0:
        results["time_scale"] = SCALE
    ctx = Ctx(stamp, results)

    def save():
        # written aside and renamed: a Ctrl+C during the write cannot leave a cut results file.
        # OneDrive or the antivirus can hold the file for a moment: the run goes on without the save.
        try:
            with open(out_path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(results, f, indent=1, ensure_ascii=False)
            os.replace(out_path + ".tmp", out_path)
            return True
        except OSError as e:
            print("Attenzione: non riesco a salvare i risultati in %s (%s)." % (out_path, e), flush=True)
            return False

    def done(code, message):
        old = ignore_ctrl_c()           # a second Ctrl+C must not cut the save or the summary short
        try:
            say(message)                # first: a warning must not wait for the save
            if not save():
                say("I risultati non sono stati salvati: rilancia la prova o copia questo riepilogo.")
            say("\n".join(summary_lines(results, out_path)))
        finally:
            restore_ctrl_c(old)
        return code

    dog = Dog(args.host, args.port)
    print("Connessione a %s:%d ..." % (args.host, args.port))
    try:
        dog.wait_ready()
    except grpc.FutureTimeoutError:
        print("Il cane non risponde. E' acceso e collegato all'hotspot?")
        return 1
    except KeyboardInterrupt:
        print("\nInterrotto prima di cominciare. Nessun comando inviato.")
        return 1

    print("\n== Parte 1: controlli senza movimento (circa 3 minuti) ==")
    try:
        for name, fn in STILL_STEPS:
            print("- %s" % STEP_TITLES[name])
            try:
                results["steps"][name] = fn(dog, ctx)
            except Exception:
                results["steps"][name] = {"exception": traceback.format_exc()}
            save()
    except KeyboardInterrupt:
        results["interrupted"] = "ctrl+c in part 1: nothing was moving, no ESTOP sent"
        return done(2, "\nInterrotto. Nessun movimento era in corso.")

    soc = battery(results)
    if args.no_motion:
        return done(0, "\nFatto (senza movimento).")
    if soc is None:
        results["motion_skipped"] = "battery not read"
        return done(0, "\nBatteria non letta: salto la parte con movimento.")
    if soc < 25:
        results["motion_skipped"] = "battery %s" % soc
        return done(0, "\nBatteria al %s%%: salto la parte con movimento. Ricarica e riprova." % soc)

    print("\n== Parte 2: movimenti brevi (circa 3 minuti) ==")
    print("Ctrl+C in qualsiasi momento = arresto di emergenza (il cane si spegne e cade a terra).")
    try:
        clear = ask("Il cane e' a terra, su un pavimento piano, con almeno 1,5 m liberi intorno, "
                    "non su un tavolo o un rialzo? Per confermare scrivi libero", FLOOR)
    except KeyboardInterrupt:
        clear = "nessuna risposta"
    results["floor_confirmed"] = clear
    if clear != "si":
        return done(0, "Niente movimento.")
    try:
        for name, fn in MOTION_STEPS:
            if not dog.estop_ready():
                results["motion_stopped"] = "%s: the ESTOP channel is not connected: no further motion" % name
                return done(3, "\nIl collegamento per l'arresto di emergenza non risponde: mi fermo qui, "
                               "senza altri movimenti.")
            print("- %s" % STEP_TITLES[name])
            try:
                res = results["steps"][name] = fn(dog, ctx)
            except KeyboardInterrupt:
                raise
            except Exception:
                results["steps"][name] = {"exception": traceback.format_exc()}
                results["motion_stopped"] = "exception in %s: no further motion" % name
                return done(3, "\nErrore inatteso nel passo %s: mi fermo qui, senza altri movimenti."
                            % STEP_TITLES[name])
            save()
            if "stopped" in res:
                results["motion_stopped"] = "%s: %s: no further motion" % (name, res["stopped"])
                return done(0, "\nMi fermo qui, senza altri movimenti: %s.\nGuarda il cane: se e' in "
                               "piedi e vuoi farlo sdraiare, usa l'app v1 (tasto F)." % res["stopped_it"])
    except KeyboardInterrupt:
        while True:                     # a second Ctrl+C before the first call is inside a try: again
            try:
                old = ignore_ctrl_c()
                break
            except KeyboardInterrupt:
                continue
        try:
            # nothing is written before the first ESTOP: a dead or stalled stdout reader cannot hold it
            results["estop"], ok = emergency_stop(dog)
            if not ok:
                results["interrupted"] = "ctrl+c in part 2: no ESTOP delivered after the watchdog window"
                return done(4, "\nATTENZIONE: il cane non ha confermato l'arresto di emergenza.\n"
                               "Stai lontano dal cane e fermalo con l'app v1 (tasto Spazio).")
            results["interrupted"] = ("ctrl+c in part 2: ESTOP sent three times" if len(results["estop"]) == 3 else
                                      "ctrl+c in part 2: ESTOP delivered on send %d" % len(results["estop"]))
            return done(2, "\nESTOP inviato. Per rialzare il cane usa l'app v1 (tasto R) o il telecomando.")
        finally:
            restore_ctrl_c(old)
    return done(0, "\nFatto.")


if __name__ == "__main__":
    try:
        code = main()
    except SystemExit as e:             # --help and argparse errors keep their exit code
        code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    except BaseException:               # still reach os._exit: an error must not hang in grpc cleanup
        try:
            traceback.print_exc()
        except BaseException:
            pass
        code = 1
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except BaseException:           # a broken pipe must not skip os._exit
            pass
    os._exit(code)                      # after a Ctrl+C, grpc's channel cleanup at exit can hang
