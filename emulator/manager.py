"""motion_manager over a Controller (contracts.md 4.1).

The 1004 result path, the 1002 servo path, the 20 Hz heartbeat and the reverse map, after
sim_bridge.md sections 2.2-2.4 and the corrected pseudo-code of 2.6. Result codes are the
numbers of motion.md section 2.8. Line references: MH = motion_handler.cpp, MD = motion_decision.cpp.
"""
from __future__ import annotations

import collections
import json
import os
import threading
import time

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ARRAYS = (("vel_des", 3), ("rpy_des", 3), ("pos_des", 3), ("acc_des", 6), ("ctrl_point", 3),
          ("foot_pose", 6), ("step_height", 2))
# switch_status -> result code (MH:302-384): ESTOP, EDAMP, LIFTED, BAN_TRANS, OVER_HEAT, LOW_BAT, 8-14.
FAIL_CODE = {2: 3021, 3: 3023, 4: 3024, 5: 3022, 6: 3025, 7: 3026,
             8: 3050, 9: 3051, 10: 3052, 11: 3053, 12: 3054, 13: 3055, 14: 3056}
DANCES = {140, 176, 177, 178}                     # MH:986-992
PROTECTED_OK = {111, 101, 102, 0, 212}            # motion_manager.cpp:270-275
HEARTBEAT, FEEDBACK, SETTLE = 0.05, 0.2, 0.015    # wall-clock (motion_macros.hpp:82,85; MH:435-441)
SERVO_TICK, SERVO_MISS = 0.05, 4                  # MH:234-249; motion_macros.hpp:83
TRANSIT, CAP = 3.0, 20.0                          # scaled: MH:538-551; cyberdog_app.cpp:753-780
RTF_FLOOR = 0.2                                   # lowest real-time factor the waits are divided by


def i8(n: int) -> int:
    return ((n + 128) % 256) - 128                # int8_t life_count_ wraps (motion_action.hpp:193)


class Manager:
    def __init__(self, ctl, emu):
        self.ctl, self.emu = ctl, emu
        with open(os.path.join(DATA, "motion_id_map.json"), encoding="utf-8") as f:
            self.map = {e["motion_id"]: e for e in json.load(f)}
        self.rev = {}
        for mid in sorted(self.map):                  # std::map order, first match wins
            m = self.map[mid]["map"]
            self.rev.setdefault((m[0], m[-1]), mid)
        self.cv = threading.Condition()
        self.st = dict(motion_id=-1, mode=-1, gait=-1, bar=0, switch=0, motor_error=[0] * 12, seen=0)
        self.plk = threading.Lock()                   # life_count and the last frame
        self.hlk = threading.Lock()                   # handler work status and decision latch
        self.wlk = threading.RLock()                  # servo state shared with the watchdog
        self.life, self.last = 0, None
        self.int_gen = 0                              # execute_cv_ notify by ESTOP / dance interruption
        self.stop_ev = threading.Event()
        self.servo_q = collections.deque(maxlen=1)    # motion_servo_cmd subscription, KEEP_LAST 1 [INF]
        self.servo_ev = threading.Event()
        self._clear()
        ctl.on_response(self._on_resp)
        for fn in (self._heartbeat, self._watchdog, self._servo_worker):
            threading.Thread(target=fn, name="mgr" + fn.__name__, daemon=True).start()
        self._publish_new(self._build(101, {}, servo=False))   # boot: the last command was a lie-down

    def _clear(self):
        self.work = "idle"                            # idle | result | servo
        self.estop_latch = False                      # MotionDecision estop_
        self.h_estop = False                          # MotionHandler estop_
        self.post_checked = self.pre_failed = False
        self.last_servo = None
        self.need_check = self.ticked = False
        self.servo = dict(frames=0, last_frame=None, last_response=None, max_gap_ms=0,
                          gaps_over_150ms=0, watchdog_stops=0, walk_stands=0, _t=None)

    def reset(self):
        with self.wlk, self.hlk:                      # same order as _stop_servo
            self._clear()
            self.int_gen += 1
        self.servo_q.clear()
        with self.cv:
            self.cv.notify_all()
        if hasattr(self.ctl, "reset"):
            self.ctl.reset()
        self._publish_new(self._build(101, {}, servo=False))

    def close(self):
        self.stop_ev.set()

    # ---- controller I/O --------------------------------------------------------------
    def _on_resp(self, r):
        mid = 0 if r["mode"] == 0 else self.rev.get((r["mode"], r["gait_id"]), -1)   # motion_action.cpp:407-459
        with self.cv:
            self.st.update(motion_id=mid, mode=r["mode"], gait=r["gait_id"], bar=r["order_process_bar"],
                           switch=r["switch_status"], motor_error=list(r["motor_error"]),
                           seen=self.st["seen"] + 1)
            self.cv.notify_all()

    def _wait(self, pred, timeout):
        with self.cv:
            return self.cv.wait_for(lambda: pred(self.st), timeout)

    def _rt(self, t):
        """A manager wait (feedback, transition, completion) in wall seconds. With a controller that has
        rtf() (Gazebo), divide by the live real-time factor, as the `timeout / RTF` of sim_bridge.md 2.6.
        Unknown RTF (no simulator_state yet) uses the floor, the most lenient wait."""
        f = getattr(self.ctl, "rtf", None)
        if f is None:
            return t
        r = f()
        return t / max(r or RTF_FLOOR, RTF_FLOOR)

    def _publish_new(self, c):                        # a new frame gets a new life_count (motion_action.cpp:56,106)
        with self.plk:
            c["life_count"] = i8(self.life)
            self.life += 1
            self.last = c
            self.ctl.publish(c)

    def _heartbeat(self):                             # same frame, same life_count, 20 Hz (motion_action.cpp:474-483)
        while not self.stop_ev.wait(HEARTBEAT):
            with self.plk:
                if self.last is not None:
                    self.ctl.publish(self.last)

    def _build(self, mid, p, servo):
        m = self.map[mid]["map"]
        value = p.get("value", 0)
        if value & 0x04:                              # SIM DEVIATION: factory remount bit (command_interface.cpp:1058-1064)
            self.emu.client_bug("value bit 0x04 (factory calibration) masked", motion_id=mid, value=value)
            value &= ~0x04
        c = {"mode": m[0], "gait_id": m[-1], "life_count": 0, "value": value}
        for f, n in ARRAYS:                           # GET_VALUE: wrong length -> zeros (motion_macros.hpp:111-121)
            v = p.get(f) or []
            c[f] = [float(x) for x in v] if len(v) == n else [0.0] * n
        if servo:
            c["contact"], c["duration"] = 0, 0        # motion_action.cpp:44,46
        else:
            c["contact"], c["duration"] = p.get("contact", 0) or 15, p.get("duration", 0)
            if (m[0], m[-1]) == (3, 3) and c["duration"] == 0:
                self.emu.client_bug("202 with duration 0 never ends on the dog: it adds rpy_des again every 2 s "
                                    "(fsm_state_balancestand.cpp:386-396)", motion_id=mid)
        return c

    # ---- rules -------------------------------------------------------------------------
    def _fail(self, code):
        return {"motion_id": self.st["motion_id"], "result": False, "code": code}

    def _post_ok(self, mid):                          # CheckPostMotion, MH:871-889
        if mid in (111, 0):
            return True
        cur = self.st["motion_id"]
        if cur == -1 or cur not in self.map:
            return False
        return self.map[mid]["map"][0] in self.map[cur]["post_motion"]

    def _check(self):                                 # CheckMotionResult (motor_error is always 0 here)
        return FAIL_CODE.get(self.st["switch"], 0)

    # ---- 1004 --------------------------------------------------------------------------
    def motion_cmd_grpc(self, req):
        """The bridge view: None means error 321 (no reply within the 20 s cap; the motion goes on)."""
        out = []
        t = threading.Thread(target=lambda: out.append(self.motion_cmd(req)), name="mgr-1004", daemon=True)
        t.start()
        t.join(self.emu.sc(CAP))
        return out[0] if out else None

    def motion_cmd(self, req):
        mid = req["motion_id"]
        if self.emu.protected() and mid not in PROTECTED_OK:
            return {"motion_id": 0, "result": False, "code": 3013}      # motion_manager.cpp:265-299
        with self.hlk:
            if self.estop_latch and mid not in (0, 111):              # MD hpp:161-176
                return self._fail(3040)
            self.estop_latch = (mid == 0)                             # MD:152
            dance = self.work == "result" and self.st["motion_id"] in DANCES and mid == 101
            self.h_estop = (mid == 0)
            if dance or mid == 0:
                self.int_gen += 1                                     # MH:640-644
            if self.work != "idle" and not dance and mid != 0:
                return {"motion_id": 0, "result": False, "code": 3011}  # MH:645-655
            self.work = "result"
        with self.cv:
            self.cv.notify_all()
        try:
            if mid != 400:                                            # IsCommandValid, MH:93-119
                if mid not in self.map:
                    return self._fail(3008)
                t, d = self.map[mid]["min_exec_time"], req.get("duration", 0)
                if not (d == 0 if t > 0 else d > 0 if t < 0 else True):
                    return self._fail(3010)
            if not self._post_ok(mid):                                # MH:686-710
                r = self._execute({"motion_id": 111})
                if not r["result"]:
                    return r
                if self.h_estop:
                    return {"motion_id": 0, "result": False, "code": 0}
            if mid == 400:                                            # gait_toml is never forwarded (MH:711-722)
                return self._fail(3031)
            if tuple(self.map[mid]["map"]) == (11, 110):              # SIM GUARD: no '# Gait Def' upload
                self.emu.refuse(1004, "SIM GUARD: 401 needs a user gait upload", motion_id=mid)
                return self._fail(3031)
            return self._execute(req)
        finally:
            with self.hlk:
                self.work = "idle"

    def _execute(self, req):                          # ExecuteResultCmd, MH:401-622
        mid = req["motion_id"]
        with self.cv:
            seen0, gen0 = self.st["seen"], self.int_gen
        t_pub = time.monotonic()
        self._publish_new(self._build(mid, req, servo=False))
        if mid == 400:
            self._publish_new(self._build(400, {}, servo=False))     # MH:429-433
        if not self._wait(lambda s: s["seen"] > seen0, self._rt(FEEDBACK)):
            return self._fail(3035)
        rest = SETTLE - (time.monotonic() - t_pub)
        if rest > 0:
            time.sleep(rest)
        sw = self.st["switch"]
        if sw in FAIL_CODE:
            return self._fail(FAIL_CODE[sw])
        if sw == 1 and not self._wait(lambda s: s["motion_id"] == mid and s["switch"] == 0,
                                      self._rt(self.emu.sc(TRANSIT))):
            return self._fail(3027)
        t, d = self.map[mid]["min_exec_time"], req.get("duration", 0)
        to = t if t > 0 else d + 500 if t < 0 else (500 if d == 0 else 2 * d)
        # execute_cv_ has no predicate: an ESTOP notify ends the wait and goes to the final check
        if not self._wait(lambda s: (s["motion_id"] == mid and s["bar"] > 95) or self.int_gen != gen0,
                          self._rt(self.emu.sc(to / 1000.0))):
            return self._fail(3028)
        code = self._check()
        if code:
            return self._fail(code)
        return {"motion_id": self.st["motion_id"], "result": True, "code": 0}

    # ---- 1002 --------------------------------------------------------------------------
    def servo_frame(self, f):
        """The 1002 RPC publishes on motion_servo_cmd and returns at once; frames are handled in order."""
        now = time.monotonic()
        with self.wlk:
            s = self.servo
            s["frames"] += 1
            s["last_frame"] = f
            if s["_t"] is not None and now - s["_t"] < 1.0:
                gap = int((now - s["_t"]) * 1000)
                s["max_gap_ms"] = max(s["max_gap_ms"], gap)
                s["gaps_over_150ms"] += gap > 150
            s["_t"] = now
        self.servo_q.append(f)
        self.servo_ev.set()

    def _servo_worker(self):
        while not self.stop_ev.is_set():
            self.servo_ev.wait(0.1)
            self.servo_ev.clear()
            while True:
                try:
                    f = self.servo_q.popleft()
                except IndexError:
                    break
                self._servo(f)

    def _servo(self, f):                              # MD:67-84, MH:164-232
        mid = f["motion_id"]
        with self.hlk:
            if self.estop_latch and mid not in (0, 111):
                return self._servo_resp(False, 3040)
            if self.work == "result":
                return self._servo_resp(False, 3011)
            if mid not in self.map:
                return self._servo_resp(False, 3008)
            self.work = "servo"
        if f["cmd_type"] == 2:
            self._stop_servo()
            return self._servo_resp(True, 0)
        if not ((self.post_checked and mid != 309) or self._post_ok(mid)):   # AllowServoCmd, MH:891-896
            with self.wlk:
                self.need_check = False
            r = self._execute({"motion_id": 118 if mid == 309 else 111})
            if not r["result"]:
                with self.hlk:
                    self.work = "idle"
                self.pre_failed = True
                return self._servo_resp(False, r["code"])
            self.post_checked = True
        with self.wlk:
            self.pre_failed = False
            self.last_servo = f                       # never cleared (motion_handler.hpp:214)
            self._publish_new(self._build(mid, f, servo=True))
            self.ticked = self.need_check = True
        self._servo_resp(True, 0)

    def _servo_resp(self, result, code):              # the 1003 push; recorded only (reverse channel is down)
        s = self.st
        self.servo["last_response"] = {"motion_id": s["motion_id"], "order_process_bar": s["bar"],
                                       "status": s["switch"], "result": result,
                                       "code": code if not result else self._check()}
        if not result:
            self.emu.refuse(1002, "servo frame refused", code=code)

    def _stop_servo(self):                            # StopServoCmd + WalkStand, MH:251-300
        with self.wlk:
            self.need_check = False
            s = self.last_servo
            if s is not None and s["motion_id"] != 201 and not self.pre_failed:
                self._publish_new(self._build(112, {"value": s.get("value", 0),
                                                    "step_height": s.get("step_height")}, servo=False))
                self.servo["walk_stands"] += 1
            with self.hlk:
                self.work = "idle"
            self.post_checked = False

    def _watchdog(self):
        miss = 0
        while not self.stop_ev.wait(SERVO_TICK):
            with self.wlk:
                if not self.need_check:
                    miss = 0
                    continue
                miss = 0 if self.ticked else miss + 1
                self.ticked = False
                if miss >= SERVO_MISS:
                    miss = 0
                    self.servo["watchdog_stops"] += 1
                    self._stop_servo()

    # ---- inspection ----------------------------------------------------------------------
    def motion_id(self):
        return self.st["motion_id"]

    def state(self):
        s = self.st
        return {"motion_id": s["motion_id"], "mode": s["mode"], "gait_id": s["gait"],
                "switch_status": s["switch"], "bar": s["bar"], "busy": self.work != "idle",
                "work": self.work, "estop_latched": self.estop_latch}

    def servo_state(self):
        with self.wlk:
            return {k: v for k, v in self.servo.items() if not k.startswith("_")}
