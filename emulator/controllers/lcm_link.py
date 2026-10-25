"""LcmController: the Controller of contracts.md 4.1 over real LCM to cyberdog_locomotion in Gazebo (part C).

Runs inside the CyberDogSim WSL distro only (python3-lcm, Python 3.8). Channels and URLs follow
sim_bridge.md 1.1; the frame layout is robot_control_cmd_lcmt.lcm (sim_bridge.md 1.2), the feedback
robot_control_responce_lcmt.lcm (1.3) and the ground truth simulator_lcmt.lcm (section 3).

Never publishes motor_ctrl: one such message latches MotorCtrl_mode_flag_ in the controller for good
(command_interface.cpp:45,84-88,153), after which every robot_control_cmd is ignored. The only channel
this class can write is robot_control_cmd.
"""
from __future__ import annotations

import math
import os
import random
import select
import struct
import sys
import threading
import time

TYPES_DIR = os.environ.get("CYBERDOG_LCM_TYPES", "/root/lcm_types_py")   # `lcm-gen -p` output in the VM
if TYPES_DIR not in sys.path:
    sys.path.insert(0, TYPES_DIR)
try:
    import lcm
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
    from robot_control_response_lcmt import robot_control_response_lcmt
    from simulator_lcmt import simulator_lcmt
except ImportError as e:                          # Windows, or a VM without python3-lcm / generated types
    raise ImportError("LcmController needs python3-lcm and the lcm-gen types in %s (CyberDogSim WSL): %s"
                      % (TYPES_DIR, e)) from e

URL_CMD = "udpm://239.255.76.67:7671?ttl=255"     # robot_control_cmd (motion_macros.hpp:87; simulation_bridge.hpp:29-42)
URL_RESP = "udpm://239.255.76.67:7670?ttl=255"    # robot_control_response (motion_macros.hpp:88; robot_runner.cpp:22)
URL_SIM = "udpm://239.255.76.67:7667?ttl=0"       # simulator_state, LCM default URL (lcmhandler.cpp:19-26) [inferred]
CH_CMD, CH_RESP, CH_SIM = "robot_control_cmd", "robot_control_response", "simulator_state"
SIM_DT = 0.002                                    # one simulator_state per 2 Gazebo steps of 1 ms (legged_plugin.cpp:184-219)
ARRAYS = (("vel_des", 3), ("rpy_des", 3), ("pos_des", 3), ("acc_des", 6), ("ctrl_point", 3),
          ("foot_pose", 6), ("step_height", 2))
VALUE_FACTORY = 0x04                              # value bit2: factory speed calibration + remount (command_interface.cpp:1058-1064)
RTF_WINDOW = 2.0                                  # wall seconds of simulator_state used for rtf()


def i8(n: int) -> int:
    return ((int(n) + 128) % 256) - 128           # int8_t life_count wraps (motion_action.hpp:193)


def _int(cmd, key, lo, hi):
    v = int(cmd.get(key, 0))
    if not lo <= v <= hi:
        raise ValueError("robot_control_cmd.%s = %d does not fit [%d, %d]" % (key, v, lo, hi))
    return v


def quat_to_rpy(w, x, y, z):
    """ZYX (yaw-pitch-roll) angles of a world-from-body quaternion. simulator_lcmt.rpy comes from Eigen
    eulerAngles(2,1,0), whose yaw is folded into [0, pi] (legged_plugin.cpp:330-333), so it is not used."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class LcmController:
    """Controller interface (contracts.md 4.1): publish, on_response, ground_truth, close.

    life_offset is added (mod 256) to every life_count the manager hands over. The controller keeps
    its last processed life_count across emulator restarts and ignores a first frame that repeats it
    (command_interface.cpp:11,983); a random offset per link makes that a 1/256 event instead of the
    certainty of every restart starting at 0. Equal life_counts stay equal, so heartbeat duplicates
    are still duplicates.
    """

    def __init__(self, life_offset: int | None = None):
        self.life_offset = random.randrange(256) if life_offset is None else int(life_offset)
        self.lc_cmd = lcm.LCM(URL_CMD)
        self.lc_resp = lcm.LCM(URL_RESP)
        self.lc_sim = lcm.LCM(URL_SIM)
        self.lc_resp.subscribe(CH_RESP, self._on_resp)
        self.lc_sim.subscribe(CH_SIM, self._on_sim)
        self.cbs = []
        self.plk = threading.Lock()               # publish
        self.slk = threading.Lock()               # simulator_state snapshot
        self.sim_raw, self.sim_n, self.sim_dec = None, 0, (-1, None)
        self.sim_marks = []                       # (wall, sim_n) for rtf()
        self.resp = None                          # last decoded response dict
        self.n_resp = self.n_pub = 0
        self.stop_ev = threading.Event()
        self.rx = threading.Thread(target=self._rx_loop, name="lcm-link-rx", daemon=True)
        self.rx.start()

    # ---- Controller interface (contracts.md 4.1) ---------------------------------------
    def publish(self, cmd: dict) -> None:
        """Encode one robot_control_cmd_lcmt and send it. The same dict published twice is a heartbeat
        duplicate (same life_count), which only refreshes the controller's 0.5 s timeout."""
        c = robot_control_cmd_lcmt()
        c.mode = _int(cmd, "mode", -128, 127)
        c.gait_id = _int(cmd, "gait_id", -128, 127)
        c.contact = _int(cmd, "contact", -128, 127)
        c.life_count = i8(int(cmd.get("life_count", 0)) + self.life_offset)
        for f, n in ARRAYS:                       # exact sizes; a wrong length is zero-filled like GET_VALUE
            v = cmd.get(f) or []
            setattr(c, f, [float(x) for x in v] if len(v) == n else [0.0] * n)
        c.value = _int(cmd, "value", -2 ** 31, 2 ** 31 - 1) & ~VALUE_FACTORY   # never the factory remount bit
        c.duration = _int(cmd, "duration", -2 ** 31, 2 ** 31 - 1)
        data = c.encode()
        with self.plk:
            if self.stop_ev.is_set():             # a heartbeat tick racing close()
                return
            self.lc_cmd.publish(CH_CMD, data)
            self.n_pub += 1

    def on_response(self, cb) -> None:
        self.cbs.append(cb)

    def ground_truth(self) -> dict:
        """Gazebo base_link state from simulator_state: p (world), attitude from quat [w,x,y,z],
        v and omega in the world frame (legged_plugin.cpp:345-351). t is simulated seconds since this
        link opened, counted in simulator_state frames (the plugin leaves `time` empty). All values are
        None until the first simulator_state arrives."""
        s, n = self._sim()
        if s is None:
            return dict.fromkeys(("x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "wz", "t"))
        roll, pitch, yaw = quat_to_rpy(*s.quat)
        return {"x": s.p[0], "y": s.p[1], "z": s.p[2], "roll": roll, "pitch": pitch, "yaw": yaw,
                "vx": s.v[0], "vy": s.v[1], "wz": s.omega[2], "t": round(n * SIM_DT, 4)}

    def close(self) -> None:
        with self.plk:
            self.stop_ev.set()
        if self.rx.is_alive() and threading.current_thread() is not self.rx:
            self.rx.join(1.0)
        self.cbs = []

    # ---- extras for tests and the control endpoint (not part of 4.1) ---------------------
    def last_response(self):
        return self.resp

    def sim_time(self) -> float:
        """Simulated seconds since this link opened (simulator_state frames x 2 ms)."""
        return self.sim_n * SIM_DT

    def rtf(self):
        """Real-time factor over the last RTF_WINDOW wall seconds, or None before enough frames."""
        with self.slk:
            m = list(self.sim_marks)
        if len(m) < 2 or m[-1][0] - m[0][0] < 0.5:
            return None
        return (m[-1][1] - m[0][1]) * SIM_DT / (m[-1][0] - m[0][0])

    def stats(self) -> dict:
        return {"published": self.n_pub, "responses": self.n_resp, "sim_frames": self.sim_n,
                "rtf": self.rtf(), "life_offset": self.life_offset}

    # ---- receive side ------------------------------------------------------------------
    def _rx_loop(self):
        lcs = {self.lc_resp.fileno(): self.lc_resp, self.lc_sim.fileno(): self.lc_sim}
        while not self.stop_ev.is_set():
            try:
                r, _, _ = select.select(list(lcs), [], [], 0.1)
            except (OSError, ValueError):
                break
            for fd in r:
                lcs[fd].handle()

    def _on_resp(self, channel, data):
        try:
            m = robot_control_response_lcmt.decode(data)
        except (ValueError, struct.error):
            return
        r = {"mode": m.mode, "gait_id": m.gait_id, "contact": m.contact,
             "order_process_bar": m.order_process_bar, "switch_status": m.switch_status,
             "ori_error": m.ori_error, "footpos_error": m.footpos_error, "motor_error": list(m.motor_error)}
        self.resp = r
        self.n_resp += 1
        for cb in list(self.cbs):
            cb(r)

    def _on_sim(self, channel, data):
        now = time.monotonic()                    # about 500 Hz x RTF: keep the bytes, decode on demand
        with self.slk:
            self.sim_raw = data
            self.sim_n += 1
            if not self.sim_marks or now - self.sim_marks[-1][0] >= 0.1:
                self.sim_marks.append((now, self.sim_n))
                while len(self.sim_marks) > 2 and now - self.sim_marks[0][0] > RTF_WINDOW:
                    self.sim_marks.pop(0)

    def _sim(self):
        with self.slk:
            raw, n = self.sim_raw, self.sim_n
            if raw is None:
                return None, n
            if self.sim_dec[0] == n:
                return self.sim_dec[1], n
        try:
            s = simulator_lcmt.decode(raw)
        except (ValueError, struct.error):
            return None, n
        with self.slk:
            self.sim_dec = (n, s)
        return s, n
