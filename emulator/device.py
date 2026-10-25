"""Device services: 1001, 1012, 10001, 11001, 11002, battery, and the dangerous nameCodes
(grpc_fidelity.md 6, 7.1, 7.8, 7.9). Dangerous calls are recorded and answered; nothing reboots,
powers off, unlocks, calibrates or upgrades.
"""
from __future__ import annotations

from emulator.common import MISSING, get, jdump

FLAGS = ("is_sn", "is_version", "is_nick_name", "is_volume", "is_mic_state", "is_voice_control", "is_wifi",
         "is_bat_info", "is_motor_temper", "is_audio_state", "is_device_model", "is_stand", "is_lowpower_control")
LOW_BATTERY = 20                                         # Protected below this soc [INF]
OTA_REPLY = {5001: "normal", 5002: "1.3.0.0", 5003: "{}", 5004: "{}", 5005: "0", 5006: "{}"}   # [INF]


class Device:
    def __init__(self, emu):
        self.emu = emu
        self.reset()

    def reset(self):
        self.soc, self.charging, self.wireless = 80, False, False
        self.volume = 50
        self.version = ""                                # version_ cache: empty until the OTA reply lands
        self.skin = dict(enabled=False, mode=None, wave_cycle_time=None)
        self.provider_ip = "10.0.0.1"                    # connector provider (never dialled by the emulator)

    def protected(self):
        return self.soc < LOW_BATTERY

    # ---- 1001 ---------------------------------------------------------------------------
    def h1001(self, doc, nc, raw):                       # app.cpp:2677-2749, query_info.hpp:157-513
        dep = self.emu.dep("query_divice_info")
        if not dep.up(3):
            return                                       # 0 messages
        flags = {k: get(doc, k, bool, False) for k in FLAGS}
        if isinstance(doc, dict) and isinstance(doc.get("is_uid"), bool):
            flags["is_version"] = doc["is_uid"]          # Get("is_uid", is_version) (:2705)
        if not dep.ready(3):
            dep.block()                                  # future.get() with no bound
        mid = self.emu.mgr.motion_id()
        out = {}
        if flags["is_sn"]:
            out["sn"] = "EMU2241CN0000001"
        if flags["is_version"]:
            if not self.version:                         # GetVersion: unconditional sleep(5), result arrives later
                self.emu.sleep(5)
                self.version = "1.3.0.0-emulator"
                out["version"] = {}
            else:
                out["version"] = self.version
        if flags["is_nick_name"]:
            out["nick_name"] = {"current_name": "铁蛋", "default_name": "铁蛋"}
        if flags["is_volume"]:
            out["volume"] = self.volume
        if flags["is_mic_state"]:
            out["mic_state"] = True
        if flags["is_voice_control"]:
            out["voice_control"] = True
        if flags["is_wifi"]:
            out["wifi"] = {"name": "emulator", "ip": "127.0.0.1", "mac": "***********", "strength": 80}
        if flags["is_bat_info"]:
            out["bat_info"] = {"capacity": 4500, "power": self.soc, "voltage": 25200, "temperature": 30,
                               "is_charging": self.charging or self.wireless, "charge_over_current": False,
                               "discharge_over_current": False, "discharge_time": 120}
        if flags["is_motor_temper"]:
            out["motor_temper"] = {k: ["35.00"] * 4 for k in ("hip", "thigh", "crus")}
        if flags["is_audio_state"]:
            out["audio_state"] = False                   # uid_ is empty
        if flags["is_device_model"]:
            out["device_model"] = "MS2241CN"
        if flags["is_stand"]:
            out["stand"] = mid == 111
        if flags["is_lowpower_control"]:
            out["lowpower_control"] = False
        out["non-standing"] = mid in (0, 101, 102)       # always present (qi.hpp:505)
        yield self.emu.msg(nc, jdump(out))

    # ---- 1012 ---------------------------------------------------------------------------
    def h1012(self, doc, nc, raw):                       # app.cpp:3111-3145
        dep = self.emu.dep("audio_volume_set")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        vol = get(doc, "volume", "int32")
        if vol is MISSING:
            self.emu.client_bug("1012 volume missing or not an int32: uninitialised on the dog (0 here)")
            vol = 0
        if not dep.ready(3):
            dep.block()
        self.volume = vol & 0xFF                         # int32 -> uint8, truncating
        yield self.emu.msg(nc, jdump({"success": True}))

    # ---- 10001 --------------------------------------------------------------------------
    def h10001(self, doc, nc, raw):                      # app.cpp:2502-2548: the cache only
        # A booted dog: the -1 cache defaults (cyberdog_app.hpp:635-659) are replaced once the topics
        # publish; self-check 0 = passed, state 0 = active, 1 = low battery (Self/StateSwitchStatus.msg).
        nav = self.emu.nav
        yield self.emu.msg(nc, jdump({
            "motion_status": {"motion_id": self.emu.mgr.motion_id()},
            "task_status": {"task_status": nav.status, "task_sub_status": nav.sub},
            "self_check_status": {"code": 0, "description": ""},
            "state_switch_status": {"state": 1 if self.protected() else 0, "code": 0},
            "charging_status": {"wired_charging": self.charging, "wireless_charging": self.wireless}}))

    # ---- skin ---------------------------------------------------------------------------
    def h11001(self, doc, nc, raw):                      # app.cpp:3495-3532
        dep = self.emu.dep("enable_elec_skin")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        data = get(doc, "data", bool)
        if data is MISSING:
            yield self.emu.err(323, nc)
            return
        if not dep.ready(5):
            yield self.emu.err(321, nc)
            return
        self.skin = dict(enabled=data, mode=None, wave_cycle_time=None)   # false restores the default skin
        yield self.emu.msg(nc, jdump({"success": True, "message": ""}))

    def h11002(self, doc, nc, raw):                      # app.cpp:3534-3572, skin_manager.cpp:107-135
        dep = self.emu.dep("set_elec_skin")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        mode, wave = get(doc, "mode", "int32"), get(doc, "wave_cycle_time", "int32")
        if mode is MISSING or wave is MISSING:
            yield self.emu.err(323, nc)
            return
        if not dep.ready(5):
            yield self.emu.err(321, nc)
            return
        if not self.skin["enabled"]:
            self.emu.refuse(nc, "skin not enabled: send 11001 {\"data\":true} first", mode=mode)
            yield self.emu.msg(nc, jdump({"success": False}))
            return
        self.skin.update(mode=mode, wave_cycle_time=wave)
        yield self.emu.msg(nc, jdump({"success": True}))

    # ---- dangerous nameCodes (recorded by the server before these run) -------------------
    def h9030(self, doc, nc, raw):                       # unlock: the cloud never answers offline [INF]
        if not self.emu.dep("unlock_develop_access").up(3):
            return
        self.emu.sleep(10)
        yield self.emu.msg(nc, jdump({"unlock_result": 100}))

    def h9031(self, doc, nc, raw):                       # reboot: answered, not simulated
        dep = self.emu.dep("reboot_machine")
        if not dep.up(3):
            return
        if not dep.ready(5):
            dep.block()
        yield self.emu.msg(nc, jdump({"reboot_result": 0}))

    def h10006(self, doc, nc, raw):                      # power off: the reply is written before acting
        if not self.emu.dep("poweroff").up(3):
            yield self.emu.err(322, nc)
            return
        yield self.emu.msg(nc, jdump({"success": True}))

    def h12001(self, doc, nc, raw):                      # leg calibration
        dep = self.emu.dep("dog_leg_calibration")
        if not dep.up(3):
            yield self.emu.err(322, nc)
            return
        if get(doc, "data", bool) is MISSING:
            self.emu.client_bug("12001 data missing or not a bool: uninitialised on the dog")
        if not dep.ready(10):
            yield self.emu.err(321, nc)
            return
        yield self.emu.msg(nc, jdump({"success": True, "message": ""}))

    def h55001(self, doc, nc, raw):                      # test disconnect: provider_ip cleared and persisted
        self.provider_ip = ""
        yield self.emu.msg(nc, "")

    def hota(self, doc, nc, raw):                        # 5001-5006 (app.cpp:995-1280)
        dep = self.emu.dep("ota_grpc")
        if not dep.up(10):
            return
        if not dep.ready(10):
            dep.block()
        yield self.emu.msg(5004 if nc == 5005 else nc, OTA_REPLY[nc])

    def hota_action(self, doc, nc, raw):                 # 5010/5011 (app.cpp:1282-1409): accepted, then cut
        if not self.emu.dep("cyberdog_ota_action").up(3):
            yield self.emu.err(322, nc)
            return
        nav = self.emu.nav
        with nav.task_init_lock:
            epoch = nav.epoch
        yield self.emu.msg(nc, jdump({"accepted": 1}))
        while nav.epoch == epoch and not self.emu.stopping.wait(0.05):
            pass                                         # RemoveRequest ends the stream; no result is written
