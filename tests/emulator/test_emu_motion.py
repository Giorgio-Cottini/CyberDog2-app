"""1004 / 1002 through motion_manager and the fake controller (sim_bridge.md 2, motion.md 2.8)."""
from __future__ import annotations

import threading
import time

from emulator.controllers.fake import FakeController


def _bg(dog, nc, params, out):
    t = threading.Thread(target=lambda: out.append(dog.j(nc, params)), daemon=True)
    t.start()
    return t


def test_1004_happy_path_stand_then_lie(dog):
    r = dog.j(1004, {"motion_id": 111})
    assert r == {"motion_id": 111, "result": True, "code": 0}
    s = dog.state()
    assert s["motion"]["motion_id"] == 111 and s["ground_truth"]["z"] > 0.2
    assert dog.j(10001)["motion_status"]["motion_id"] == 111
    assert dog.j(1004, {"motion_id": 101}) == {"motion_id": 101, "result": True, "code": 0}


def test_unknown_motion_id_is_3008(dog):
    r = dog.j(1004, {"motion_id": 999})
    assert r["result"] is False and r["code"] == 3008


def test_duration_rule_is_3010(dog):
    assert dog.j(1004, {"motion_id": 111, "duration": 1000})["code"] == 3010   # min_exec_time > 0 needs 0
    assert dog.j(1004, {"motion_id": 212})["code"] == 3010                     # min_exec_time < 0 needs > 0


def test_estop_latch_is_3040_until_recovery_stand(dog):
    dog.j(1004, {"motion_id": 0})
    assert dog.state()["motion"]["estop_latched"] is True
    assert dog.j(1004, {"motion_id": 101})["code"] == 3040
    assert dog.j(1004, {"motion_id": 111})["result"] is True
    assert dog.state()["motion"]["estop_latched"] is False


def test_busy_is_3011(dog):
    out = []
    t = _bg(dog, 1004, {"motion_id": 141}, out)  # auto 111 (2.0 s) then 141 (8.6 s), scaled by 0.05
    dog.wait(lambda s: s["motion"]["busy"], timeout=3)
    r = dog.j(1004, {"motion_id": 101})
    t.join(5)
    assert r == {"motion_id": 0, "result": False, "code": 3011}
    assert out[0]["result"] is True


def test_ban_trans_is_3022(dog):
    dog.j(1004, {"motion_id": 111})
    r = dog.j(1004, {"motion_id": 120})       # mode 17 passes CheckPostMotion but not the RecoveryStand table
    assert r["result"] is False and r["code"] == 3022


def test_auto_111_before_a_standing_motion(dog):
    r = dog.j(1004, {"motion_id": 141})       # from lying: 62 is not a post motion of 101
    assert r == {"motion_id": 141, "result": True, "code": 0}


def test_20s_cap_gives_321_while_the_motion_goes_on(dog):
    dog.j(1004, {"motion_id": 111})
    t0 = time.monotonic()
    msgs = dog.call(1004, {"motion_id": 151})  # 27 s dance: longer than the 20 s cap
    assert msgs == [(1004, '{"error_code":321}')]
    assert time.monotonic() - t0 < 2.0
    assert dog.j(1004, {"motion_id": 101})["code"] == 3011   # still busy: 0.35 s of the dance remain
    s = dog.wait(lambda s: not s["motion"]["busy"], timeout=5)
    assert s["motion"]["motion_id"] in (151, 212)   # finished, then the PoseCtrl filler


def test_estop_concurrent_with_an_inflight_1004(dog):
    dog.j(1004, {"motion_id": 111})
    out = []
    t = _bg(dog, 1004, {"motion_id": 151}, out)
    time.sleep(0.3)
    t0 = time.monotonic()
    r = dog.j(1004, {"motion_id": 0})
    assert time.monotonic() - t0 < 1.0
    assert r["motion_id"] == 0
    t.join(5)
    assert out, "the in-flight 1004 must return"
    s = dog.state()
    assert s["motion"]["motion_id"] == 0 and s["motion"]["estop_latched"] is True


def test_protected_low_battery_is_3013(dog):
    dog.post("/battery", {"soc": 15})
    assert dog.j(10001)["state_switch_status"]["state"] == 1
    assert dog.j(1004, {"motion_id": 141}) == {"motion_id": 0, "result": False, "code": 3013}
    assert dog.j(1004, {"motion_id": 111})["result"] is True


def test_json_typing_traps_are_client_bugs(dog):
    r = dog.j(1004, {"motion_id": 111.0})      # 111.0 is not an int32: motion_id stays 0 = ESTOP
    assert r["motion_id"] == 0
    s = dog.state()
    assert s["motion"]["estop_latched"] is True
    assert any("not an int32" in b["bug"] for b in s["client_bugs"])
    dog.j(1004, {"motion_id": 111})
    assert dog.call(1002, {"motion_id": 303, "cmd_type": 1, "vel_des": [0.3, 0, 0]}) == [(1002, "")]
    s = dog.wait(lambda s: s["servo"]["frames"] == 1)
    assert s["servo"]["last_frame"]["vel_des"] == [0.3]
    assert any("int literal stops" in b["bug"] for b in s["client_bugs"])


def test_untimed_202_never_completes_and_keeps_tilting(dog):
    # fsm_state_balancestand.cpp:338-396: duration 0 -> 1000-tick periods, rpy_des added each period
    dog.j(1004, {"motion_id": 111})
    r = dog.j(1004, {"motion_id": 202, "rpy_des": [0.0, 0.05, 0.0], "duration": 0})
    assert r["result"] is False and r["code"] == 3028, r
    s = dog.state()
    assert any("202 with duration 0" in b["bug"] for b in s["client_bugs"])
    p1 = s["ground_truth"]["pitch"]
    dog.wait(lambda s: s["ground_truth"]["pitch"] > p1 + 0.08, timeout=2.0)     # more than one rpy_des
    dog.j(1004, {"motion_id": 111})
    dog.post("/reset")


def test_timed_202_completes_once(dog):
    dog.j(1004, {"motion_id": 111})
    r = dog.j(1004, {"motion_id": 202, "rpy_des": [0.0, 0.05, 0.0], "duration": 1000})
    assert r == {"motion_id": 202, "result": True, "code": 0}, r
    assert not any("202" in b["bug"] for b in dog.state()["client_bugs"])


def test_value_bit_4_is_masked(dog):
    dog.j(1004, {"motion_id": 111, "value": 4})
    assert any("0x04" in b["bug"] for b in dog.state()["client_bugs"])


def test_401_is_refused_by_the_sim_guard(dog):
    dog.j(1004, {"motion_id": 111})
    assert dog.j(1004, {"motion_id": 401})["code"] == 3031


def test_servo_stream_then_watchdog_walkstand(dog):
    dog.j(1004, {"motion_id": 111})
    x0 = dog.state()["ground_truth"]["x"]
    for _ in range(12):                        # 50 ms frames, as the user's app sends them
        assert dog.call(1002, {"motion_id": 303, "cmd_type": 1, "vel_des": [0.3, 0.0, 0.0],
                               "step_height": [0.05, 0.05]}) == [(1002, "")]
        time.sleep(0.05)
    s = dog.wait(lambda s: s["servo"]["watchdog_stops"] == 1, timeout=3)   # 4 missed ticks of 50 ms
    assert s["servo"]["frames"] == 12 and s["servo"]["walk_stands"] == 1
    assert s["ground_truth"]["x"] > x0
    s = dog.wait(lambda s: s["motion"]["motion_id"] == 112, timeout=3)
    assert s["motion"]["busy"] is False


def test_servo_end_frame_sends_walkstand(dog):
    dog.j(1004, {"motion_id": 111})
    dog.call(1002, {"motion_id": 303, "cmd_type": 1, "vel_des": [0.2, 0.0, 0.0]})
    dog.call(1002, {"motion_id": 303, "cmd_type": 2})
    s = dog.wait(lambda s: s["servo"]["walk_stands"] == 1, timeout=3)
    assert s["servo"]["watchdog_stops"] == 0


def test_status_10001_fields(dog):
    r = dog.j(10001)
    assert r == {"motion_status": {"motion_id": 101},
                 "task_status": {"task_status": 101, "task_sub_status": 0},
                 "self_check_status": {"code": 0, "description": ""},
                 "state_switch_status": {"state": 0, "code": 0},
                 "charging_status": {"wired_charging": False, "wireless_charging": False}}
    dog.post("/battery", {"charging": True})
    assert dog.j(10001)["charging_status"]["wired_charging"] is True


def test_1001_flags_and_uid_bug(dog):
    r = dog.j(1001, {"is_stand": True, "is_bat_info": True})
    assert list(r) == ["bat_info", "stand", "non-standing"]
    assert r["bat_info"]["power"] == 80 and r["stand"] is False and r["non-standing"] is True
    assert dog.j(1001, {"is_uid": True})["version"] == {}               # is_uid writes is_version; first call
    assert dog.j(1001, {"is_version": True, "is_uid": False}) == {"non-standing": True}


def test_fake_controller_drops_to_pure_damper_after_half_a_second():
    c = FakeController(time_scale=0.05)
    seen = []
    c.on_response(lambda r: seen.append((time.monotonic(), r["mode"])))
    try:
        cmd = {"mode": 12, "gait_id": 0, "contact": 15, "life_count": 1, "value": 0, "duration": 0,
               "vel_des": [0.0] * 3, "rpy_des": [0.0] * 3, "pos_des": [0.0] * 3, "acc_des": [0.0] * 6,
               "ctrl_point": [0.0] * 3, "foot_pose": [0.0] * 6, "step_height": [0.0] * 2}
        t0 = time.monotonic()
        c.publish(cmd)
        time.sleep(0.3)
        assert seen[-1][1] == 12
        time.sleep(0.5)
        first7 = next(t for t, m in seen if m == 7 and t > t0)
        assert 0.45 < first7 - t0 < 0.7
    finally:
        c.close()
