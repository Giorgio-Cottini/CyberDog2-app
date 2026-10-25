"""Connect, status and the 1004 result motions: the app's stand/sit/trick flows, ESTOP and recovery."""
from __future__ import annotations

import socket
import threading
import time

import pytest

from cyberdog_sdk import CyberDog, DogUnreachable, ForbiddenCommand, RiskyMotion
from cyberdog_sdk.client import SIT_IDS

KEYMAP_TRICKS = (141, 123, 146, 144, 145, 174, 173)      # keys 2-8 (contracts.md 6); 140 (key 9) below


def test_connect_status_and_device_info(dog, ctl):
    s = dog.status()
    assert (s.motion_id, s.task_status, s.task_sub_status, s.state, s.self_check_code) == (101, 101, 0, 0, 0)
    assert not s.wired_charging and not s.wireless_charging
    assert dog.nav.state().name == "idle"
    info = dog.device_info()
    assert info["bat_info"]["power"] == 80 and info["stand"] is False and "wifi" in info
    assert dog.set_volume(35) is True
    assert dog.device_info("volume")["volume"] == 35
    ctl.post("/battery", {"soc": 42, "charging": True})
    s = dog.status()
    assert s.wired_charging and dog.device_info("bat_info")["bat_info"]["power"] == 42


def test_connect_to_a_closed_port_is_unreachable():
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()                                    # nothing listens there now
    d = CyberDog(host="127.0.0.1", port=port)
    t0 = time.monotonic()
    with pytest.raises(DogUnreachable):
        d.connect(timeout=0.5)
    assert time.monotonic() - t0 < 2.0
    d.close()


def test_stand_sit_and_the_after_sit_rule(dog, ctl):
    r = dog.stand()
    assert (r.motion_id, r.result, r.code, r.still_running) == (111, True, 0, False)
    s = ctl.state()
    assert s["motion"]["motion_id"] == 111 and s["ground_truth"]["z"] > 0.2
    assert dog.device_info("stand")["stand"] is True

    r = dog.sit()
    assert r.result and r.code == 0
    # after a mode-62 trick the controller runs the 21/3 filler, reported as 214 (sim_bridge.md 1.2, motion.md 4)
    s = ctl.wait(lambda s: s["motion"]["motion_id"] == 214 and not s["motion"]["busy"])
    assert dog.status().motion_id in SIT_IDS

    r = dog.trick(141)                              # sitting: 111 first (motion.md 6, VP SDK motion.cpp:565-583)
    assert r.result and r.code == 0
    assert dog.rec.motions() == [111, 143, 111, 141]

    # 214 again, but after 141 (not a sit): the dog stands, so the sit swing gets its 143 first
    ctl.wait(lambda s: s["motion"]["motion_id"] == 214 and not s["motion"]["busy"])
    assert dog.status().motion_id == 214 and dog.last_motion == 141
    r = dog.trick(148)
    assert r.result
    assert dog.rec.motions()[-2:] == [143, 148]

    r = dog.trick(149)                              # 214 after 148: still sitting, no second 143
    assert r.result
    assert dog.rec.motions()[-3:] == [143, 148, 149]


def test_sit_swing_from_standing_sends_143_first(standing, ctl):
    r = standing.trick(149)
    assert r.result and r.code == 0
    assert standing.rec.motions() == [111, 143, 149]


def test_keymap_tricks(standing, ctl):
    for mid in KEYMAP_TRICKS:
        before = standing.status().motion_id
        n = len(standing.rec.motions())
        r = standing.trick(mid)
        assert (r.motion_id, r.result, r.code, r.still_running) == (mid, True, 0, False), (mid, r)
        sent = standing.rec.motions()[n:]
        if before in SIT_IDS:
            assert sent == [111, mid], (mid, before)
        else:   # the 214 filler after a mode-62 trick can arrive between this read and trick()'s own
            assert sent in ([mid], [111, mid]), (mid, before, sent)
        ctl.wait(lambda s: not s["motion"]["busy"])
    assert ctl.state()["calls"]["1004"] == len(standing.rec.motions())


def test_dance_140_returns_still_running(standing, ctl):
    t0 = time.monotonic()
    r = standing.trick(140)                         # 60 s on the dog: the bridge's 20 s cap answers 321
    elapsed = time.monotonic() - t0
    assert (r.motion_id, r.result, r.code, r.still_running) == (140, False, 321, True)
    assert "may still be running" in r.message
    assert elapsed < 20 * 0.05 + 1.0
    s = ctl.state()
    assert s["motion"]["busy"] and s["motion"]["motion_id"] == 140
    busy = standing.motion(101)                     # the dance still runs: 101 interrupts it (MH:637-644)
    assert busy.motion_id in (101, 140)
    ctl.wait(lambda s: not s["motion"]["busy"], timeout=5)


def test_lie_down(standing, ctl):
    r = standing.lie_down()
    assert r.result and r.code == 0
    s = ctl.wait(lambda s: s["ground_truth"]["z"] < 0.08)
    assert s["motion"]["motion_id"] == 101
    assert standing.status().motion_id == 101


def test_busy_is_3011(standing, ctl):
    out = []
    th = threading.Thread(target=lambda: out.append(standing.trick(174)), daemon=True)
    th.start()
    ctl.wait(lambda s: s["motion"]["busy"] and s["motion"]["motion_id"] == 174, timeout=3)
    r = standing.motion(146)
    th.join(5)
    assert (r.result, r.code) == (False, 3011) and "busy" in r.message
    assert out and out[0].result


def test_estop_during_a_motion_then_recovery(standing, ctl):
    out = []
    th = threading.Thread(target=lambda: out.append(standing.trick(140)), daemon=True)
    th.start()
    ctl.wait(lambda s: s["motion"]["motion_id"] == 140, timeout=3)
    t0 = time.monotonic()
    r = standing.estop()
    assert time.monotonic() - t0 < 2.5              # three ESTOPs, the last ESTOP_LATE_S after the first
    assert (r.motion_id, r.result, r.code) == (0, True, 0)
    th.join(5)
    assert out, "the in-flight 1004 must return"
    s = ctl.state()
    assert s["motion"]["estop_latched"] and s["motion"]["motion_id"] == 0
    assert standing.rec.motions().count(0) == 3     # always three times; the repeats are harmless

    r = standing.motion(101)
    assert (r.result, r.code) == (False, 3040)
    r = standing.stand()
    assert r.result and r.code == 0
    s = ctl.wait(lambda s: s["motion"]["motion_id"] == 111 and not s["motion"]["busy"])
    assert s["motion"]["estop_latched"] is False


def test_motion_guards_and_codes(standing, ctl):
    before = ctl.calls().get("1004", 0)
    with pytest.raises(ValueError):
        standing.motion(212, rpy_des=[0.0, 0.1, 0.0])        # min_exec_time -1: needs duration > 0 (3010)
    with pytest.raises(ValueError):
        standing.motion(111, duration=1000)                  # min_exec_time > 0: needs 0
    with pytest.raises(ForbiddenCommand):
        standing.motion(303)                                 # a gait: Driver only
    with pytest.raises(RiskyMotion):
        standing.motion(130)                                 # R3 jump
    with pytest.raises(RiskyMotion):
        standing.trick(121)                                  # R4 backflip
    with pytest.raises(RiskyMotion):
        standing.motion(999)                                 # not in the catalog
    assert ctl.calls().get("1004", 0) == before

    r = standing.motion(999, allow_risky=True)               # reaches the dog: unknown id
    assert (r.result, r.code) == (False, 3008)
    r = standing.motion(211, duration=1000, pos_des=[0.0, 0.0, 0.2])
    assert r.result and r.code == 0
    s = ctl.state()
    assert abs(s["ground_truth"]["z"] - 0.1925) < 0.01      # 21/5 settles 7.5 mm below the target (sim_baseline)


def test_low_battery_is_3013(standing, ctl):
    ctl.post("/battery", {"soc": 15})
    assert standing.status().state == 1
    r = standing.trick(146)
    assert (r.result, r.code) == (False, 3013) and "Protected" in r.message
    assert standing.lie_down().result                        # 101 is allowed in the Protected state
