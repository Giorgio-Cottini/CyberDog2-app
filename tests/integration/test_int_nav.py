"""Maps and navigation: 6003 / 6002 / 6004 / 6009 / 6010 and the 10001 task status (nav_map.md 2-5)."""
from __future__ import annotations

import math
import time

import pytest


def wait_for(fn, timeout=5.0, step=0.02):
    end = time.monotonic() + timeout
    while True:
        v = fn()
        if v:
            return v
        if time.monotonic() > end:
            raise AssertionError("condition not met in %.1f s" % timeout)
        time.sleep(step)


def localize(dog):
    t = dog.nav.relocalize()
    wait_for(lambda: t.accepted is not None and (t.accepted != 1 or t.feedback), 3)   # feedback lands after the accept
    assert t.accepted == 1 and t.feedback[0][0] == 3100
    st = wait_for(lambda: (lambda s: s if s.localized else None)(dog.nav.state()), 5)
    assert (st.status, st.name, st.stop_type) == (8, "laser localized", 0)


def test_get_map_and_labels(dog, ctl):
    m = dog.nav.get_map()
    assert (m.name, m.success, m.is_outdoor, m.width, m.height) == ("casa", 0, False, 80, 60)
    assert m.resolution == pytest.approx(0.05) and m.origin == (-2.0, -1.5) and m.yaw == 0.0
    assert len(m.data) == 80 * 60 and set(m.data) == {0, 100}
    assert [(n, round(x, 3), round(y, 3)) for n, x, y in m.labels] == [("cucina", 1.3, 0.7), ("divano", -1.1, -0.6)]

    # the complete list, integer coordinates included: the SDK sends them as doubles (contracts.md 1, rule 3)
    assert dog.nav.set_labels("casa", [("porta", 1, 2), ("cucina", 1.3, 0.7)]) is True
    assert [n for n, _, _ in dog.nav.get_map().labels] == ["porta", "cucina"]
    assert dog.nav.set_labels("casa", [{"labelName": "porta", "physicX": 1.0, "physicY": 2.0}],
                              only_delete=True) is True
    assert [n for n, _, _ in dog.nav.get_map().labels] == ["cucina"]
    assert dog.nav.set_labels("nessuna", [("x", 0.0, 0.0)]) is False      # unknown map: success 1


def test_mapping_then_save(dog, ctl):
    t = dog.nav.start_mapping()
    wait_for(lambda: t.accepted is not None and (t.accepted != 1 or t.feedback), 3)
    assert t.accepted == 1 and t.feedback[0][0] == 6                       # mapping started
    st = dog.nav.state()
    assert (st.status, st.task_type, st.stop_type, st.feedback) == (5, 5, 5, "mapping started")
    s = dog.nav.save_map("ufficio")
    assert s.wait(10) and s.error is None and s.result == 0
    assert dog.nav.state().status == 101
    assert ctl.state()["nav"]["maps"] == {"casa": ["cucina", "divano"], "ufficio": []}


def test_stop_mapping_without_saving(dog, ctl):
    t = dog.nav.start_mapping()
    wait_for(lambda: t.accepted == 1, 3)
    assert dog.nav.stop(5) == 0
    assert dog.nav.state().status == 101
    assert list(ctl.state()["nav"]["maps"]) == ["casa"]


def test_go_to_needs_localization(dog, ctl):
    t = dog.nav.go_to(1.0, 0.5)
    assert t.wait(3) and t.accepted == 2                                   # status 101, not 8: refused
    assert dog.nav.state().status == 101


def test_relocalize_go_to_dock(dog, ctl):
    localize(dog)
    goal = (1.0, 0.5)
    t = dog.nav.go_to(*goal)
    wait_for(lambda: t.accepted is not None and (t.accepted != 1 or t.feedback), 3)
    assert t.accepted == 1 and t.feedback[0][0] == 309
    task = ctl.state()["nav"]["task"]
    assert task["type"] == 1 and task["goal"] == list(goal)
    dists = []

    def arrived():
        s = ctl.state()["nav"]
        dists.append(math.hypot(goal[0] - s["pose"]["x"], goal[1] - s["pose"]["y"]))
        return s["task"] is None
    wait_for(arrived, 5, step=0.01)
    assert dists[0] == pytest.approx(math.hypot(*goal), abs=1e-6)          # it starts where the dog stands
    assert any(0.05 < d < dists[0] - 0.05 for d in dists)                  # seen on the way
    assert all(b <= a + 1e-9 for a, b in zip(dists, dists[1:]))            # never moves away from the goal
    assert dists[-1] < 1e-6                                                # and stays there
    st = dog.nav.state()
    assert st.status == 8 and st.localized and st.sub_status == 0          # arrived: still localized

    t2 = dog.nav.dock()                                                    # needs 101/6/16
    assert t2.wait(3) and t2.accepted == 2
    assert dog.nav.stop(st.stop_type) == 0                                 # ResetNav (6009 type 0)
    assert dog.nav.state().status == 101
    d = dog.nav.dock()
    wait_for(lambda: d.accepted is not None and (d.accepted != 1 or d.feedback), 3)
    assert d.accepted == 1 and d.feedback[0][0] == 500
    assert dog.nav.state().name == "auto docking" or dog.status().wireless_charging
    wait_for(lambda: dog.status().wireless_charging, 3)
    assert dog.nav.state().status == 101


def test_go_to_stopped_before_the_goal_is_sent(dog, ctl):
    # before feedback 300 a type 1 stop is a no-op and the goal still goes out (nav_map.md §5 d)
    localize(dog)
    x0 = ctl.state()["nav"]["pose"]["x"]
    t = dog.nav.go_to(3.0, 0.0)
    wait_for(lambda: t.accepted == 1, 3)
    st = dog.nav.state()
    assert st.status == 1 and st.sub_status not in (300, 307) and st.stop_type == 0, st
    assert dog.nav.stop(st.stop_type) == 0
    assert dog.nav.state().status == 101
    time.sleep(1.2)                                                        # the goal would have gone out at 0.8 s
    s = ctl.state()["nav"]
    assert (s["status"], s["task"], s["pose"]["x"]) == (101, None, x0)


def test_go_to_stop_midway_and_attach(dog, ctl):
    localize(dog)
    t = dog.nav.go_to(3.0, 0.0)
    wait_for(lambda: t.accepted == 1, 3)
    wait_for(lambda: 0.3 < ctl.state()["nav"]["pose"]["x"] < 2.5, 5, step=0.01)
    a = dog.nav.attach(1)                                                  # 6010: re-attach, latest feedback replayed
    wait_for(lambda: a.accepted is not None and (a.accepted != 1 or a.feedback), 3)
    assert a.accepted == 1 and a.feedback and a.feedback[0][0] in (300, 307)   # goal sent / navigating
    assert dog.nav.stop(1) == 0
    x = ctl.state()["nav"]["pose"]["x"]
    assert 0.3 < x < 3.0
    time.sleep(0.3)
    assert ctl.state()["nav"]["pose"]["x"] == x                            # stopped where it was
    st = dog.nav.state()
    assert st.status == 8 and ctl.state()["nav"]["task"] is None
