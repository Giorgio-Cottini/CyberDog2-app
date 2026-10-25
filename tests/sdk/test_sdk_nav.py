"""Maps and navigation: 6002 guards, 6004 bodies, NavTask streams, 6009/6010, NavState decoding."""
from __future__ import annotations

import math

import pytest

from cyberdog_sdk import DogError, ForbiddenCommand
from cyberdog_sdk.nav import NavState, decode_state


# ---- 6002 labels ---------------------------------------------------------------------------
def test_set_labels_floats_and_shape(dog, fake):
    fake.replies[6002] = [{"success": 0}]
    ok = dog.nav.set_labels("casa", [("cucina", 1, -2), {"labelName": "porta", "physicX": 3.5, "physicY": 0}])
    assert ok is True
    s = fake.of(6002)[0]
    assert s.raw == ('{"mapName":"casa","is_outdoor":false,"only_delete":false,"locationLabelInfo":['
                     '{"labelName":"cucina","physicX":1.0,"physicY":-2.0},'
                     '{"labelName":"porta","physicX":3.5,"physicY":0.0}]}')
    assert s.timeout >= 15.0


def test_set_labels_failure_and_error(dog, fake):
    fake.replies[6002] = [{"success": 1}]
    assert dog.nav.set_labels("casa", [("a", 0.0, 0.0)]) is False
    fake.replies[6002] = [{"error_code": 321}]
    with pytest.raises(DogError):
        dog.nav.set_labels("casa", [("a", 0.0, 0.0)])


def test_delete_all_guard(dog, fake):
    with pytest.raises(ForbiddenCommand):
        dog.nav.set_labels("casa", [], only_delete=True)
    assert fake.sent == []
    fake.replies[6002] = [{"success": 0}]
    assert dog.nav.set_labels("casa", [], only_delete=True, confirm_delete_all=True)
    assert fake.of(6002)[0].params == {"mapName": "casa", "is_outdoor": False, "only_delete": True,
                                       "locationLabelInfo": []}


def test_delete_one_label_is_not_delete_all(dog, fake):
    fake.replies[6002] = [{"success": 0}]
    dog.nav.set_labels("casa", [("porta", 0, 0)], only_delete=True)
    assert fake.of(6002)[0].raw.endswith('"only_delete":true,"locationLabelInfo":[{"labelName":"porta","physicX":0.0,"physicY":0.0}]}')


@pytest.mark.parametrize("label", [("", 1.0, 1.0), (None, 1.0, 1.0), ("a", "1", 0.0), ("a", float("inf"), 0.0),
                                   ("a", True, 0.0)])
def test_invalid_labels(dog, fake, label):
    with pytest.raises(ValueError):
        dog.nav.set_labels("casa", [label])
    with pytest.raises(ValueError):
        dog.nav.set_labels("casa", [label], only_delete=True)    # an invalid list is not a delete-all
    assert fake.sent == []


# ---- 6003 map ------------------------------------------------------------------------------
def test_get_map(dog, fake):
    fake.replies[6003] = [{"mapName": "casa", "success": 0, "is_outdoor": False,
                           "locationLabelInfo": [{"labelName": "porta", "physicX": 0.10000000149011612,
                                                  "physicY": 2.0}],
                           "map": {"resolution": 0.05, "width": 2, "height": 2, "px": -1.0, "py": -2.0,
                                   "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": math.sin(0.25), "qw": math.cos(0.25),
                                   "data": [0, 100, -1, 0]}}]
    m = dog.nav.get_map()
    assert (m.name, m.success, m.is_outdoor, m.resolution, m.width, m.height) == ("casa", 0, False, 0.05, 2, 2)
    assert m.origin == (-1.0, -2.0) and m.yaw == pytest.approx(0.5) and m.data == [0, 100, -1, 0]
    assert m.labels == [("porta", 0.10000000149011612, 2.0)]
    s = fake.of(6003)[0]
    assert s.raw == '{"mapName":""}' and s.timeout >= 15.0


def test_get_map_missing_and_timeout(dog, fake):
    fake.replies[6003] = [{"mapName": "", "success": 1, "is_outdoor": False, "locationLabelInfo": [],
                           "map": {"resolution": 0.0, "width": 0, "height": 0, "data": []}}]
    m = dog.nav.get_map("casa")
    assert m.success == 1 and m.data == [] and m.width == 0
    fake.replies[6003] = [{"error_code": 321}, ""]
    with pytest.raises(DogError) as e:
        dog.nav.get_map()
    assert e.value.code == 321


# ---- 6004 tasks ----------------------------------------------------------------------------
@pytest.mark.parametrize("start,raw", [
    (lambda n: n.start_mapping(), '{"type":5,"outdoor":false,"keep_distance":0.0}'),
    (lambda n: n.relocalize(), '{"type":7,"outdoor":false,"keep_distance":0.0}'),
    (lambda n: n.dock(), '{"type":9,"outdoor":false,"keep_distance":0.0}'),
    (lambda n: n.go_to(1, -2), '{"type":1,"outdoor":false,"keep_distance":0.0,"goalX":1.0,"goalY":-2.0,"theta":0.0}'),
])
def test_task_bodies(dog, fake, start, raw):
    task = start(dog.nav)
    assert task.wait(2.0)
    s = fake.of(6004)[0]
    assert s.raw == raw and s.timeout >= 7300.0


def test_nav_task_collects_stream(dog, fake):
    fake.replies[6004] = [{"accepted": 1}, {"feedback_code": 3100, "feedback_msg": "check"},
                          {"feedback_code": 500, "feedback_msg": "stale dock"},
                          {"feedback_code": 3108, "feedback_msg": "ok"}]
    task = dog.nav.relocalize()
    assert task.wait(2.0)
    assert task.accepted == 1 and task.feedback == [(3100, "check"), (3108, "ok")]
    assert task.last_feedback == 3108 and task.result is None and task.error is None


def test_nav_task_keeps_500_for_dock(dog, fake):
    fake.replies[6004] = [{"accepted": 1}, {"feedback_code": 500, "feedback_msg": ""}]
    task = dog.nav.dock()
    assert task.wait(2.0) and task.feedback == [(500, "")]


def test_nav_task_rejected_and_error(dog, fake):
    fake.replies[6004] = [{"accepted": 2}]
    t = dog.nav.start_mapping()
    assert t.wait(2.0) and t.accepted == 2
    fake.replies[6004] = [{"error_code": 322}]
    t = dog.nav.start_mapping()
    assert t.wait(2.0) and t.error == 322


def test_nav_task_transport_failure(dog, fake):
    def down(rec):
        raise DogError("down")
    fake.replies[6004] = down
    t = dog.nav.dock()
    assert t.wait(2.0) and "down" in t.error


def test_go_to_needs_numbers(dog, fake):
    with pytest.raises(ValueError):
        dog.nav.go_to("1", 2)
    with pytest.raises(ValueError):
        dog.nav.go_to(float("nan"), 2)
    assert fake.sent == []


def test_6004_type_0_never_sent(dog, fake):
    with pytest.raises(ForbiddenCommand):
        dog.nav._start({"type": 0})
    with pytest.raises(ForbiddenCommand):
        fake.stream(6004, {"type": 0, "outdoor": False, "keep_distance": 0.0})
    assert fake.sent == []


# ---- 6009 / 6010 ----------------------------------------------------------------------------
def test_stop(dog, fake):
    fake.replies[6009] = [{"result": 0}]
    assert dog.nav.stop(1) == 0
    s = fake.of(6009)[0]
    assert s.raw == '{"type":1}' and s.timeout >= 65.0
    fake.replies[6009] = [{"result": 10}]
    assert dog.nav.stop(0) == 10
    for bad in (7, 8, 3, 2, -1):
        with pytest.raises(ValueError):
            dog.nav.stop(bad)
    assert len(fake.of(6009)) == 2


def test_save_map(dog, fake):
    fake.replies[6009] = [{"result": 0}]
    t = dog.nav.save_map("casa")
    assert t.wait(2.0) and t.result == 0 and t.kind == "save"
    s = fake.of(6009)[0]
    assert s.raw == '{"type":5,"map_name":"casa"}' and s.timeout >= 65.0
    with pytest.raises(ValueError):
        dog.nav.save_map("")


def test_attach(dog, fake):
    fake.replies[6010] = [{"accepted": 3}]
    t = dog.nav.attach(7)
    assert t.wait(2.0) and t.accepted == 3 and t.kind == "attach"
    assert fake.of(6010)[0].raw == '{"type":7}' and fake.of(6010)[0].timeout >= 7300.0
    with pytest.raises(ValueError):
        dog.nav.attach(0)


# ---- NavState ------------------------------------------------------------------------------
@pytest.mark.parametrize("status,task_type,stop_type,localized,wedged", [
    (101, None, None, False, False), (100, None, None, False, False),
    (5, 5, 5, False, False), (15, 5, 5, False, False),
    (7, 7, 0, False, False), (17, 7, 0, False, False),
    (8, None, 0, True, False), (18, None, 0, True, False),
    (6, None, None, False, False), (16, None, None, False, False),
    (1, 1, 0, False, False), (21, 1, 0, False, False),
    (9, 9, 9, False, False), (11, 11, 11, False, False),
    (13, 13, 13, False, False), (3, 13, 13, False, False),
    (0, None, None, False, True),
])
def test_decode_state(status, task_type, stop_type, localized, wedged):
    s = decode_state(status, 0)
    assert (s.task_type, s.stop_type, s.localized, s.wedged) == (task_type, stop_type, localized, wedged)
    assert s.name != "unknown"


@pytest.mark.parametrize("sub,stop_type", [
    (0, 0), (309, 0), (310, 0), (1000, 0), (1001, 0), (500, 0),   # before 300: type 1 is a no-op
    (300, 1), (307, 1),
])
def test_ab_stop_type_depends_on_the_goal_being_sent(sub, stop_type):
    for status in (1, 21):
        assert decode_state(status, sub).stop_type == stop_type


@pytest.mark.parametrize("status,sub,feedback", [
    (7, 3107, "not localized yet: drive slowly"),
    (1, 308, "arrived"),
    (5, 6, "mapping started"),
    (101, 500, ""),                  # stale dock feedback outside types 9/13
    (1, 500, ""),
    (9, 500, "started"),
    (13, 501, "waiting for target selection"),
    (11, 11, "UWB target lost"),
    (1, 11, ""),
    (101, 0, ""),
])
def test_decode_feedback(status, sub, feedback):
    assert decode_state(status, sub).feedback == feedback


def test_state_from_10001(dog, fake):
    fake.replies[10001] = [{"motion_status": {"motion_id": 111},
                            "task_status": {"task_status": 7, "task_sub_status": 3107}}]
    st = dog.nav.state()
    assert isinstance(st, NavState) and (st.status, st.sub_status, st.stop_type) == (7, 3107, 0)
    assert fake.of(10001)[0].raw == "{}"
