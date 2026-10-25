"""Motion catalog (motion.md §6) and its cross-check against motion_id_map.json."""
from __future__ import annotations

import json
import os

import pytest

from cyberdog_sdk import catalog

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# every id listed in motion.md §6
SECTION_6 = ([0, 101, 102, 111, 112, 113, 114, 115, 117, 118, 119, 124, 138, 125,
              141, 142, 143, 148, 149, 150, 144, 145, 146, 173, 155, 156, 161, 164, 165, 174, 157, 158,
              152, 159, 160, 167, 169, 170, 171, 151, 140, 176, 177, 178, 153, 154, 162, 163, 166, 168,
              139, 172, 179, 400, 123, 175, 130, 131, 132, 133, 134, 135, 136, 137, 147, 500, 126,
              120, 121, 122, 127, 128, 129, 506, 507, 508, 509, 510, 511, 512, 513]
             + list(range(501, 506)) + list(range(180, 200)))


def test_every_section_6_id_is_in_the_catalog():
    missing = [i for i in SECTION_6 if catalog.get(i) is None]
    assert missing == []


def test_fields_and_types():
    for m in catalog.all():
        assert isinstance(m.id, int) and m.key and m.name_it and m.name_en
        assert m.risk in ("R0", "R1", "R2", "R3", "R4")
        assert isinstance(m.lr_safe, bool) and isinstance(m.min_ms, int)
        assert m.typical_s is None or m.typical_s > 0
        assert isinstance(m.needs, tuple)
        assert not (m.lr_safe and m.risk in ("R2", "R3", "R4")), m


@pytest.mark.parametrize("ids,risk", [
    ((0, 101, 102, 111, 112, 141, 142, 143, 144, 145, 146, 148, 149, 150, 173), "R0"),
    ((155, 156, 161, 164, 165, 174, 157, 158, 123, 175), "R1"),
    ((117, 118, 119, 124, 138, 125, 152, 159, 160, 167, 169, 170, 171, 151, 140, 176, 177, 178,
      501, 502, 503, 504, 505), "R2"),
    ((153, 154, 162, 163, 166, 168, 130, 131, 132, 133, 134, 135, 136, 137, 147, 500, 126), "R3"),
    ((139, 172, 179, 120, 121, 122, 127, 128, 129, 506, 507, 508, 509, 510, 511, 512, 513), "R4"),
])
def test_risk_classes(ids, risk):
    assert {i: catalog.get(i).risk for i in ids} == {i: risk for i in ids}


def test_lr_safe_flags():
    safe = (101, 111, 141, 142, 143, 148, 149, 150, 144, 145, 146, 173, 155, 156, 161, 164, 165,
            174, 123, 175)
    assert all(catalog.get(i).lr_safe for i in safe)
    assert not any(catalog.get(i).lr_safe for i in (102, 112, 157, 158, 152, 140, 130, 139))


def test_needs():
    assert all(catalog.get(i).needs == ("sit",) for i in (148, 149, 150))
    assert all("never" in catalog.get(i).needs for i in (113, 114, 115, 400, 203, 390, 391, 392))
    assert all("servo" in catalog.get(i).needs for i in (303, 304, 305, 308))
    assert all("pose" in catalog.get(i).needs for i in (201, 202, 211, 212, 213, 214))
    assert catalog.get(143).needs == () and catalog.get(111).needs == ()


def test_typical_lengths_from_the_doc():
    expected = {101: 3.0, 111: 6.0, 141: 8.6, 142: 8.6, 143: 1.3, 144: 3.9, 145: 3.9, 146: 3.3,
                174: 14.0, 152: 9.0, 151: 27.0, 123: 8.7, 130: 0.79, 131: 0.92, 132: 0.79, 136: 0.76,
                140: 60.0, 176: 120.0, 505: 120.0}
    assert {i: catalog.get(i).typical_s for i in expected} == expected


def test_min_exec_time_matches_motion_id_map():
    with open(os.path.join(ROOT, "emulator", "data", "motion_id_map.json"), encoding="utf-8") as f:
        table = {e["motion_id"]: e["min_exec_time"] for e in json.load(f)}
    diff = {m.id: (m.min_ms, table[m.id]) for m in catalog.all() if m.id in table and m.min_ms != table[m.id]}
    assert diff == {}
    assert [i for i in SECTION_6 if i not in table] == []


def test_by_risk_and_all():
    ids = [m.id for m in catalog.all()]
    assert ids == sorted(ids) and len(ids) == len(set(ids))
    safe = catalog.by_risk("R0", "R1")
    assert {m.risk for m in safe} == {"R0", "R1"} and catalog.get(143) in safe
    assert catalog.by_risk() == []
    assert catalog.get(999) is None
