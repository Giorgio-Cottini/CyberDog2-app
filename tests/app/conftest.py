"""Fixtures for the app server tests: `h` / `make_harness` run app2/server.py in-process
with a StubDog (see apphelp.py). The emulator layer has its own fixtures."""
from __future__ import annotations

import pytest

from apphelp import Harness


@pytest.fixture
def make_harness(tmp_path):
    made = []

    def make(**kw):
        h = Harness(tmp_path, **kw)
        made.append(h)
        return h
    yield make
    for h in made:
        h.close()


@pytest.fixture
def h(make_harness):
    return make_harness()
