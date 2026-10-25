"""Layer (a): /api/events (SSE): format, "state" on change only and at most 5 per second,
keepalive comments, a "log" event per outcome and refusal, slow clients never block."""
from __future__ import annotations

import threading
import time

from apphelp import RawStream, clock, server


def open_events(h, **kw):
    r = RawStream(h.port, "/api/events", **kw)
    assert r.status == 200 and r.headers["content-type"].startswith("text/event-stream")
    assert not [k for k in r.headers if k.startswith("access-control-")]
    return r


def test_format_first_frames(h):
    r = open_events(h)
    try:
        f = r.frame()
        assert f[1:] == ("retry", "2000")
        t, kind, data = r.frame()
        assert kind == "state" and data["connected"] is True and data["armed"] is False
        assert data == h.state() or set(data) == set(h.state())
    finally:
        r.close()


def test_log_event_per_outcome_and_refusal(h):
    r = open_events(h)
    try:
        h.post("/api/drive", {"x": 1})                               # refused: gate off
        f = r.until(lambda f: f[1] == "log")
        assert f[2]["level"] == "warn" and "rifiutato /api/drive" in f[2]["msg"]
        assert set(f[2]) == {"t", "level", "msg"} and isinstance(f[2]["t"], float)
        h.arm()
        f = r.until(lambda f: f[1] == "log")
        assert "ABILITATO" in f[2]["msg"]
        h.post("/api/motion", {"id": 141})
        f = r.until(lambda f: f[1] == "log" and "fatto" in f[2]["msg"])
        assert f is not None and f[2]["level"] == "info"
        h.dog.results[123] = (False, 3011)
        h.post("/api/motion", {"id": 123})
        f = r.until(lambda f: f[1] == "log" and f[2]["level"] == "error")
        assert "occupato" in f[2]["msg"]
        f = r.until(lambda f: f[1] == "state" and f[2]["motion"]["last_result"]
                    and f[2]["motion"]["last_result"]["id"] == 123)
        assert f[2]["motion"]["last_result"]["ok"] is False and f[2]["motion"]["busy"] is False
    finally:
        r.close()


def test_keepalive_comment(make_harness):
    h = make_harness(keepalive_s=0.3)
    r = open_events(h)
    try:
        r.frame()
        r.frame()
        f = r.until(lambda f: f[1] == "comment", 1.0)
        assert f is not None and f[2] == "keepalive"
    finally:
        r.close()


def test_default_keepalive_is_10_s():
    assert server.KEEPALIVE_S == 10.0 and server.STATE_HZ == 5


def test_state_only_on_change(make_harness):
    h = make_harness(poll_s=5.0, info_every_s=5.0)      # nothing changes by itself
    h.wait(lambda: h.app.emulator is not None and h.app.soc == 80, what="first 1001 answer")
    r = open_events(h)
    try:
        r.frame()
        r.frame()
        assert [f for f in r.frames(0.8) if f[1] == "state"] == []
        h.post("/api/speed", {"level": 0.7})
        states = [f for f in r.frames(0.6) if f[1] == "state"]
        assert len(states) == 1 and states[0][2]["drive"]["speed"] == 0.7
    finally:
        r.close()


def test_state_at_most_5_per_second(h):
    r = open_events(h)
    stop = threading.Event()

    def churn():
        i = 0
        while not stop.is_set():
            i += 1
            h.post("/api/speed", {"level": 0.1 + (i % 9) / 10})
            time.sleep(0.01)
    th = threading.Thread(target=churn, daemon=True)
    try:
        first = r.frame()
        assert first[1] == "retry"
        t0 = r.frame()[0]
        th.start()
        states = [t0] + [f[0] for f in r.frames(1.6) if f[1] == "state"]
        gaps = [b - a for a, b in zip(states, states[1:])]
        assert len(states) >= 5, len(states)                 # it keeps flowing while the state changes
        assert min(gaps) >= 0.17, gaps                       # 5 Hz: 200 ms apart (socket jitter allowed)
        assert len(states) <= 1.6 * 5 + 2
    finally:
        stop.set()
        th.join(2)
        r.close()


def test_event_hub_never_blocks_and_drops_an_overflowing_client():
    hub = server.EventHub(maxsize=3)
    s = hub.subscribe()
    t0 = clock()
    for i in range(10):
        hub.publish("log", '{"i":%d}' % i)
    assert clock() - t0 < 0.05
    assert s.dead is True and s.get(0) is None
    s2 = hub.subscribe()
    hub.publish("state", "{}")
    assert s2.dead is False and s2.get(0) == ("state", b"event: state\ndata: {}\n\n")
    hub.unsubscribe(s)
    assert hub.count() == 1


def test_a_client_that_never_reads_does_not_block(h):
    stuck = open_events(h, rcvbuf=4096)     # never read again from here
    fresh = None
    try:
        h.wait(lambda: h.app.hub.count() == 1)
        sub = next(iter(h.app.hub.subs))
        msg = "x" * 4000
        worst = 0.0
        for _ in range(5000):
            t0 = clock()
            h.app.log("info", msg)
            worst = max(worst, clock() - t0)
            if sub.dead:
                break
        assert sub.dead, "the stuck client was never dropped"
        assert worst < 0.1, worst                # publishing never waited for the socket
        t0 = clock()
        assert h.state()["connected"] is True and clock() - t0 < 1.0
        h.arm()
        assert h.post("/api/drive", {"x": 1})[0] == 200
        fresh = open_events(h)
        assert fresh.frame()[1] == "retry" and fresh.frame()[1] == "state"
        h.app.log("info", "dopo")
        assert fresh.until(lambda f: f[1] == "log" and f[2]["msg"] == "dopo") is not None
    finally:
        stuck.close()
        if fresh is not None:
            fresh.close()
