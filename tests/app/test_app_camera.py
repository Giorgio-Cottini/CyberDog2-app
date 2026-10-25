"""Layer (a): /stream.mjpg only while the camera is on; one shared grabber; it stops when
the last viewer leaves or the camera is switched off."""
from __future__ import annotations

import time

from apphelp import RawStream


def open_stream(h):
    return RawStream(h.port, "/stream.mjpg")


def test_stream_refused_while_camera_off(h):
    r = open_stream(h)
    try:
        assert r.status == 409
        b = r.body_json()
        assert b["ok"] is False and "fotocamera spenta" in b["error"]
    finally:
        r.close()
    assert h.dog.camera.starts == 0


def test_camera_on_alone_does_not_start_the_grabber(h):
    assert h.post("/api/camera", {"on": True}) == (200, {"ok": True, "on": True})
    time.sleep(0.3)
    assert h.dog.camera.starts == 0
    cam = h.state()["camera"]
    assert cam["on"] is True and cam["fps"] == 0.0 and cam["viewers"] == 0


def test_frames_and_one_shared_grabber(h):
    h.post("/api/camera", {"on": True})
    a, b = open_stream(h), open_stream(h)
    try:
        for r in (a, b):
            assert r.status == 200
            assert r.headers["content-type"] == "multipart/x-mixed-replace; boundary=cyberdogframe"
        seqs = []
        for r in (a, b):
            for _ in range(3):
                ctype, jpeg = r.part()
                assert ctype == "image/jpeg" and jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")
                seqs.append(jpeg)
        assert h.dog.camera.starts == 1
        cam = h.state()["camera"]
        assert cam["viewers"] == 2 and cam["fps"] == 20.0
    finally:
        a.close()
        b.close()
    h.wait(lambda: h.dog.camera.stops >= 1, 3.0, "grabber stopped after the last viewer")
    h.wait(lambda: h.state()["camera"]["viewers"] == 0)
    assert h.dog.camera.running is False and h.app.camera.running_on is None
    assert h.state()["camera"] == {"on": True, "fps": 0.0, "viewers": 0}


def test_grabber_keeps_running_while_one_viewer_stays(h):
    h.post("/api/camera", {"on": True})
    a, b = open_stream(h), open_stream(h)
    try:
        a.part()
        b.part()
        a.close()
        h.wait(lambda: h.state()["camera"]["viewers"] == 1, 3.0)
        assert h.dog.camera.stops == 0
        assert b.part() is not None
    finally:
        b.close()
    h.wait(lambda: h.dog.camera.stops == 1, 3.0)


def test_switching_off_ends_open_streams(h):
    h.post("/api/camera", {"on": True})
    r = open_stream(h)
    try:
        assert r.part() is not None
        assert h.post("/api/camera", {"on": False}) == (200, {"ok": True, "on": False})
        end = time.perf_counter() + 3.0
        while time.perf_counter() < end and not r.eof:
            r.part(0.5)
        assert r.eof, "the stream did not end"
    finally:
        r.close()
    h.wait(lambda: h.dog.camera.stops == 1, 3.0)
    assert h.dog.camera.running is False
    r = open_stream(h)
    assert r.status == 409
    r.close()


def test_camera_body_validation(h):
    assert h.post("/api/camera", {"on": "yes"})[0] == 400
    assert h.post("/api/camera", {})[0] == 400
