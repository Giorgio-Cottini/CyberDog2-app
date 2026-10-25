"""Camera grabber over 4002 / getFile (ported from app/server.py)."""
from __future__ import annotations

import time

from cyberdog_sdk import wire
from cyberdog_sdk.camera import Camera


def test_camera_start_pull_stop(fake):
    n = {"pull": 0}

    def picture(rec):
        n["pull"] += 1
        if n["pull"] == 2:
            return [wire.FileChunk(error_code=5)]          # INVALID_STATE: re-arm
        return [wire.FileChunk(0, "p.jpg", 4, b"\xff\xd8"), wire.FileChunk(0, "p.jpg", 4, b"\xff\xd9")]
    fake.file_replies[4002] = picture
    cam = Camera(fake, workers=2)
    assert cam.latest() is None
    cam.start()
    got = cam.wait(0, timeout=2.0)
    assert got is not None and got[1] == b"\xff\xd8\xff\xd9" and got[0] >= 1
    time.sleep(0.3)
    cam.stop()
    time.sleep(0.3)
    cmds = [s.params["command"] for s in fake.of(4002) if s.method == wire.SEND_MSG]
    assert cmds[0] == 7 and cmds[-1] == 8 and 7 in cmds[1:-1]      # start, re-arm after error 5, stop
    pulls = [s for s in fake.of(4002) if s.method == wire.GET_FILE]
    assert pulls and all(s.raw == '{"command":1}' for s in pulls)
    assert {s.lane for s in pulls} <= set(cam._channels) and len({s.lane for s in pulls}) == 2
    assert all(s.lane not in ("main", "priority") for s in pulls)   # own channels, never the control lanes
    seq = cam.latest()[0]
    time.sleep(0.2)
    assert cam.latest()[0] == seq                                    # stopped
    cam.close()


def test_camera_close_without_start_sends_nothing(fake):
    cam = Camera(fake)
    cam.close()
    assert fake.sent == []


def test_a_pull_in_flight_at_stop_does_not_restart_the_stream(fake):
    import threading
    gate, inside = threading.Event(), threading.Event()

    def picture(rec):
        inside.set()
        gate.wait(2)
        return [wire.FileChunk(error_code=5)]          # INVALID_STATE, answered after stop()
    fake.file_replies[4002] = picture
    cam = Camera(fake, workers=1)
    cam.start()
    assert inside.wait(2)
    cam.stop()
    gate.set()
    time.sleep(0.4)
    cmds = [s.params["command"] for s in fake.of(4002) if s.method == wire.SEND_MSG]
    assert cmds == [7, 8], cmds
    cam.close()


def test_stop_does_not_wait_behind_a_start_and_stop_is_last(fake):
    import threading
    gate, inside = threading.Event(), threading.Event()

    def command(rec):
        if rec.params["command"] == 7:
            inside.set()
            gate.wait(2)                               # the dog's START can take up to 10 s
        return [{"result": 0}]
    fake.replies[4002] = command
    fake.file_replies[4002] = [wire.FileChunk(error_code=7)]
    cam = Camera(fake, workers=1)
    th = threading.Thread(target=cam.start)
    th.start()
    assert inside.wait(2)
    t0 = time.monotonic()
    cam.stop()
    assert time.monotonic() - t0 < 0.5
    gate.set()
    th.join(2)
    cmds = [s.params["command"] for s in fake.of(4002) if s.method == wire.SEND_MSG]
    assert cmds == [7, 8, 8], cmds                     # the START came back after stop(): STOP again
    cam.close()


def test_fps_starts_again_from_zero_after_a_restart(fake):
    fake.file_replies[4002] = [wire.FileChunk(0, "p.jpg", 2, b"\xff\xd8")]
    cam = Camera(fake, workers=1)
    cam.start()
    assert cam.wait(3, timeout=2.0) is not None
    time.sleep(0.05)
    assert cam.fps > 0
    cam.stop()
    time.sleep(0.3)
    fake.file_replies[4002] = [wire.FileChunk(error_code=7)]          # no frames after the restart
    cam.start()
    assert cam.fps == 0.0                              # the old run's frames no longer count
    cam.close()


def test_fps_drops_to_zero_when_frames_stop(fake, monkeypatch):
    from cyberdog_sdk import camera
    fake.file_replies[4002] = [wire.FileChunk(0, "p.jpg", 2, b"\xff\xd8")]
    cam = Camera(fake, workers=1)
    cam.start()
    assert cam.wait(2, timeout=2.0) is not None
    time.sleep(0.05)
    assert cam.fps > 0
    fake.file_replies[4002] = [wire.FileChunk(error_code=7)]          # camera service unavailable
    monkeypatch.setattr(camera, "STALE_S", 0.3)
    time.sleep(0.5)
    assert cam.fps == 0.0
    cam.close()


def _frame(n):
    return [wire.FileChunk(0, "p.jpg", 1, bytes([n]))]


def test_a_restart_serves_no_frame_from_before_the_stop(fake):
    import threading
    n, gate = {"pull": 0}, threading.Event()

    def picture(rec):
        n["pull"] += 1
        if n["pull"] == 2:          # in flight across stop() and start(): taken before the stop
            gate.wait(2)
        return _frame(n["pull"])
    fake.file_replies[4002] = picture
    cam = Camera(fake, workers=1)
    cam.start()
    assert cam.wait(0, timeout=2.0)[1] == b"\x01"
    time.sleep(0.1)                 # the worker is inside pull 2
    cam.stop()
    cam.start()
    assert cam.latest() is None     # the old frame is not served as live
    gate.set()
    got = cam.wait(0, timeout=2.0)
    assert got is not None and got[1] not in (b"\x01", b"\x02")
    cam.close()


def test_wait_with_no_frame_yet_waits(fake):
    cam = Camera(fake, workers=1)
    t0 = time.perf_counter()
    assert cam.wait(0, timeout=0.3) is None
    assert time.perf_counter() - t0 >= 0.25        # app2's MJPEG loop must not spin


def test_a_failed_start_leaves_the_camera_stopped(fake, monkeypatch):
    cam = Camera(fake, workers=2)
    real = fake.new_channel

    def broken():
        raise OSError("ca-cert.pem: offline placeholder")
    monkeypatch.setattr(fake, "new_channel", broken)
    try:
        cam.start()
        raise AssertionError("start() did not raise")
    except OSError:
        pass
    assert not cam.running and fake.of(4002) == []
    monkeypatch.setattr(fake, "new_channel", real)
    fake.file_replies[4002] = lambda rec: _frame(1)
    cam.start()                     # tries again for real
    assert cam.running and cam.wait(0, timeout=2.0) is not None
    cam.close()


def test_a_refused_stop_is_sent_again(fake):
    answers = iter([[{"result": 7}], [{"result": 0}]])
    fake.replies[4002] = lambda rec: [{"result": 0}] if rec.params["command"] == 7 else next(answers)
    fake.file_replies[4002] = lambda rec: _frame(1)
    cam = Camera(fake, workers=1)
    cam.start()
    cam.stop()
    stops = [s for s in fake.of(4002) if s.method == wire.SEND_MSG and s.params["command"] == 8]
    assert len(stops) == 2 and stops[1].t - stops[0].t >= 0.45
    assert cam.last_error == "4002 8: result 7"
    cam.close()


def test_a_start_during_the_stop_retry_gets_no_stop(fake):
    import threading
    first_stop = threading.Event()

    def reply(rec):
        if rec.params["command"] == 8:
            first_stop.set()
            return [{"result": 7}]                                   # refused: retried 0.5 s later
        return [{"result": 0}]
    fake.replies[4002] = reply
    fake.file_replies[4002] = lambda rec: _frame(1)
    cam = Camera(fake, workers=1)
    cam.start()
    th = threading.Thread(target=cam.stop)
    th.start()
    assert first_stop.wait(2.0)
    cam.start()                                                      # inside the 0.5 s retry gap
    th.join(3.0)
    cmds = [s.params["command"] for s in fake.of(4002) if s.method == wire.SEND_MSG]
    assert cmds == [7, 8, 7]                                         # the new stream is not stopped
    cam.close()
