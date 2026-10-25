"""tools/probe_dog.py control flow, in process, against a fake dog (no gRPC, no emulator).

main() runs with scripted answers in place of input(), Part 1 cut down to the battery read,
and --time-scale 0.2 on 127.0.0.1. The fake records every call in order, so a test can say
which moves went out and when.
"""
from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PROBE = os.path.join(ROOT, "tools", "probe_dog.py")
YES_TO_ALL = ["libero", "s", "s", "s", "s", "giu", "s", "s", "s", "s"]


@pytest.fixture(scope="module")
def probe():
    spec = importlib.util.spec_from_file_location("probe_dog_flow", PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeDog:
    """A dog that does every move it accepts. refuse holds the indexes of 1004 calls answered
    result false; interrupt_at is a motion_id whose 1004 raises KeyboardInterrupt (a Ctrl+C)."""

    def __init__(self, soc=80, refuse=(), interrupt_at=None, estop_error=None, estop_delay=0.0,
                 estop_ready_for=None):
        self.soc, self.refuse, self.interrupt_at = soc, set(refuse), interrupt_at
        self.estop_error, self.estop_delay = estop_error, estop_delay
        self.estop_ready_for = estop_ready_for      # None: always; n: the ESTOP channel drops after n checks
        self.log, self.moves, self.estops = [], [], []
        self.motion_id = 101

    def wait_ready(self, timeout=8.0):
        pass

    def estop_ready(self, timeout=2.0):
        if self.estop_ready_for is None:
            return True
        self.estop_ready_for -= 1
        return self.estop_ready_for >= 0

    def call(self, name_code, params, timeout=10.0):
        rec = {"name_code": name_code, "params": params, "data": [], "error": None, "elapsed": 0.001}
        if name_code != 10001:
            self.log.append((name_code, params))
        if name_code == 1004:
            mid = params["motion_id"]
            if mid == self.interrupt_at:
                raise KeyboardInterrupt
            ok = len(self.moves) not in self.refuse
            self.moves.append(mid)
            if ok:
                self.motion_id = mid
            rec["data"].append(json.dumps({"motion_id": mid, "result": ok, "code": 0 if ok else 3010}))
        elif name_code == 10001:
            rec["data"].append(json.dumps({"motion_status": {"motion_id": self.motion_id},
                                           "task_status": {"task_status": 101}}))
        elif name_code == 1001:
            rec["data"].append(json.dumps({"bat_info": {} if self.soc is None else {"power": self.soc}}))
        elif name_code != 1002:
            rec["data"].append(json.dumps({"result": 0}))
        return rec

    def get_file(self, name_code, params, timeout=40.0):
        self.log.append((name_code, params))
        return {"name_code": name_code, "params": params, "error_codes": [0], "file_name": "x.jpg",
                "file_size": 1, "bytes": 1, "error": None, "elapsed": 0.0}

    def estop(self):
        self.estops.append(time.time())
        if self.estop_delay and len(self.estops) == 1:
            time.sleep(self.estop_delay)
        if self.estop_error:
            return {"data": [], "error": self.estop_error, "elapsed": 0.0}
        return {"data": [json.dumps({"motion_id": 0, "result": True, "code": 0})], "error": None,
                "elapsed": 0.0}


def run_main(probe, monkeypatch, tmp_path, dog, answers):
    """main() with the fake dog and the answers; returns (exit code, results)."""
    left = list(answers)

    def scripted(prompt=""):
        print(prompt)
        if not left:
            raise EOFError
        return left.pop(0)

    monkeypatch.setattr(probe, "Dog", lambda host, port: dog)
    monkeypatch.setattr(probe, "STILL_STEPS", [("device_info", probe.step_device_info)])
    monkeypatch.setattr("builtins.input", scripted)
    out = str(tmp_path / "results.json")
    code = probe.main(["--host", "127.0.0.1", "--time-scale", "0.2", "--out", out])
    with open(out, encoding="utf-8") as f:
        return code, json.load(f)


def test_yes_to_all_stands_before_the_tilt(probe, monkeypatch, tmp_path, capsys):
    dog = FakeDog()
    code, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert code == 0 and "motion_stopped" not in r
    assert dog.moves == [111, 143, 111, 141, 111, 111, 101]
    # the 201 stream starts only after the 111 that follows the paw
    order = [(c, p.get("motion_id")) for c, p in dog.log if c in (1002, 1004)]
    first_201 = order.index((1002, 201))
    assert order[first_201 - 1] == (1004, 111) and order[first_201 - 2] == (1004, 141)
    out = capsys.readouterr().out
    assert "Il cane si alzera' in piedi (111)." in out and "Il cane si sdraiera' (101)." in out
    assert "Il cane si rialzera' in piedi (111)." in out


@pytest.mark.parametrize("answers, moves, stopped", [
    (["libero", "n"], [111], "stand: the person answered no after 111"),
    (["libero", "s", "n"], [111, 143], "sit: the person answered no after 143"),
    (["libero", "s", "s", "n"], [111, 143, 111], "sit: the person answered no after 111"),
    (["libero", "s", "s", "s", "n"], [111, 143, 111, 141], "trick: the person answered no after 141"),
])
def test_a_no_answer_stops_part_2(probe, monkeypatch, tmp_path, capsys, answers, moves, stopped):
    dog = FakeDog()
    code, r = run_main(probe, monkeypatch, tmp_path, dog, answers)
    assert code == 0 and dog.moves == moves
    assert r["motion_stopped"] == stopped + ": no further motion"
    assert not any(c == 1002 for c, _ in dog.log)
    assert "Mi fermo qui, senza altri movimenti: hai risposto no" in capsys.readouterr().out


@pytest.mark.parametrize("refused, answers, moves, stopped", [
    (0, ["libero", "s"], [111], "stand: 111 not accepted"),
    (2, ["libero", "s", "s", "s"], [111, 143, 111], "sit: 111 not accepted"),
    (4, ["libero", "s", "s", "s", "s"], [111, 143, 111, 141, 111], "posture: 111 not accepted"),
])
def test_a_refused_move_stops_part_2(probe, monkeypatch, tmp_path, refused, answers, moves, stopped):
    dog = FakeDog(refuse=[refused])
    code, r = run_main(probe, monkeypatch, tmp_path, dog, answers)
    assert code == 0 and dog.moves == moves
    assert r["motion_stopped"] == stopped + ": no further motion"
    assert not any(c == 1002 for c, _ in dog.log)


def test_battery_not_read_skips_part_2(probe, monkeypatch, tmp_path, capsys):
    dog = FakeDog(soc=None)
    code, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert code == 0 and dog.moves == [] and r["motion_skipped"] == "battery not read"
    assert "floor_confirmed" not in r
    assert "Batteria non letta: salto la parte con movimento." in capsys.readouterr().out


def test_a_plain_yes_does_not_confirm_the_floor(probe, monkeypatch, tmp_path, capsys):
    dog = FakeDog()
    code, r = run_main(probe, monkeypatch, tmp_path, dog, ["s", "si", "s"])
    assert code == 0 and dog.moves == [] and r["floor_confirmed"] == "nessuna risposta"
    assert capsys.readouterr().out.count("Scrivi una di queste risposte: libero/n") == 3


def test_ctrl_c_sends_estop_three_times_and_restores_the_handler(probe, monkeypatch, tmp_path, capsys):
    before = signal.getsignal(signal.SIGINT)
    dog = FakeDog(interrupt_at=143)
    code, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert code == 2 and r["interrupted"] == "ctrl+c in part 2: ESTOP sent three times"
    assert len(dog.estops) == 3 and 0.3 <= dog.estops[1] - dog.estops[0] < 0.6
    assert 0.95 <= dog.estops[2] - dog.estops[0] < 1.3     # monotonic in the probe, time.time here
    assert "ESTOP inviato." in capsys.readouterr().out
    assert signal.getsignal(signal.SIGINT) is before


def test_estop_not_delivered_is_said_loudly(probe, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(probe, "ESTOP_RETRY_S", 1.0)
    dog = FakeDog(interrupt_at=111, estop_error="UNAVAILABLE failed to connect")
    code, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert code == 4
    assert r["interrupted"] == "ctrl+c in part 2: no ESTOP delivered after the watchdog window"
    assert len(r["estop"]) == len(dog.estops) > 2               # sent again while there was no reply
    out = capsys.readouterr().out
    assert "ATTENZIONE: il cane non ha confermato l'arresto di emergenza." in out
    assert "ESTOP inviato" not in out


def test_repeats_are_timed_from_the_first_send(probe):
    dog = FakeDog(estop_delay=1.0)                              # the first reply takes 1 s
    t0 = time.time()
    recs = probe.estop_burst(dog)
    assert [r["error"] for r in recs] == [None, None, None]
    assert 0.3 <= dog.estops[1] - dog.estops[0] < 0.6 and 0.95 <= dog.estops[2] - dog.estops[0] < 1.3
    assert time.time() - t0 >= 1.0                              # it still waits for the first reply


def test_skin_and_camera_clean_up_on_ctrl_c(probe, monkeypatch):
    monkeypatch.setattr(probe, "SCALE", 0.01)

    def no_answer(prompt=""):
        raise EOFError

    monkeypatch.setattr("builtins.input", no_answer)
    dog = FakeDog()
    with pytest.raises(KeyboardInterrupt):
        probe.step_skin(dog, None)
    assert dog.log[-1] == (11001, {"data": False})

    class Cut(FakeDog):
        def get_file(self, name_code, params, timeout=40.0):
            raise KeyboardInterrupt

    dog = Cut()
    with pytest.raises(KeyboardInterrupt):
        probe.step_camera_delete(dog, None)
    assert dog.log[-1] == (4002, {"command": 8})


def test_answers_ignore_case_and_accents(probe, monkeypatch):
    assert probe.plain(" Giù ") == "giu"
    monkeypatch.setattr("builtins.input", lambda prompt="": "GIÙ")
    assert probe.ask("?", {"su": "su", "giu": "giu", "n": "non si e' mossa"}) == "giu"


def test_save_replaces_the_file_whole(probe, monkeypatch, tmp_path):
    dog = FakeDog()
    run_main(probe, monkeypatch, tmp_path, dog, ["n"])
    assert sorted(os.listdir(tmp_path)) == ["results.json"]


def test_no_motion_step_starts_without_the_estop_channel(probe, monkeypatch, tmp_path, capsys):
    dog = FakeDog(estop_ready_for=1)                            # ready for the stand, gone before the sit
    code, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert code == 3 and dog.moves == [111]
    assert r["motion_stopped"] == "sit: the ESTOP channel is not connected: no further motion"
    assert "arresto di emergenza non risponde" in capsys.readouterr().out


def test_a_failed_save_still_prints_the_warning(probe, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(probe, "ESTOP_RETRY_S", 1.0)
    real = os.replace

    def locked(src, dst):
        if str(dst).endswith("results.json") and dog.estops:    # OneDrive holds the file after the Ctrl+C
            raise PermissionError(32, "file in use")
        return real(src, dst)
    monkeypatch.setattr(probe.os, "replace", locked)
    dog = FakeDog(interrupt_at=111, estop_error="UNAVAILABLE failed to connect")
    code, _ = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    out = capsys.readouterr().out
    assert code == 4
    assert out.index("non conferma ancora l'arresto") < out.index("ATTENZIONE: il cane non ha confermato")
    assert out.index("ATTENZIONE: il cane non ha confermato") < out.index("non riesco a salvare i risultati")
    assert "I risultati non sono stati salvati" in out


class BrokenStdout:
    """A stdout whose reader died with the Ctrl+C (| tee, | more): every write raises."""

    def write(self, text):
        raise OSError(22, "Invalid argument")

    def flush(self):
        raise OSError(22, "Invalid argument")


@pytest.mark.parametrize("estop_error, code", [(None, 2), ("UNAVAILABLE failed to connect", 4)])
def test_a_broken_stdout_does_not_stop_the_estop(probe, monkeypatch, tmp_path, estop_error, code):
    monkeypatch.setattr(probe, "ESTOP_RETRY_S", 1.0)

    class CutPipe(FakeDog):
        def call(self, name_code, params, timeout=10.0):
            if name_code == 1004 and params["motion_id"] == 143:
                monkeypatch.setattr(sys, "stdout", BrokenStdout())
                raise KeyboardInterrupt
            return FakeDog.call(self, name_code, params, timeout)

    dog = CutPipe(estop_error=estop_error)
    got, r = run_main(probe, monkeypatch, tmp_path, dog, YES_TO_ALL)
    assert got == code and r["interrupted"].startswith("ctrl+c in part 2")
    assert len(dog.estops) == 3 if estop_error is None else len(dog.estops) > 2


@pytest.mark.parametrize("argv, code", [(["--help"], 0), (["--bogus"], 2)])
def test_argument_errors_keep_their_exit_code(argv, code):
    p = subprocess.run([sys.executable, PROBE] + argv, capture_output=True, universal_newlines=True,
                       encoding="utf-8", timeout=60)
    assert p.returncode == code and "Traceback" not in p.stderr
