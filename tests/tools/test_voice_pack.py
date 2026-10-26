"""tools/voice_pack.py: the plan, the sound.toml merge, the dog bodies and the dog steps.

The dog bodies run for real, in process, against temporary folders: FakeDog executes each body
with the globals the VP engine's header provides (os, sys, time, threading, cyberdog). Its os
renames like Linux (over an existing file) and has statvfs, so the bodies also run on Windows.
The pyflakes check needs pyflakes (the CyberDogSim WSL distro has it); the audio tests need ffmpeg.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "voice_pack.py")
TOML = ('[config]\r\ntts_ids = [31000, 31038]\r\n'
        'tts_names = ["basis/a.opus", "basis/self_check_start.opus"]\r\n'
        'music_ids = [60003]\r\nmusic_names = ["music/m.opus"]\r\n')


@pytest.fixture(scope="module")
def vp_tool():
    spec = importlib.util.spec_from_file_location("voice_pack_test", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def row(pid, choice, where="sd", file="", it="Ciao."):
    return {"id": str(pid), "choice": choice, "where": where, "file": file, "it": it}


# ---- plan and paths ---------------------------------------------------------------------
def test_check_rel(vp_tool):
    for ok in ("basis/self_check_start.opus", "custom/124.opus", "power/power100.opus"):
        assert vp_tool.check_rel(ok) == ok
    for bad in ("../x.opus", "/abs.opus", "a//b.opus", "yaml/sound.toml", "a b.opus",
                "x.opus;rm", "", "basis/..opus", None):
        with pytest.raises(ValueError):
            vp_tool.check_rel(bad)


def test_plan_sources(vp_tool, tmp_path):
    rec = tmp_path / "rec"
    rec.mkdir()
    for name in ("2.m4a", "3.wav", "4.wav"):
        (rec / name).write_bytes(b"x")
    rows = [row(1, "rec", file="basis/one.opus"),           # no recording yet: speech
            row(2, "rec", file="basis/two.opus"),           # recording
            row(3, "tts", file="basis/three.opus"),         # a recording wins over tts
            row(4, "keep", file="basis/four.opus"),         # keep wins over a recording
            row(51, "click", where="board", file="camera_start_photos"),
            row(6, "silence", file="basis/six.opus")]
    got = {pid: (dest, where, kind) for pid, dest, where, (kind, _) in vp_tool.plan(rows, str(rec))}
    assert got == {"1": ("basis/one.opus", "sd", "tts"), "2": ("basis/two.opus", "sd", "rec"),
                   "3": ("basis/three.opus", "sd", "rec"), "51": ("custom/51.opus", "board", "click"),
                   "6": ("basis/six.opus", "sd", "silence")}


@pytest.mark.parametrize("rows", [
    [row(1, "tts", file="a/b.opus"), row(1, "tts", file="a/c.opus")],     # id twice
    [row(1, "tts", file="a/b.opus"), row(2, "tts", file="a/b.opus")],     # one destination twice
    [row(1, "loud", file="a/b.opus")],                                    # unknown choice
    [row(1, "tts", file="a/b.opus", it=" ")],                             # no Italian line
    [row(1, "tts", file="../b.opus")],                                    # destination outside
    [row(1, "tts", where="usb", file="a/b.opus")],
])
def test_plan_refuses(vp_tool, tmp_path, rows):
    with pytest.raises(ValueError):
        vp_tool.plan(rows, str(tmp_path))


def test_phrase_table_plans(vp_tool):
    """The committed table itself: every row has a valid choice and destination."""
    items = vp_tool.plan(vp_tool.load_rows(), os.path.join(ROOT, "no_such_folder"))
    assert len(items) > 150
    assert all(dest.startswith("custom/") == (where == "board") for _, dest, where, _ in items)


# ---- sound.toml -------------------------------------------------------------------------
def test_merge_toml(vp_tool):
    merged = vp_tool.merge_toml(TOML, [(124, "custom/124.opus"), (51, "custom/51.opus")])
    assert merged.split("\r\n")[1] == "tts_ids = [31000, 31038, 124, 51]"
    assert merged.split("\r\n")[2] == ('tts_names = ["basis/a.opus", "basis/self_check_start.opus", '
                                       '"custom/124.opus", "custom/51.opus"]')
    assert merged.count("\r\n") == TOML.count("\r\n") and "\n" not in merged.replace("\r\n", "")
    assert merged.split("\r\n")[3:] == TOML.split("\r\n")[3:]
    assert vp_tool.merge_toml(merged, [(124, "custom/124.opus")]) == merged     # idempotent


def test_merge_toml_refuses(vp_tool):
    with pytest.raises(ValueError):                     # 31038 is an SD prompt already
        vp_tool.merge_toml(TOML, [(31038, "custom/31038.opus")])
    with pytest.raises(ValueError):
        vp_tool.merge_toml(TOML.replace('"basis/a.opus", ', ""), [])
    with pytest.raises(ValueError):
        vp_tool.merge_toml("[config]\r\n", [])


def test_batches(vp_tool):
    files = [("a", b"1" * 4), ("b", b"2" * 4), ("c", b"3" * 9), ("d", b"4")]
    assert [[rel for rel, _ in g] for g in vp_tool.batches(files, 8)] == [["a", "b"], ["c"], ["d"]]
    assert vp_tool.batches([], 8) == []


def test_body_needs_every_value(vp_tool):
    with pytest.raises(ValueError):
        vp_tool.body("push", "/home/mi/Camera/r.json")
    with pytest.raises(ValueError):
        vp_tool.body("play", "/home/mi/Camera/r.json")


# ---- dog bodies, run in process ---------------------------------------------------------
class LinuxOs:
    """os as the bodies see it on the dog: rename replaces, statvfs exists."""

    def __getattr__(self, name):
        return getattr(os, name)

    @staticmethod
    def rename(src, dst):
        os.replace(src, dst)

    @staticmethod
    def statvfs(path):
        return types.SimpleNamespace(f_bavail=1000, f_frsize=4096)


class FakeDog:
    def __init__(self, mod, tmp):
        self.mod, self.tmp = mod, tmp
        self.sound, self.orig = str(tmp / "sound"), str(tmp / "sound.orig")
        self.ops, self.played = [], []
        audio = types.SimpleNamespace(offline_play=lambda i, v: self.played.append((i, v)) or "ok")
        self.cyberdog = types.SimpleNamespace(audio=audio)

    def raw(self, op, **values):
        report = str(self.tmp / "report.json")
        code = self.mod.body(op, report, sound=self.sound, orig=self.orig, **values)
        exec(compile(code, "body_%s.py" % op, "exec"),
             {"os": LinuxOs(), "sys": sys, "time": time, "threading": threading,
              "cyberdog": self.cyberdog, "__name__": "__main__"})
        with open(report, encoding="utf-8") as f:
            rep = json.load(f)
        os.remove(report)
        if "files" in rep:                             # relpath gives \\ on Windows
            rep["files"] = {k.replace(os.sep, "/"): v for k, v in rep["files"].items()}
        return rep

    def run(self, op, describe, timeout=0.0, **values):
        self.ops.append(op)
        rep = self.raw(op, **values)
        if rep.get("error"):
            raise RuntimeError(rep["error"])
        return rep


def sha(data):
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def setup(vp_tool, tmp_path, monkeypatch):
    sound = tmp_path / "sound"
    for rel, data in (("basis/self_check_start.opus", b"orig-31038"), ("music/m.opus", b"music"),
                      ("yaml/sound.toml", TOML.encode())):
        (sound / rel).parent.mkdir(parents=True, exist_ok=True)
        (sound / rel).write_bytes(data)
    out = tmp_path / "out"
    out.mkdir()
    files = {}
    for pid, dest, where, data in (("31038", "basis/self_check_start.opus", "sd", b"new-31038"),
                                   ("124", "custom/124.opus", "board", b"new-124")):
        (out / (pid + ".opus")).write_bytes(data)
        files[pid] = {"dest": dest, "where": where, "source": "tts", "key": "k",
                      "sha256": sha(data), "bytes": len(data)}
    (out / "manifest.json").write_text(json.dumps({"voice": "Elsa", "chain": 1, "files": files}))
    monkeypatch.setattr(vp_tool, "BACKUP_TOML", str(tmp_path / "backup" / "sound.toml"))
    args = argparse.Namespace(dir=str(out), ids=None, batch=10, id=124)
    return FakeDog(vp_tool, tmp_path), args, sound


def test_status_body(setup):
    dog, _, sound = setup
    rep = dog.raw("status")
    assert rep["orig"] is False and rep["free"] == 4096000
    assert rep["files"]["music/m.opus"] == [5, sha(b"music")]
    assert set(rep["files"]) == {"basis/self_check_start.opus", "music/m.opus", "yaml/sound.toml"}


def test_push_body_checks(setup):
    dog, _, sound = setup
    payload = [("basis/self_check_start.opus", sha(b"x"), "eQ==")]       # "y", not "x"
    assert "no backup" in dog.raw("push", FILES=payload)["error"]
    os.makedirs(dog.orig)
    assert "checksum mismatch" in dog.raw("push", FILES=payload)["error"]
    assert (sound / "basis/self_check_start.opus").read_bytes() == b"orig-31038"


def test_dog_steps(vp_tool, setup, capsys):
    dog, args, sound = setup
    with pytest.raises(SystemExit):                     # no backup yet
        vp_tool.cmd_push(dog, args)

    vp_tool.cmd_backup(dog, args)
    with open(vp_tool.BACKUP_TOML, encoding="utf-8", newline="") as f:
        assert f.read() == TOML                         # CRLF kept
    assert os.path.isfile(os.path.join(dog.orig, "basis", "self_check_start.opus"))
    vp_tool.cmd_backup(dog, args)                       # once only: the copy stays the original
    assert "gia presente" in capsys.readouterr().out

    with pytest.raises(SystemExit):                     # custom/124.opus is not on the dog yet
        vp_tool.cmd_toml(dog, args)

    dog.ops.clear()
    args.batch = 9                                      # 9 + 7 bytes: two push tasks
    vp_tool.cmd_push(dog, args)
    assert dog.ops == ["status", "push", "push"]
    assert (sound / "basis/self_check_start.opus").read_bytes() == b"new-31038"
    assert (sound / "custom/124.opus").read_bytes() == b"new-124"
    dog.ops.clear()
    vp_tool.cmd_push(dog, args)                         # nothing left to send
    assert dog.ops == ["status"]

    vp_tool.cmd_toml(dog, args)
    toml = (sound / "yaml/sound.toml").read_bytes().decode()
    assert "tts_ids = [31000, 31038, 124]\r\n" in toml and '"custom/124.opus"]\r\n' in toml
    dog.ops.clear()
    vp_tool.cmd_toml(dog, args)                         # already merged: no write
    assert dog.ops == ["status"]

    capsys.readouterr()
    vp_tool.cmd_status(dog, args)
    out = capsys.readouterr().out
    assert "presente" in out and "modificato" in out and "2 file uguali" in out

    vp_tool.cmd_play(dog, args)
    assert dog.played == [(124, -1)]
    dog.cyberdog.audio.offline_play = lambda i, v: "- state:\n  - code = 7949\n"
    capsys.readouterr()
    vp_tool.cmd_play(dog, args)
    assert "Nota" in capsys.readouterr().out

    vp_tool.cmd_restore(dog, args)
    assert (sound / "basis/self_check_start.opus").read_bytes() == b"orig-31038"
    assert (sound / "yaml/sound.toml").read_bytes() == TOML.encode()
    assert (sound / "custom/124.opus").exists()         # restore keeps custom/


def test_toml_refuses_foreign_toml(vp_tool, setup):
    dog, args, sound = setup
    vp_tool.cmd_backup(dog, args)
    vp_tool.cmd_push(dog, args)
    (sound / "yaml/sound.toml").write_bytes(TOML.replace("60003", "60004").encode())
    with pytest.raises(SystemExit):
        vp_tool.cmd_toml(dog, args)


def test_bodies_pass_pyflakes(vp_tool):
    """The engine deletes a task whose file draws any pyflakes warning."""
    api = pytest.importorskip("pyflakes.api")
    reporter_mod = pytest.importorskip("pyflakes.reporter")
    import io
    header = "import os\nimport sys\nimport time\nimport threading\ncyberdog = None\n"
    footer = "print(sys, threading)\n"
    values = {"push": {"FILES": [("custom/1.opus", "0" * 64, "eA==")]}, "toml": {"TEXT": TOML},
              "play": {"ID": 124}}
    for op in vp_tool.WORK:
        code = header + vp_tool.body(op, "/home/mi/Camera/r.json", **values.get(op, {})) + footer
        out = io.StringIO()
        n = api.check(code, "body_%s.py" % op, reporter_mod.Reporter(out, out))
        assert n == 0, "%s: %s" % (op, out.getvalue())


# ---- audio (needs ffmpeg) ---------------------------------------------------------------
def ffmpeg_or_skip(mod):
    if not (shutil.which(mod.FFMPEG) or os.path.isfile(mod.FFMPEG)):
        pytest.skip("ffmpeg not found")
    return mod.FFMPEG


def fake_tts(ffmpeg):
    """Writes, for each job line, 0.4 s of silence, a quiet tone and 0.5 s of silence."""
    def runner(voice, job_path):
        runner.calls.append(voice)
        with open(job_path, encoding="utf-8") as f:
            job = json.load(f)
        for item in job:
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-f", "lavfi", "-i",
                            "sine=f=%d:d=1:r=16000" % (300 + 10 * len(item["text"])),
                            "-af", "volume=0.05,adelay=400,apad=pad_dur=0.5", "-ac", "1", item["out"]],
                           check=True)
    runner.calls = []
    return runner


def test_build(vp_tool, tmp_path):
    ffmpeg = ffmpeg_or_skip(vp_tool)
    rows = [row(1, "tts", file="basis/one.opus", it="Uno."),
            row(2, "tts", file="basis/two.opus", it="Due, piu lungo."),
            row(51, "click", where="board", file="camera_start_photos"),
            row(9, "keep", file="basis/nine.opus")]
    out, tts = str(tmp_path / "out"), str(tmp_path / "tts")
    runner = fake_tts(ffmpeg)
    files = vp_tool.build(rows, "Elsa", out, str(tmp_path / "rec"), tts, tts_runner=runner)
    assert sorted(files) == ["1", "2", "51"] and files["51"]["dest"] == "custom/51.opus"
    assert runner.calls == ["Elsa"]
    probe = subprocess.run([ffmpeg.replace("ffmpeg", "ffprobe"), "-v", "error", "-show_entries",
                            "stream=codec_name,sample_rate,channels:format=duration", "-of", "json",
                            os.path.join(out, "1.opus")], stdout=subprocess.PIPE, check=True)
    info = json.loads(probe.stdout)
    st = info["streams"][0]
    assert (st["codec_name"], st["sample_rate"], st["channels"]) == ("opus", "48000", 1)
    assert float(info["format"]["duration"]) < 1.4         # 1.9 s source, silence trimmed
    meter = subprocess.run([ffmpeg, "-hide_banner", "-nostats", "-i", os.path.join(out, "1.opus"),
                            "-af", "loudnorm=print_format=json", "-f", "null", "-"],
                           stderr=subprocess.PIPE, check=True)
    lufs = float(vp_tool._I_RE.search(meter.stderr.decode("utf-8", "replace")).group(1))
    assert abs(lufs - vp_tool.TARGET_LUFS) < 1.0

    before = {p: e["sha256"] for p, e in files.items()}
    rows[1]["it"] = "Due, cambiato."
    files = vp_tool.build(rows, "Elsa", out, str(tmp_path / "rec"), tts, tts_runner=runner)
    assert runner.calls == ["Elsa", "Elsa"]
    with open(os.path.join(tts, "elsa", "index.json"), encoding="utf-8") as f:
        assert json.load(f)["2"] == "Due, cambiato."
    assert files["1"]["sha256"] == before["1"] and files["2"]["sha256"] != before["2"]

    rows[0]["choice"] = "keep"                             # a row switched to keep leaves the build
    files = vp_tool.build(rows, "Elsa", out, str(tmp_path / "rec"), tts, tts_runner=runner)
    assert sorted(files) == ["2", "51"] and not os.path.exists(os.path.join(out, "1.opus"))
