"""Italian voice pack for the CyberDog 2: build the prompts on the PC, install them on the dog.

The dog's audio node (cyberdog_audio) plays a prompt id in one of two ways:
  - an id listed in /SDCARD/sound/yaml/sound.toml: the audio board fetches
    http://192.168.44.1:8001/<name> from the dog's nginx (root /SDCARD/sound) at every play, so a
    replaced file plays at its next use, with no reboot;
  - any other id: the audio board plays its built-in file. A sound.toml entry that points the id
    to custom/<id>.opus overrides it. cyberdog_audio reads sound.toml once when it starts, so the
    dog needs one reboot after `toml`.

    python tools\\voice_pack.py build [--voice Elsa] [--dir voices\\out]   PC only
    python tools\\voice_pack.py backup      copies /SDCARD/sound to /SDCARD/sound.orig, once
    python tools\\voice_pack.py push        writes the built files that differ on the dog
    python tools\\voice_pack.py toml        adds the built-in ids to sound.toml (then reboot)
    python tools\\voice_pack.py play 31038  plays one prompt on the dog
    python tools\\voice_pack.py status      compares the dog with the build
    python tools\\voice_pack.py restore     puts the original files back (then reboot)

Sources, per row of voices/phrases.csv (column choice): a recording voices/rec/<id>.<wav|m4a|
mp3|ogg|flac> wins over every choice except keep; rec without a recording and tts speak column
it with a Windows Italian voice (tools/voice_tts.ps1, cached in voices/tts/<voice>/); silence is
0.3 s of silence; click is a synthesized shutter click; keep leaves the dog's file alone.
Recordings and speech go through one chain: trim the silence at both ends, compress, set the
loudness to -12 LUFS (the level of Xiaomi's prompts), limit the peaks to -2.5 dBFS, and encode
Ogg Opus 48 kHz mono 64 kb/s. The build writes <dir>/<id>.opus and <dir>/manifest.json.

Each dog step is one VP debug task (2002) that runs as user mi and returns its report through
/home/mi/Camera and getFile 4004 (cyberdog_sdk/vp.py). The engine kills the running debug task
when a new one arrives: close app2 while this tool works on the dog.
"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import textwrap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# pylibs/ holds Windows builds (grpcio's cygrpc is a .pyd); on Linux it would shadow pip3's grpcio.
sys.path[:0] = ([os.path.join(ROOT, "pylibs")] if os.name == "nt" else []) + [ROOT]

VOICES = os.path.join(ROOT, "voices")
CSV_PATH = os.path.join(VOICES, "phrases.csv")
REC_DIR = os.path.join(VOICES, "rec")
TTS_DIR = os.path.join(VOICES, "tts")
OUT_DIR = os.path.join(VOICES, "out")
BACKUP_TOML = os.path.join(VOICES, "backup", "sound.toml")
TTS_PS1 = os.path.join(ROOT, "tools", "voice_tts.ps1")
FFMPEG = os.environ.get("FFMPEG") or shutil.which("ffmpeg") or "ffmpeg"
REC_EXT = (".wav", ".m4a", ".mp3", ".ogg", ".flac")
CHOICES = ("rec", "tts", "keep", "silence", "click")

SOUND = "/SDCARD/sound"
ORIG = "/SDCARD/sound.orig"
TOML_REL = "yaml/sound.toml"
BATCH_BYTES = 200 * 1024      # raw bytes per push task; base64 makes the body about 4/3 of it
REPORT_TIMEOUT = 240.0

TARGET_LUFS = -12.0
CHAIN_VERSION = 2             # part of every cache key: change it with the chain to encode again
PRE = ("highpass=f=80,"
       "silenceremove=start_periods=1:start_threshold=-50dB:start_silence=0.05,areverse,"
       "silenceremove=start_periods=1:start_threshold=-50dB:start_silence=0.08,areverse,"
       "acompressor=threshold=0.125:ratio=4:attack=5:release=80,aresample=48000")
# peaks to -2.5 dBFS: Opus decoding overshoots by up to about 2 dB (-1 dBFS measured +1.1 dBTP)
POST = "alimiter=limit=0.75:attack=5:release=50:level=false,apad=pad_dur=0.1"
OPUS = ["-ar", "48000", "-ac", "1", "-c:a", "libopus", "-b:a", "64k", "-map_metadata", "-1"]
# two noise bursts 75 ms apart, band-limited: a camera shutter
CLICK = ("aevalsrc='(random(0)*2-1)*(exp(-t*350)*lt(t,0.02)"
         "+0.6*exp(-(t-0.075)*450)*between(t,0.075,0.095))':s=48000:d=0.15,"
         "highpass=f=1000,lowpass=f=9000")

_REL_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_./-]*\.opus$")
_I_RE = re.compile(r'"input_i"\s*:\s*"([-0-9.inf]+)"')
APP2_WARNING = "Chiudi app2 mentre lo strumento lavora sul cane: i suoi task VP fermerebbero i nostri."


# ---- phrase table and plan ---------------------------------------------------------------
def load_rows(path=CSV_PATH):
    with open(path, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f, delimiter=";"))


def check_rel(rel):
    """A destination under /SDCARD/sound: a relative .opus path, no '..', no odd characters."""
    if not isinstance(rel, str) or not _REL_RE.match(rel) or ".." in rel or "//" in rel:
        raise ValueError("destination refused: %r" % (rel,))
    return rel


def dest_of(row):
    if row["where"] == "board":
        return "custom/%d.opus" % int(row["id"])
    if row["where"] != "sd":
        raise ValueError("id %s: where must be sd or board, got %r" % (row["id"], row["where"]))
    return check_rel(row["file"])


def find_recording(rec_dir, pid):
    for ext in REC_EXT:
        path = os.path.join(rec_dir, pid + ext)
        if os.path.isfile(path):
            return path
    return None


def plan(rows, rec_dir=REC_DIR):
    """[(id, dest, where, (kind, arg))] for every row that gets a file. kind is rec (arg: the
    recording), tts (arg: the Italian line), silence or click (arg: None)."""
    out, seen = [], set()
    for row in rows:
        pid, choice = row["id"].strip(), row["choice"].strip()
        if choice not in CHOICES:
            raise ValueError("id %s: unknown choice %r" % (pid, choice))
        if pid in seen:
            raise ValueError("id %s is listed twice" % pid)
        seen.add(pid)
        if choice == "keep":
            continue
        rec = find_recording(rec_dir, pid)
        if rec:
            src = ("rec", rec)
        elif choice in ("rec", "tts"):
            text = row["it"].strip()
            if not text:
                raise ValueError("id %s: choice %s needs a line in column it" % (pid, choice))
            src = ("tts", text)
        else:
            src = (choice, None)
        out.append((pid, dest_of(row), row["where"], src))
    dests = [p[1] for p in out]
    if len(set(dests)) != len(dests):
        raise ValueError("two rows share one destination file")
    return out


# ---- audio ------------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _load_json(path, default):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return default


def _save_json(path, data):
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1, sort_keys=True)
    os.replace(path + ".tmp", path)


def run_tts(voice, job_path):
    subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", TTS_PS1,
                    "-Voice", voice, "-Job", job_path], check=True)


def tts_wavs(items, voice, cache_root=TTS_DIR, runner=run_tts):
    """{id: wav} for [(id, text)]. Speaks, in one PowerShell run, only the lines that are new or
    whose text changed since the cache index recorded them."""
    folder = os.path.join(cache_root, voice.lower())
    os.makedirs(folder, exist_ok=True)
    index_path = os.path.join(folder, "index.json")
    index = _load_json(index_path, {})
    wav = {pid: os.path.join(folder, pid + ".wav") for pid, _ in items}
    todo = [(pid, text) for pid, text in items if index.get(pid) != text or not os.path.isfile(wav[pid])]
    if todo:
        print("Sintesi vocale (%s): %d frasi..." % (voice, len(todo)), flush=True)
        job_path = os.path.join(folder, "job.json")
        with open(job_path, "w", encoding="utf-8") as f:
            json.dump([{"text": text, "out": wav[pid]} for pid, text in todo], f, ensure_ascii=False)
        runner(voice, job_path)
        os.remove(job_path)
        index.update(dict(todo))
        _save_json(index_path, index)
    return wav


def _ffmpeg(args, ffmpeg):
    r = subprocess.run([ffmpeg, "-hide_banner", "-nostdin"] + args,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    err = r.stderr.decode("utf-8", "replace")
    if r.returncode != 0:
        raise RuntimeError("ffmpeg failed (%d): %s" % (r.returncode, err[-800:]))
    return err


def loudness(src, ffmpeg=FFMPEG):
    """Integrated loudness (LUFS) of src after PRE. loudnorm prints its JSON among other text."""
    err = _ffmpeg(["-nostats", "-i", src, "-af", PRE + ",loudnorm=print_format=json", "-f", "null", "-"],
                  ffmpeg)
    m = _I_RE.search(err)
    if not m or "inf" in m.group(1):
        raise RuntimeError("no loudness measured for %s (silent file?)" % src)
    return float(m.group(1))


def encode(src, out, ffmpeg=FFMPEG):
    """One recording or speech file to the dog's format at TARGET_LUFS."""
    gain = TARGET_LUFS - loudness(src, ffmpeg)
    tmp = out[:-len(".opus")] + ".tmp.opus"
    _ffmpeg(["-y", "-loglevel", "error", "-i", src,
             "-af", "%s,volume=%.2fdB,%s" % (PRE, gain, POST)] + OPUS + [tmp], ffmpeg)
    os.replace(tmp, out)


def synth(kind, out, ffmpeg=FFMPEG):
    """The fixed sounds: 0.3 s of silence, or the shutter click."""
    src = ["-t", "0.3", "-f", "lavfi", "-i", "anullsrc=r=48000:cl=mono"] if kind == "silence" \
        else ["-f", "lavfi", "-i", CLICK]
    tmp = out[:-len(".opus")] + ".tmp.opus"
    _ffmpeg(["-y", "-loglevel", "error"] + src + OPUS + [tmp], ffmpeg)
    os.replace(tmp, out)


def build(rows, voice, out_dir, rec_dir=REC_DIR, tts_dir=TTS_DIR, ids=None, ffmpeg=FFMPEG,
          tts_runner=run_tts):
    """Writes <out_dir>/<id>.opus for every planned row (ids: only these) and the manifest.
    A file is encoded again only when its source, its kind or CHAIN_VERSION changed."""
    items = plan(rows, rec_dir)
    if ids:
        items = [it for it in items if it[0] in ids]
    os.makedirs(out_dir, exist_ok=True)
    wavs = tts_wavs([(pid, src[1]) for pid, _, _, src in items if src[0] == "tts"],
                    voice, tts_dir, tts_runner)
    man_path = os.path.join(out_dir, "manifest.json")
    files = _load_json(man_path, {}).get("files", {})
    if not ids:                                # rows now keep, or gone: drop their files
        planned = set(it[0] for it in items)
        for pid in [p for p in files if p not in planned]:
            del files[pid]
            if os.path.exists(os.path.join(out_dir, pid + ".opus")):
                os.remove(os.path.join(out_dir, pid + ".opus"))
    made = 0
    try:
        for n, (pid, dest, where, (kind, arg)) in enumerate(items, 1):
            src = arg if kind == "rec" else wavs.get(pid)
            key = hashlib.sha256(("%d|%s|%s" % (CHAIN_VERSION, kind, sha256_file(src) if src else ""))
                                 .encode()).hexdigest()
            out = os.path.join(out_dir, pid + ".opus")
            old = files.get(pid)
            if (old and old["key"] == key and old["dest"] == dest and os.path.isfile(out)
                    and sha256_file(out) == old["sha256"]):
                continue
            if kind in ("rec", "tts"):
                encode(src, out, ffmpeg)
            else:
                synth(kind, out, ffmpeg)
            files[pid] = {"dest": dest, "where": where, "source": kind, "key": key,
                          "sha256": sha256_file(out), "bytes": os.path.getsize(out)}
            made += 1
            if made % 20 == 0:
                print("  %d/%d" % (n, len(items)), flush=True)
    finally:
        _save_json(man_path, {"voice": voice, "chain": CHAIN_VERSION, "files": files})
    counts = {}
    for _, _, _, (kind, _) in items:
        counts[kind] = counts.get(kind, 0) + 1
    print("Build in %s: %d file, %d rifatti. Fonti: %s" % (
        out_dir, len(items), made, ", ".join("%s %d" % kv for kv in sorted(counts.items()))))
    return files


# ---- sound.toml -------------------------------------------------------------------------
def _toml_line(lines, key):
    for i, line in enumerate(lines):
        m = re.match(r"^%s\s*=\s*\[(.*)\]\s*$" % key, line)
        if m:
            return i, m.group(1)
    raise ValueError("sound.toml has no %s line" % key)


def merge_toml(text, entries):
    """text with the (id, name) pairs appended to tts_ids/tts_names; line ends kept. A pair whose
    id is listed already is skipped when its name matches, refused when it does not."""
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.split(nl)
    i_ids, ids_s = _toml_line(lines, "tts_ids")
    i_names, names_s = _toml_line(lines, "tts_names")
    ids = [int(x) for x in ids_s.split(",") if x.strip()]
    names = re.findall(r'"([^"]*)"', names_s)
    if len(ids) != len(names):
        raise ValueError("sound.toml: %d tts_ids but %d tts_names" % (len(ids), len(names)))
    for pid, name in entries:
        if pid in ids:
            if names[ids.index(pid)] != name:
                raise ValueError("id %d is in sound.toml already, as %s" % (pid, names[ids.index(pid)]))
            continue
        ids.append(pid)
        names.append(check_rel(name))
    lines[i_ids] = "tts_ids = [%s]" % ", ".join(str(x) for x in ids)
    lines[i_names] = "tts_names = [%s]" % ", ".join('"%s"' % x for x in names)
    return nl.join(lines)


# ---- dog bodies -------------------------------------------------------------------------
# The engine puts each body after a header that imports os, sys, time, threading and creates
# `cyberdog`, then runs pyflakes on the file: any warning deletes it (cyberdog_sdk/vp.py).
# Bodies import what they use, use every name they bind inside a function, and keep pc_ names.
STATUS_WORK = """\
def pc_sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()
pc_files = {}
for pc_dir, pc_subs, pc_names in os.walk(@SOUND@):
    for pc_n in pc_names:
        pc_p = os.path.join(pc_dir, pc_n)
        pc_files[os.path.relpath(pc_p, @SOUND@)] = [os.path.getsize(pc_p), pc_sha(pc_p)]
pc_rep['files'] = pc_files
pc_rep['orig'] = os.path.isdir(@ORIG@)
pc_st = os.statvfs(@SOUND@)
pc_rep['free'] = pc_st.f_bavail * pc_st.f_frsize
"""

BACKUP_WORK = """\
pc_rep['existed'] = os.path.isdir(@ORIG@)
if not pc_rep['existed']:
    if os.path.exists(@ORIG@ + '.tmp'):
        shutil.rmtree(@ORIG@ + '.tmp')
    shutil.copytree(@SOUND@, @ORIG@ + '.tmp', symlinks=True)
    os.rename(@ORIG@ + '.tmp', @ORIG@)
pc_n = 0
for pc_dir, pc_subs, pc_names in os.walk(@ORIG@):
    pc_n += len(pc_names)
pc_rep['count'] = pc_n
with open(os.path.join(@ORIG@, @TOML@), 'rb') as pc_f:
    pc_rep['toml'] = pc_f.read().decode('utf-8')
"""

PUSH_WORK = """\
if not os.path.isdir(@ORIG@):
    raise RuntimeError('no backup in ' + @ORIG@ + ': run backup first')
pc_done = []
for pc_rel, pc_sum, pc_b64 in @FILES@:
    pc_data = base64.b64decode(pc_b64)
    if hashlib.sha256(pc_data).hexdigest() != pc_sum:
        raise RuntimeError('checksum mismatch on ' + pc_rel)
    pc_dst = os.path.join(@SOUND@, pc_rel)
    pc_parent = os.path.dirname(pc_dst)
    if not os.path.isdir(pc_parent):
        os.makedirs(pc_parent)
        os.chmod(pc_parent, 0o755)
    with open(pc_dst + '.tmp', 'wb') as pc_f:
        pc_f.write(pc_data)
    os.chmod(pc_dst + '.tmp', 0o644)
    os.rename(pc_dst + '.tmp', pc_dst)
    with open(pc_dst, 'rb') as pc_f:
        pc_done.append([pc_rel, hashlib.sha256(pc_f.read()).hexdigest()])
pc_rep['done'] = pc_done
"""

TOML_WORK = """\
if not os.path.isdir(@ORIG@):
    raise RuntimeError('no backup in ' + @ORIG@ + ': run backup first')
pc_dst = os.path.join(@SOUND@, @TOML@)
with open(pc_dst + '.tmp', 'wb') as pc_f:
    pc_f.write(@TEXT@.encode('utf-8'))
os.chmod(pc_dst + '.tmp', os.stat(pc_dst).st_mode & 0o777)
os.rename(pc_dst + '.tmp', pc_dst)
pc_rep['written'] = True
"""

PLAY_WORK = """\
pc_rep['result'] = str(cyberdog.audio.offline_play(@ID@, -1))
"""

RESTORE_WORK = """\
if not os.path.isdir(@ORIG@):
    raise RuntimeError('no backup in ' + @ORIG@)
pc_back = []
for pc_dir, pc_subs, pc_names in os.walk(@ORIG@):
    for pc_n in pc_names:
        pc_src = os.path.join(pc_dir, pc_n)
        pc_rel = os.path.relpath(pc_src, @ORIG@)
        pc_dst = os.path.join(@SOUND@, pc_rel)
        if os.path.isfile(pc_dst) and filecmp.cmp(pc_src, pc_dst, shallow=False):
            continue
        if not os.path.isdir(os.path.dirname(pc_dst)):
            os.makedirs(os.path.dirname(pc_dst))
        shutil.copy2(pc_src, pc_dst + '.tmp')
        os.rename(pc_dst + '.tmp', pc_dst)
        pc_back.append(pc_rel)
pc_rep['restored'] = pc_back
"""

IMPORTS = {"status": ("hashlib",), "backup": ("shutil",), "push": ("base64", "hashlib"),
           "toml": (), "play": (), "restore": ("filecmp", "shutil")}
WORK = {"status": STATUS_WORK, "backup": BACKUP_WORK, "push": PUSH_WORK, "toml": TOML_WORK,
        "play": PLAY_WORK, "restore": RESTORE_WORK}


def body(op, report, sound=SOUND, orig=ORIG, **values):
    """The full body of one dog step. Every @KEY@ in the work becomes repr(value); the report
    goes to `report` as .tmp, then renamed, so 4004 never sends a half-written file."""
    values.update(SOUND=sound, ORIG=orig, TOML=TOML_REL)
    work = WORK[op]
    missing = set(re.findall(r"@([A-Z]+)@", work)) - set(values)
    if missing:
        raise ValueError("body %s: no value for %s" % (op, sorted(missing)))
    for key, value in values.items():
        work = work.replace("@%s@" % key, repr(value))
    parts = ["import json\n"] + ["import %s\n" % m for m in IMPORTS[op]]
    parts += ["pc_rep = {'op': %r, 'started': time.time()}\n" % op, "try:\n",
              textwrap.indent(work, "    "),
              "except Exception as e:\n    pc_rep['error'] = repr(e)\n",
              "pc_rep['finished'] = time.time()\n",
              "with open(%r, 'w') as pc_f:\n    pc_f.write(json.dumps(pc_rep))\n" % (report + ".tmp"),
              "os.rename(%r, %r)\n" % (report + ".tmp", report)]
    return "".join(parts)


def batches(entries, limit=BATCH_BYTES):
    """Splits [(rel, data)] into lists of at most `limit` bytes (a bigger file goes alone)."""
    out, cur, size = [], [], 0
    for rel, data in entries:
        if cur and size + len(data) > limit:
            out.append(cur)
            cur, size = [], 0
        cur.append((rel, data))
        size += len(data)
    if cur:
        out.append(cur)
    return out


# ---- dog steps --------------------------------------------------------------------------
class Dog:
    def __init__(self, host):
        from cyberdog_sdk.transport import Transport
        self.t = Transport(host=host)
        self.t.connect(timeout=8.0)

    def run(self, op, describe, timeout=REPORT_TIMEOUT, **values):
        from cyberdog_sdk import vp
        tid = vp.new_task_id("voice")
        vp.send_task(self.t, vp.debug_task(body(op, vp.CAMERA_DIR + tid + ".json", **values),
                                           describe, tid))
        rep = vp._fetch_json(self.t, tid + ".json", timeout=timeout, poll=2.0)
        if rep.get("error"):
            raise RuntimeError("il cane ha risposto con un errore (%s): %s" % (op, rep["error"]))
        return rep

    def close(self):
        self.t.close()


def load_manifest(out_dir):
    man = _load_json(os.path.join(out_dir, "manifest.json"), None)
    if not man:
        raise SystemExit("Nessun build in %s: esegui prima build." % out_dir)
    for entry in man["files"].values():
        check_rel(entry["dest"])
    return man["files"]


def board_entries(files):
    return sorted((int(pid), e["dest"]) for pid, e in files.items() if e["where"] == "board")


def cmd_backup(dog, args):
    rep = dog.run("backup", "pc voice backup", timeout=600.0)
    os.makedirs(os.path.dirname(BACKUP_TOML), exist_ok=True)
    with open(BACKUP_TOML, "w", encoding="utf-8", newline="") as f:
        f.write(rep["toml"])
    print("%s %s (%d file). sound.toml originale salvato in %s" % (
        "Backup gia presente in" if rep["existed"] else "Backup creato in", ORIG, rep["count"],
        os.path.relpath(BACKUP_TOML, ROOT)))


def cmd_push(dog, args):
    files = load_manifest(args.dir)
    st = dog.run("status", "pc voice status")
    if not st["orig"]:
        raise SystemExit("Sul cane non c'e il backup %s: esegui prima backup." % ORIG)
    todo = []
    for pid, e in sorted(files.items(), key=lambda kv: int(kv[0])):
        if args.ids and pid not in args.ids:
            continue
        if st["files"].get(e["dest"], [0, ""])[1] != e["sha256"]:
            with open(os.path.join(args.dir, pid + ".opus"), "rb") as f:
                data = f.read()
            if hashlib.sha256(data).hexdigest() != e["sha256"]:
                raise SystemExit("%s.opus non corrisponde al manifest: esegui di nuovo build." % pid)
            todo.append((e["dest"], data))
    if not todo:
        print("Il cane ha gia tutti i file del build.")
        return
    groups = batches(todo, args.batch)
    print("Invio %d file (%d KB) in %d task..." % (len(todo), sum(len(d) for _, d in todo) // 1024,
                                                    len(groups)), flush=True)
    for n, group in enumerate(groups, 1):
        payload = [(rel, hashlib.sha256(d).hexdigest(), base64.b64encode(d).decode("ascii"))
                   for rel, d in group]
        rep = dog.run("push", "pc voice push %d/%d" % (n, len(groups)), FILES=payload)
        if sorted(map(tuple, rep["done"])) != sorted((rel, s) for rel, s, _ in payload):
            raise RuntimeError("task %d: il cane ha scritto %r" % (n, rep["done"]))
        print("  task %d/%d: %d file scritti" % (n, len(groups), len(group)), flush=True)
    print("Fatto. I file in /SDCARD/sound valgono dalla prossima riproduzione; gli id integrati "
          "(custom/) solo dopo toml e un riavvio.")


def cmd_toml(dog, args):
    files = load_manifest(args.dir)
    if not os.path.isfile(BACKUP_TOML):
        raise SystemExit("Manca %s: esegui prima backup." % os.path.relpath(BACKUP_TOML, ROOT))
    with open(BACKUP_TOML, encoding="utf-8", newline="") as f:
        original = f.read()
    entries = board_entries(files)
    merged = merge_toml(original, entries)
    st = dog.run("status", "pc voice status")
    missing = [d for pid, d in entries if st["files"].get(d, [0, ""])[1] != files[str(pid)]["sha256"]]
    if missing:
        raise SystemExit("Sul cane mancano %d file custom/ (%s...): esegui prima push."
                         % (len(missing), missing[0]))
    current = st["files"].get(TOML_REL, [0, ""])[1]
    if current == sha256_text(merged):
        print("sound.toml sul cane e gia aggiornato (%d id integrati)." % len(entries))
        return
    if current != sha256_text(original):
        raise SystemExit("sound.toml sul cane non e ne l'originale ne quello del build: non lo tocco. "
                         "Controlla con status, o usa restore.")
    dog.run("toml", "pc voice toml", TEXT=merged)
    print("sound.toml aggiornato con %d id integrati. Riavvia il cane (tasto di accensione): "
          "cyberdog_audio legge sound.toml solo all'avvio." % len(entries))


def cmd_play(dog, args):
    rep = dog.run("play", "pc voice play %d" % args.id, ID=args.id)
    if "code = 7949" in rep["result"]:
        # the VP call to the audio service times out, yet the dog plays the prompt (seen in syslog)
        print("Inviato %d. Nota: il servizio audio non ha confermato in tempo (codice 7949), "
              "ma di solito la frase viene riprodotta lo stesso." % args.id)
    else:
        print("Riprodotto %d: %s" % (args.id, rep["result"]))


def cmd_status(dog, args):
    st = dog.run("status", "pc voice status")
    print("Backup %s: %s. Spazio libero: %d MB." % (ORIG, "presente" if st["orig"] else "ASSENTE",
                                                    st["free"] // (1 << 20)))
    if os.path.isfile(BACKUP_TOML):
        with open(BACKUP_TOML, encoding="utf-8", newline="") as f:
            original = f.read()
        current = st["files"].get(TOML_REL, [0, ""])[1]
        print("sound.toml: %s." % ("originale" if current == sha256_text(original) else "modificato"))
    man = _load_json(os.path.join(args.dir, "manifest.json"), None)
    if not man:
        print("Nessun build in %s." % args.dir)
        return
    same, diff = 0, []
    for pid, e in sorted(man["files"].items(), key=lambda kv: int(kv[0])):
        if st["files"].get(e["dest"], [0, ""])[1] == e["sha256"]:
            same += 1
        else:
            diff.append(pid)
    print("Build %s: %d file uguali sul cane, %d diversi o mancanti%s" % (
        args.dir, same, len(diff), (": " + " ".join(diff[:30]) + (" ..." if len(diff) > 30 else ""))
        if diff else "."))


def cmd_restore(dog, args):
    rep = dog.run("restore", "pc voice restore", timeout=600.0)
    print("Ripristinati %d file da %s (custom/ resta, ma senza voce in sound.toml dopo il riavvio). "
          "Riavvia il cane se sound.toml era modificato." % (len(rep["restored"]), ORIG))


DOG_CMDS = {"backup": cmd_backup, "push": cmd_push, "toml": cmd_toml, "play": cmd_play,
            "status": cmd_status, "restore": cmd_restore}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["build"] + sorted(DOG_CMDS))
    ap.add_argument("id", nargs="?", type=int, help="play: the prompt id")
    ap.add_argument("--host", default=os.environ.get("DOG_IP", "192.168.137.115"))
    ap.add_argument("--dir", default=OUT_DIR, help="build folder (default voices/out)")
    ap.add_argument("--voice", default="Elsa", help="Windows Italian voice: Elsa or Cosimo")
    ap.add_argument("--ids", type=lambda s: set(x.strip() for x in s.split(",") if x.strip()),
                    help="build/push: only these ids, comma separated")
    ap.add_argument("--batch", type=int, default=BATCH_BYTES, help="push: bytes per task")
    args = ap.parse_args(argv)
    args.dir = os.path.abspath(args.dir)
    if args.cmd == "build":
        build(load_rows(), args.voice, args.dir, ids=args.ids)
        return
    if args.cmd == "play" and args.id is None:
        ap.error("play needs an id")
    print(APP2_WARNING, flush=True)
    dog = Dog(args.host)
    try:
        DOG_CMDS[args.cmd](dog, args)
    finally:
        dog.close()


if __name__ == "__main__":
    main()
