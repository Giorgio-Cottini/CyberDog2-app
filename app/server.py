#!/usr/bin/env python3
# CyberDog 2 PC controller - local bridge.
#
# Two gRPC channels to the dog (both mTLS, SNI-overridden, certs from the
# MiRoboticsLab/bridges repo):
#   FORWARD : PC -> dog 192.168.137.115:50052  GrpcApp.sendMsg   (we send commands)
#   REVERSE : dog -> PC   0.0.0.0:8981          GrpcApp.sendMsg + heartbeat
#             (the dog pushes: WebRTC answer/ICE on 4001, servo resp 1003,
#              and heartbeat Ticks -> battery/wifi/motion_id)
#
# A local HTTP server (127.0.0.1:8793) serves the UI, takes commands by POST,
# relays WebRTC signalling, and streams telemetry + answers to the browser over
# Server-Sent Events. No third-party deps beyond grpcio.
#
# KINEMATICS (authoritative, from Xiaomi SDK cyberdog_vp_abilityset/src/sdk/motion.cpp
# and protocol/ros/msg/MotionID.msg):
#   Pose changes are ONE-SHOT result commands (nameCode 1004, ROS service
#   motion_result_cmd):
#       111 RECOVERYSTAND  stand up
#       101 GETDOWN        lie down
#       112 WALK_STAND     standing posture
#         0 ESTOP          emergency stop
#   Walking is a STREAMED servo command (nameCode 1002, ROS topic
#   motion_servo_cmd). You must publish Data frames continuously at ~50 Hz
#   (>=20 Hz required) and send one End frame to stop:
#       cmd_type   1 = Data frame, 2 = End frame
#       cmd_source 0 (app)
#       motion_id  303 WALK_USERTROT  (user trot gait; 304 = WALK_ADAPTIVELY)
#       vel_des    [x, y, yaw]  x,y <= 1.5 m/s ; yaw <= 2.0 rad/s
#       step_height[front, back] in m, recommended [0.05, 0.05]
#   A single 1004 cannot produce continuous walking; that is why movement uses
#   the 1002 servo stream, not 1004.

import collections
import json
import os
import queue
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
CDOG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(CDOG, "pylibs"))
sys.path.insert(0, os.path.join(CDOG, "proto"))

import grpc  # noqa: E402
import cyberdog_app_pb2 as pb  # noqa: E402
import cyberdog_app_pb2_grpc as rpc  # noqa: E402

# ---- config -----------------------------------------------------------------
DOG_IP = os.environ.get("DOG_IP", "192.168.137.115")
DOG_PORT = os.environ.get("DOG_PORT", "50052")
REVERSE_PORT = os.environ.get("DOG_REVERSE_PORT", "8981")
HTTP_HOST = os.environ.get("DOG_UI_HOST", "127.0.0.1")
HTTP_PORT = int(os.environ.get("DOG_UI_PORT", "8793"))
CERTS = os.path.join(CDOG, "certs")

# Gentle manual-drive velocities (well under the 1.5 m/s / 2.0 rad/s limits).
VX = float(os.environ.get("DOG_VX", "0.2"))      # forward/back  m/s
VY = float(os.environ.get("DOG_VY", "0.15"))     # strafe        m/s
VYAW = float(os.environ.get("DOG_VYAW", "0.4"))  # turn          rad/s
STEP_H = float(os.environ.get("DOG_STEP_H", "0.05"))  # leg lift  m
GAIT = int(os.environ.get("DOG_GAIT", "303"))    # WALK_USERTROT
SERVO_HZ = 50
# Camera grabber pool. The dog's gRPC file service allows up to 5 concurrent
# transfers (TransmitFiles::thread_counts_ > 5 -> busy). Four parallel pullers
# roughly double the still-frame rate vs one serial puller (~1 fps -> ~2 fps),
# measured against the dog. Frames stay full resolution (the gRPC 4002 handler
# ignores width/height), so this raises frame RATE, not resolution.
NUM_GRABBERS = int(os.environ.get("DOG_GRABBERS", "4"))

# Motion IDs
ID_ESTOP, ID_GETDOWN, ID_STAND, ID_WALK_STAND = 0, 101, 111, 112
# Camera (nameCode 4002) commands
CAM_TAKE_PICTURE, CAM_START_STREAM, CAM_STOP_STREAM = 1, 7, 8


def _read(p):
    with open(p, "rb") as f:
        return f.read()


# ---- shared state -----------------------------------------------------------
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.safety_on = False          # movement gate; OFF by default (charger)
        self.vx = self.vy = self.vyaw = 0.0
        self.walking = False
        self.streaming = False          # camera live stream active
        self.telemetry = {}
        self.subscribers = []           # list[queue.Queue] for SSE

    def publish(self, event):
        data = json.dumps(event)
        with self.lock:
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait(data)
            except Exception:
                pass


ST = State()


# ---- forward channel (commands) --------------------------------------------
def _make_channel():
    creds = grpc.ssl_channel_credentials(
        root_certificates=_read(os.path.join(CERTS, "ca-cert.pem")),
        private_key=_read(os.path.join(CERTS, "client-key.pem")),
        certificate_chain=_read(os.path.join(CERTS, "client-cert.pem")),
    )
    opts = (("grpc.ssl_target_name_override", "cyberdog2.server"),
            ("grpc.default_authority", "cyberdog2.server"),
            ("grpc.max_receive_message_length", 32 * 1024 * 1024))
    return grpc.secure_channel("%s:%s" % (DOG_IP, DOG_PORT), creds, options=opts)


class Forward:
    def __init__(self):
        # Control channel for commands, and a separate channel for camera file
        # pulls so a ~1 s snapshot never blocks a stand/lie/estop command.
        self.ch = _make_channel()
        self.stub = rpc.GrpcAppStub(self.ch)
        self.lock = threading.Lock()
        self.fch = _make_channel()
        self.fstub = rpc.GrpcAppStub(self.fch)
        self.flock = threading.Lock()

    def send(self, name_code, params, timeout=8):
        req = pb.SendRequest(nameCode=name_code, params=json.dumps(params))
        out = []
        with self.lock:
            for rsp in self.stub.sendMsg(req, timeout=timeout):
                out.append(rsp.data)
        return out

    def get_file(self, name_code, params, timeout=30):
        req = pb.SendRequest(nameCode=name_code, params=json.dumps(params))
        buf = bytearray()
        name = err = None
        with self.flock:
            for chunk in self.fstub.getFile(req, timeout=timeout):
                err = chunk.error_code
                name = chunk.file_name
                if chunk.buffer:
                    buf += chunk.buffer
        return err, name, bytes(buf)


FWD = Forward()


# ---- camera grabber pool (parallel full-res stills -> MJPEG) ----------------
class Grabber:
    # N worker threads, each on its OWN gRPC channel so they are not serialized
    # by a shared lock. Each loops TAKE_PICTURE over getFile and publishes the
    # newest JPEG to a single latest-frame slot guarded by a condition. The
    # MJPEG endpoint and /snapshot read that slot; a new frame wakes any waiter.
    def __init__(self, n):
        self.n = max(1, n)
        self.want = False
        self.lock = threading.Lock()
        self.cond = threading.Condition()
        self.latest = b""
        self.gen = 0
        self.fps = 0.0
        self._stamps = collections.deque(maxlen=12)
        self._channels = [_make_channel() for _ in range(self.n)]
        self._stubs = [rpc.GrpcAppStub(c) for c in self._channels]

    def start(self):
        with self.lock:
            if self.want:
                return
            self.want = True
        try:
            FWD.send(4002, {"command": CAM_START_STREAM})
        except Exception as e:
            ST.publish({"type": "log", "msg": "grab start-stream: %s" % e})
        with ST.lock:
            ST.streaming = True
        for i in range(self.n):
            threading.Thread(target=self._work, args=(i,), daemon=True).start()
        ST.publish({"type": "log", "msg": "grabber pool started (%d workers)" % self.n})

    def stop(self):
        with self.lock:
            self.want = False
        try:
            FWD.send(4002, {"command": CAM_STOP_STREAM})
        except Exception:
            pass
        with ST.lock:
            ST.streaming = False
        ST.publish({"type": "log", "msg": "grabber pool stopped"})

    def _work(self, i):
        stub = self._stubs[i]
        while True:
            with self.lock:
                if not self.want:
                    return
            try:
                req = pb.SendRequest(nameCode=4002,
                                     params=json.dumps({"command": CAM_TAKE_PICTURE}))
                buf = bytearray()
                err = None
                for chunk in stub.getFile(req, timeout=30):
                    err = chunk.error_code
                    if chunk.buffer:
                        buf += chunk.buffer
                if err == 5:  # RESULT_INVALID_STATE: camera dropped streaming; re-arm
                    try:
                        FWD.send(4002, {"command": CAM_START_STREAM})
                    except Exception:
                        pass
                    time.sleep(0.2)
                    continue
                if buf and err in (0, None):
                    data = bytes(buf)
                    with self.cond:
                        self.latest = data
                        self.gen += 1
                        now = time.time()
                        self._stamps.append(now)
                        if len(self._stamps) >= 2:
                            self.fps = (len(self._stamps) - 1) / (self._stamps[-1] - self._stamps[0])
                        self.cond.notify_all()
                else:
                    time.sleep(0.1)
            except Exception:
                time.sleep(0.2)

    def wait_frame(self, last_gen, timeout):
        with self.cond:
            if self.gen == last_gen:
                self.cond.wait(timeout)
            return self.latest, self.gen


GRAB = Grabber(NUM_GRABBERS)


# ---- servo streaming loop (accurate walking kinematics) ---------------------
def servo_loop():
    period = 1.0 / SERVO_HZ
    was_walking = False
    while True:
        with ST.lock:
            want = ST.walking and ST.safety_on
            vx, vy, vyaw = ST.vx, ST.vy, ST.vyaw
        try:
            if want:
                FWD.send(1002, {
                    "motion_id": GAIT,
                    "cmd_type": 1,            # Data frame
                    "cmd_source": 0,
                    "vel_des": [vx, vy, vyaw],
                    "step_height": [STEP_H, STEP_H],
                }, timeout=2)
                was_walking = True
            elif was_walking:
                # End frame: stop the gait cleanly.
                FWD.send(1002, {
                    "motion_id": GAIT,
                    "cmd_type": 2,            # End frame
                    "cmd_source": 0,
                }, timeout=2)
                was_walking = False
        except Exception as e:
            ST.publish({"type": "log", "msg": "servo: %s" % e})
        time.sleep(period)


# ---- reverse server (dog -> PC pushes) --------------------------------------
class GrpcAppServicer(rpc.GrpcAppServicer):
    def sendMsg(self, request, context):
        # The dog pushes a SendRequest; we relay the interesting ones and
        # return an empty response stream (the dog just drains it).
        try:
            code = request.nameCode
            params = request.params or ""
            if code == 4001:  # WebRTC signalling answer / ICE from the dog
                try:
                    obj = json.loads(params)
                except Exception:
                    obj = {}
                if "answer_sdp" in obj:
                    ST.publish({"type": "answer", "sdp": obj["answer_sdp"]})
                elif "c_sdp" in obj:
                    ST.publish({"type": "candidate", "c_sdp": obj["c_sdp"]})
                elif "error" in obj:
                    ST.publish({"type": "log", "msg": "dog webrtc error: %s" % obj["error"]})
            elif code == 1003:  # servo response
                ST.publish({"type": "servo_resp", "data": params})
        except Exception as e:
            ST.publish({"type": "log", "msg": "reverse sendMsg: %s" % e})
        return iter(())  # empty stream

    def heartbeat(self, request, context):
        t = request
        tel = {
            "battery": t.battery_power,
            "wifi": t.wifi_strength,
            "motion_id": t.motion_status.motion_id if t.HasField("motion_status") else None,
            "wired_charging": t.charging_status.wired_charging if t.HasField("charging_status") else None,
            "wireless_charging": t.charging_status.wireless_charging if t.HasField("charging_status") else None,
            "sn": t.sn,
            "ip": t.ip,
        }
        with ST.lock:
            ST.telemetry = tel
        ST.publish({"type": "telemetry", "tel": tel})
        return pb.Result(result="SUCCESS")


def start_reverse_server():
    server = grpc.server(futures_thread_pool())
    rpc.add_GrpcAppServicer_to_server(GrpcAppServicer(), server)
    server_creds = grpc.ssl_server_credentials(
        [(_read(os.path.join(CERTS, "client-key.pem")),
          _read(os.path.join(CERTS, "client-cert.pem")))],
        root_certificates=_read(os.path.join(CERTS, "ca-cert.pem")),
        require_client_auth=True,
    )
    server.add_secure_port("0.0.0.0:%s" % REVERSE_PORT, server_creds)
    server.start()
    ST.publish({"type": "log", "msg": "reverse gRPC server on :%s" % REVERSE_PORT})
    return server


def futures_thread_pool():
    from concurrent import futures
    return futures.ThreadPoolExecutor(max_workers=8)


# ---- command dispatch -------------------------------------------------------
def dispatch(action, body):
    if action == "stand":
        return FWD.send(1004, {"motion_id": ID_STAND})
    if action == "lie":
        return FWD.send(1004, {"motion_id": ID_GETDOWN})
    if action == "stand_posture":
        return FWD.send(1004, {"motion_id": ID_WALK_STAND})
    if action == "estop":
        with ST.lock:
            ST.walking = False
            ST.vx = ST.vy = ST.vyaw = 0.0
        return FWD.send(1004, {"motion_id": ID_ESTOP})
    if action == "info":
        return FWD.send(1001, {"is_stand": True, "is_bat_info": True})
    if action == "volume":
        return FWD.send(1012, {"volume": int(body.get("value", 50))})
    if action == "audio":
        return FWD.send(1014, {"enable": bool(body.get("enable", True))})
    if action == "safety":
        with ST.lock:
            ST.safety_on = bool(body.get("enabled", False))
            if not ST.safety_on:
                ST.walking = False
                ST.vx = ST.vy = ST.vyaw = 0.0
            on = ST.safety_on
        return ["safety=%s" % on]
    if action == "move":
        # Browser sends the current velocity vector while keys are held.
        with ST.lock:
            ST.vx = max(-VX, min(VX, float(body.get("vx", 0.0))))
            ST.vy = max(-VY, min(VY, float(body.get("vy", 0.0))))
            ST.vyaw = max(-VYAW, min(VYAW, float(body.get("vyaw", 0.0))))
            ST.walking = bool(body.get("walking", False)) and ST.safety_on
            gated = not ST.safety_on
        return ["gated"] if gated else ["ok"]
    if action == "cam_start":
        GRAB.start()
        return ["ok"]
    if action == "cam_stop":
        GRAB.stop()
        return ["ok"]
    if action == "signal":
        # WebRTC signalling from the browser -> forward to the dog (4001).
        params = body.get("params", {})
        return FWD.send(4001, params)
    raise ValueError("unknown action %r" % action)


def take_snapshot():
    # The dog only produces a still once the camera is streaming, so make sure
    # the stream is on, then pull one full-res JPEG over the forward channel.
    with ST.lock:
        streaming = ST.streaming
    if not streaming:
        FWD.send(4002, {"command": CAM_START_STREAM})
        with ST.lock:
            ST.streaming = True
    return FWD.get_file(4002, {"command": CAM_TAKE_PICTURE}, timeout=30)


# ---- HTTP + SSE -------------------------------------------------------------
UI_PATH = os.path.join(HERE, "ui.html")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            with open(UI_PATH, "rb") as f:
                self._send(200, "text/html; charset=utf-8", f.read())
            return
        if self.path == "/events":
            self.handle_sse()
            return
        if self.path.startswith("/stream.mjpg"):
            self.handle_mjpeg()
            return
        if self.path.startswith("/snapshot"):
            # Long-poll: ?after=N blocks until the pool has a frame newer than N,
            # so the client gets exactly one transfer per new frame. after=-1
            # (or absent) returns the current frame at once. Browser renders each
            # JPEG as a blob <img>, which this pane does reliably (MJPEG did not).
            try:
                q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                after = int(q.get("after", ["-1"])[0])
                data, gen = GRAB.wait_frame(after, 10)
                err = 0
                if not data:            # pool idle: pull one still directly
                    err, name, data = take_snapshot()
                    gen = GRAB.gen
                if data and (err in (0, None)):
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Gen", str(gen))
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self._send(503, "application/json",
                               json.dumps({"ok": False, "error": "camera err %s" % err}).encode())
            except Exception as e:
                self._send(503, "application/json",
                           json.dumps({"ok": False, "error": str(e)}).encode())
            return
        self._send(404, "text/plain", b"not found")

    def handle_mjpeg(self):
        # multipart/x-mixed-replace: push each new pooled frame to the <img>.
        GRAB.start()  # idempotent: ensures the grabber pool is running
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            while True:
                frame, gen = GRAB.wait_frame(last, 15)
                if gen == last:           # 15 s with no new frame: keep waiting
                    continue
                last = gen
                if not frame:
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                    % len(frame))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except Exception:
            pass  # client disconnected; the pool keeps running until cam_stop

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(n) if n else b"{}"
        try:
            body = json.loads(raw or b"{}")
            action = body.get("action")
            result = dispatch(action, body)
            self._send(200, "application/json", json.dumps({"ok": True, "result": result}).encode())
        except Exception as e:
            self._send(200, "application/json", json.dumps({"ok": False, "error": str(e)}).encode())

    def handle_sse(self):
        q = queue.Queue(maxsize=256)
        with ST.lock:
            ST.subscribers.append(q)
            tel = dict(ST.telemetry)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self.wfile.write(b": connected\n\n")
            if tel:
                self.wfile.write(("data: %s\n\n" % json.dumps({"type": "telemetry", "tel": tel})).encode())
                self.wfile.flush()
            while True:
                try:
                    data = q.get(timeout=15)
                    self.wfile.write(("data: %s\n\n" % data).encode())
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                self.wfile.flush()
        except Exception:
            pass
        finally:
            with ST.lock:
                if q in ST.subscribers:
                    ST.subscribers.remove(q)


def telemetry_poll():
    # Battery/posture come back on the FORWARD channel via 1001, so we poll it
    # rather than depend on the dog-initiated reverse heartbeat (which needs a
    # cyberdog2.client cert we do not have).
    while True:
        try:
            out = FWD.send(1001, {"is_stand": True, "is_bat_info": True}, timeout=6)
            obj = json.loads(out[0]) if out else {}
            bat = obj.get("bat_info", {}) or {}
            tel = {
                "battery": bat.get("power"),
                "voltage": bat.get("voltage"),
                "temp": bat.get("temperature"),
                "wired_charging": bool(bat.get("is_charging")),
                "wireless_charging": None,
                "standing": obj.get("stand"),
                "wifi": None,
                "motion_id": None,
            }
            with ST.lock:
                ST.telemetry = tel
            ST.publish({"type": "telemetry", "tel": tel})
        except Exception as e:
            ST.publish({"type": "log", "msg": "poll: %s" % e})
        time.sleep(2.5)


def main():
    threading.Thread(target=servo_loop, daemon=True).start()
    threading.Thread(target=telemetry_poll, daemon=True).start()
    # The reverse server only becomes useful once we have a cert valid for the
    # name "cyberdog2.client" (needed for WebRTC video + the dog's heartbeat).
    # Until then it just makes the dog retry and logs TLS noise, so it is
    # opt-in: set DOG_REVERSE=1 to enable.
    if os.environ.get("DOG_REVERSE") == "1":
        try:
            rev = start_reverse_server()  # noqa: F841
        except Exception as e:
            print("WARN reverse server failed:", e)
    httpd = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), Handler)
    url = "http://%s:%d/" % (HTTP_HOST, HTTP_PORT)
    print("CyberDog controller on", url)
    print("dog forward %s:%s  reverse :%s" % (DOG_IP, DOG_PORT, REVERSE_PORT))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
