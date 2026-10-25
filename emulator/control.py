"""HTTP control and inspection endpoint for tests (contracts.md 4.2).

GET /state; POST /reset, /fault {"dep","mode"}, /battery {"soc","charging"[,"wireless"]},
/led {"client","target","occupation","mode","effect","r","g","b"} (another LED client, e.g. bms).
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FAULT_MODES = ("ok", "absent", "timeout", "hang")


class Control:
    def __init__(self, emu, host, port):
        self.emu = emu
        ctl = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, obj):
                body = json.dumps(obj, ensure_ascii=False, default=str).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.split("?")[0] == "/state":
                    return self._send(200, ctl.emu.state())
                self._send(404, {"error": "unknown path"})

            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                    body = json.loads(self.rfile.read(n) or b"{}")
                    if not isinstance(body, dict):
                        raise ValueError("body must be a JSON object")
                    code, out = ctl.post(self.path.split("?")[0], body)
                except (ValueError, KeyError, TypeError) as e:
                    code, out = 400, {"error": str(e)}
                self._send(code, out)

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, name="control", daemon=True).start()

    def post(self, path, b):
        emu = self.emu
        if path == "/reset":
            emu.reset()
            return 200, {"ok": True}
        if path == "/fault":
            if b.get("mode") not in FAULT_MODES or not isinstance(b.get("dep"), str):
                raise ValueError("need dep (ROS service name) and mode in %s" % (FAULT_MODES,))
            emu.dep(b["dep"]).set(b["mode"])
            return 200, {"ok": True, "faults": emu.faults()}
        if path == "/battery":
            d = emu.device
            if "soc" in b:
                d.soc = max(0, min(100, int(b["soc"])))
            if "charging" in b:
                d.charging = bool(b["charging"])
            if "wireless" in b:
                d.wireless = bool(b["wireless"])
            return 200, {"ok": True, "soc": d.soc, "charging": d.charging, "wireless": d.wireless}
        if path == "/led":
            code = emu.vp.leds.execute(bool(b.get("occupation", True)), str(b.get("client", "bms")),
                                       int(b["target"]), int(b.get("mode", 1)), int(b.get("effect", 0xA1)),
                                       int(b.get("r", 0)), int(b.get("g", 0)), int(b.get("b", 0)))
            return 200, {"ok": True, "code": code, "leds": emu.vp.leds.state()}
        return 404, {"error": "unknown path"}

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
