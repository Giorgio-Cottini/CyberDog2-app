"""python -m emulator --port 50052 [--controller fake|lcm] [--time-scale 1.0] [--control-port 50080] [--host H]"""
from __future__ import annotations

import argparse
import signal
import sys

from emulator.server import Emulator


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m emulator", description="CyberDog 2 emulator (fake dog)")
    ap.add_argument("--port", type=int, default=50052)
    ap.add_argument("--host", default="127.0.0.1", help="bind address (0.0.0.0 inside WSL)")
    ap.add_argument("--controller", choices=("fake", "lcm"), default="fake")
    ap.add_argument("--time-scale", type=float, default=1.0, help="fake controller only")
    ap.add_argument("--control-port", type=int, default=50080)
    a = ap.parse_args(argv)
    if a.controller == "lcm" and a.time_scale != 1.0:
        print("emulator: --time-scale is ignored with --controller lcm (Gazebo sets the pace)", file=sys.stderr)
    try:
        emu = Emulator(port=a.port, host=a.host, controller=a.controller, time_scale=a.time_scale,
                       control_port=a.control_port)
    except ImportError as e:
        if a.controller != "lcm":
            raise
        print("emulator: --controller lcm needs python3-lcm, available only in the CyberDogSim VM (%s)" % e,
              file=sys.stderr)
        return 2
    port, cport = emu.start()
    print("emulator: gRPC mTLS %s:%d, control http://%s:%d, controller %s, time scale %g, files %s"
          % (a.host, port, a.host, cport, a.controller, emu.time_scale, emu.fs_root), flush=True)
    signal.signal(signal.SIGTERM, lambda *_: emu.stopping.set())
    try:
        while not emu.stopping.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        emu.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
