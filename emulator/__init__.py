"""CyberDog 2 emulator: the fake dog (docs/design/contracts.md section 4).

Start it with `python -m emulator`, or in-process with `emulator.server.Emulator`.
"""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

try:                                    # WSL: grpcio from pip3. Windows: grpcio from pylibs/.
    import grpc  # noqa: F401
except ImportError:
    sys.path.insert(0, os.path.join(ROOT, "pylibs"))
