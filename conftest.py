"""Root pytest config: put pylibs/ (Windows packages) and the repo root on sys.path.
pylibs/ holds Windows builds (grpcio's cygrpc is a .pyd): on Linux (WSL) it would shadow pip3's grpcio."""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
for p in ((os.path.join(ROOT, "pylibs"),) if os.name == "nt" else ()) + (ROOT,):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
