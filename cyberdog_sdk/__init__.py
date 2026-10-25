"""CyberDog 2 SDK: gRPC client for the dog's GrpcApp service (docs/design/contracts.md §3)."""
from __future__ import annotations

from . import motions as catalog
from .client import CyberDog, MotionResult, Status
from .servo import Driver, Gait, Posture
from .transport import DogError, DogUnreachable, FileResult, ForbiddenCommand, RiskyMotion, Transport

__all__ = ["CyberDog", "Transport", "Driver", "Posture", "Gait", "MotionResult", "Status",
           "DogError", "DogUnreachable", "ForbiddenCommand", "RiskyMotion", "catalog", "FileResult"]
