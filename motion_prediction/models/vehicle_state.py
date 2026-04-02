from __future__ import annotations
from dataclasses import dataclass
from motion_prediction.math.vector import Vec2


@dataclass(frozen=True)
class BackgroundSnapshot:
    position: Vec2
    velocity: Vec2
    acceleration: Vec2
    confidence: float
    timestamp: float


@dataclass(frozen=True)
class DisplaySnapshot:
    position: Vec2
    heading_deg: float
    confidence: float
    timestamp: float
