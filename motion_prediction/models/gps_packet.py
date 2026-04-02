from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GPSPacket:

    vehicle_id: str
    lat: float
    lon: float
    timestamp: float
    speed_mps: Optional[float] = None

    # -----------------------------
    # Validation helpers
    # -----------------------------

    def is_valid(self) -> bool:

        if not self.vehicle_id:
            return False

        if not (-90.0 <= self.lat <= 90.0):
            return False

        if not (-180.0 <= self.lon <= 180.0):
            return False

        if self.timestamp <= 0:
            return False

        if self.speed_mps is not None and self.speed_mps < 0:
            return False

        return True
