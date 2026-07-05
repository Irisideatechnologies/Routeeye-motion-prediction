from __future__ import annotations

from typing import Optional, List, Tuple
import time

from motion_prediction.core.kalman_engine import KalmanEngine
from motion_prediction.models.gps_packet import GPSPacket

# GPS speed below this (km/h) is treated as stopped
SPEED_STOP_THRESHOLD_KMPH = 4.0


class MotionPredictionAdapter:

    def __init__(self, enabled: bool = True):
        self._engine = KalmanEngine(enabled=enabled)

    def enable(self) -> None:
        self._engine.set_enabled(True)

    def disable(self) -> None:
        self._engine.set_enabled(False)

    def remove_vehicle(self, device_id: str) -> None:
        self._engine.remove_vehicle(device_id)

    def ingest_gps_packet(
        self,
        device_id: str,
        latitude: str,
        longitude: str,
        timestamp: int,
        speed: str | None = None,
    ) -> None:
        try:
            lat = float(latitude)
            lon = float(longitude)
            ts = timestamp / 1000.0

            speed_mps = None
            if speed is not None:
                speed_kmph = float(speed)
                if speed_kmph < SPEED_STOP_THRESHOLD_KMPH:
                    speed_mps = 0.0
                else:
                    speed_mps = speed_kmph / 3.6

        except (ValueError, TypeError):
            return

        packet = GPSPacket(
            vehicle_id=device_id,
            lat=lat,
            lon=lon,
            timestamp=ts,
            speed_mps=speed_mps,
        )

        if packet.is_valid():
            self._engine.ingest_gps(packet)

    def update_route_context(
        self,
        device_id: str,
        route_polyline: Optional[List[Tuple[float, float]]],
        stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:
        self._engine.update_route_context(device_id, route_polyline, stops)

    def get_display_position(
        self,
        vehicle_id: str,
        now_ts: Optional[float] = None,
    ) -> Optional[dict]:
        if now_ts is None:
            now_ts = time.time()

        result = self._engine.get_display_position(vehicle_id, now_ts)
        if result is None:
            return None

        lat, lon, confidence, speed_mps, is_off_route = result
        speed_mps = speed_mps if speed_mps is not None else 0.0
        speed_kmph = speed_mps * 3.6

        return {
            "device_id": vehicle_id,
            "latitude": str(lat),
            "longitude": str(lon),
            "speed": str(round(speed_kmph, 2)),
            "timestamp": int(now_ts * 1000),
            "confidence": confidence,
            "is_off_route": is_off_route,
            "lat": lat,
            "lon": lon,
            "speed_kmph": round(speed_kmph, 2),
        }
