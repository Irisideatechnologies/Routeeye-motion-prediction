from __future__ import annotations
from typing import Optional, List, Tuple
import time

from motion_prediction.core.engine import PredictiveEngine
from motion_prediction.models.gps_packet import GPSPacket


class MotionPredictionAdapter:

    def __init__(self, enabled: bool = True):
        self._engine = PredictiveEngine(enabled=enabled)
        self._avg_speed_mps: Optional[float] = None  # System-provided avg speed

    
    # Feature toggle
    

    def enable(self) -> None:
        self._engine.set_enabled(True)

    def disable(self) -> None:
        self._engine.set_enabled(False)

    
    # System avg speed setter
    

    def set_avg_speed(self, avg_speed_kmph: float) -> None:

        self._avg_speed_mps = avg_speed_kmph / 3.6

    
    # GPS ingestion
    

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

            # Convert ms → seconds
            ts = timestamp / 1000.0

            # Speed handling (GPS sends kmph)
            speed_mps = None
            if speed is not None:
                speed_val = float(speed)

                # STOP threshold
                if speed_val < 3.0:
                    speed_mps = 0.0

                # Use raw GPS speed between 5–7 km/h
                elif 3.0 < speed_val <= 7.0:
                    speed_mps = 12.0 / 3.6

                # Above 7 km/h → use preset average speed if available
                elif speed_val > 7.0:
                    if self._avg_speed_mps is not None:
                        speed_mps = self._avg_speed_mps
                    else:
                        speed_mps = speed_val / 3.6

                # 3–7 km/h → use actual GPS speed (bus decelerating near stop)
                else:
                    speed_mps = speed_val / 3.6

            else:
                speed_mps = None

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

    
    # Route context management
    

    def update_route_context(
            self,
            device_id: str,
            route_polyline: Optional[List[Tuple[float, float]]],
            stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:

        self._engine.update_route_context(device_id, route_polyline, stops)

    
    # UI-facing query 
    

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

        try:
            lat, lon, confidence, speed_mps = result
        except ValueError:
            return None

        speed_mps = speed_mps if speed_mps is not None else 0.0
        speed_kmph = speed_mps * 3.6

        return {
            "lat": lat,
            "lon": lon,
            "confidence": confidence,
            "speed_kmph": speed_kmph,
            "timestamp": now_ts,
        }