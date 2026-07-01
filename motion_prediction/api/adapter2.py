from __future__ import annotations
from typing import Optional, List, Tuple, Dict
from collections import deque
import time

from motion_prediction.core.engine import PredictiveEngine
from motion_prediction.models.gps_packet import GPSPacket

# Sliding window configuration
SPEED_WINDOW_SIZE = 12         # Number of packets to average over
SPEED_COLLECT_THRESHOLD = 8.0  # Only collect speeds above this (km/h)
SPEED_DIRECT_THRESHOLD = 16.0  # Above this, use raw GPS speed directly (km/h)
SPEED_STOP_THRESHOLD = 4.0     # Below this -> vehicle is stopped (km/h)
PRESET_AVG_SPEED_KMPH = 14.0   # Default avg speed until window is full (8-15 km/h range)


class MotionPredictionAdapter:

    def __init__(self, enabled: bool = True):
        self._engine = PredictiveEngine(enabled=enabled)
        # Per-device sliding window: stores last N valid speeds in km/h
        self._speed_windows: Dict[str, deque] = {}

    
    # Feature toggle
    

    def enable(self) -> None:
        self._engine.set_enabled(True)

    def disable(self) -> None:
        self._engine.set_enabled(False)

    def remove_vehicle(self, device_id: str) -> None:
        """Delete the predictor for a device so a fresh one is created on
        the next GPS packet."""
        self._engine.remove_vehicle(device_id)
        self._speed_windows.pop(device_id, None)

    
    # Sliding window speed helpers
    

    def _get_speed_window(self, device_id: str) -> deque:
        """Get or create the sliding window for a device."""
        if device_id not in self._speed_windows:
            self._speed_windows[device_id] = deque(maxlen=SPEED_WINDOW_SIZE)
        return self._speed_windows[device_id]

    def _get_avg_speed_kmph(self, device_id: str) -> float:
        """Return the current average speed for a device.
        If the window is full, return the mean of the window.
        Otherwise, return the preset average speed (12 km/h).
        """
        window = self._get_speed_window(device_id)
        if len(window) >= SPEED_WINDOW_SIZE:
            return sum(window) / len(window)
        return PRESET_AVG_SPEED_KMPH

    
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

            # Speed handling (GPS sends km/h)
            speed_mps = None
            if speed is not None:
                speed_val = float(speed)

                # STOP: GPS speed below 4 km/h -> force to 0
                if speed_val < SPEED_STOP_THRESHOLD:
                    speed_mps = 0.0

                # HIGH SPEED: >= 16 km/h -> use raw GPS speed directly
                elif speed_val >= SPEED_DIRECT_THRESHOLD:
                    speed_mps = speed_val / 3.6

                # MODERATE: 8-15 km/h -> use window avg or preset 
                elif speed_val > SPEED_COLLECT_THRESHOLD:
                    window = self._get_speed_window(device_id)
                    window.append(speed_val)
                    avg_kmph = self._get_avg_speed_kmph(device_id)
                    speed_mps = avg_kmph / 3.6

                # LOW: 4-8 km/h -> use actual GPS speed
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
            # Try 5-tuple first (new API)
            lat, lon, confidence, speed_mps, is_off_route = result
        except ValueError:
            # Fallback for 4-tuple
            lat, lon, confidence, speed_mps = result
            is_off_route = False

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
            
            # Keep these for backward compatibility with older UI clients 
            # if they haven't updated to standard schema yet
            "lat": lat,
            "lon": lon,
            "speed_kmph": round(speed_kmph, 2),
        }