from __future__ import annotations
from typing import Dict, Optional, List, Tuple
import time

from motion_prediction.core.predictor import VehiclePredictor
from motion_prediction.models.gps_packet import GPSPacket


class PredictiveEngine:

    def __init__(self, enabled: bool = True):
        self._enabled = enabled
        self._predictors: Dict[str, VehiclePredictor] = {}

    # -----------------------------
    # Feature toggle
    # -----------------------------

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def remove_vehicle(self, vehicle_id: str) -> None:
        """Delete the predictor for a vehicle so a fresh one is created on
        the next GPS packet.  Used when a device goes silent for too long
        and the accumulated state becomes stale."""
        self._predictors.pop(vehicle_id, None)

    # -----------------------------
    # GPS ingestion 
    # -----------------------------

    def ingest_gps(self, packet: GPSPacket) -> None:

        if not self._enabled:
            return

        predictor = self._predictors.get(packet.vehicle_id)

        if predictor is None:
            predictor = VehiclePredictor(packet)
            self._predictors[packet.vehicle_id] = predictor
            return

        predictor.ingest_gps(packet)

    # -----------------------------
    # Route context management
    # -----------------------------

    def update_route_context(
            self,
            device_id: str,
            route_polyline: Optional[List[Tuple[float, float]]],
            stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:

        predictor = self._predictors.get(device_id)

        if predictor is None:
            # No predictor yet - route will be set when first GPS arrives
            return

        predictor.update_route_context(route_polyline, stops)

    # -----------------------------
    # UI output 
    # -----------------------------

    def get_display_position(
            self,
            vehicle_id: str,
            now_ts: Optional[float] = None,
    ) -> Optional[tuple[float, float, float, float]]:
        """
        Returns:
            (lat, lon, confidence, speed_mps)
        """

        if not self._enabled:
            return None

        predictor = self._predictors.get(vehicle_id)
        if predictor is None:
            return None

        if now_ts is None:
            now_ts = time.time()

        # Predictor is the single source of truth
        result = self._predictors[vehicle_id].get_display_position(now_ts)
        if result is None:
            return None

        # Return 5-tuple
        lat, lon, confidence, speed_mps, is_off_route = result
        return lat, lon, confidence, speed_mps, is_off_route