from __future__ import annotations

import time
from typing import Dict, Optional, List, Tuple

from motion_prediction.core.kalman_predictor import KalmanVehiclePredictor
from motion_prediction.core.route_kalman import KalmanTuning
from motion_prediction.models.gps_packet import GPSPacket


class KalmanEngine:
    """Multi-vehicle manager for route-constrained Kalman predictors."""

    def __init__(self, enabled: bool = True, tuning: KalmanTuning | None = None):
        self._enabled = enabled
        self._tuning = tuning
        self._predictors: Dict[str, KalmanVehiclePredictor] = {}

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def remove_vehicle(self, vehicle_id: str) -> None:
        self._predictors.pop(vehicle_id, None)

    def ingest_gps(self, packet: GPSPacket) -> None:
        if not self._enabled:
            return

        predictor = self._predictors.get(packet.vehicle_id)
        if predictor is None:
            predictor = KalmanVehiclePredictor(packet, tuning=self._tuning)
            self._predictors[packet.vehicle_id] = predictor
        else:
            predictor.ingest_gps(packet)

    def update_route_context(
        self,
        device_id: str,
        route_polyline: Optional[List[Tuple[float, float]]],
        stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:
        predictor = self._predictors.get(device_id)
        if predictor is None:
            return
        predictor.update_route_context(route_polyline, stops)

    def get_display_position(
        self,
        vehicle_id: str,
        now_ts: Optional[float] = None,
    ) -> Optional[tuple[float, float, float, float, bool]]:
        if not self._enabled:
            return None

        predictor = self._predictors.get(vehicle_id)
        if predictor is None:
            return None

        if now_ts is None:
            now_ts = time.time()

        return predictor.get_display_position(now_ts)
