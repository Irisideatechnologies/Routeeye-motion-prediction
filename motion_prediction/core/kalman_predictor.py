from __future__ import annotations

from typing import Optional, List, Tuple

from motion_prediction.core.route_geometry import RouteGeometry
from motion_prediction.core.route_follower import RouteFollower
from motion_prediction.core.route_kalman import RouteKalmanFilter, KalmanTuning
from motion_prediction.models.gps_packet import GPSPacket
from motion_prediction.utils.geo import latlon_to_xy, xy_to_latlon
from motion_prediction.math.vector import Vec2

# Route / off-route thresholds (meters)
GPS_SNAP_DIST_M = 50.0
OFF_ROUTE_THRESHOLD_M = 75.0
ON_ROUTE_THRESHOLD_M = 40.0
BYPASS_PREDICTION_THRESHOLD_M = 120.0
STOP_SPEED_MPS = 0.3


class KalmanVehiclePredictor:
    """Per-vehicle route-constrained Kalman predictor."""

    def __init__(self, first_packet: GPSPacket, tuning: KalmanTuning | None = None):
        self.device_id = first_packet.vehicle_id
        self._ref_lat = first_packet.lat
        self._ref_lon = first_packet.lon

        self._filter = RouteKalmanFilter(tuning)
        self._route_follower: Optional[RouteFollower] = None
        self._route_length: float = 0.0

        self._last_filter_ts: float = first_packet.timestamp
        self._last_gps_ts: float = first_packet.timestamp

        self._off_route: bool = False
        self._bypass_prediction: bool = False

        # Fallback when no route or off-route: hold latest GPS
        self._last_gps_lat = first_packet.lat
        self._last_gps_lon = first_packet.lon
        self._last_gps_speed_mps: Optional[float] = first_packet.speed_mps

        # Apply first packet immediately (fixes legacy first-packet skip)
        self.ingest_gps(first_packet)

    # ------------------------------------------------------------------ #
    # Route context
    # ------------------------------------------------------------------ #

    def update_route_context(
        self,
        route_polyline: Optional[List[Tuple[float, float]]],
        stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:
        if route_polyline is None or len(route_polyline) < 2:
            self._route_follower = None
            self._route_length = 0.0
            return

        route_data = {
            "id": "dynamic",
            "routeId": "dynamic",
            "name": "Dynamic Route",
            "routeType": "BI_DIRECTIONAL",
            "routeCoordinates": [
                {"order": i + 1, "latitude": str(lat), "longitude": str(lon)}
                for i, (lat, lon) in enumerate(route_polyline)
            ],
        }

        stops_data = []
        if stops:
            stops_data = [
                {
                    "id": i,
                    "stopId": stop_id,
                    "name": stop_id,
                    "latitude": str(lat),
                    "longitude": str(lon),
                }
                for i, (stop_id, lat, lon) in enumerate(stops)
            ]

        route = RouteGeometry.from_api_data(
            route_data=route_data,
            stops_data=stops_data,
            direction="OUTBOUND",
            ref_lat=self._ref_lat,
            ref_lon=self._ref_lon,
        )

        self._route_follower = RouteFollower(route)
        self._route_length = self._route_follower._total_route_length

        # Seed Kalman state from last known GPS position
        gps_pos = latlon_to_xy(self._last_gps_lat, self._last_gps_lon, self._ref_lat, self._ref_lon)
        s_meas = self._route_follower.route_distance_of(gps_pos)
        v_init = self._last_gps_speed_mps if self._last_gps_speed_mps is not None else 0.0
        self._filter.initialize(s_meas, v_init, self._last_gps_ts)
        self._last_filter_ts = self._last_gps_ts

        dist = self._route_follower.nearest_route_distance(gps_pos)
        self._off_route = dist > OFF_ROUTE_THRESHOLD_M
        self._bypass_prediction = dist > BYPASS_PREDICTION_THRESHOLD_M

    # ------------------------------------------------------------------ #
    # GPS ingestion
    # ------------------------------------------------------------------ #

    def ingest_gps(self, packet: GPSPacket) -> None:
        self._last_gps_lat = packet.lat
        self._last_gps_lon = packet.lon
        self._last_gps_speed_mps = packet.speed_mps

        gps_ts = packet.timestamp
        gps_pos = latlon_to_xy(packet.lat, packet.lon, self._ref_lat, self._ref_lon)
        speed_mps = packet.speed_mps
        self._last_gps_ts = gps_ts

        if self._route_follower is None:
            return

        dist = self._route_follower.nearest_route_distance(gps_pos)
        self._update_off_route_state(dist, gps_pos)

        if self._bypass_prediction or self._off_route:
            return

        if gps_ts > self._last_filter_ts:
            self._predict_to(gps_ts)

        s_meas = self._route_follower.route_distance_of(gps_pos)

        if dist <= GPS_SNAP_DIST_M:
            if speed_mps is not None and speed_mps < STOP_SPEED_MPS:
                self._filter.initialize(s_meas, 0.0, gps_ts)
            elif speed_mps is not None:
                self._filter.update_position_speed(s_meas, speed_mps)
            else:
                self._filter.update_position(s_meas)
        else:
            self._off_route = True
            return

        self._last_filter_ts = gps_ts

        if (
            self._route_length > 0
            and self._filter.s >= self._route_length - 1.0
            and speed_mps is not None
            and speed_mps > STOP_SPEED_MPS
        ):
            self._off_route = True

    def _update_off_route_state(self, dist: float, gps_pos: Vec2) -> None:
        if dist > BYPASS_PREDICTION_THRESHOLD_M:
            if not self._bypass_prediction:
                self._bypass_prediction = True
                self._off_route = True
        elif not self._off_route and dist > OFF_ROUTE_THRESHOLD_M:
            self._off_route = True
        elif self._off_route and dist < ON_ROUTE_THRESHOLD_M:
            self._off_route = False
            self._bypass_prediction = False
            if self._route_follower is not None:
                self._route_follower._initialized = False
                s_meas = self._route_follower.route_distance_of(gps_pos)
                v_init = self._last_gps_speed_mps if self._last_gps_speed_mps is not None else 0.0
                self._filter.initialize(s_meas, v_init, self._last_gps_ts)
                self._last_filter_ts = self._last_gps_ts

    # ------------------------------------------------------------------ #
    # Display query (called every ~1 s)
    # ------------------------------------------------------------------ #

    def get_display_position(
        self, now_ts: float
    ) -> Optional[tuple[float, float, float, float, bool]]:
        if now_ts > self._last_filter_ts:
            self._predict_to(now_ts)

        is_off = self._off_route or self._bypass_prediction

        if is_off or self._route_follower is None or not self._filter.initialized:
            speed = self._last_gps_speed_mps if self._last_gps_speed_mps is not None else 0.0
            if speed < STOP_SPEED_MPS:
                speed = 0.0
            conf = 0.5 if is_off else 1.0
            return (
                self._last_gps_lat,
                self._last_gps_lon,
                conf,
                speed,
                is_off,
            )

        pos, _ = self._route_follower.position_at_distance(self._filter.s)
        lat, lon = xy_to_latlon(pos, self._ref_lat, self._ref_lon)
        speed = self._filter.v
        if speed < STOP_SPEED_MPS:
            speed = 0.0

        return lat, lon, self._filter.confidence(), speed, False

    # ------------------------------------------------------------------ #
    # Internal
    # ------------------------------------------------------------------ #

    def _predict_to(self, target_ts: float) -> None:
        if target_ts <= self._last_filter_ts:
            return

        dt = target_ts - self._last_filter_ts
        self._filter.predict(dt, max_route_dist=self._route_length or None)
        self._last_filter_ts = target_ts
