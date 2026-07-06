from __future__ import annotations

import math
from typing import Optional, List, Tuple

from motion_prediction.core.route_geometry import RouteGeometry
from motion_prediction.core.route_follower import RouteFollower
from motion_prediction.core.route_kalman import RouteKalmanFilter, KalmanTuning
from motion_prediction.models.gps_packet import GPSPacket
from motion_prediction.utils.geo import latlon_to_xy, xy_to_latlon, distance_m
from motion_prediction.math.vector import Vec2

# Route / off-route thresholds (meters)
OFF_ROUTE_THRESHOLD_M = 75.0
ON_ROUTE_THRESHOLD_M = 40.0
STOP_SPEED_MPS = 0.3

# Dead-reckoning limits between GPS fixes
MAX_DEAD_RECKON_M = 80.0
MAX_DEAD_RECKON_SEC = 12.0
DEAD_RECKON_DECAY_SEC = 5.0
MIN_DR_CAP_M = 12.0  # minimum forward cap for very slow vehicles

# Speed inference from GPS position delta
INFER_SPEED_MIN_DT = 1.0
INFER_SPEED_TOLERANCE = 1.15
INFER_SPEED_MARGIN_MPS = 0.5
MIN_DS_FOR_SPEED_CAP_M = 15.0
MIN_HEADING_MOVE_M = 3.0
MAX_INFER_WHEN_DEVICE_STOPPED_MPS = 8.0  # ~29 km/h — reject GPS-teleport inference when device says stopped

# Geographic jump above this uses global route projection (not hint window).
GPS_TELEPORT_THRESHOLD_M = 150.0
# Hinted vs global arc-length disagree → GPS jumped to another route segment.
HINT_GLOBAL_DIVERGENCE_M = 80.0
# Snap display to raw GPS when stuck far from latest fix or GPS teleports.
DISPLAY_GPS_SNAP_DIVERGENCE_M = 300.0
GPS_TELEPORT_DISPLAY_SNAP_M = 400.0


class KalmanVehiclePredictor:
    """Per-vehicle hybrid predictor: route Kalman on-route, GPS dead-reckoning off-route."""

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
        self._peak_route_s: float = 0.0

        self._last_display_lat: float = first_packet.lat
        self._last_display_lon: float = first_packet.lon
        self._display_last_ts: float = first_packet.timestamp
        self._display_speed_mps: float = 0.0
        self._travel_heading_rad: float = 0.0
        self._just_reentered_route: bool = False

        self._last_gps_lat = first_packet.lat
        self._last_gps_lon = first_packet.lon
        self._last_gps_speed_mps: Optional[float] = first_packet.speed_mps

        # Route-based speed inference (on-route Kalman fusion)
        self._s_at_last_gps: float = 0.0
        self._last_gps_fusion_ts: float = first_packet.timestamp
        self._prev_gps_s: Optional[float] = None
        self._prev_gps_ts: Optional[float] = None

        # Raw GPS speed / heading (off-route free-space tracking)
        self._prev_raw_gps_lat: Optional[float] = None
        self._prev_raw_gps_lon: Optional[float] = None
        self._prev_raw_gps_ts: Optional[float] = None

        self._free_lat: float = first_packet.lat
        self._free_lon: float = first_packet.lon
        self._free_speed_mps: float = 0.0
        self._free_heading_rad: float = 0.0
        self._free_last_ts: float = first_packet.timestamp
        self._free_gps_fusion_ts: float = first_packet.timestamp

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

        gps_pos = latlon_to_xy(self._last_gps_lat, self._last_gps_lon, self._ref_lat, self._ref_lon)
        s_meas = self._route_follower.route_distance_of(gps_pos)
        v_init = self._last_gps_speed_mps if self._last_gps_speed_mps is not None else 0.0
        self._filter.initialize(s_meas, v_init, self._last_gps_ts)
        self._last_filter_ts = self._last_gps_ts
        self._s_at_last_gps = s_meas
        self._peak_route_s = s_meas
        self._last_gps_fusion_ts = self._last_gps_ts
        self._prev_gps_s = s_meas
        self._prev_gps_ts = self._last_gps_ts

        dist = self._route_follower.nearest_route_distance(gps_pos)
        self._off_route = dist > OFF_ROUTE_THRESHOLD_M
        self._seed_free_space(self._last_gps_lat, self._last_gps_lon, v_init, self._last_gps_ts)
        if not self._off_route:
            self._free_heading_rad = self._route_heading_at(gps_pos)
            self._travel_heading_rad = self._free_heading_rad
        self._display_speed_mps = v_init

    # ------------------------------------------------------------------ #
    # GPS ingestion
    # ------------------------------------------------------------------ #

    def ingest_gps(self, packet: GPSPacket) -> None:
        prev_lat = self._last_gps_lat
        prev_lon = self._last_gps_lon

        self._last_gps_lat = packet.lat
        self._last_gps_lon = packet.lon
        self._last_gps_speed_mps = packet.speed_mps

        gps_ts = packet.timestamp
        gps_pos = latlon_to_xy(packet.lat, packet.lon, self._ref_lat, self._ref_lon)
        speed_mps = packet.speed_mps
        self._last_gps_ts = gps_ts

        prev_heading = self._travel_heading_rad
        geo_jump = distance_m(prev_lat, prev_lon, packet.lat, packet.lon)
        self._update_travel_heading(prev_lat, prev_lon, packet.lat, packet.lon)
        self._display_last_ts = gps_ts
        self._maybe_snap_display_to_gps(packet.lat, packet.lon, geo_jump)

        if self._route_follower is None:
            eff = self._effective_speed_raw(
                self._infer_raw_gps_speed(packet.lat, packet.lon, gps_ts),
                speed_mps,
            )
            self._display_speed_mps = eff
            self._nudge_display_toward_gps(packet.lat, packet.lon, prev_heading)
            self._seed_free_space(packet.lat, packet.lon, eff, gps_ts)
            self._commit_raw_gps_history(packet.lat, packet.lon, gps_ts)
            return

        dist = self._route_follower.nearest_route_distance(gps_pos)
        self._update_off_route_state(dist, gps_pos)

        if self._off_route:
            inferred_raw = self._infer_raw_gps_speed(packet.lat, packet.lon, gps_ts)
            effective_speed = self._effective_speed_raw(inferred_raw, speed_mps)
            self._display_speed_mps = effective_speed
            self._update_heading_from_gps(packet.lat, packet.lon, prev_lat, prev_lon, gps_pos)
            self._sync_free_space_from_gps(
                packet.lat, packet.lon, effective_speed, gps_ts
            )
            self._nudge_display_toward_gps(packet.lat, packet.lon, prev_heading)
            self._commit_raw_gps_history(packet.lat, packet.lon, gps_ts)
            return

        if gps_ts > self._last_filter_ts:
            self._predict_to(gps_ts)

        s_meas = self._route_s_from_gps(gps_pos, dist, geo_jump)
        effective_speed = self._effective_speed_mps(s_meas, gps_ts, speed_mps)
        self._display_speed_mps = effective_speed
        self._fuse_gps_measurement(s_meas, effective_speed, gps_ts, geo_jump)
        self._nudge_display_toward_gps(packet.lat, packet.lon, prev_heading)
        self._commit_raw_gps_history(packet.lat, packet.lon, gps_ts)

        if (
            self._route_length > 0
            and self._filter.s >= self._route_length - 1.0
            and effective_speed > STOP_SPEED_MPS
        ):
            self._off_route = True
            self._sync_free_space_from_gps(
                packet.lat, packet.lon, effective_speed, gps_ts
            )
            self._update_heading_from_gps(packet.lat, packet.lon, prev_lat, prev_lon, gps_pos)

    def _route_s_from_gps(
        self, gps_pos: Vec2, dist_to_route: float, geo_jump_m: float
    ) -> float:
        """Project GPS onto route; use global search when GPS teleports."""
        if self._route_follower is None:
            return 0.0

        hint = self._filter.s if self._filter.initialized else None
        use_hint = (
            hint is not None
            and geo_jump_m < GPS_TELEPORT_THRESHOLD_M
            and dist_to_route <= OFF_ROUTE_THRESHOLD_M * 2
        )
        if not use_hint:
            return self._route_follower.route_distance_of(gps_pos, hint_dist=None)

        s_hinted = self._route_follower.route_distance_of(gps_pos, hint_dist=hint)
        s_global = self._route_follower.route_distance_of(gps_pos, hint_dist=None)
        if abs(s_global - s_hinted) > HINT_GLOBAL_DIVERGENCE_M:
            return s_global
        return s_hinted

    def _fuse_gps_measurement(
        self,
        s_meas: float,
        effective_speed: float,
        gps_ts: float,
        geo_jump_m: float = 0.0,
    ) -> None:
        prev_s = self._filter.s if self._filter.initialized else 0.0

        if (
            self._filter.initialized
            and geo_jump_m >= GPS_TELEPORT_THRESHOLD_M
            and s_meas < self._peak_route_s - self._filter._tuning.innovation_reset_m
        ):
            self._peak_route_s = s_meas

        s_meas = self._clamp_forward_s_meas(s_meas)

        forward_jump = (
            self._filter.initialized
            and s_meas > prev_s + self._filter._tuning.innovation_reset_m
        )
        if forward_jump:
            self._peak_route_s = s_meas

        if effective_speed < STOP_SPEED_MPS:
            if not self._filter.initialized:
                self._filter.initialize(s_meas, 0.0, gps_ts)
            else:
                self._filter.update_position(s_meas)
                self._filter.clamp_v(0.0)
        elif not self._filter.initialized:
            self._filter.initialize(s_meas, effective_speed, gps_ts)
        else:
            self._filter.update_position_speed(s_meas, effective_speed)
            if effective_speed >= STOP_SPEED_MPS:
                self._filter.ensure_min_v(effective_speed)

        self._last_filter_ts = gps_ts
        self._s_at_last_gps = self._filter.s
        self._peak_route_s = max(self._peak_route_s, self._filter.s)
        self._last_gps_fusion_ts = gps_ts
        self._prev_gps_s = s_meas
        self._prev_gps_ts = gps_ts

    def _update_off_route_state(self, dist: float, gps_pos: Vec2) -> None:
        if not self._off_route and dist > OFF_ROUTE_THRESHOLD_M:
            self._off_route = True
            speed = self._last_gps_speed_mps if self._last_gps_speed_mps is not None else 0.0
            self._sync_free_space_from_gps(
                self._last_gps_lat, self._last_gps_lon, speed, self._last_gps_ts
            )
            self._free_heading_rad = self._route_heading_at(gps_pos)
        elif self._off_route and dist < ON_ROUTE_THRESHOLD_M:
            self._off_route = False
            self._just_reentered_route = True
            if self._route_follower is not None:
                hint = self._filter.s if self._filter.initialized else None
                free_pos = latlon_to_xy(
                    self._free_lat, self._free_lon, self._ref_lat, self._ref_lon
                )
                display_pos = latlon_to_xy(
                    self._last_display_lat,
                    self._last_display_lon,
                    self._ref_lat,
                    self._ref_lon,
                )
                s_free = self._route_follower.route_distance_of(free_pos, hint_dist=hint)
                s_display = self._route_follower.route_distance_of(
                    display_pos, hint_dist=hint
                )
                self._peak_route_s = max(self._peak_route_s, s_free, s_display)
                geo_jump = distance_m(
                    self._last_display_lat,
                    self._last_display_lon,
                    self._last_gps_lat,
                    self._last_gps_lon,
                )
                s_meas = self._route_follower.route_distance_of(gps_pos, hint_dist=None)
                v_init = self._effective_speed_mps(
                    s_meas, self._last_gps_ts, self._last_gps_speed_mps
                )
                self._fuse_gps_measurement(
                    s_meas, v_init, self._last_gps_ts, geo_jump
                )

    # ------------------------------------------------------------------ #
    # Display query (called every ~1 s)
    # ------------------------------------------------------------------ #

    def get_display_position(
        self, now_ts: float
    ) -> Optional[tuple[float, float, float, float, bool]]:
        if self._route_follower is None or not self._filter.initialized:
            prev_lat, prev_lon = self._last_display_lat, self._last_display_lon
            lat, lon, speed = self._step_display_gps_led(now_ts, prev_lat, prev_lon)
            lat, lon = self._clamp_display_forward_strict(lat, lon, prev_lat, prev_lon)
            speed = 0.0 if speed < STOP_SPEED_MPS else speed
            self._last_display_lat, self._last_display_lon = lat, lon
            return lat, lon, 0.5, speed, True

        if self._off_route:
            prev_lat, prev_lon = self._last_display_lat, self._last_display_lon
            lat, lon, speed = self._predict_free_space(now_ts)
            lat, lon = self._clamp_display_forward_strict(lat, lon, prev_lat, prev_lon)
            if speed < STOP_SPEED_MPS:
                speed = 0.0
            gps_age = max(0.0, now_ts - self._free_gps_fusion_ts)
            conf = max(0.35, min(0.55, 0.55 - gps_age * 0.01))
            self._last_display_lat, self._last_display_lon = lat, lon
            return lat, lon, conf, speed, True

        if now_ts > self._last_filter_ts:
            self._predict_to(now_ts)

        prev_lat, prev_lon = self._last_display_lat, self._last_display_lon
        lat, lon, speed = self._step_display_gps_led(now_ts, prev_lat, prev_lon)
        lat, lon = self._snap_display_to_route_if_forward(lat, lon, prev_lat, prev_lon)
        lat, lon = self._clamp_display_forward_strict(lat, lon, prev_lat, prev_lon)
        if self._just_reentered_route:
            self._just_reentered_route = False

        if speed < STOP_SPEED_MPS:
            speed = 0.0

        self._record_progress_along_route(lat, lon)
        self._last_display_lat, self._last_display_lon = lat, lon
        return lat, lon, self._filter.confidence(), speed, False

    # ------------------------------------------------------------------ #
    # GPS-led display (continuous 1 Hz, follow GPS heading)
    # ------------------------------------------------------------------ #

    def _speed_scaled_dr_cap_m(self, speed: float, gps_age: float) -> float:
        """Max distance display may lead last raw GPS — scales with speed."""
        if speed < STOP_SPEED_MPS:
            return 0.0
        window = min(max(0.0, gps_age), MAX_DEAD_RECKON_SEC)
        return min(MAX_DEAD_RECKON_M, max(MIN_DR_CAP_M, speed * window))

    def _update_travel_heading(
        self, prev_lat: float, prev_lon: float, lat: float, lon: float
    ) -> None:
        move_m = distance_m(prev_lat, prev_lon, lat, lon)
        if move_m >= MIN_HEADING_MOVE_M:
            p1 = latlon_to_xy(prev_lat, prev_lon, self._ref_lat, self._ref_lon)
            p2 = latlon_to_xy(lat, lon, self._ref_lat, self._ref_lon)
            delta = p2 - p1
            self._travel_heading_rad = math.atan2(delta.x, delta.y)

    def _maybe_snap_display_to_gps(
        self, gps_lat: float, gps_lon: float, geo_jump_m: float
    ) -> None:
        """Trust raw GPS when it teleports or display has drifted to a wrong place."""
        display_gap = distance_m(
            self._last_display_lat, self._last_display_lon, gps_lat, gps_lon
        )
        if (
            display_gap < DISPLAY_GPS_SNAP_DIVERGENCE_M
            and geo_jump_m < GPS_TELEPORT_DISPLAY_SNAP_M
        ):
            return

        self._last_display_lat = gps_lat
        self._last_display_lon = gps_lon
        self._free_lat = gps_lat
        self._free_lon = gps_lon

        if self._route_follower is not None:
            gps_pos = latlon_to_xy(gps_lat, gps_lon, self._ref_lat, self._ref_lon)
            s_meas = self._route_follower.route_distance_of(gps_pos, hint_dist=None)
            self._peak_route_s = max(self._peak_route_s, s_meas)

    def _nudge_display_toward_gps(
        self, gps_lat: float, gps_lon: float, heading_rad: float
    ) -> None:
        """When GPS moves forward along travel heading, pull display toward it."""
        forward_m = self._geographic_forward_m(
            self._last_display_lat,
            self._last_display_lon,
            gps_lat,
            gps_lon,
            heading_rad,
        )
        if forward_m < 0.5:
            return
        p1 = latlon_to_xy(
            self._last_display_lat, self._last_display_lon, self._ref_lat, self._ref_lon
        )
        p2 = latlon_to_xy(gps_lat, gps_lon, self._ref_lat, self._ref_lon)
        delta = p2 - p1
        if delta.magnitude() < 1e-6:
            return
        step = min(forward_m, delta.magnitude())
        new_pos = p1 + delta.normalized() * step
        self._last_display_lat, self._last_display_lon = xy_to_latlon(
            new_pos, self._ref_lat, self._ref_lon
        )

    def _step_display_gps_led(
        self, now_ts: float, anchor_lat: float, anchor_lon: float
    ) -> tuple[float, float, float]:
        """Advance display along GPS-derived heading between fixes."""
        if now_ts <= self._display_last_ts:
            return anchor_lat, anchor_lon, self._display_speed_mps

        dt = now_ts - self._display_last_ts
        speed = self._display_speed_mps
        gps_age = max(0.0, now_ts - self._last_gps_ts)

        if gps_age > MAX_DEAD_RECKON_SEC and speed >= STOP_SPEED_MPS:
            over = gps_age - MAX_DEAD_RECKON_SEC
            factor = max(0.0, 1.0 - over / DEAD_RECKON_DECAY_SEC)
            speed *= factor

        if speed < STOP_SPEED_MPS:
            self._display_last_ts = now_ts
            return anchor_lat, anchor_lon, 0.0

        dist_step = speed * dt
        anchor = latlon_to_xy(anchor_lat, anchor_lon, self._ref_lat, self._ref_lon)
        dx = dist_step * math.sin(self._travel_heading_rad)
        dy = dist_step * math.cos(self._travel_heading_rad)
        new_pos = Vec2(anchor.x + dx, anchor.y + dy)

        gps_anchor = latlon_to_xy(
            self._last_gps_lat, self._last_gps_lon, self._ref_lat, self._ref_lon
        )
        gps_age_end = max(0.0, now_ts - self._last_gps_ts)
        max_from_gps = self._speed_scaled_dr_cap_m(speed, gps_age_end)
        drift = new_pos - gps_anchor
        if drift.magnitude() > max_from_gps and drift.magnitude() > 1e-6:
            new_pos = gps_anchor + drift.normalized() * max_from_gps

        lat, lon = xy_to_latlon(new_pos, self._ref_lat, self._ref_lon)
        self._display_last_ts = now_ts
        return lat, lon, speed

    def _snap_display_to_route_if_forward(
        self, lat: float, lon: float, prev_lat: float, prev_lon: float
    ) -> tuple[float, float]:
        """Light route snap only when it does not move display backward."""
        if self._route_follower is None:
            return lat, lon
        pos = latlon_to_xy(lat, lon, self._ref_lat, self._ref_lon)
        if self._route_follower.nearest_route_distance(pos) > OFF_ROUTE_THRESHOLD_M:
            return lat, lon
        hint = self._peak_route_s if self._peak_route_s > 0 else None
        s = self._route_follower.route_distance_of(pos, hint_dist=hint)
        route_pos, _ = self._route_follower.position_at_distance(s)
        rlat, rlon = xy_to_latlon(route_pos, self._ref_lat, self._ref_lon)
        forward_m = self._geographic_forward_m(
            prev_lat, prev_lon, rlat, rlon, self._travel_heading_rad
        )
        if forward_m >= 0.0:
            return rlat, rlon
        return lat, lon

    def _clamp_display_forward_strict(
        self, lat: float, lon: float, prev_lat: float, prev_lon: float
    ) -> tuple[float, float]:
        """Display never moves backward along GPS travel heading."""
        back_m = -self._geographic_forward_m(
            prev_lat, prev_lon, lat, lon, self._travel_heading_rad
        )
        if back_m > 0.05:
            return prev_lat, prev_lon
        return lat, lon

    # ------------------------------------------------------------------ #
    # Off-route free-space tracking
    # ------------------------------------------------------------------ #

    def _free_space_display(
        self, now_ts: float
    ) -> tuple[float, float, float, float, bool]:
        lat, lon, speed = self._predict_free_space(now_ts)
        if speed < STOP_SPEED_MPS:
            speed = 0.0

        gps_age = max(0.0, now_ts - self._free_gps_fusion_ts)
        conf = max(0.35, min(0.55, 0.55 - gps_age * 0.01))
        return lat, lon, conf, speed, True

    def _predict_free_space(self, target_ts: float) -> tuple[float, float, float]:
        if target_ts <= self._free_last_ts:
            return self._free_lat, self._free_lon, self._free_speed_mps

        dt = target_ts - self._free_last_ts
        speed = self._free_speed_mps
        if speed < STOP_SPEED_MPS:
            return self._free_lat, self._free_lon, 0.0

        gps_age = max(0.0, target_ts - self._free_gps_fusion_ts)
        if gps_age > MAX_DEAD_RECKON_SEC:
            over = gps_age - MAX_DEAD_RECKON_SEC
            factor = max(0.0, 1.0 - over / DEAD_RECKON_DECAY_SEC)
            speed *= factor

        max_dist = self._speed_scaled_dr_cap_m(speed, gps_age)
        dist = min(speed * dt, max_dist)

        anchor = latlon_to_xy(self._free_lat, self._free_lon, self._ref_lat, self._ref_lon)
        dx = dist * math.sin(self._free_heading_rad)
        dy = dist * math.cos(self._free_heading_rad)
        new_pos = Vec2(anchor.x + dx, anchor.y + dy)

        # Never drift farther from last raw GPS than dead-reckon cap allows.
        gps_anchor = latlon_to_xy(
            self._last_gps_lat, self._last_gps_lon, self._ref_lat, self._ref_lon
        )
        if (new_pos - gps_anchor).magnitude() > max_dist:
            delta = new_pos - gps_anchor
            if delta.magnitude() > 1e-6:
                new_pos = gps_anchor + delta.normalized() * max_dist

        self._free_lat, self._free_lon = xy_to_latlon(new_pos, self._ref_lat, self._ref_lon)
        self._free_last_ts = target_ts
        return self._free_lat, self._free_lon, speed

    def _seed_free_space(
        self, lat: float, lon: float, speed_mps: float, ts: float
    ) -> None:
        self._free_lat = lat
        self._free_lon = lon
        self._free_speed_mps = max(0.0, speed_mps)
        self._free_last_ts = ts
        self._free_gps_fusion_ts = ts

    def _sync_free_space_from_gps(
        self, lat: float, lon: float, speed_mps: float, gps_ts: float
    ) -> None:
        self._free_lat = lat
        self._free_lon = lon
        self._free_speed_mps = max(0.0, speed_mps)
        self._free_last_ts = gps_ts
        self._free_gps_fusion_ts = gps_ts

    def _update_heading_from_gps(
        self,
        lat: float,
        lon: float,
        prev_lat: float,
        prev_lon: float,
        gps_pos: Vec2,
    ) -> None:
        move_m = distance_m(prev_lat, prev_lon, lat, lon)
        if move_m >= MIN_HEADING_MOVE_M:
            p1 = latlon_to_xy(prev_lat, prev_lon, self._ref_lat, self._ref_lon)
            p2 = latlon_to_xy(lat, lon, self._ref_lat, self._ref_lon)
            delta = p2 - p1
            self._free_heading_rad = math.atan2(delta.x, delta.y)
            self._travel_heading_rad = self._free_heading_rad
        elif self._route_follower is not None:
            self._free_heading_rad = self._route_heading_at(gps_pos)

    def _route_heading_at(self, gps_pos: Vec2) -> float:
        if self._route_follower is None:
            return self._free_heading_rad
        s = self._route_follower.route_distance_of(gps_pos)
        pos_a, _ = self._route_follower.position_at_distance(max(0.0, s - 5.0))
        pos_b, _ = self._route_follower.position_at_distance(s + 5.0)
        delta = pos_b - pos_a
        if delta.magnitude() < 1e-6:
            return self._free_heading_rad
        return math.atan2(delta.x, delta.y)

    def _commit_raw_gps_history(self, lat: float, lon: float, ts: float) -> None:
        self._prev_raw_gps_lat = lat
        self._prev_raw_gps_lon = lon
        self._prev_raw_gps_ts = ts

    def _infer_raw_gps_speed(self, lat: float, lon: float, gps_ts: float) -> Optional[float]:
        if (
            self._prev_raw_gps_lat is None
            or self._prev_raw_gps_lon is None
            or self._prev_raw_gps_ts is None
        ):
            return None
        dt = gps_ts - self._prev_raw_gps_ts
        if dt < INFER_SPEED_MIN_DT:
            return None
        dist = distance_m(self._prev_raw_gps_lat, self._prev_raw_gps_lon, lat, lon)
        if dist < 0.5:
            return 0.0
        return dist / dt

    def _effective_speed_raw(
        self,
        inferred: Optional[float],
        device_speed_mps: Optional[float],
    ) -> float:
        device = device_speed_mps if device_speed_mps is not None else 0.0
        device_stopped = device < STOP_SPEED_MPS

        if inferred is None:
            return 0.0 if device_stopped else device

        if device_stopped:
            if STOP_SPEED_MPS < inferred <= MAX_INFER_WHEN_DEVICE_STOPPED_MPS:
                return inferred
            return 0.0

        if inferred < STOP_SPEED_MPS:
            return device

        cap = inferred * INFER_SPEED_TOLERANCE + INFER_SPEED_MARGIN_MPS
        return min(device, cap)

    def _clamp_forward_s_meas(self, s_meas: float) -> float:
        """Never snap more than max_backward_innovation behind recent forward progress."""
        if not self._filter.initialized:
            return s_meas
        max_back = self._filter._tuning.max_backward_innovation_m
        floor_s = self._peak_route_s - max_back
        floor_s = max(floor_s, self._filter.s - max_back)
        return max(s_meas, max(0.0, floor_s))

    def _record_progress_along_route(self, lat: float, lon: float) -> None:
        if self._route_follower is None:
            return
        pos = latlon_to_xy(lat, lon, self._ref_lat, self._ref_lon)
        hint = self._peak_route_s if self._peak_route_s > 0 else None
        s = self._route_follower.route_distance_of(pos, hint_dist=hint)
        self._peak_route_s = max(self._peak_route_s, s)

    def _geographic_forward_m(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float,
        heading_rad: float,
    ) -> float:
        p1 = latlon_to_xy(lat1, lon1, self._ref_lat, self._ref_lon)
        p2 = latlon_to_xy(lat2, lon2, self._ref_lat, self._ref_lon)
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        return dx * math.sin(heading_rad) + dy * math.cos(heading_rad)

    # ------------------------------------------------------------------ #
    # On-route helpers
    # ------------------------------------------------------------------ #

    def _effective_speed_mps(
        self,
        s_meas: float,
        gps_ts: float,
        device_speed_mps: Optional[float],
    ) -> float:
        """Prefer speed implied by GPS progress along route; cap overstated device speed."""
        inferred: Optional[float] = None
        ds = 0.0
        if self._prev_gps_s is not None and self._prev_gps_ts is not None:
            dt = gps_ts - self._prev_gps_ts
            if dt >= INFER_SPEED_MIN_DT:
                ds = s_meas - self._prev_gps_s
                if ds > 0.5:
                    inferred = ds / dt
                elif ds < -5.0:
                    inferred = 0.0
                elif abs(ds) <= 0.5:
                    inferred = 0.0

        device = device_speed_mps if device_speed_mps is not None else 0.0
        device_stopped = device < STOP_SPEED_MPS

        if inferred is None:
            return 0.0 if device_stopped else device

        if device_stopped:
            if STOP_SPEED_MPS < inferred <= MAX_INFER_WHEN_DEVICE_STOPPED_MPS:
                return inferred
            return 0.0

        if inferred < STOP_SPEED_MPS or abs(ds) < MIN_DS_FOR_SPEED_CAP_M:
            return device

        cap = inferred * INFER_SPEED_TOLERANCE + INFER_SPEED_MARGIN_MPS
        return min(device, cap)

    def _predict_to(self, target_ts: float) -> None:
        if target_ts <= self._last_filter_ts:
            return

        dt = target_ts - self._last_filter_ts
        self._filter.predict(dt, max_route_dist=self._route_length or None)

        gps_age = max(0.0, target_ts - self._last_gps_fusion_ts)
        max_s = self._s_at_last_gps + self._speed_scaled_dr_cap_m(
            self._filter.v, gps_age
        )
        if self._filter.s > max_s:
            self._filter.clamp_s(max_s)

        if gps_age > MAX_DEAD_RECKON_SEC:
            over = gps_age - MAX_DEAD_RECKON_SEC
            factor = max(0.0, 1.0 - over / DEAD_RECKON_DECAY_SEC)
            self._filter.clamp_v(self._filter.v * factor)

        self._last_filter_ts = target_ts
