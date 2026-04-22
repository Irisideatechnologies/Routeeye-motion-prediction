from __future__ import annotations
from typing import Optional, List, Tuple

from motion_prediction.core.state_background import BackgroundState
from motion_prediction.core.state_display import DisplayState
from motion_prediction.core.route_geometry import RouteGeometry
from motion_prediction.core.route_follower import RouteFollower
from motion_prediction.models.gps_packet import GPSPacket
from motion_prediction.config.settings import PHYSICS_DT
from motion_prediction.utils.geo import latlon_to_xy


class VehiclePredictor:

    def __init__(self, first_packet: GPSPacket):
        self._bg = BackgroundState.initialize_from_gps(
            lat=first_packet.lat,
            lon=first_packet.lon,
            ts=first_packet.timestamp,
        )

        # 🔑 Seed speed immediately
        if first_packet.speed_mps is not None and first_packet.speed_mps > 0.3:
            self._bg.last_confirmed_speed_mps = first_packet.speed_mps

        self._display = DisplayState(
            ref_lat=first_packet.lat,
            ref_lon=first_packet.lon,
        )

        self._last_tick_ts = first_packet.timestamp

        # Store reference coordinates for route conversion
        self._ref_lat = first_packet.lat
        self._ref_lon = first_packet.lon

    # -----------------------------
    # Route context management (UNCHANGED)
    # -----------------------------

    def update_route_context(
            self,
            route_polyline: Optional[List[Tuple[float, float]]],
            stops: Optional[List[Tuple[str, float, float]]] = None,
    ) -> None:
        
        print(f"Predictor.update_route_context called:")
        print(f"   Route polyline: {len(route_polyline) if route_polyline else 0} waypoints")
        print(f"   Stops: {len(stops) if stops else 0} stops")

        if route_polyline is None:
            # Clear route
            self._bg.set_route_follower(None)
            self._display.set_bus_stops([])
            return

        # Create route geometry from simple lists
        route_data = {
            "id": "dynamic",
            "routeId": "dynamic",
            "name": "Dynamic Route",
            "routeType": "BI_DIRECTIONAL",
            "routeCoordinates": [
                {"order": i + 1, "latitude": str(lat), "longitude": str(lon)}
                for i, (lat, lon) in enumerate(route_polyline)
            ]
        }

        # Create stops data
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

        # Create route geometry
        route = RouteGeometry.from_api_data(
            route_data=route_data,
            stops_data=stops_data,
            direction="OUTBOUND",
            ref_lat=self._ref_lat,
            ref_lon=self._ref_lon,
        )

        # Set route follower for background state
        route_follower = RouteFollower(route)
        self._bg.set_route_follower(route_follower)

        # Set bus stops for display state (magnetic wells)
        self._display.set_bus_stops(route.stops)

    # -----------------------------
    # GPS ingestion 
    # -----------------------------

    def ingest_gps(self, packet: GPSPacket) -> None:
        self._bg.apply_gps_fix(
            lat=packet.lat,
            lon=packet.lon,
            ts=packet.timestamp,
            speed_mps=packet.speed_mps,
        )

    # -----------------------------
    # Tick 
    # -----------------------------

    def tick(self, now_ts: float) -> None:
        while self._last_tick_ts + PHYSICS_DT <= now_ts:
            self._last_tick_ts += PHYSICS_DT
            self._bg.propagate(self._last_tick_ts)

            # Calculate time since last GPS
            time_since_gps = self._bg.last_update_ts - self._bg.last_gps_ts

            # Pass marker position to display state
            self._display.push_background_state(
                ts=self._bg.last_update_ts,
                position=self._bg.position,
                velocity=self._bg.velocity,
                confidence=self._bg.confidence,
                gps_speed_cap=self._bg.last_confirmed_speed_mps,
                time_since_gps=time_since_gps,
                marker_position=self._bg.marker_position,  
                marker_velocity=self._bg.marker_velocity,  
            )

    # -----------------------------
    # Display position 
    # -----------------------------

    def get_display_position(self, now_ts: float) -> Optional[tuple[float, float, float, float]]:
        self.tick(now_ts)

        result = self._display.get_display_position(now_ts)
        if result is None:
            return None

        lat, lon, confidence, speed_mps = result

        return lat, lon, confidence, speed_mps
