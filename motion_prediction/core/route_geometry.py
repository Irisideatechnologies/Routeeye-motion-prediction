from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional

from motion_prediction.math.vector import Vec2
from motion_prediction.utils.geo import latlon_to_xy


@dataclass
class BusStop:
    """Represents a bus stop on a route"""
    stop_id: str
    name: str  # Display name
    position: Vec2  # In local XY coordinates
    sequence: int  # Order in route (0-indexed)
    lat: float  # Original latitude
    lon: float  # Original longitude


@dataclass
class RouteGeometry:

    route_id: str
    route_name: str
    route_type: str  # "BI_DIRECTIONAL" or "UNI_DIRECTIONAL"
    direction: str  # "OUTBOUND" or "INBOUND"

    # Polyline in local XY coordinates (ordered waypoints)
    polyline: List[Vec2]

    # Bus stops in sequence
    stops: List[BusStop]

    # Reference point for coordinate conversion
    ref_lat: float
    ref_lon: float

    @classmethod
    def from_api_data(
            cls,
            route_data: dict,
            stops_data: List[dict],
            direction: str,
            ref_lat: Optional[float] = None,
            ref_lon: Optional[float] = None,
    ) -> "RouteGeometry":


        # Extract route coordinates
        route_coords = sorted(
            route_data.get("routeCoordinates", []),
            key=lambda x: x["order"]
        )

        if not route_coords:
            raise ValueError(f"Route {route_data['routeId']} has no coordinates")

        # Use first coordinate as reference if not provided
        if ref_lat is None or ref_lon is None:
            ref_lat = float(route_coords[0]["latitude"])
            ref_lon = float(route_coords[0]["longitude"])

        # Convert polyline to XY coordinates
        polyline = []
        for coord in route_coords:
            lat = float(coord["latitude"])
            lon = float(coord["longitude"])
            xy = latlon_to_xy(lat, lon, ref_lat, ref_lon)
            polyline.append(xy)

        # Convert stops to XY coordinates
        stops = []
        for idx, stop in enumerate(stops_data):
            lat = float(stop["latitude"])
            lon = float(stop["longitude"])
            xy = latlon_to_xy(lat, lon, ref_lat, ref_lon)

            stops.append(BusStop(
                stop_id=stop["stopId"],
                name=stop["name"],
                position=xy,
                sequence=idx,
                lat=lat,
                lon=lon,
            ))

        # Reverse for INBOUND direction
        if direction == "INBOUND":
            polyline = list(reversed(polyline))
            # Reverse stops and update sequence
            stops = list(reversed(stops))
            for idx, stop in enumerate(stops):
                stop.sequence = idx

        return cls(
            route_id=str(route_data["id"]),
            route_name=route_data["routeId"],
            route_type=route_data["routeType"],
            direction=direction,
            polyline=polyline,
            stops=stops,
            ref_lat=ref_lat,
            ref_lon=ref_lon,
        )