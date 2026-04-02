from __future__ import annotations
import math
from typing import Tuple

from motion_prediction.math.vector import Vec2

# Mean Earth radius in meters
EARTH_RADIUS_M = 6_371_000.0


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def latlon_to_xy(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float,
) -> Vec2:

    lat_rad = _deg_to_rad(lat)
    lon_rad = _deg_to_rad(lon)
    ref_lat_rad = _deg_to_rad(ref_lat)
    ref_lon_rad = _deg_to_rad(ref_lon)

    d_lat = lat_rad - ref_lat_rad
    d_lon = lon_rad - ref_lon_rad

    x = d_lon * math.cos(ref_lat_rad) * EARTH_RADIUS_M
    y = d_lat * EARTH_RADIUS_M

    return Vec2(x, y)


def xy_to_latlon(
    pos: Vec2,
    ref_lat: float,
    ref_lon: float,
) -> Tuple[float, float]:
    """
    Convert local XY meters back to lat/lon using the same reference origin.
    """

    ref_lat_rad = _deg_to_rad(ref_lat)
    ref_lon_rad = _deg_to_rad(ref_lon)

    lat = ref_lat_rad + (pos.y / EARTH_RADIUS_M)
    lon = ref_lon_rad + (pos.x / (EARTH_RADIUS_M * math.cos(ref_lat_rad)))

    return (
        lat * 180.0 / math.pi,
        lon * 180.0 / math.pi,
    )


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Approximate distance in meters between two lat/lon points.
    Uses the same local-plane approximation.
    """

    lat1_rad = _deg_to_rad(lat1)
    lat2_rad = _deg_to_rad(lat2)
    d_lat = lat2_rad - lat1_rad
    d_lon = _deg_to_rad(lon2 - lon1)

    x = d_lon * math.cos((lat1_rad + lat2_rad) * 0.5)
    y = d_lat

    return math.hypot(x, y) * EARTH_RADIUS_M


def bearing_deg(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:

    lat1_rad = _deg_to_rad(lat1)
    lat2_rad = _deg_to_rad(lat2)
    d_lon = _deg_to_rad(lon2 - lon1)

    y = math.sin(d_lon) * math.cos(lat2_rad)
    x = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon)
    )

    return math.degrees(math.atan2(y, x))
