from __future__ import annotations

import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)


def polyline_from_route_data(route_data: dict) -> List[Tuple[float, float]]:
    """Build a polyline from routeCoordinates in the routes API response (no Directions call)."""
    coords = route_data.get("routeCoordinates") or []
    if not coords:
        return []

    sorted_coords = sorted(coords, key=lambda c: c.get("order", 0))
    path: List[Tuple[float, float]] = []
    for coord in sorted_coords:
        try:
            lat = float(coord.get("latitude"))
            lon = float(coord.get("longitude"))
            path.append((lat, lon))
        except (TypeError, ValueError):
            continue
    return path


def get_route_polyline(
    route_id: str,
    route_data: dict,
    directions_base_url: str = "",
    fetch_directions_fn=None,
) -> List[Tuple[float, float]]:
    """
    Prefer routeCoordinates from routes API; optionally fall back to Directions API.

    fetch_directions_fn: callable(route_id) -> list, injected by redis_daemon when needed.
    """
    path = polyline_from_route_data(route_data)
    if len(path) >= 2:
        logger.info(
            "Route '%s': loaded %d waypoints from routeCoordinates (no Directions call)",
            route_id,
            len(path),
        )
        return path

    if directions_base_url and fetch_directions_fn is not None:
        logger.info(
            "Route '%s': routeCoordinates missing/short — falling back to Directions API",
            route_id,
        )
        return fetch_directions_fn(route_id)

    logger.warning(
        "Route '%s': no usable routeCoordinates and Directions fallback unavailable",
        route_id,
    )
    return []
