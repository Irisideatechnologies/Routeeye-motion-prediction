import os
import time
import json
import logging
from typing import Dict, Set

import redis
import requests

from motion_prediction.api.adapter2 import MotionPredictionAdapter
from motion_prediction.config.env_loader import (
    REDIS_URL,
    API_ROUTES_URL,
    API_BEARER_TOKEN,
    TOPIC_PREDICTED_OUT,
)

logger = logging.getLogger(__name__)

device_to_route: Dict[str, str] = {}

adapter = MotionPredictionAdapter(enabled=True)


def fetch_route_device_mappings() -> dict:
    """Fetch active route-device mappings from the backend API."""
    headers = {"Authorization": f"Bearer {API_BEARER_TOKEN}"}

    resp = requests.get(API_ROUTES_URL, headers=headers, timeout=30)
    resp.raise_for_status()
    routes_json = resp.json()

    mappings = {}
    seen_devices: Set[str] = set()

    for route in routes_json:
        route_id = route.get("routeId")
        if not route_id:
            continue

        active_devices: Set[str] = set()
        for vehicle in route.get("vehicles", []):
            if vehicle.get("deletedAt") is not None:
                continue
            if vehicle.get("isDeleted", False):
                continue
            device_id = vehicle.get("device_id")
            if device_id and device_id not in seen_devices:
                active_devices.add(device_id)
                seen_devices.add(device_id)
            elif device_id and device_id in seen_devices:
                logger.warning("Device '%s' duplicate on route '%s', skipping.", device_id, route_id)

        if active_devices:
            mappings[route_id] = {
                "devices": active_devices,
                "route_data": route,
            }

    return mappings


def _process_gps_packet(device_id: str, payload_str: str) -> None:
    """Parse and ingest a raw GPS packet into the prediction adapter."""
    data = json.loads(payload_str)
    adapter.ingest_gps_packet(
        device_id=device_id,
        latitude=str(data.get("lat")) if data.get("lat") is not None else "0",
        longitude=str(data.get("lon")) if data.get("lon") is not None else "0",
        timestamp=int(data.get("timestamp", time.time() * 1000)),
        speed=str(data["speed"]) if data.get("speed") is not None else None,
    )


def start_redis_daemon():
    """Single-threaded daemon: ingests GPS, runs predictions, publishes at 1Hz."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger.info("Starting Redis Prediction Daemon (Single-Thread)...")

    # Fetch route-device mappings from API -- let errors propagate
    mappings = fetch_route_device_mappings()

    if not mappings:
        logger.warning("No active routes found.")

    # Build device-route lookup and parse polylines per route
    route_polylines: Dict[str, list] = {}

    for route_id, info in mappings.items():
        for device_id in info["devices"]:
            device_to_route[device_id] = route_id

        # Parse polyline once per route
        coords = info["route_data"].get("routeCoordinates", [])
        sorted_coords = sorted(coords, key=lambda c: c.get("order", 0))
        polyline = [
            (float(c["latitude"]), float(c["longitude"]))
            for c in sorted_coords
            if c.get("latitude") and c.get("longitude")
        ]
        route_polylines[route_id] = polyline

    logger.info("%d devices across %d routes", len(device_to_route), len(mappings))
    for route_id, info in mappings.items():
        logger.info("  Route '%s' -> %s (%d waypoints)",
                     route_id, info['devices'], len(route_polylines[route_id]))

    # Redis connections
    r_sub = redis.Redis.from_url(REDIS_URL, decode_responses=True)
    r_pub = redis.Redis.from_url(REDIS_URL, decode_responses=True)

    # Subscribe to each device channel
    pubsub = r_sub.pubsub()
    all_device_ids = list(device_to_route.keys())

    if not all_device_ids:
        logger.warning("No device channels to subscribe to.")
        return

    pubsub.subscribe(*all_device_ids)
    logger.info("Subscribed to %d channels. Waiting for GPS...", len(all_device_ids))

    # Tracking state
    last_seen: Dict[str, float] = {}
    polyline_pending: Set[str] = set(all_device_ids)
    last_publish_time = time.time()

    # --- Single-threaded event loop ---
    while True:
        # Poll for GPS messages (non-blocking, with short timeout)
        message = pubsub.get_message(ignore_subscribe_messages=True, timeout=0.1)

        if message and message["type"] == "message":
            device_id = message["channel"]
            payload = message["data"]

            route_id = device_to_route.get(device_id)
            if route_id is None:
                continue

            # Ingest GPS packet
            try:
                _process_gps_packet(device_id, payload)
            except Exception:
                logger.exception("GPS processing failed for %s on route '%s'", device_id, route_id)
                continue

            last_seen[device_id] = time.time()

            # Apply polyline after first GPS (predictor now exists)
            if device_id in polyline_pending:
                polyline = route_polylines.get(route_id, [])
                if polyline:
                    try:
                        adapter.update_route_context(
                            device_id=device_id,
                            route_polyline=polyline,
                            stops=None,
                        )
                        polyline_pending.discard(device_id)
                        logger.info("Polyline applied for %s on route '%s'", device_id, route_id)
                    except Exception:
                        logger.exception("Failed to apply polyline for %s on route '%s'", device_id, route_id)

        # --- Publish predictions every 1 second ---
        now = time.time()
        if now - last_publish_time >= 1.0:
            last_publish_time = now

            # Timeout inactive devices (120s)
            expired = [d for d, ts in last_seen.items() if now - ts > 120]
            for d in expired:
                del last_seen[d]
                polyline_pending.discard(d)
                logger.warning("Device %s silent >120s, removing.", d)

            # Publish predictions for all active devices
            for device_id in list(last_seen.keys()):
                try:
                    predicted_pos = adapter.get_display_position(device_id)
                    if predicted_pos is not None:
                        r_pub.publish(
                            f"{TOPIC_PREDICTED_OUT}:{device_id}",
                            json.dumps(predicted_pos),
                        )
                except Exception:
                    logger.exception("Failed to predict/publish for device %s", device_id)
