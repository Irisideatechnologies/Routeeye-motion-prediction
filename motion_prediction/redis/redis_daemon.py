import os
import time
import json
import logging
import threading
import queue
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

route_queues: Dict[str, queue.Queue] = {}
device_to_route: Dict[str, str] = {}
routes_lock = threading.Lock()

adapter = MotionPredictionAdapter(enabled=True)


def fetch_route_device_mappings() -> dict:
    
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


def _process_gps_on_route_thread(device_id: str, payload_str: str) -> None:
    """Parse and ingest a raw GPS packet. Runs on route thread, no lock needed.

    """
    data = json.loads(payload_str)
    adapter.ingest_gps_packet(
        device_id=device_id,
        latitude=str(data.get("lat")) if data.get("lat") is not None else "0",
        longitude=str(data.get("lon")) if data.get("lon") is not None else "0",
        timestamp=int(data.get("timestamp", time.time() * 1000)),
        speed=str(data["speed"]) if data.get("speed") is not None else None,
    )


def route_thread_worker(route_id: str, device_ids: Set[str],
                        route_data: dict, r_pub: redis.Redis) -> None:
    """Dedicated thread for one route. Drains GPS queue, polls predictions exactly at 1Hz (1s)."""
    work_queue = route_queues[route_id]

    # Devices added here only after their first GPS arrives (avoids premature timeout)
    last_seen_local: Dict[str, float] = {}
    polyline_pending: Set[str] = set(device_ids)

    # Parse polyline once
    coords = route_data.get("routeCoordinates", [])
    sorted_coords = sorted(coords, key=lambda c: c.get("order", 0))
    polyline = [
        (float(c["latitude"]), float(c["longitude"]))
        for c in sorted_coords
        if c.get("latitude") and c.get("longitude")
    ]

    logger.info("Route '%s' started | %d devices | %d waypoints", route_id, len(device_ids), len(polyline))

    while True:
        # Drain queue
        try:
            while True:
                msg = work_queue.get_nowait()
                device_id = msg["device_id"]
                try:
                    _process_gps_on_route_thread(device_id, msg["payload"])
                except Exception:
                    logger.exception("GPS processing failed for %s on route '%s'", device_id, route_id)
                    continue
                last_seen_local[device_id] = time.time()

                # Apply polyline after first GPS (predictor now exists)
                if device_id in polyline_pending and polyline:
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
        except queue.Empty:
            pass

        # Timeout inactive devices (120s)
        now = time.time()
        expired = [d for d, ts in last_seen_local.items() if now - ts > 120]
        for d in expired:
            del last_seen_local[d]
            polyline_pending.discard(d)
            logger.warning("Device %s on route '%s' silent >120s, removing.", d, route_id)

        # Exit if all devices that ever sent GPS have expired
        if last_seen_local or not polyline_pending:
            if not last_seen_local:
                logger.info("Route '%s' exiting — no active devices.", route_id)
                with routes_lock:
                    route_queues.pop(route_id, None)
                break

        # Publish predictions
        try:
            for device_id in list(last_seen_local.keys()):
                predicted_pos = adapter.get_display_position(device_id)
                if predicted_pos is not None:
                    r_pub.publish(
                        f"{TOPIC_PREDICTED_OUT}:{device_id}",
                        json.dumps(predicted_pos),
                    )
        except Exception:
            logger.exception("Failed to predict/publish for route '%s'", route_id)

        # Sleep to achieve 1 publish per second
        time.sleep(1.0)


def start_redis_daemon():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger.info("Starting Redis Prediction Daemon (Thread-Per-Route)...")

    # Fetch route-device mappings from API -- let errors propagate
    mappings = fetch_route_device_mappings()

    if not mappings:
        logger.warning("No active routes found.")

    # Build device-route lookup
    for route_id, info in mappings.items():
        for device_id in info["devices"]:
            device_to_route[device_id] = route_id

    logger.info("%d devices across %d routes", len(device_to_route), len(mappings))
    for route_id, info in mappings.items():
        logger.info("  Route '%s' -> %s", route_id, info['devices'])

    # Redis connections
    r_sub = redis.Redis.from_url(REDIS_URL, decode_responses=True)
    r_pub = redis.Redis.from_url(REDIS_URL, decode_responses=True)

    # Spawn route threads
    for route_id, info in mappings.items():
        route_queues[route_id] = queue.Queue()
        t = threading.Thread(
            target=route_thread_worker,
            args=(route_id, info["devices"], info["route_data"], r_pub),
            daemon=True,
            name=f"RouteThread-{route_id}",
        )
        t.start()

    # Subscribe to each device channel
    pubsub = r_sub.pubsub()
    all_device_ids = list(device_to_route.keys())

    if not all_device_ids:
        logger.warning("No device channels to subscribe to.")
        return

    pubsub.subscribe(*all_device_ids)
    logger.info("Subscribed to %d channels. Waiting for GPS...", len(all_device_ids))

    # Main loop -- dispatch GPS to route threads
    for message in pubsub.listen():
        if message["type"] == "message":
            device_id = message["channel"]
            payload = message["data"]

            route_id = device_to_route.get(device_id)
            if route_id is None:
                continue

            q = route_queues.get(route_id)
            if q is not None:
                q.put({"device_id": device_id, "payload": payload})




