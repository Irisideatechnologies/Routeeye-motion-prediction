import time
import json
import threading
import queue
from typing import Dict, Set

import redis
import requests

from motion_prediction.api.adapter2 import MotionPredictionAdapter

REDIS_URL = "----------"
API_ROUTES_URL = "----------"
API_BEARER_TOKEN = "----------"
TOPIC_PREDICTED_OUT = "gps:predicted"

route_queues: Dict[str, queue.Queue] = {}
device_to_route: Dict[str, str] = {}
routes_lock = threading.Lock()

adapter = MotionPredictionAdapter(enabled=True)


def fetch_route_device_mappings() -> dict:
    """Fetch all routes with vehicles from the backend API.
    Returns {route_id: {"devices": set, "route_data": dict}}.
    Deduplicates devices — first route wins if a device appears on multiple routes.
    """
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
                print(f"[WARN] Device '{device_id}' duplicate on route '{route_id}', skipping.")

        if active_devices:
            mappings[route_id] = {
                "devices": active_devices,
                "route_data": route,
            }

    return mappings


def _process_gps_on_route_thread(device_id: str, payload_str: str) -> None:
    """Parse and ingest a raw GPS packet. Runs on route thread, no lock needed."""
    try:
        data = json.loads(payload_str)
        adapter.ingest_gps_packet(
            device_id=device_id,
            latitude=str(data.get("lat")) if data.get("lat") is not None else "0",
            longitude=str(data.get("lon")) if data.get("lon") is not None else "0",
            timestamp=int(data.get("timestamp", time.time() * 1000)),
            speed=str(data["speed"]) if data.get("speed") is not None else None,
        )
    except Exception as e:
        print(f"[ERROR] GPS processing failed for {device_id}: {e}")


def route_thread_worker(route_id: str, device_ids: Set[str],
                        route_data: dict, r_pub: redis.Redis) -> None:
    """Dedicated thread for one route. Drains GPS queue, polls predictions at ~4Hz."""
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

    print(f"[ROUTE START] '{route_id}' | {len(device_ids)} devices | {len(polyline)} waypoints")

    while True:
        # Drain queue
        try:
            while True:
                msg = work_queue.get_nowait()
                device_id = msg["device_id"]
                _process_gps_on_route_thread(device_id, msg["payload"])
                last_seen_local[device_id] = time.time()

                # Apply polyline after first GPS (predictor now exists)
                if device_id in polyline_pending and polyline:
                    adapter.update_route_context(
                        device_id=device_id,
                        route_polyline=polyline,
                        stops=None,
                    )
                    polyline_pending.discard(device_id)
                    print(f"[CONTEXT] Polyline applied for {device_id} on '{route_id}'")
        except queue.Empty:
            pass

        # Timeout inactive devices (120s)
        now = time.time()
        expired = [d for d, ts in last_seen_local.items() if now - ts > 120]
        for d in expired:
            del last_seen_local[d]
            polyline_pending.discard(d)
            print(f"[TIMEOUT] {d} on '{route_id}' silent >120s")

        # Exit if all devices that ever sent GPS have expired
        if last_seen_local or not polyline_pending:
            if not last_seen_local:
                print(f"[ROUTE EXIT] '{route_id}' — no active devices")
                with routes_lock:
                    route_queues.pop(route_id, None)
                break

        # Publish predictions
        for device_id in list(last_seen_local.keys()):
            predicted_pos = adapter.get_display_position(device_id)
            if predicted_pos is not None:
                r_pub.publish(
                    f"{TOPIC_PREDICTED_OUT}:{device_id}",
                    json.dumps(predicted_pos),
                )

        time.sleep(0.25)


def main():
    print("Starting Redis Prediction Daemon (Thread-Per-Route)...")

    # Fetch route-device mappings from API
    try:
        mappings = fetch_route_device_mappings()
    except requests.RequestException as e:
        print(f"[FATAL] API request failed: {e}")
        return
    except Exception as e:
        print(f"[FATAL] Unexpected error: {e}")
        return

    if not mappings:
        print("[WARN] No active routes found.")

    # Build device → route lookup
    for route_id, info in mappings.items():
        for device_id in info["devices"]:
            device_to_route[device_id] = route_id

    print(f"[INIT] {len(device_to_route)} devices across {len(mappings)} routes")
    for route_id, info in mappings.items():
        print(f"  '{route_id}' → {info['devices']}")

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
        print("[WARN] No device channels to subscribe to.")
        return

    pubsub.subscribe(*all_device_ids)
    print(f"[INIT] Subscribed to {len(all_device_ids)} channels. Waiting for GPS...\n")

    # Main loop — dispatch GPS to route threads
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


if __name__ == "__main__":
    main()
