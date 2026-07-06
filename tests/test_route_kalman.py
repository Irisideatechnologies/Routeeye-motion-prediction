"""Unit tests for the route-constrained Kalman filter."""

import time

from motion_prediction.core.route_kalman import RouteKalmanFilter, KalmanTuning
from motion_prediction.core.kalman_predictor import KalmanVehiclePredictor
from motion_prediction.models.gps_packet import GPSPacket
from motion_prediction.utils.geo import distance_m


def test_route_kalman_predict_and_update():
    kf = RouteKalmanFilter()
    kf.initialize(s=0.0, v=10.0)

    kf.predict(1.0)
    assert abs(kf.s - 10.0) < 0.01
    assert abs(kf.v - 10.0) < 0.01

    kf.update_position_speed(s_meas=12.0, v_meas=10.0)
    assert kf.s > 10.0
    assert kf.confidence() > 0.0


def test_route_kalman_innovation_reset():
    kf = RouteKalmanFilter(KalmanTuning(innovation_reset_m=20.0))
    kf.initialize(s=100.0, v=5.0)
    kf.update_position(s_meas=200.0)  # large forward jump → reset
    assert abs(kf.s - 200.0) < 0.01


def test_route_kalman_backward_innovation_clamped():
    kf = RouteKalmanFilter(KalmanTuning(max_backward_innovation_m=20.0))
    kf.initialize(s=3700.0, v=2.4)
    kf.update_position(s_meas=3511.0)  # 189 m behind — must not teleport backward
    assert kf.s > 3680.0
    assert kf.s <= 3700.0


def test_route_kalman_dead_reckon_clamp():
    kf = RouteKalmanFilter()
    kf.initialize(s=100.0, v=10.0)
    kf.predict(20.0)  # would advance 200 m without cap at predictor layer
    assert abs(kf.s - 300.0) < 0.01
    kf.clamp_s(150.0)
    assert abs(kf.s - 150.0) < 0.01


def test_effective_speed_caps_device_overshoot():
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=0.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    # Device claims 12 km/h but only moved ~70 m in 10 s (~7 km/h along route)
    p2 = GPSPacket("dev1", 13.199497, 77.699750, ts + 10.0, speed_mps=12.0 / 3.6)
    predictor.ingest_gps(p2)

    lat1, lon1, _, _, _ = predictor.get_display_position(ts + 11.0)
    lat2, lon2, _, _, _ = predictor.get_display_position(ts + 16.0)
    assert lon2 > lon1  # still moving forward
    assert lon2 - lon1 < 0.001  # capped — should not sprint east at 12 km/h


def test_dead_reckon_cap_prevents_large_overshoot():
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=8.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    # No second GPS — dead-reckon for 15 s
    lat1, lon1, _, _, _ = predictor.get_display_position(ts + 1.0)
    lat2, lon2, _, _, _ = predictor.get_display_position(ts + 15.0)
    assert lon2 >= lon1
    assert lon2 - lon1 < 0.0013


def test_kalman_predictor_without_route():
    ts = time.time()
    packet = GPSPacket(
        vehicle_id="dev1",
        lat=13.0352,
        lon=77.5970,
        timestamp=ts,
        speed_mps=5.0,
    )
    predictor = KalmanVehiclePredictor(packet)
    result = predictor.get_display_position(ts + 1.0)
    assert result is not None
    lat, lon, conf, speed, is_off = result
    assert abs(lat - 13.0352) < 1e-4
    assert abs(lon - 77.5970) < 1e-4


def test_kalman_predictor_with_route():
    ts = time.time()
    polyline = [
        (13.0350, 77.5970),
        (13.0355, 77.5975),
        (13.0360, 77.5980),
        (13.0365, 77.5985),
    ]
    p1 = GPSPacket("dev1", 13.0350, 77.5970, ts, speed_mps=8.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.0353, 77.5973, ts + 10.0, speed_mps=8.0)
    predictor.ingest_gps(p2)

    lat, lon, conf, speed, is_off = predictor.get_display_position(ts + 11.0)
    assert not is_off
    assert speed > 0.0
    assert 13.034 <= lat <= 13.037
    assert 77.596 <= lon <= 77.599


def test_off_route_follows_gps_not_route_polyline():
    """Off-route: display tracks raw GPS, not a distant point on the route."""
    ts = time.time()
    polyline = [
        (13.211100, 77.678670),
        (13.210365, 77.677144),
        (13.199798, 77.679343),
        (13.199665, 77.683207),
        (13.199619, 77.686824),
        (13.199641, 77.688379),
        (13.199569, 77.694717),
        (13.199525, 77.697618),
        (13.199528, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
        (13.199817, 77.707588),
        (13.196487, 77.708962),
        (13.199860, 77.712279),
        (13.199472, 77.715425),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.700481, ts, speed_mps=8.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    raw_lat, raw_lon = 13.199800, 77.708600
    p2 = GPSPacket("dev1", raw_lat, raw_lon, ts + 10.0, speed_mps=8.64 / 3.6)
    predictor.ingest_gps(p2)

    lat1, lon1, _, speed1, is_off1 = predictor.get_display_position(ts + 11.0)
    lat2, lon2, _, speed2, is_off2 = predictor.get_display_position(ts + 12.0)

    assert is_off1
    assert is_off2
    assert speed1 > 0.0
    assert speed2 > 0.0
    # Must be near raw GPS, not snapped to route ~250 m away
    assert distance_m(lat1, lon1, raw_lat, raw_lon) < 90.0
    assert distance_m(lat2, lon2, raw_lat, raw_lon) < 100.0
    # Still advances every second between GPS fixes
    assert distance_m(lat1, lon1, lat2, lon2) > 0.5


def test_very_far_off_route_tracks_gps():
    """Even >200 m from polyline, prediction stays near raw GPS and keeps moving."""
    ts = time.time()
    polyline = [
        (13.211100, 77.678670),
        (13.210365, 77.677144),
        (13.199798, 77.679343),
        (13.199665, 77.683207),
        (13.199619, 77.686824),
        (13.199641, 77.688379),
        (13.199569, 77.694717),
        (13.199525, 77.697618),
        (13.199528, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
        (13.199817, 77.707588),
        (13.196487, 77.708962),
        (13.199860, 77.712279),
        (13.199472, 77.715425),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.700481, ts, speed_mps=5.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    raw_lat, raw_lon = 13.195500, 77.707300
    p2 = GPSPacket("dev1", raw_lat, raw_lon, ts + 10.0, speed_mps=10.26 / 3.6)
    predictor.ingest_gps(p2)

    lat1, lon1, _, speed1, is_off1 = predictor.get_display_position(ts + 11.0)
    lat2, lon2, _, speed2, _ = predictor.get_display_position(ts + 12.0)

    assert is_off1
    assert speed1 > 0.0
    assert speed2 > 0.0
    assert distance_m(lat1, lon1, raw_lat, raw_lon) < 90.0
    assert lat2 != lat1 or lon2 != lon1


def test_off_route_stopped_freezes():
    """Off-route with zero speed: hold position, no route drift."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=8.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.199900, 77.708600, ts + 10.0, speed_mps=0.0)
    predictor.ingest_gps(p2)

    lat1, lon1, _, speed1, is_off1 = predictor.get_display_position(ts + 11.0)
    lat2, lon2, _, speed2, _ = predictor.get_display_position(ts + 15.0)

    assert is_off1
    assert speed1 == 0.0
    assert speed2 == 0.0
    assert abs(lat1 - lat2) < 1e-6
    assert abs(lon1 - lon2) < 1e-6


def test_returns_to_route_prediction_when_back_on_route():
    """Re-entering corridor switches back to route-constrained display."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=8.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.199900, 77.708600, ts + 10.0, speed_mps=8.0 / 3.6)
    predictor.ingest_gps(p2)
    _, _, _, _, is_off = predictor.get_display_position(ts + 11.0)
    assert is_off

    p3 = GPSPacket("dev1", 13.199497, 77.700200, ts + 20.0, speed_mps=8.0 / 3.6)
    predictor.ingest_gps(p3)
    _, _, _, speed, is_off_back = predictor.get_display_position(ts + 21.0)
    assert not is_off_back
    assert speed > 0.0


def test_speed_inferred_when_device_reports_zero():
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=0.0)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.199497, 77.699600, ts + 10.0, speed_mps=0.0)
    predictor.ingest_gps(p2)

    _, lon1, _, speed1, _ = predictor.get_display_position(ts + 11.0)
    _, lon2, _, speed2, _ = predictor.get_display_position(ts + 12.0)
    assert speed1 > 0.0
    assert lon2 > lon1


def test_off_route_low_device_speed_not_inferred_as_teleport():
    """3 km/h device speed must not become ~40 km/h from a noisy GPS jump."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.199109, 77.706415, ts, speed_mps=13.5 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.199217, 77.706518, ts + 1.4, speed_mps=3.24 / 3.6)
    predictor.ingest_gps(p2)

    _, _, _, speed, is_off = predictor.get_display_position(ts + 2.0)
    assert is_off
    assert speed * 3.6 < 10.0


def test_reenter_route_does_not_snap_backward():
    """Returning on-route must not jump far behind off-route dead-reckoning."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
        (13.199497, 77.706500),
        (13.199497, 77.707500),
        (13.199497, 77.709500),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.706000, ts, speed_mps=13.5 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    p2 = GPSPacket("dev1", 13.199900, 77.710200, ts + 10.0, speed_mps=3.24 / 3.6)
    predictor.ingest_gps(p2)
    _, lon_off, _, speed_off, is_off = predictor.get_display_position(ts + 11.0)
    assert is_off
    assert speed_off * 3.6 < 10.0

    for i in range(2):
        _, lon_off, _, _, _ = predictor.get_display_position(ts + 12.0 + i)

    p3 = GPSPacket("dev1", 13.199497, 77.708200, ts + 20.0, speed_mps=0.0)
    predictor.ingest_gps(p3)
    _, lon_on, _, _, is_off_back = predictor.get_display_position(ts + 21.0)
    assert not is_off_back
    # Must not snap far behind last off-route display (Shuttle 2 regression).
    assert lon_on >= lon_off - 0.0015


def test_gps_teleport_forward_not_projected_backward():
    """Large GPS jump must not snap prediction backward on route (Shuttle 2)."""
    ts = time.time()
    polyline = [
        (13.211100, 77.678670),
        (13.210365, 77.677144),
        (13.199798, 77.679343),
        (13.199665, 77.683207),
        (13.199497, 77.705000),
    ]
    p1 = GPSPacket("dev1", 13.211100, 77.678670, ts, speed_mps=6.48 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    _, lon_before, _, _, _ = predictor.get_display_position(ts + 1.0)

    p2 = GPSPacket("dev1", 13.199600, 77.679300, ts + 2.0, speed_mps=8.64 / 3.6)
    predictor.ingest_gps(p2)

    _, lon_after, _, speed, is_off = predictor.get_display_position(ts + 3.0)
    assert not is_off
    assert speed > 0.0
    assert lon_after >= lon_before - 0.0005


def test_on_route_display_never_jumps_backward():
    """Stopped GPS must not pull display backward along route (Shuttle 3)."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
        (13.199497, 77.707500),
        (13.199497, 77.709500),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.707000, ts, speed_mps=14.0 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    for i in range(5):
        predictor.get_display_position(ts + 1.0 + i)

    _, lon_advancing, _, _, _ = predictor.get_display_position(ts + 6.0)

    # GPS reports stopped slightly behind predicted position along route
    p2 = GPSPacket("dev1", 13.199497, 77.707200, ts + 7.0, speed_mps=0.0)
    predictor.ingest_gps(p2)

    _, lon_after, _, _, _ = predictor.get_display_position(ts + 8.0)
    assert lon_after >= lon_advancing - 0.0005


def test_no_route_advances_continuously():
    """Without route polyline, display still moves at 1 Hz along GPS heading."""
    ts = time.time()
    p1 = GPSPacket("dev1", 13.199497, 77.699089, ts, speed_mps=5.0)
    predictor = KalmanVehiclePredictor(p1)
    p2 = GPSPacket("dev1", 13.199500, 77.699200, ts + 2.0, speed_mps=5.0)
    predictor.ingest_gps(p2)

    _, lon1, _, speed1, _ = predictor.get_display_position(ts + 3.0)
    _, lon2, _, speed2, _ = predictor.get_display_position(ts + 4.0)
    assert speed1 > 0.0
    assert speed2 > 0.0
    assert lon2 > lon1


def test_display_never_moves_backward_on_route():
    """GPS-led display must not reverse along travel heading."""
    ts = time.time()
    polyline = [
        (13.199497, 77.699089),
        (13.199498, 77.700481),
        (13.199497, 77.705000),
        (13.199497, 77.707500),
    ]
    p1 = GPSPacket("dev1", 13.199497, 77.707000, ts, speed_mps=10.0 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    lons = []
    for i in range(8):
        _, lon, _, _, _ = predictor.get_display_position(ts + 1.0 + i)
        lons.append(lon)

    for i in range(1, len(lons)):
        assert lons[i] >= lons[i - 1] - 1e-6


def test_display_snaps_when_stuck_far_from_gps():
    """Bad GPS fix then correction must not leave display kilometers away (Shuttle 1)."""
    ts = time.time()
    polyline = [
        (13.211100, 77.678670),
        (13.210365, 77.677144),
        (13.199798, 77.679343),
        (13.199665, 77.683207),
        (13.199497, 77.705000),
        (13.199497, 77.709500),
    ]
    p1 = GPSPacket("dev1", 13.211100, 77.678670, ts, speed_mps=10.0 / 3.6)
    predictor = KalmanVehiclePredictor(p1)
    predictor.update_route_context(polyline)

    bad = GPSPacket("dev1", 13.199817, 77.707580, ts + 5.0, speed_mps=21.0 / 3.6)
    predictor.ingest_gps(bad)
    _, lon_bad, _, _, _ = predictor.get_display_position(ts + 6.0)

    good = GPSPacket("dev1", 13.209700, 77.677100, ts + 10.0, speed_mps=21.0 / 3.6)
    predictor.ingest_gps(good)
    _, lon_fixed, _, _, _ = predictor.get_display_position(ts + 11.0)

    assert abs(lon_bad - 77.70758) < 0.001
    assert abs(lon_fixed - 77.6771) < 0.002

