"""Unit tests for the route-constrained Kalman filter."""

import time

from motion_prediction.core.route_kalman import RouteKalmanFilter, KalmanTuning
from motion_prediction.core.kalman_predictor import KalmanVehiclePredictor
from motion_prediction.models.gps_packet import GPSPacket


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
    kf.update_position(s_meas=200.0)  # large jump → reset
    assert abs(kf.s - 200.0) < 0.01


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
    # Simple straight-ish route in Bangalore area
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
