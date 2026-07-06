"""Tests for route polyline extraction from routeCoordinates."""

from motion_prediction.utils.route_polyline import polyline_from_route_data, get_route_polyline


def test_polyline_from_route_data_sorted_by_order():
    route = {
        "routeId": "R1",
        "routeCoordinates": [
            {"order": 3, "latitude": "13.03", "longitude": "77.60"},
            {"order": 1, "latitude": "13.01", "longitude": "77.58"},
            {"order": 2, "latitude": "13.02", "longitude": "77.59"},
        ],
    }
    path = polyline_from_route_data(route)
    assert path == [(13.01, 77.58), (13.02, 77.59), (13.03, 77.60)]


def test_polyline_from_route_data_empty():
    assert polyline_from_route_data({}) == []
    assert polyline_from_route_data({"routeCoordinates": []}) == []


def test_get_route_polyline_prefers_coordinates():
    route = {
        "routeCoordinates": [
            {"order": 1, "latitude": "13.01", "longitude": "77.58"},
            {"order": 2, "latitude": "13.02", "longitude": "77.59"},
        ],
    }
    path = get_route_polyline("R1", route)
    assert len(path) == 2
