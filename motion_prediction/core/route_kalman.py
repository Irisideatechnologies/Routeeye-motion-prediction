from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class KalmanTuning:
    """Process and measurement noise for the route Kalman filter."""

    process_accel_var: float = 0.8       # m²/s⁴ — acceleration uncertainty
    meas_pos_var: float = 9.0            # m²  — ~3 m GPS along-route std
    meas_speed_var: float = 2.25         # (m/s)² — ~1.5 m/s speed std
    init_pos_var: float = 16.0           # m²
    init_speed_var: float = 4.0          # (m/s)²
    innovation_reset_m: float = 80.0     # hard reset if forward innovation exceeds this
    max_backward_innovation_m: float = 20.0  # clamp backward GPS snaps per update


class RouteKalmanFilter:
    """
    1-D constant-velocity Kalman filter in route arc-length space.

    State x = [s, v]  where s = meters along route, v = speed along route (m/s).
    """

    def __init__(self, tuning: KalmanTuning | None = None):
        self._tuning = tuning or KalmanTuning()
        self.x = np.zeros(2)
        self.P = np.diag([self._tuning.init_pos_var, self._tuning.init_speed_var])
        self.initialized = False
        self._last_update_ts: float | None = None

    def initialize(self, s: float, v: float, ts: float | None = None) -> None:
        self.x = np.array([max(0.0, s), max(0.0, v)])
        self.P = np.diag([self._tuning.init_pos_var, self._tuning.init_speed_var])
        self.initialized = True
        if ts is not None:
            self._last_update_ts = ts

    def predict(self, dt: float, max_route_dist: float | None = None) -> None:
        if not self.initialized or dt <= 0:
            return

        F = np.array([[1.0, dt], [0.0, 1.0]])

        # Discrete white-noise acceleration model
        q = self._tuning.process_accel_var
        G = np.array([[0.5 * dt * dt], [dt]])
        Q = G @ G.T * q

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        self._clamp_state(max_route_dist)

    def update_position(self, s_meas: float) -> bool:
        """Update with arc-length measurement only. Returns False if rejected."""
        H = np.array([[1.0, 0.0]])
        R = np.array([[self._tuning.meas_pos_var]])
        return self._update(np.array([s_meas]), H, R)

    def update_position_speed(self, s_meas: float, v_meas: float) -> bool:
        """Update with arc-length and speed measurements. Returns False if rejected."""
        H = np.eye(2)
        R = np.diag([self._tuning.meas_pos_var, self._tuning.meas_speed_var])
        return self._update(np.array([s_meas, v_meas]), H, R)

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> bool:
        z = np.array(z, dtype=float, copy=True)
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R

        # Large GPS corrections: soften backward snaps, hard-reset only on forward teleports.
        if self.initialized and innovation.size >= 1:
            innov_s = float(innovation[0])
            sigma = math.sqrt(max(float(S[0, 0]), 1e-6))
            gate = max(self._tuning.innovation_reset_m, 3.0 * sigma)
            if innov_s < -self._tuning.max_backward_innovation_m:
                # GPS behind dead-reckoned position — clamp, do not jump backward.
                z[0] = max(float(z[0]), self.s - self._tuning.max_backward_innovation_m)
                innovation = z - H @ self.x
            elif innov_s > gate:
                # Large forward teleport — re-initialize to GPS.
                self.initialize(float(z[0]), float(z[1]) if z.size > 1 else max(0.0, self.v))
                return False

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P
        self.initialized = True
        return True

    def clamp_s(self, max_s: float) -> None:
        """Cap arc-length (e.g. dead-reckoning limit between GPS fixes)."""
        self.x[0] = min(self.s, max(0.0, max_s))

    def clamp_v(self, max_v: float) -> None:
        """Cap speed (e.g. decay when GPS is stale)."""
        self.x[1] = min(self.v, max(0.0, max_v))

    def ensure_min_v(self, min_v: float) -> None:
        """Floor speed after GPS fusion when device reports movement."""
        self.x[1] = max(self.v, max(0.0, min_v))

    def _clamp_state(self, max_route_dist: float | None) -> None:
        self.x[0] = max(0.0, self.x[0])
        self.x[1] = max(0.0, self.x[1])
        if max_route_dist is not None:
            self.x[0] = min(self.x[0], max_route_dist)

    @property
    def s(self) -> float:
        return float(self.x[0])

    @property
    def v(self) -> float:
        return float(self.x[1])

    def confidence(self) -> float:
        """Map position variance to a 0–1 confidence score."""
        sigma_s = math.sqrt(max(float(self.P[0, 0]), 1e-6))
        # Floor keeps predictions usable between sparse GPS updates (10–15 s).
        return max(0.25, min(1.0, 1.0 / (1.0 + sigma_s / 20.0)))
