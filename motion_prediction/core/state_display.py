from __future__ import annotations
from dataclasses import dataclass
from typing import Deque, Optional, List
from collections import deque

from motion_prediction.math.vector import Vec2
from motion_prediction.utils.geo import xy_to_latlon
from motion_prediction.config.settings import (
    DISPLAY_LAG_SEC,
    MAX_VISUAL_LEAD_M,
    DISPLAY_SMOOTHING,
    DEAD_RECKON_START_SEC,
)
from motion_prediction.core.magnetic_well import MagneticWell

MAX_DISPLAY_ACCEL = 1.4  # m/s² (UI only)


@dataclass
class DisplaySample:
    ts: float
    position: Vec2
    velocity: Vec2
    confidence: float
    gps_speed_cap: Optional[float] = None
    time_since_gps: float = 0.0
    marker_position: Optional[Vec2] = None  
    marker_velocity: Optional[Vec2] = None
    is_off_route: bool = False


class DisplayState:

    def __init__(self, ref_lat: float, ref_lon: float):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon

        self._history: Deque[DisplaySample] = deque(maxlen=64)

        self._display_position: Optional[Vec2] = None
        self._display_velocity: Vec2 = Vec2.zero()
        self._last_display_ts: Optional[float] = None

        # Magnetic well for bus stops
        self._magnetic_well = MagneticWell()

        # Shared route follower (owned by BackgroundState, read-only here)
        self._route_follower = None
        self._display_route_dist: Optional[float] = None

    
    def set_bus_stops(self, stops: List) -> None:

        from motion_prediction.core.magnetic_well import BusStop as MagneticBusStop

        magnetic_stops = []
        for stop in stops:
            magnetic_stops.append(MagneticBusStop(
                stop_id=stop.stop_id,
                position=stop.position,
            ))

        self._magnetic_well.set_stops(magnetic_stops)

    def set_route_follower(self, follower) -> None:
        self._route_follower = follower

    
    # Background sampling 
    

    def push_background_state(
            self,
            ts: float,
            position: Vec2,
            velocity: Vec2,
            confidence: float,
            gps_speed_cap: Optional[float] = None,
            time_since_gps: float = 0.0,
            marker_position: Optional[Vec2] = None,
            marker_velocity: Optional[Vec2] = None,
            is_off_route: bool = False,
    ) -> None:

        self._history.append(
            DisplaySample(
                ts=ts,
                position=position,
                velocity=velocity,
                confidence=confidence,
                gps_speed_cap=gps_speed_cap,
                time_since_gps=time_since_gps,
                marker_position=marker_position,
                marker_velocity=marker_velocity,
                is_off_route=is_off_route,
            )
        )

    
    # Display extraction 
    

    def get_display_position(
            self,
            now_ts: float,
    ) -> Optional[tuple[float, float, float, float]]:

        target_ts = now_ts - DISPLAY_LAG_SEC

        sample = self._interpolated_sample(target_ts)
        if sample is None:
            return None

        # --- Initialize display state ---
        if self._display_position is None:
            self._display_position = sample.position
            self._display_velocity = sample.velocity
            self._last_display_ts = now_ts
            
            if self._route_follower:
                # Clamp initial position to route (read-only, doesn't mutate cursor)
                self._display_position = self._route_follower.clamp_to_route(self._display_position)
                self._display_route_dist = self._route_follower.route_distance_of(self._display_position)

        dt = max(now_ts - (self._last_display_ts or now_ts), 1e-3)

        # --- Smooth velocity ---
        prev_velocity = self._display_velocity

        if not sample.velocity.is_near_zero():
            blended_dir = (
                    prev_velocity.normalized() * (1.0 - DISPLAY_SMOOTHING)
                    + sample.velocity.normalized() * DISPLAY_SMOOTHING
            )

            if not blended_dir.is_near_zero():
                candidate_velocity = blended_dir.normalized() * sample.velocity.magnitude()
            else:
                candidate_velocity = sample.velocity
        else:
            candidate_velocity = sample.velocity

        self._display_velocity = candidate_velocity

        # --- Prevent sudden speed increase ---
        prev_speed = prev_velocity.magnitude()
        new_speed = self._display_velocity.magnitude()

        if new_speed > prev_speed:
            max_allowed_increase = MAX_DISPLAY_ACCEL * dt

            if (new_speed - prev_speed) > max_allowed_increase:
                if new_speed > 1e-3:
                    self._display_velocity = (
                            self._display_velocity.normalized() * (prev_speed + max_allowed_increase)
                    )

        # --- GPS speed cap ---
        if sample.gps_speed_cap is not None and sample.time_since_gps < DEAD_RECKON_START_SEC:
            final_speed = self._display_velocity.magnitude()
            if final_speed > sample.gps_speed_cap:
                if final_speed > 1e-3:
                    self._display_velocity = self._display_velocity.normalized() * sample.gps_speed_cap

        # # --- OFF-ROUTE HANDLING ---
        # # When the vehicle is off-route (depot, parking, wrong route), bypass
        # # prediction entirely and show the raw GPS position directly.
        # # The marker will update each time a new GPS packet arrives (~10-15s).
        # # When the vehicle returns on-route, prediction resumes seamlessly
        # # because _display_position is already near the vehicle's actual location.
        # if sample.is_off_route:
        #     # Jump display position to the latest raw GPS position (no prediction)
        #     self._display_position = sample.position
        #     self._display_velocity = Vec2.zero()
        #     self._last_display_ts = now_ts

        #     lat, lon = xy_to_latlon(
        #         self._display_position,
        #         self.ref_lat,
        #         self.ref_lon,
        #     )
        #     # Pass through the actual GPS speed so the marker doesn't report 0
        #     speed_mps = sample.gps_speed_cap if sample.gps_speed_cap is not None else 0.0
        #     return lat, lon, sample.confidence, speed_mps, sample.is_off_route

        # --- OFF-ROUTE HANDLING ---
        # Bypass prediction and show raw GPS when vehicle is off-route.
        if sample.is_off_route:
            # Jump display position to the latest raw GPS position (no prediction)
            self._display_position = sample.position
            self._display_velocity = Vec2.zero()
            self._last_display_ts = now_ts
            # Reset cached route distance so re-engagement starts fresh
            self._display_route_dist = None

            lat, lon = xy_to_latlon(
                self._display_position,
                self.ref_lat,
                self.ref_lon,
            )
            # Pass through the actual GPS speed so the marker doesn't report 0
            speed_mps = sample.gps_speed_cap if sample.gps_speed_cap is not None else 0.0
            return lat, lon, sample.confidence, speed_mps, sample.is_off_route

        # --- Proposed forward motion ---
        proposed_pos = self._display_position + self._display_velocity * dt

        # --- No backward motion rule ---
        forward_vec = proposed_pos - self._display_position
        sample_vec = sample.position - self._display_position

        if forward_vec.dot(sample_vec) < 0:
            proposed_pos = self._display_position

        # --- Smooth Rubber-Banding Catch-up ---
        error_vec = sample.position - proposed_pos
        dist_to_sample = error_vec.magnitude()

        if dist_to_sample > 500.0:
            # Massive jump (e.g. depot start / route reassignment)
            # Instantly snap display marker
            proposed_pos = sample.position
        elif dist_to_sample > 0.1:
            # Proportional catch-up: faster when further behind
            catchup_speed = min(dist_to_sample * 1.0, 15.0)
            catchup_dist = catchup_speed * dt
            
            if catchup_dist > dist_to_sample:
                catchup_dist = dist_to_sample
                
            proposed_pos += error_vec.normalized() * catchup_dist

        # MAGNETIC WELL — applied to DISPLAY position
        well_check_velocity = sample.marker_velocity if sample.marker_velocity else self._display_velocity

        proposed_pos, self._display_velocity = self._magnetic_well.apply_well(
            position=proposed_pos,         
            velocity=well_check_velocity,  
            gps_speed_mps=sample.gps_speed_cap,
        )

        # --- STRICT ROUTE CLAMPING ---
        # Clamp proposed position to route polyline and ensure strict monotonic forward progress.
        if self._route_follower:
            # Check if the physics engine auto-reversed the route beneath us
            current_rev_count = getattr(self._route_follower, 'reverse_count', 0)
            if current_rev_count != getattr(self, '_last_reverse_count', 0):
                self._display_route_dist = None
                self._last_reverse_count = current_rev_count

            # 1. Use global search if display is far behind to find correct position on route.
            use_hint = self._display_route_dist if dist_to_sample <= 50.0 else None
            new_dist = self._route_follower.route_distance_of(proposed_pos, hint_dist=use_hint)
            
            # 2. Enforce forward-only constraint (with safety valve)
            if self._display_route_dist is not None:
                if dist_to_sample > 500.0:
                    # Massive jump (depot start / reassignment) - allow backward jump
                    self._display_route_dist = new_dist
                elif dist_to_sample > 15.0 and sample.gps_speed_cap is not None and sample.gps_speed_cap > 1.0:
                    # Unstick display if it falls >15m behind a moving vehicle.
                    self._display_route_dist = new_dist
                else:
                    # Allow up to 2m backward slip to handle curve tangent projections
                    self._display_route_dist = max(self._display_route_dist - 2.0, new_dist)
            else:
                self._display_route_dist = new_dist
                
            # 3. Extract exact 2D position at the constrained route distance
            proposed_pos, route_dir = self._route_follower.position_at_distance(self._display_route_dist)

            # 4. Align display velocity to route direction to prevent freezing on sharp curves.
            speed = self._display_velocity.magnitude()
            if speed > 0.1:
                self._display_velocity = route_dir * speed

        # --- Commit ---
        self._display_position = proposed_pos
        self._last_display_ts = now_ts

        lat, lon = xy_to_latlon(
            self._display_position,
            self.ref_lat,
            self.ref_lon,
        )

        # Velocity floor: snap to zero below 1 km/h (matches background state floor)
        VELOCITY_FLOOR_MPS = 1.0 / 3.6  # 1 km/h
        if self._display_velocity.magnitude() < VELOCITY_FLOOR_MPS:
            speed_mps = 0.0
        else:
            speed_mps = self._display_velocity.magnitude()

        return lat, lon, sample.confidence, speed_mps, sample.is_off_route

    # Internal helpers 
    

    def _interpolated_sample(
            self,
            target_ts: float,
    ) -> Optional[DisplaySample]:

        if not self._history:
            return None
            
        if len(self._history) == 1:
            return self._history[0]

        # If target is older than our oldest history, return the oldest
        if target_ts <= self._history[0].ts:
            return self._history[0]

        # If target is newer than our newest history, return the newest
        if target_ts >= self._history[-1].ts:
            return self._history[-1]

        for i in range(len(self._history) - 1):
            a = self._history[i]
            b = self._history[i + 1]

            if a.ts <= target_ts <= b.ts:
                t = (target_ts - a.ts) / max(b.ts - a.ts, 1e-6)
                pos = a.position * (1 - t) + b.position * t
                vel = a.velocity * (1 - t) + b.velocity * t
                conf = a.confidence * (1 - t) + b.confidence * t

                gps_cap = b.gps_speed_cap if b.gps_speed_cap is not None else a.gps_speed_cap
                time_since = b.time_since_gps

                # Interpolate marker position and velocity
                marker_pos = None
                marker_vel = None
                if a.marker_position and b.marker_position:
                    marker_pos = a.marker_position * (1 - t) + b.marker_position * t
                if a.marker_velocity and b.marker_velocity:
                    marker_vel = a.marker_velocity * (1 - t) + b.marker_velocity * t

                return DisplaySample(
                    target_ts, pos, vel, conf, gps_cap, time_since,
                    marker_pos, marker_vel,
                    is_off_route=b.is_off_route,
                )

        return self._history[-1]