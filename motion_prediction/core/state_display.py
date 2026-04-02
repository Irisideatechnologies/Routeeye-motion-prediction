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


class DisplayState:

    def __init__(self, ref_lat: float, ref_lon: float):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon

        self._history: Deque[DisplaySample] = deque(maxlen=32)

        self._display_position: Optional[Vec2] = None
        self._display_velocity: Vec2 = Vec2.zero()
        self._last_display_ts: Optional[float] = None

        # Magnetic well for bus stops
        self._magnetic_well = MagneticWell()

    
    # Bus stop management 
    

    def set_bus_stops(self, stops: List) -> None:

        from motion_prediction.core.magnetic_well import BusStop as MagneticBusStop

        magnetic_stops = []
        for stop in stops:
            magnetic_stops.append(MagneticBusStop(
                stop_id=stop.stop_id,
                position=stop.position,
            ))

        self._magnetic_well.set_stops(magnetic_stops)

    
    # Background sampling 
    

    def push_background_state(
            self,
            ts: float,
            position: Vec2,
            velocity: Vec2,
            confidence: float,
            gps_speed_cap: Optional[float] = None,
            time_since_gps: float = 0.0,
            marker_position: Optional[Vec2] = None,  # NEW
            marker_velocity: Optional[Vec2] = None,  # NEW
    ) -> None:

        self._history.append(
            DisplaySample(
                ts=ts,
                position=position,
                velocity=velocity,
                confidence=confidence,
                gps_speed_cap=gps_speed_cap,
                time_since_gps=time_since_gps,
                marker_position=marker_position,  # NEW
                marker_velocity=marker_velocity,  # NEW
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

        if dist_to_sample > 0.1:
           
            catchup_speed = min(dist_to_sample * 0.5, 8.0)
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

        # --- Commit ---
        self._display_position = proposed_pos
        self._last_display_ts = now_ts

        lat, lon = xy_to_latlon(
            self._display_position,
            self.ref_lat,
            self.ref_lon,
        )

        speed_mps = self._display_velocity.magnitude()

        return lat, lon, sample.confidence, speed_mps

    # Internal helpers 
    

    def _interpolated_sample(
            self,
            target_ts: float,
    ) -> Optional[DisplaySample]:

        if len(self._history) < 2:
            return None

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
                    marker_pos, marker_vel  # NEW
                )

        return None