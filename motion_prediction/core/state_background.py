from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import math


from motion_prediction.math.vector import Vec2
from motion_prediction.math.kinematics import (
    integrate_position,
    integrate_velocity,
    apply_acceleration_limit,
    apply_velocity_damping,
    infer_acceleration,
    apply_braking_bias,
)
from motion_prediction.utils.geo import latlon_to_xy
from motion_prediction.config.settings import *
from motion_prediction.core.route_follower import RouteFollower


@dataclass
class BackgroundState:
    ref_lat: float
    ref_lon: float

    position: Vec2
    velocity: Vec2
    acceleration: Vec2

    last_update_ts: float
    last_gps_ts: float

    confidence: float
    dead_reckon_dist: float

    last_gps_position: Vec2
    last_motion_state: str = "STOPPED"
    last_confirmed_speed_mps: Optional[float] = None
    last_heading_dir: Optional[Vec2] = None

    # Route-following support
    route_follower: Optional['RouteFollower'] = None

    # Off-route detection: when vehicle is far from route (parking, depot),
    # skip route following and just hold at raw GPS position.
    _off_route: bool = False

   
    marker_position: Vec2 = None  # Current visual marker position
    marker_velocity: Vec2 = None  # Current visual marker velocity

    
    # Construction
    

    @classmethod
    def initialize_from_gps(cls, lat: float, lon: float, ts: float) -> "BackgroundState":
        pos = Vec2.zero()
        return cls(
            ref_lat=lat,
            ref_lon=lon,
            position=pos,
            velocity=Vec2.zero(),
            acceleration=Vec2.zero(),
            last_update_ts=ts,
            last_gps_ts=ts,
            confidence=1.0,
            dead_reckon_dist=0.0,
            last_gps_position=pos,
            last_motion_state="STOPPED",
            marker_position=pos,  # Initialize marker at GPS position
            marker_velocity=Vec2.zero(),

        )

    
    # Route management
   

    def set_route_follower(self, follower: Optional['RouteFollower']) -> None:

        self.route_follower = follower

   
    # GPS correction 
    

    def apply_gps_fix(self, lat: float, lon: float, ts: float, speed_mps: Optional[float]) -> None:

        gps_pos = latlon_to_xy(lat, lon, self.ref_lat, self.ref_lon)
        dt = max(ts - self.last_gps_ts, 1e-3)

        # --- OFF-ROUTE DETECTION (runs before all branches) ---
        # Check distance from route polyline on every GPS fix.
        # If vehicle is >75m from route (parking, depot), disable route following.
        # Threshold is set high enough to tolerate GPS noise (~35m) without
        # falsely triggering off-route mode.
        OFF_ROUTE_THRESHOLD_M = 75.0
        ON_ROUTE_THRESHOLD_M = 40.0  # Hysteresis: must come closer to re-engage

        if self.route_follower:
            dist = self.route_follower.nearest_route_distance(gps_pos)
            if not self._off_route and dist > OFF_ROUTE_THRESHOLD_M:
                self._off_route = True
                import logging
                logging.getLogger(__name__).info(
                    "Vehicle went OFF-ROUTE (%.0fm from polyline). Holding at GPS position.", dist
                )
            elif self._off_route and dist < ON_ROUTE_THRESHOLD_M:
                self._off_route = False
                # Re-initialize route follower for clean handoff
                self.route_follower._initialized = False
                import logging
                logging.getLogger(__name__).info(
                    "Vehicle returned ON-ROUTE (%.0fm from polyline). Resuming route following.", dist
                )

        # --- GPS SNAP TO ROUTE (filters noisy GPS that drifts off-road) ---
        # When vehicle is on-route, project raw GPS onto the nearest route
        # segment so the marker never leaves the road, even with ~35m GPS error.
        if self.route_follower and not self._off_route:
            gps_pos = self.route_follower.snap_to_route(gps_pos)

        # --- STOP ---
        if speed_mps is not None and speed_mps < 0.3:
            self.position = gps_pos
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()
            self.last_confirmed_speed_mps = 0.0
            self.confidence = 1.0
            self.dead_reckon_dist = 0.0
            self.last_heading_dir = None
            self.last_motion_state = "STOPPED"
            self.last_gps_ts = ts
            self.last_gps_position = gps_pos
            return

        # --- TELEPORT DETECTION (Bus-specific: max 100 km/h) ---
        position_delta = (gps_pos - self.last_gps_position).magnitude()
        time_delta = dt

        MAX_BUS_SPEED_MPS = 27.78  # 100 km/h
        max_reasonable_distance = MAX_BUS_SPEED_MPS * time_delta * 1.3

        if position_delta > max_reasonable_distance and position_delta > 100.0:
            # True teleport
            self.position = gps_pos
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()
            self.last_confirmed_speed_mps = speed_mps
            self.confidence = 1.0
            self.dead_reckon_dist = 0.0
            self.last_heading_dir = None
            self.last_motion_state = "MOVING"
            self.last_gps_ts = ts
            self.last_gps_position = gps_pos
            return

        # --- MOVING ---
        if speed_mps is not None and speed_mps > 0.3:

            # Resolve direction with fallback chain
            direction = None

            # 1. Try position delta (real movement)
            delta = gps_pos - self.last_gps_position
            if delta.magnitude() > 0.5:
                direction = delta.normalized()
                
                # --- AUTO-REVERSE ROUTE FOLLOWER IF MOVING BACKWARDS ---
                if self.route_follower and not getattr(self.route_follower, '_direction_verified', False) and delta.magnitude() > 5.0:
                    p1, seg1, prog1 = self.route_follower._global_locate(self.last_gps_position)
                    p2, seg2, prog2 = self.route_follower._global_locate(gps_pos)
                    
                    dist1 = self.route_follower._route_dist_at(seg1, prog1)
                    dist2 = self.route_follower._route_dist_at(seg2, prog2)
                    
                    if dist2 < dist1 - 2.0:
                        import logging
                        logging.getLogger(__name__).info("Auto-reversing route follower for inbound vehicle!")
                        self.route_follower.reverse()
                    
                    self.route_follower._direction_verified = True

            # 2. Try current velocity direction
            elif self.velocity.magnitude() > 0.5:
                direction = self.velocity.normalized()

            # 3. Try last known heading
            elif self.last_heading_dir is not None:
                direction = self.last_heading_dir

            # 4. No direction available - can't create velocity yet
            if direction is None:
                self.position = gps_pos
                self.last_confirmed_speed_mps = speed_mps
                self.confidence = 1.0
                self.last_motion_state = "MOVING"
                self.last_gps_ts = ts
                self.last_gps_position = gps_pos
                return

            inferred_velocity = direction * speed_mps
            self.last_heading_dir = direction

            inferred_accel = infer_acceleration(self.velocity, inferred_velocity, dt)
            inferred_accel = apply_acceleration_limit(inferred_accel, MAX_ACCEL)

            self.position = gps_pos
            self.velocity = inferred_velocity
            self.acceleration = inferred_accel
            self.last_confirmed_speed_mps = speed_mps
            self.confidence = 1.0
            self.dead_reckon_dist = 0.0
            self.last_motion_state = "MOVING"
            self.last_gps_ts = ts
            self.last_gps_position = gps_pos

   
    # Propagation 
    

    def propagate(self, now_ts: float) -> None:

        dt = now_ts - self.last_update_ts
        if dt <= 0:
            return

        target_speed_mps = self.last_confirmed_speed_mps if self.last_confirmed_speed_mps is not None else 0.0
        time_since_gps = now_ts - self.last_gps_ts
        dead_reckoning = time_since_gps >= DEAD_RECKON_START_SEC

        # Integrate motion
        self.velocity = integrate_velocity(self.velocity, self.acceleration, dt)

        # --- Hold GPS speed ---

        if time_since_gps <= SPEED_HOLD_SEC:
            if target_speed_mps > 0.1 and self.velocity.magnitude() > 0.1:
                current_direction = self.velocity.normalized()
                self.velocity = current_direction * target_speed_mps  # Lock to GPS speed
        # --- Post-hold damping ---
        else:
            self.velocity = apply_velocity_damping(
                self.velocity,
                VELOCITY_DAMPING ** dt
            )

        # --- Brake bias ---
        if self.confidence < 0.43:
            self.velocity = apply_braking_bias(self.velocity, BRAKE_STRENGTH, dt)

        # --- Velocity floor: snap to zero below 1 km/h (0.2778 m/s) ---
        VELOCITY_FLOOR_MPS = 1.0 / 3.6  # 1 km/h
        if self.velocity.magnitude() < VELOCITY_FLOOR_MPS:
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()

        # --- Hard stop ---
        if self.confidence <= DEAD_RECKON_STOP_CONF:
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()
            self.last_heading_dir = None

        speed_before_route = self.velocity.magnitude()
        
        # ROUTE-FOLLOWING: active when route is available AND vehicle is on-route
        

        if self.route_follower and not self._off_route:
            # Vehicle is on-route: use route-following to keep marker on polyline
            self.position, self.velocity = self.route_follower.advance_along_route(
                current_position=self.position,
                speed=self.velocity.magnitude(),
                dt=dt,
                confidence=self.confidence,
            )
        elif self._off_route:
            # Vehicle is OFF-ROUTE (parking, depot, etc.)
            # Just hold at the last GPS position — no prediction, no dead-reckoning.
            # Position is already set to gps_pos in apply_gps_fix(), so do nothing.
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()
        else:
            # No route available - use normal physics-based position
            prev = self.position
            self.position = integrate_position(self.position, self.velocity, dt)
            self.dead_reckon_dist += (self.position - prev).magnitude()

        if time_since_gps <= SPEED_HOLD_SEC and target_speed_mps > 0.1:
            if self.velocity.magnitude() > 0.1:
                self.velocity = self.velocity.normalized() * target_speed_mps
        
        # MARKER POSITION CALCULATION
        
        # Calculate where marker will appear on screen (with display lag)
        

        
        if self.marker_position is None:
            self.marker_position = self.position
            self.marker_velocity = self.velocity
        else:
            alpha = 1 - math.exp(-8 * dt)

            self.marker_position += (self.position - self.marker_position) * alpha
            self.marker_velocity += (self.velocity - self.marker_velocity) * alpha
        # --- Confidence decay  ---
        if time_since_gps <= DEAD_RECKON_START_SEC:
            self.confidence = 0.5 ** (time_since_gps / DEAD_RECKON_START_SEC)
        else:
            extra = time_since_gps - DEAD_RECKON_START_SEC
            self.confidence = 0.5 * (CONFIDENCE_DECAY_PER_SEC ** extra)

        self.confidence = max(0.0, min(1.0, self.confidence))
        self.last_update_ts = now_ts