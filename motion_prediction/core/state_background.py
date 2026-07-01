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
    device_id: str
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

    # Off-route detection: skip route following if vehicle is far from route (e.g. parking).
    _off_route: bool = False

    # Bypass prediction entirely when vehicle is >120m from route.
    _bypass_prediction: bool = False

   
    marker_position: Vec2 = None  # Current visual marker position
    marker_velocity: Vec2 = None  # Current visual marker velocity

    
    # Construction
    

    @classmethod
    def initialize_from_gps(cls, lat: float, lon: float, ts: float, device_id: str = "") -> "BackgroundState":
        pos = Vec2.zero()
        return cls(
            device_id=device_id,
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
        # Disable route following if vehicle >75m from route (tolerates ~35m noise).
        OFF_ROUTE_THRESHOLD_M = 75.0
        ON_ROUTE_THRESHOLD_M = 40.0  # Hysteresis: must come closer to re-engage
        BYPASS_PREDICTION_THRESHOLD_M = 120.0  # Beyond this, bypass prediction entirely

        if self.route_follower:
            dist = self.route_follower.nearest_route_distance(gps_pos)

            # --- Bypass prediction tier (>120m) ---
            if dist > BYPASS_PREDICTION_THRESHOLD_M:
                if not self._bypass_prediction:
                    self._bypass_prediction = True
                    self._off_route = True
                    import logging
                    logging.getLogger(__name__).info(
                        "[%s] Vehicle VERY FAR from route (%.0fm). Bypassing prediction entirely.",
                        self.device_id, dist
                    )
            # --- Normal off-route tier (75-120m) ---
            elif not self._off_route and dist > OFF_ROUTE_THRESHOLD_M:
                self._off_route = True
                import logging
                logging.getLogger(__name__).info(
                    "[%s] Vehicle went OFF-ROUTE (%.0fm from polyline). Holding at GPS position.",
                    self.device_id, dist
                )
            # --- Re-engage route (<40m) ---
            elif self._off_route and dist < ON_ROUTE_THRESHOLD_M:
                self._off_route = False
                self._bypass_prediction = False
                # Re-initialize route follower for clean handoff
                self.route_follower._initialized = False
                self.route_follower._direction_verified = False  # Allow auto-reverse check
                import logging
                logging.getLogger(__name__).info(
                    "[%s] Vehicle returned ON-ROUTE (%.0fm from polyline). Resuming route following.",
                    self.device_id, dist
                )

        # --- END-OF-ROUTE DETECTION ---
        # Release to off-route mode immediately if vehicle reaches polyline end but keeps moving.
        if (self.route_follower
                and not self._off_route
                and self.route_follower.at_route_end
                and speed_mps is not None
                and speed_mps > 1.0):  # 3.6 km/h — clearly still driving
            self._off_route = True
            import logging
            logging.getLogger(__name__).info(
                "[%s] Vehicle reached END OF ROUTE and still moving (%.1f m/s). "
                "Switching to off-route mode.", self.device_id, speed_mps
            )

        # --- GPS SNAP TO ROUTE ---
        # Project raw GPS within 50m onto nearest route segment to filter noise.
        MAX_SNAP_DIST_M = 50.0
        if self.route_follower and not self._off_route:
            snapped_pos = self.route_follower.snap_to_route(gps_pos)
            snap_dist = (snapped_pos - gps_pos).magnitude()
            if snap_dist <= MAX_SNAP_DIST_M:
                gps_pos = snapped_pos

        # --- SPEED INFERENCE FROM POSITION DELTA ---
        # Infer speed from position delta if GPS incorrectly reports 0 speed while moving.
        position_delta_for_speed = (gps_pos - self.last_gps_position).magnitude()
        if (speed_mps is not None
                and speed_mps < 0.3
                and dt > 2.0
                and position_delta_for_speed > 5.0):
            inferred_speed = position_delta_for_speed / dt
            # Cap inferred speed at a reasonable max (100 km/h for bus)
            inferred_speed = min(inferred_speed, 27.78)
            speed_mps = inferred_speed

        # --- STOP ---
        if speed_mps is not None and speed_mps < 0.3:
            self.position = gps_pos
            self.velocity = Vec2.zero()
            self.acceleration = Vec2.zero()
            self.last_confirmed_speed_mps = 0.0
            self.confidence = 1.0
            self.dead_reckon_dist = 0.0
            # Preserve last_heading_dir so vehicle can instantly resume moving.
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
                        logging.getLogger(__name__).info("[%s] Auto-reversing route follower for inbound vehicle!", self.device_id)
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

        # --- Confidence decay (computed BEFORE velocity decisions) ---
        if time_since_gps <= DEAD_RECKON_START_SEC:
            self.confidence = 0.5 ** (time_since_gps / DEAD_RECKON_START_SEC)
        else:
            extra = time_since_gps - DEAD_RECKON_START_SEC
            self.confidence = 0.5 * (CONFIDENCE_DECAY_PER_SEC ** extra)
        self.confidence = max(0.0, min(1.0, self.confidence))

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

        # --- Brake bias (only during dead reckoning) ---
        if dead_reckoning and self.confidence < 0.43:
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
            # Deliberately NOT clearing self.last_heading_dir here so the 
            # vehicle can resume moving instantly on the next GPS ping.

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
            # Use dead reckoning for off-route vehicle to maintain smooth movement.
            prev = self.position
            self.position = integrate_position(self.position, self.velocity, dt)
            self.dead_reckon_dist += (self.position - prev).magnitude()
            
        else:
            # No route available - use normal physics-based position
            prev = self.position
            self.position = integrate_position(self.position, self.velocity, dt)
            self.dead_reckon_dist += (self.position - prev).magnitude()
        
        # MARKER POSITION CALCULATION
        
        # Calculate where marker will appear on screen (with display lag)
        

        
        if self.marker_position is None:
            self.marker_position = self.position
            self.marker_velocity = self.velocity
        else:
            alpha = 1 - math.exp(-8 * dt)

            self.marker_position += (self.position - self.marker_position) * alpha
            self.marker_velocity += (self.velocity - self.marker_velocity) * alpha

        self.last_update_ts = now_ts