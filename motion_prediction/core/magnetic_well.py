from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional

from motion_prediction.math.vector import Vec2


@dataclass
class BusStop:
    """Represents a bus stop with magnetic well properties"""
    stop_id: str
    position: Vec2  # In local XY coordinates

    # Behavioral thresholds
    awareness_radius: float = 100.0  # Marker knows stop is coming (~14s at 25 km/h)
    slowdown_radius: float = 40.0  # Start active deceleration (~5.7s at 25 km/h)
    snap_radius: float = 8.0  # Snap to exact stop position


@dataclass
class StopApproachState:
    """Tracks the current state of approaching a stop"""
    stop: BusStop
    distance: float
    phase: str  # "AWARE", "SLOWING", "SNAPPED", "STOPPED"


class MagneticWell:



    def __init__(self):
        self._stops: List[BusStop] = []
        self._approach_state: Optional[StopApproachState] = None
        self._is_stopped: bool = False

    def set_stops(self, stops: List[BusStop]) -> None:
        """Update the list of bus stops for this route"""
        self._stops = stops

    def apply_well(
            self,
            position: Vec2,
            velocity: Vec2,
            gps_speed_mps: Optional[float],
    ) -> tuple[Vec2, Vec2]:


        # --- Check for departure from stopped state ---
        if self._is_stopped:
            if gps_speed_mps is not None and gps_speed_mps > 0.8:  # 2.88 km/h (lower for 25 km/h max)
                # Bus is moving again - release from stop
                self._is_stopped = False
                self._approach_state = None
                return position, velocity
            else:
                # Still stopped - hold at stop position
                if self._approach_state and self._approach_state.stop:
                    return self._approach_state.stop.position, Vec2.zero()
                return position, Vec2.zero()

        # --- Find next stop ahead ---
        next_stop = self._find_next_stop_ahead(position, velocity)
        if next_stop is None:
            self._approach_state = None
            return position, velocity

        stop, distance = next_stop

        # --- Determine approach phase ---
        if distance > stop.awareness_radius:
            # Too far - no effect
            self._approach_state = None
            return position, velocity

        elif distance > stop.slowdown_radius:
            # PHASE 1: AWARE (100m - 50m)
            # Marker knows stop is coming but no deceleration yet
            self._approach_state = StopApproachState(
                stop=stop,
                distance=distance,
                phase="AWARE"
            )
            return position, velocity

        elif distance > stop.snap_radius:
            # PHASE 2: SLOWING (50m - 10m)
            # Active deceleration aligned with GPS speed
            self._approach_state = StopApproachState(
                stop=stop,
                distance=distance,
                phase="SLOWING"
            )

            # Calculate target speed based on distance
            # Linear deceleration: speed reduces proportionally to distance
            # At 50m: current GPS speed
            # At 10m: near zero
            decel_progress = (distance - stop.snap_radius) / (stop.slowdown_radius - stop.snap_radius)

            # Target speed decreases as we approach
            if gps_speed_mps is not None:
                target_speed = gps_speed_mps * decel_progress
            else:
                # No GPS - use current velocity
                target_speed = velocity.magnitude() * decel_progress

            # Apply speed reduction while maintaining direction
            if velocity.magnitude() > 0.1:
                adjusted_velocity = velocity.normalized() * target_speed
            else:
                adjusted_velocity = velocity

            # Gentle position pull toward stop (prevent overshoot)
            to_stop = stop.position - position
            pull_strength = (1.0 - decel_progress) * 0.2  # Up to 20% pull
            adjusted_position = position + (to_stop * pull_strength * 0.1)

            return adjusted_position, adjusted_velocity

        else:
            # PHASE 3: SNAPPED (< 10m)
            # Lock to exact stop position
            self._approach_state = StopApproachState(
                stop=stop,
                distance=distance,
                phase="SNAPPED"
            )

            # Check if GPS confirms we've stopped
            if gps_speed_mps is not None and gps_speed_mps < 0.3:  # 1.08 km/h (lower for city buses)
                # PHASE 4: STOPPED
                self._is_stopped = True
                return stop.position, Vec2.zero()

            # Approaching stop but not fully stopped yet
            # Snap position, but allow tiny movement until GPS confirms stop
            return stop.position, velocity * 0.1

    def _find_next_stop_ahead(
            self,
            position: Vec2,
            velocity: Vec2
    ) -> Optional[tuple[BusStop, float]]:

        if not self._stops:
            return None

        # Movement direction
        if velocity.magnitude() < 0.1:
            # Not moving - just find nearest
            return self._find_nearest_stop(position)

        direction = velocity.normalized()

        # Find stops that are ahead of us
        candidates = []
        for stop in self._stops:
            to_stop = stop.position - position
            distance = to_stop.magnitude()

            # Check if stop is generally ahead (dot product > 0)
            if to_stop.dot(direction) > 0:
                candidates.append((stop, distance))

        if not candidates:
            return None

        # Return closest stop ahead
        return min(candidates, key=lambda x: x[1])

    def _find_nearest_stop(self, position: Vec2) -> Optional[tuple[BusStop, float]]:
        """Find the nearest stop to given position"""
        if not self._stops:
            return None

        nearest = None
        min_distance = float('inf')

        for stop in self._stops:
            dist = (stop.position - position).magnitude()
            if dist < min_distance:
                min_distance = dist
                nearest = stop

        return (nearest, min_distance) if nearest else None

    def is_stopped_at_stop(self) -> bool:
        """Check if marker is currently stopped at a bus stop"""
        return self._is_stopped

    def get_approach_state(self) -> Optional[StopApproachState]:
        """Get current approach state for debugging/monitoring"""
        return self._approach_state