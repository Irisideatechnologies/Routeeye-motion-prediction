from __future__ import annotations
from typing import Optional, Tuple
import math

from motion_prediction.math.vector import Vec2
from motion_prediction.core.route_geometry import RouteGeometry

GPS_POSITION_THRESHOLD_M = 10.0
class RouteFollower:


    def __init__(self, route: RouteGeometry):
        self.route = route

        # Current position on route
        self._current_segment_idx: int = 0
        self._segment_progress: float = 0.0  # 0.0 to 1.0 within segment

        
        self._last_route_point: Optional[Vec2] = None
        if len(route.polyline) >= 2:
            overall = route.polyline[-1] - route.polyline[0]
            self._forward_dir: Vec2 = overall.normalized() if overall.magnitude() > 1e-6 else Vec2(0.0, 1.0)
        else:
            self._forward_dir = Vec2(0.0, 1.0)

        self._is_reversed: bool = False
    def locate_on_route(self, position: Vec2) -> Tuple[Vec2, int, float]:

        if len(self.route.polyline) < 2:
            return position, 0, 0.0

        min_distance = float('inf')
        best_point = position
        best_segment = 0
        best_progress = 0.0

        # Check each segment of the polyline
        for i in range(len(self.route.polyline) - 1):
            p1 = self.route.polyline[i]
            p2 = self.route.polyline[i + 1]

            # Project position onto this segment
            point, progress = self._project_onto_segment(position, p1, p2)
            distance = (position - point).magnitude()

            if distance < min_distance:
                min_distance = distance
                best_point = point
                best_segment = i
                best_progress = progress

        self._current_segment_idx = best_segment
        self._segment_progress = best_progress
        self._last_route_point = best_point

        if min_distance <= GPS_POSITION_THRESHOLD_M:
            return best_point, best_segment, best_progress
        return position, best_segment, best_progress
    def advance_along_route(
            self,
            current_position: Vec2,
            speed: float,
            dt: float,
            confidence: float,
    ) -> Tuple[Vec2, Vec2]:


        # ALWAYS locate on route (project onto polyline)
        route_point, segment_idx, progress = self.locate_on_route(current_position)

        # Check if extremely far from route (>100m)
        distance_from_route = (current_position - route_point).magnitude()
        if distance_from_route > 100.0:
            
            # Trust route position, but zero velocity until GPS corrects
            return route_point, Vec2.zero()


        blend_factor = 0.3 + 0.7 * (1.0 - confidence)

        # Distance to travel this tick
        distance_to_travel = speed * dt

        # Advance along route from current position
        new_point, new_direction = self._traverse_route(
            segment_idx,
            progress,
            distance_to_travel
        )

        # Blend between current position and route position
        

        blended_position = current_position + (new_point - current_position) * blend_factor

        # Velocity ALWAYS follows route direction
        new_velocity = new_direction * speed

        return blended_position, new_velocity

    def reverse(self) -> None:
        
        
        self.route.polyline = list(reversed(self.route.polyline))
        self._is_reversed = not self._is_reversed
        self._forward_dir = Vec2(-self._forward_dir.x, -self._forward_dir.y)
        
        self._current_segment_idx = 0
        self._segment_progress = 0.0
        self._last_route_point = None
    def _traverse_route(
            self,
            start_segment: int,
            start_progress: float,
            distance: float,
    ) -> Tuple[Vec2, Vec2]:

        if len(self.route.polyline) < 2:
            return self.route.polyline[0], Vec2(0, 1)

        current_segment = start_segment
        current_progress = start_progress
        remaining_distance = distance

        # Traverse segments until distance exhausted
        while remaining_distance > 0 and current_segment < len(self.route.polyline) - 1:
            p1 = self.route.polyline[current_segment]
            p2 = self.route.polyline[current_segment + 1]

            segment_vector = p2 - p1
            segment_length = segment_vector.magnitude()

            if segment_length < 1e-6:
                # Degenerate segment, skip
                current_segment += 1
                current_progress = 0.0
                continue

            # Distance from current progress to end of segment
            distance_to_segment_end = segment_length * (1.0 - current_progress)

            if remaining_distance <= distance_to_segment_end:
                # We'll stop within this segment
                progress_delta = remaining_distance / segment_length
                current_progress += progress_delta
                remaining_distance = 0.0
            else:
                # Move to next segment
                remaining_distance -= distance_to_segment_end
                current_segment += 1
                current_progress = 0.0

        # Clamp to route end
        if current_segment >= len(self.route.polyline) - 1:
            current_segment = len(self.route.polyline) - 2
            current_progress = 1.0

        # Calculate final position
        p1 = self.route.polyline[current_segment]
        p2 = self.route.polyline[current_segment + 1]

        final_position = p1 + (p2 - p1) * current_progress

        # Calculate direction (tangent to route at this point)
        direction = (p2 - p1).normalized()

        # Update cached state
        self._current_segment_idx = current_segment
        self._segment_progress = current_progress
        self._last_route_point = final_position

        return final_position, direction

    def _project_onto_segment(
            self,
            point: Vec2,
            seg_start: Vec2,
            seg_end: Vec2,
    ) -> Tuple[Vec2, float]:

        segment = seg_end - seg_start
        segment_length_sq = segment.dot(segment)

        if segment_length_sq < 1e-12:
            # Degenerate segment
            return seg_start, 0.0

        # Project point onto infinite line
        to_point = point - seg_start
        t = to_point.dot(segment) / segment_length_sq

        # Clamp to segment bounds
        t = max(0.0, min(1.0, t))

        projected = seg_start + segment * t

        return projected, t

    def get_distance_from_route(self, position: Vec2) -> float:

        if self._last_route_point is None:
            self.locate_on_route(position)

        return (position - self._last_route_point).magnitude()

    def is_near_route_end(self, threshold_meters: float = 50.0) -> bool:

        if len(self.route.polyline) < 2:
            return False

        # Check if on last segment and near the end
        is_last_segment = self._current_segment_idx >= len(self.route.polyline) - 2

        if is_last_segment and self._last_route_point:
            end_point = self.route.polyline[-1]
            distance_to_end = (self._last_route_point - end_point).magnitude()
            return distance_to_end < threshold_meters

        return False