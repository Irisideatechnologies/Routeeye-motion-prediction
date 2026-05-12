from __future__ import annotations
from typing import Optional, Tuple, List
import math

from motion_prediction.math.vector import Vec2
from motion_prediction.core.route_geometry import RouteGeometry

GPS_POSITION_THRESHOLD_M = 10.0

# How many segments backward from the current position we allow GPS correction.
# This prevents the bus from teleporting to a parallel return lane while still
# allowing minor GPS jitter corrections.
BACKWARD_SEARCH_SEGMENTS = 3

# Maximum forward jump in segments allowed from a single GPS fix.
# Prevents matching to a topologically distant but geometrically close segment.
FORWARD_SEARCH_SEGMENTS = 30


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

        # --- Monotonic route-distance tracking ---
        # Pre-compute cumulative arc-length at each waypoint for O(1) distance lookups.
        # _cumulative_dist[i] = total distance from polyline[0] to polyline[i].
        self._cumulative_dist: List[float] = self._build_cumulative_distances()

        # Current scalar position along the route (meters from start).
        self._current_route_dist: float = 0.0

        # Total route length for convenience.
        self._total_route_length: float = (
            self._cumulative_dist[-1] if self._cumulative_dist else 0.0
        )

        # Whether locate_on_route has been called at least once (first call
        # uses global search to establish initial position).
        self._initialized: bool = False

    # ------------------------------------------------------------------ #
    #  Pre-computation
    # ------------------------------------------------------------------ #

    def _build_cumulative_distances(self) -> List[float]:
        """Build a cumulative arc-length table for the polyline."""
        polyline = self.route.polyline
        if len(polyline) < 2:
            return [0.0] * len(polyline)

        cumulative = [0.0]
        for i in range(1, len(polyline)):
            seg_len = (polyline[i] - polyline[i - 1]).magnitude()
            cumulative.append(cumulative[-1] + seg_len)
        return cumulative

    def _segment_start_dist(self, seg_idx: int) -> float:
        """Get the cumulative distance at the start of segment seg_idx."""
        return self._cumulative_dist[seg_idx]

    def _route_dist_at(self, seg_idx: int, progress: float) -> float:
        """Get an absolute route distance for a (segment, progress) pair."""
        if seg_idx >= len(self.route.polyline) - 1:
            return self._cumulative_dist[-1]
        seg_len = self._cumulative_dist[seg_idx + 1] - self._cumulative_dist[seg_idx]
        return self._cumulative_dist[seg_idx] + seg_len * progress

    # ------------------------------------------------------------------ #
    #  Core: locate on route (monotonic-forward search)
    # ------------------------------------------------------------------ #

    def locate_on_route(self, position: Vec2) -> Tuple[Vec2, int, float]:

        if len(self.route.polyline) < 2:
            return position, 0, 0.0

        num_segments = len(self.route.polyline) - 1

        # --- First call: global search to seed initial position ---
        if not self._initialized:
            best_point, best_seg, best_prog = self._global_locate(position)
            self._initialized = True
            self._current_segment_idx = best_seg
            self._segment_progress = best_prog
            self._current_route_dist = self._route_dist_at(best_seg, best_prog)
            self._last_route_point = best_point
            dist = (position - best_point).magnitude()
            if dist <= GPS_POSITION_THRESHOLD_M:
                return best_point, best_seg, best_prog
            return position, best_seg, best_prog

        # --- Subsequent calls: windowed search around current position ---
        search_start = max(0, self._current_segment_idx - BACKWARD_SEARCH_SEGMENTS)
        search_end = min(num_segments, self._current_segment_idx + FORWARD_SEARCH_SEGMENTS)

        min_distance = float('inf')
        best_point = position
        best_segment = self._current_segment_idx
        best_progress = self._segment_progress

        for i in range(search_start, search_end):
            p1 = self.route.polyline[i]
            p2 = self.route.polyline[i + 1]

            point, progress = self._project_onto_segment(position, p1, p2)
            distance = (position - point).magnitude()

            candidate_dist = self._route_dist_at(i, progress)

            # --- Monotonic constraint ---
            # Only accept candidates that are at or ahead of the current
            # position minus a small backward tolerance (for GPS jitter).
            backward_tolerance = 15.0  # meters
            if candidate_dist < self._current_route_dist - backward_tolerance:
                continue

            if distance < min_distance:
                min_distance = distance
                best_point = point
                best_segment = i
                best_progress = progress

        # Update state — route distance can only move forward (with jitter tolerance)
        new_route_dist = self._route_dist_at(best_segment, best_progress)
        if new_route_dist >= self._current_route_dist - 15.0:
            self._current_route_dist = max(self._current_route_dist, new_route_dist)

        self._current_segment_idx = best_segment
        self._segment_progress = best_progress
        self._last_route_point = best_point

        if min_distance <= GPS_POSITION_THRESHOLD_M:
            return best_point, best_segment, best_progress
        return position, best_segment, best_progress

    def _global_locate(self, position: Vec2) -> Tuple[Vec2, int, float]:
        """Unrestricted global search — used only for initialisation."""
        min_distance = float('inf')
        best_point = position
        best_segment = 0
        best_progress = 0.0

        for i in range(len(self.route.polyline) - 1):
            p1 = self.route.polyline[i]
            p2 = self.route.polyline[i + 1]

            point, progress = self._project_onto_segment(position, p1, p2)
            distance = (position - point).magnitude()

            if distance < min_distance:
                min_distance = distance
                best_point = point
                best_segment = i
                best_progress = progress

        return best_point, best_segment, best_progress

    # ------------------------------------------------------------------ #
    #  Advance along route
    # ------------------------------------------------------------------ #

    def nearest_route_distance(self, position: Vec2) -> float:
        """Return distance from position to nearest point on route polyline.
        
        This is a READ-ONLY method — it does NOT mutate any internal state.
        Used for off-route detection before deciding whether to engage route following.
        """
        if len(self.route.polyline) < 2:
            return float('inf')

        min_dist = float('inf')
        for i in range(len(self.route.polyline) - 1):
            p1 = self.route.polyline[i]
            p2 = self.route.polyline[i + 1]
            point, _ = self._project_onto_segment(position, p1, p2)
            dist = (position - point).magnitude()
            if dist < min_dist:
                min_dist = dist
            if min_dist < 1.0:  # Close enough, no need to keep searching
                break
        return min_dist

    def advance_along_route(
            self,
            current_position: Vec2,
            speed: float,
            dt: float,
            confidence: float,
    ) -> Tuple[Vec2, Vec2]:

        # ALWAYS locate on route (project onto polyline — now windowed)
        route_point, segment_idx, progress = self.locate_on_route(current_position)

        # Check if extremely far from route (>100m)
        distance_from_route = (current_position - route_point).magnitude()
        if distance_from_route > 100.0:
            # Vehicle is way off-route — don't snap, just return current position
            return current_position, Vec2.zero()


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

        # Rebuild cumulative distances and reset route distance
        self._cumulative_dist = self._build_cumulative_distances()
        self._total_route_length = (
            self._cumulative_dist[-1] if self._cumulative_dist else 0.0
        )
        self._current_route_dist = 0.0
        self._initialized = False

    # ------------------------------------------------------------------ #
    #  Traverse route (unchanged logic, now updates route distance)
    # ------------------------------------------------------------------ #

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

        # Update monotonic route distance (can only increase via traversal)
        new_dist = self._route_dist_at(current_segment, current_progress)
        self._current_route_dist = max(self._current_route_dist, new_dist)

        return final_position, direction

    # ------------------------------------------------------------------ #
    #  Segment projection (unchanged)
    # ------------------------------------------------------------------ #

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

    # ------------------------------------------------------------------ #
    #  Query helpers
    # ------------------------------------------------------------------ #

    def get_distance_from_route(self, position: Vec2) -> float:

        if self._last_route_point is None:
            self.locate_on_route(position)

        return (position - self._last_route_point).magnitude()

    def get_route_progress(self) -> float:
        """Return the current progress along the route as a fraction 0.0–1.0."""
        if self._total_route_length < 1e-6:
            return 0.0
        return min(1.0, self._current_route_dist / self._total_route_length)

    def get_route_distance(self) -> float:
        """Return the current absolute distance along the route in meters."""
        return self._current_route_dist

    def is_near_route_end(self, threshold_meters: float = 50.0) -> bool:

        if len(self.route.polyline) < 2:
            return False

        remaining = self._total_route_length - self._current_route_dist
        return remaining < threshold_meters