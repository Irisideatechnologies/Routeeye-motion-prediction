from __future__ import annotations
import math

from motion_prediction.math.vector import Vec2




def integrate_position(
    position: Vec2,
    velocity: Vec2,
    dt: float,
) -> Vec2:
    """
    First-order position integration.
    """
    return position + velocity * dt


def integrate_velocity(
    velocity: Vec2,
    acceleration: Vec2,
    dt: float,
) -> Vec2:
    """
    First-order velocity integration.
    """
    return velocity + acceleration * dt



# Soft constraints


def apply_acceleration_limit(
    accel: Vec2,
    max_accel: float,
) -> Vec2:
    
    return accel.limit(max_accel)


def apply_velocity_damping(
    velocity: Vec2,
    damping: float,
) -> Vec2:
    
    return velocity * damping


def limit_turn_rate(
    velocity: Vec2,
    desired_velocity: Vec2,
    max_turn_rate_rad: float,
    dt: float,
) -> Vec2:
    

    if velocity.is_near_zero():
        return desired_velocity

    v_mag = velocity.magnitude()
    d_mag = desired_velocity.magnitude()

    if d_mag < 1e-6:
        return Vec2.zero()

    # Current & desired headings
    h0 = velocity.heading_rad()
    h1 = desired_velocity.heading_rad()

    # Shortest angular difference
    dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))

    max_dh = max_turn_rate_rad * dt

    if abs(dh) <= max_dh:
        return desired_velocity

    # Rotate velocity by allowed delta
    new_heading = h0 + math.copysign(max_dh, dh)

    return Vec2(
        math.cos(new_heading) * d_mag,
        math.sin(new_heading) * d_mag,
    )


# Motion intent heuristics


def infer_acceleration(
    prev_velocity: Vec2,
    new_velocity: Vec2,
    dt: float,
) -> Vec2:
    
    if dt <= 0:
        return Vec2.zero()
    return (new_velocity - prev_velocity) / dt


def apply_braking_bias(
    velocity: Vec2,
    brake_strength: float,
    dt: float,
) -> Vec2:
    
    speed = velocity.magnitude()
    if speed < 0.1:
        return Vec2.zero()

    decel = min(brake_strength * dt, speed)
    return velocity * ((speed - decel) / speed)
