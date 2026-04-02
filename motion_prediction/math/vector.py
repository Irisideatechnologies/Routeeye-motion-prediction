from __future__ import annotations
from dataclasses import dataclass
import math


_EPS = 1e-9


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    # ---------- Basic arithmetic ----------

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Vec2":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vec2":
        if abs(scalar) < _EPS:
            raise ZeroDivisionError("Division by zero in Vec2")
        return Vec2(self.x / scalar, self.y / scalar)

    # ---------- Vector properties ----------

    def magnitude(self) -> float:
        """Euclidean length"""
        return math.hypot(self.x, self.y)

    def magnitude_sq(self) -> float:
        """Squared magnitude (avoids sqrt)"""
        return self.x * self.x + self.y * self.y

    def normalized(self) -> "Vec2":
        """Unit vector (returns zero vector if magnitude ~ 0)"""
        mag = self.magnitude()
        if mag < _EPS:
            return Vec2(0.0, 0.0)
        return self / mag

    # ---------- Direction & angles ----------

    def heading_rad(self) -> float:
        """Angle in radians, range [-pi, pi]"""
        return math.atan2(self.y, self.x)

    def heading_deg(self) -> float:
        """Angle in degrees, range [-180, 180]"""
        return math.degrees(self.heading_rad())

    # ---------- Projections & limits ----------

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def limit(self, max_magnitude: float) -> "Vec2":
        """Clamp vector magnitude without changing direction"""
        mag = self.magnitude()
        if mag <= max_magnitude:
            return self
        if mag < _EPS:
            return Vec2(0.0, 0.0)
        return self * (max_magnitude / mag)

    # ---------- Utilities ----------

    def is_near_zero(self, eps: float = 1e-6) -> bool:
        return abs(self.x) < eps and abs(self.y) < eps

    @staticmethod
    def zero() -> "Vec2":
        return Vec2(0.0, 0.0)
