"""
Timebase utilities for fixed-timestep simulation.
"""

from __future__ import annotations
from typing import Iterator


def fixed_steps(
    last_ts: float,
    now_ts: float,
    dt: float,
) -> Iterator[float]:
    """
    Yield timestamps at fixed dt intervals between last_ts and now_ts.
    """
    t = last_ts
    while t + dt <= now_ts:
        t += dt
        yield t
