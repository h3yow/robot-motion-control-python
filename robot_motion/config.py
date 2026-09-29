"""Configuration: joint limits, queue, timing and simulated clock settings.

Units
-----
* position    : degrees
* velocity    : degrees / second
* acceleration: degrees / second**2
* time        : milliseconds (all controller/clock timestamps)

All items are frozen dataclasses; the controller never mutates its config.
Tests build their own ``MotionConfig`` to get deterministic, fast virtual time.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointLimits:
    """Hard limits for a single joint (inclusive bounds)."""

    joint_id: int
    min_position: float
    max_position: float
    max_velocity: float
    max_acceleration: float


@dataclass(frozen=True)
class MotionConfig:
    """Whole-controller configuration.

    Attributes
    ----------
    joint_limits:
        One entry per joint; ``joint_id`` values must be unique and contiguous
        starting at 0.
    queue_capacity:
        Maximum number of *waiting* commands (the running command is not
        counted). Submissions beyond this are rejected with QUEUE_FULL.
    queue_timeout_ms:
        A command that stays QUEUED for longer than this is cancelled by the
        engine with QUEUE_TIMEOUT (checked on every clock tick).
    clock_tick_ms:
        Virtual time advanced per tick of the simulated clock.
    clock_real_ms_per_tick:
        How many real milliseconds one tick sleeps for. ``0.0`` means "sleep
        zero" (tests still pace the ticker, deterministically), while e.g.
        ``10.0`` with ``clock_tick_ms=10`` makes virtual time run at real rate.
    """

    joint_limits: tuple[JointLimits, ...]
    queue_capacity: int
    queue_timeout_ms: int
    clock_tick_ms: int
    clock_real_ms_per_tick: float

    @staticmethod
    def default() -> "MotionConfig":
        """3-joint arm defaults; virtual time runs ~10x faster than real."""
        return MotionConfig(
            joint_limits=(
                JointLimits(0, -180.0, 180.0, 100.0, 200.0),
                JointLimits(1, -90.0, 90.0, 80.0, 160.0),
                JointLimits(2, -120.0, 120.0, 120.0, 240.0),
            ),
            queue_capacity=8,
            queue_timeout_ms=5_000,
            clock_tick_ms=10,
            clock_real_ms_per_tick=1.0,
        )

    def limits_for(self, joint_id: int) -> JointLimits | None:
        for lim in self.joint_limits:
            if lim.joint_id == joint_id:
                return lim
        return None
