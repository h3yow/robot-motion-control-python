"""Shared pytest fixtures and logging configuration.

Tests run on a *fast deterministic* simulated clock: 10 virtual ms per tick,
zero real sleep per tick, so motions of e.g. 1000 virtual ms take only a few
real milliseconds. All timing decisions still use the simulated clock.
"""
from __future__ import annotations

import logging
import sys

import pytest

from robot_motion.config import JointLimits, MotionConfig
from robot_motion.controller import MotionController

# Print request parameters and decision basis in test logs.
logging.basicConfig(
    level=logging.INFO,
    stream=sys.stderr,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)


def fast_config(
    queue_capacity: int = 4,
    queue_timeout_ms: int = 20_000,
    real_ms_per_tick: float = 0.05,
) -> MotionConfig:
    return MotionConfig(
        joint_limits=(
            JointLimits(0, -180.0, 180.0, 100.0, 200.0),
            JointLimits(1, -90.0, 90.0, 80.0, 160.0),
            JointLimits(2, -120.0, 120.0, 120.0, 240.0),
        ),
        queue_capacity=queue_capacity,
        queue_timeout_ms=queue_timeout_ms,
        clock_tick_ms=10,
        clock_real_ms_per_tick=real_ms_per_tick,
    )


def joint(jid: int, target: float, vel: float = 50.0, acc: float = 100.0) -> dict:
    return {
        "joint_id": jid,
        "target_position": target,
        "velocity": vel,
        "acceleration": acc,
    }


@pytest.fixture
def config() -> MotionConfig:
    return fast_config()


@pytest.fixture
def controller(config: MotionConfig) -> MotionController:
    ctrl = MotionController(config)
    yield ctrl
    ctrl.close()


def wait_for_status(
    ctrl: MotionController,
    request_id: str,
    expected: object,
    timeout: float = 5.0,
    poll: float = 0.002,
):
    """Poll until the command reaches ``expected`` status or fail."""
    import time

    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        rec = ctrl.get_status(request_id)
        last = rec.status if rec else None
        if last == expected:
            return rec
        time.sleep(poll)
    raise AssertionError(
        f"request {request_id!r} did not reach {expected}; last={last}"
    )
