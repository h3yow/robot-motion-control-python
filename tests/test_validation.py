"""Out-of-range / malformed requests are rejected with distinguishable reasons.

Each test also captures the decision log and asserts that the request
parameters and the basis for the rejection are printed (explainability).
"""
from __future__ import annotations

import logging

import pytest

from robot_motion.models import RejectReason

from .conftest import joint


@pytest.mark.parametrize(
    "payload,expected",
    [
        ([joint(0, 181.0)], RejectReason.POSITION_OUT_OF_RANGE),
        ([joint(0, -181.0)], RejectReason.POSITION_OUT_OF_RANGE),
        ([joint(1, 91.0)], RejectReason.POSITION_OUT_OF_RANGE),
        ([joint(2, 90.0, vel=121.0)], RejectReason.VELOCITY_OUT_OF_RANGE),
        ([joint(0, 90.0, vel=101.0)], RejectReason.VELOCITY_OUT_OF_RANGE),
        ([joint(0, 90.0, acc=201.0)], RejectReason.ACCELERATION_OUT_OF_RANGE),
        ([joint(2, 10.0, acc=241.0)], RejectReason.ACCELERATION_OUT_OF_RANGE),
    ],
)
def test_parameter_violations_are_distinguishable(
    controller, caplog, payload, expected
):
    caplog.set_level(logging.WARNING, logger="robot_motion.controller")
    rid = f"bad-{expected.value}-{payload[0]['joint_id']}-{payload[0]['target_position']}"
    result = controller.submit(rid, payload)

    assert result.accepted is False
    assert result.reject_reason is expected
    assert result.record is not None
    assert result.record.status.value == "REJECTED"
    assert expected.value in result.message or expected.value == result.reject_reason.value

    # explainability: request params + basis must appear in logs
    joined = "\n".join(r.getMessage() for r in caplog.records)
    assert rid in joined
    assert "reason=" + expected.value in joined
    assert "basis=" in joined


@pytest.mark.parametrize(
    "payload,fragment",
    [
        ([], "non-empty"),
        ("not-a-list", "non-empty"),
        ([joint(0, 10)], "unknown"),  # joint id replaced below
        ([{"joint_id": 0, "target_position": 1}], "missing fields"),
        ([{"joint_id": "x", "target_position": 1, "velocity": 1, "acceleration": 1}],
         "non-numeric"),
        ([joint(0, 10, vel=-1)], "velocity must be > 0"),
        ([joint(0, 10, acc=0)], "acceleration must be > 0"),
    ],
)
def test_malformed_payloads(controller, payload, fragment):
    if fragment == "unknown":
        payload = [joint(42, 10)]
    result = controller.submit(f"mal-{fragment[:4]}", payload)
    assert result.reject_reason is RejectReason.INVALID_PAYLOAD
    assert fragment in result.message


def test_duplicate_joint_in_one_request(controller):
    payload = [joint(0, 10), joint(0, 20)]
    result = controller.submit("dup-joint", payload)
    assert result.reject_reason is RejectReason.INVALID_PAYLOAD
    assert "appears twice" in result.message


def test_boundary_values_are_accepted(controller):
    # exactly on the inclusive limits is legal
    for rid, jm in [
        ("b-180", joint(0, -180.0, vel=100.0, acc=200.0)),
        ("b-180hi", joint(0, 180.0, vel=100.0, acc=200.0)),
        ("b-v", joint(1, 0.0, vel=80.0, acc=160.0)),
        ("b-a", joint(2, 0.0, vel=120.0, acc=240.0)),
    ]:
        assert controller.submit(rid, [jm]).accepted, rid
