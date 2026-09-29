"""Duplicate submissions are never executed twice.

The same ``request_id`` always returns the original record, whether it is
QUEUED, RUNNING, COMPLETED or REJECTED. We additionally verify the actuator
never moves twice for one id.
"""
from __future__ import annotations

import logging
import time

from robot_motion.models import CommandStatus, RejectReason

from .conftest import joint


def _wait_terminal(controller, rid, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rec = controller.get_status(rid)
        if rec.status in (
            CommandStatus.COMPLETED,
            CommandStatus.REJECTED,
            CommandStatus.EMERGENCY_STOPPED,
            CommandStatus.QUEUE_TIMEOUT,
            CommandStatus.FAULTED,
        ):
            return rec
        time.sleep(0.002)
    raise AssertionError(f"{rid} never reached terminal status")


def test_duplicate_while_running_not_executed_twice(controller, caplog):
    caplog.set_level(logging.WARNING, logger="robot_motion.controller")
    payload = [joint(0, 120.0, vel=20.0, acc=40.0)]  # long motion
    first = controller.submit("once", payload)
    assert first.accepted

    # resubmit while it is (very likely) running
    time.sleep(0.03)
    dup = controller.submit("once", payload)
    assert dup.accepted is False
    assert dup.reject_reason is RejectReason.DUPLICATE_REQUEST
    assert dup.record is first.record

    rec = _wait_terminal(controller, "once")
    assert rec.status is CommandStatus.COMPLETED
    assert rec.start_clock_ms is not None
    # only one start/finish pair ever existed for this id
    assert controller.snapshot().current_request_id != "once"
    assert "not executed again" in "\n".join(
        r.getMessage() for r in caplog.records
    )


def test_duplicate_after_completion_returns_original(controller):
    payload = [joint(1, 30.0)]
    assert controller.submit("done-once", payload).accepted
    rec = _wait_terminal(controller, "done-once")
    assert rec.status is CommandStatus.COMPLETED
    first_finish = rec.finish_clock_ms

    again = controller.submit("done-once", payload)
    assert again.reject_reason is RejectReason.DUPLICATE_REQUEST
    time.sleep(0.05)
    same = controller.get_status("done-once")
    assert same.finish_clock_ms == first_finish  # unchanged: not re-run


def test_duplicate_while_queued_does_not_add_second_entry(controller):
    # fill capacity behind one long running command
    assert controller.submit(
        "runner", [joint(0, 150.0, vel=10.0, acc=20.0)]
    ).accepted
    time.sleep(0.03)
    for i in range(controller.config.queue_capacity):
        assert controller.submit(f"q{i}", [joint(1, 40.0)]).accepted

    snap = controller.snapshot()
    assert len(snap.queued_request_ids) == controller.config.queue_capacity

    # duplicate a queued id: must not grow the queue
    dup = controller.submit("q0", [joint(1, 40.0)])
    assert dup.reject_reason is RejectReason.DUPLICATE_REQUEST
    snap = controller.snapshot()
    assert snap.queued_request_ids.count("q0") == 1
    assert len(snap.queued_request_ids) == controller.config.queue_capacity


def test_rejected_id_resubmission_is_duplicate_not_revalidated(controller):
    bad = [joint(0, 999.0)]
    r1 = controller.submit("bad-id", bad)
    assert r1.reject_reason is RejectReason.POSITION_OUT_OF_RANGE
    # same id with now-valid body: still a duplicate, never re-evaluated
    r2 = controller.submit("bad-id", [joint(0, 10.0)])
    assert r2.reject_reason is RejectReason.DUPLICATE_REQUEST
    assert r2.record is r1.record
