"""Safety interlock, emergency stop, external fault, reset, queue timeout."""
from __future__ import annotations

import logging
import time

from robot_motion.config import JointLimits, MotionConfig
from robot_motion.controller import MotionController
from robot_motion.models import CommandStatus, MotionState, RejectReason

from .conftest import fast_config, joint, wait_for_status


def test_estop_freezes_running_motion_immediately(controller, caplog):
    caplog.set_level(logging.WARNING)
    r = controller.submit(
        "moving", [joint(0, 150.0, vel=10.0, acc=20.0)]
    )
    assert r.accepted
    wait_for_status(controller, "moving", CommandStatus.RUNNING)
    time.sleep(0.05)  # let it move partway

    before = {s.joint_id: s.position for s in controller.snapshot().joints}
    controller.emergency_stop()
    time.sleep(0.05)  # give the frozen joint time: it must not move
    after = {s.joint_id: s.position for s in controller.snapshot().joints}

    rec = controller.get_status("moving")
    assert rec.status is CommandStatus.EMERGENCY_STOPPED
    assert controller.snapshot().state is MotionState.ESTOPPED
    # frozen: position did not keep drifting towards 150
    assert abs(after[0] - before[0]) < 1e-9, (before, after)
    assert abs(after[0]) < 150.0  # stopped before reaching target


def test_estop_cancels_entire_queue(controller):
    assert controller.submit(
        "runner", [joint(0, 160.0, vel=10.0, acc=20.0)]
    ).accepted
    time.sleep(0.05)
    queued_ids = []
    for i in range(3):
        rid = f"q{i}"
        assert controller.submit(rid, [joint(1, 30.0)]).accepted
        queued_ids.append(rid)

    controller.emergency_stop()
    snap = controller.snapshot()
    assert snap.queued_request_ids == []
    assert controller.get_status("runner").status is CommandStatus.EMERGENCY_STOPPED
    for rid in queued_ids:
        assert controller.get_status(rid).status is CommandStatus.EMERGENCY_STOPPED


def test_no_command_executes_until_reset(controller):
    assert controller.submit(
        "runner", [joint(0, 100.0, vel=20.0, acc=40.0)]
    ).accepted
    time.sleep(0.03)
    controller.emergency_stop()

    r = controller.submit("blocked", [joint(1, 10.0)])
    assert r.reject_reason is RejectReason.INTERLOCK_ENGAGED
    assert r.record is None or r.record.status is CommandStatus.REJECTED
    time.sleep(0.1)
    # still tripped, still nothing running
    assert controller.snapshot().state is MotionState.ESTOPPED
    assert controller.snapshot().current_request_id is None

    controller.reset()
    assert controller.snapshot().state is MotionState.IDLE
    # and new commands execute again from the frozen position
    ok = controller.submit("after", [joint(1, 20.0)])
    assert ok.accepted
    wait_for_status(controller, "after", CommandStatus.COMPLETED)


def test_reset_is_stable_and_rejects_when_running(controller):
    # reset from IDLE is a no-op refusal
    assert controller.reset() is None  # method returns None either way
    # start a motion then try resetting without estop: refused, keeps running
    assert controller.submit("r", [joint(0, 60.0, vel=20.0, acc=40.0)]).accepted
    wait_for_status(controller, "r", CommandStatus.RUNNING)
    controller.reset()  # must not clear RUNNING
    assert controller.snapshot().state is MotionState.RUNNING


def test_external_fault_then_reset(controller):
    assert controller.submit(
        "f-run", [joint(0, 100.0, vel=20.0, acc=40.0)]
    ).accepted
    wait_for_status(controller, "f-run", CommandStatus.RUNNING)
    controller.external_fault()
    assert controller.snapshot().state is MotionState.FAULT
    assert controller.get_status("f-run").status is CommandStatus.FAULTED
    blocked = controller.submit("f-blocked", [joint(1, 10.0)])
    assert blocked.reject_reason is RejectReason.INTERLOCK_ENGAGED
    controller.reset()
    assert controller.snapshot().state is MotionState.IDLE


def test_estop_is_idempotent(controller):
    controller.emergency_stop()
    controller.emergency_stop()
    assert controller.snapshot().state is MotionState.ESTOPPED
    controller.reset()
    assert controller.snapshot().state is MotionState.IDLE


def test_queue_timeout_cancels_deterministically():
    cfg = fast_config(queue_capacity=3, queue_timeout_ms=300)
    ctrl = MotionController(cfg)
    try:
        # long runner on joint 0
        assert ctrl.submit(
            "slow", [joint(0, 170.0, vel=8.0, acc=16.0)]
        ).accepted
        time.sleep(0.05)
        # fill the queue; these will wait longer than 300 virtual ms
        ids = []
        for i in range(3):
            rid = f"wait{i}"
            assert ctrl.submit(rid, [joint(1, 10.0)]).accepted
            ids.append(rid)
        for rid in ids:
            wait_for_status(
                ctrl, rid, CommandStatus.QUEUE_TIMEOUT, timeout=6.0
            )
            rec = ctrl.get_status(rid)
            assert "queued for" in rec.detail and "ms > limit 300ms" in rec.detail
        assert ctrl.snapshot().queued_request_ids == []
    finally:
        ctrl.close()


def test_completed_motion_returns_to_idle(controller):
    assert controller.submit("fin", [joint(2, 45.0)]).accepted
    wait_for_status(controller, "fin", CommandStatus.COMPLETED)
    snap = controller.snapshot()
    assert snap.state is MotionState.IDLE
    assert snap.current_request_id is None
    pos = {j.joint_id: j.position for j in snap.joints}[2]
    assert abs(pos - 45.0) < 1e-9
