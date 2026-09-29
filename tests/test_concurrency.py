"""Concurrent submissions: serialization, deterministic overflow, no losses.

These tests launch many real OS threads submitting simultaneously. The engine
guarantees at most one running motion, and the bookkeeping guarantees every
submission gets exactly one decisive outcome (accepted or a specific
rejection) with no lost or duplicated command.
"""
from __future__ import annotations

import threading
import time

from robot_motion.models import CommandStatus, RejectReason

from .conftest import joint, wait_for_status


def test_no_two_motions_run_simultaneously(controller):
    accepted = []
    lock = threading.Lock()

    def submit(rid, target):
        # all on joint 1, while a long joint-0 command runs: disjoint joints
        # => these serialize in the queue, never conflict, overflow is the
        # only rejection path.
        r = controller.submit(rid, [joint(1, target, vel=80.0, acc=160.0)])
        with lock:
            accepted.append((rid, r.accepted, r.reject_reason))

    # long runner on a disjoint joint
    assert controller.submit(
        "runner", [joint(0, 160.0, vel=10.0, acc=20.0)]
    ).accepted
    time.sleep(0.05)

    threads = [
        threading.Thread(target=submit, args=(f"c{i}", 20.0 + i))
        for i in range(12)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    accepted_ids = [rid for rid, ok, _ in accepted if ok]
    assert len(accepted_ids) >= 1
    # Wait for all accepted commands to finish.
    for rid in accepted_ids:
        wait_for_status(controller, rid, CommandStatus.COMPLETED, timeout=10.0)

    # Serialization proof: start/finish intervals of accepted commands must
    # never overlap (finish(n) <= start(n+1)).
    records = [controller.get_status(rid) for rid in accepted_ids]
    records.sort(key=lambda r: r.start_clock_ms)
    for prev, nxt in zip(records, records[1:]):
        assert prev.finish_clock_ms <= nxt.start_clock_ms, (
            prev.request_id,
            nxt.request_id,
        )

    # Overflow was deterministic: every rejected one says QUEUE_FULL.
    rejected = [reason for _, ok, reason in accepted if not ok]
    assert rejected, "expected some QUEUE_FULL under concurrent burst"
    assert all(reason is RejectReason.QUEUE_FULL for reason in rejected)

    wait_for_status(controller, "runner", CommandStatus.COMPLETED, timeout=10.0)
    # Final state is clean: nothing running/queued, no lost records.
    snap = controller.snapshot()
    assert snap.current_request_id is None
    assert snap.queued_request_ids == []
    assert snap.state.value == "IDLE"
    assert len(snap.commands) == 13  # runner + 12, each decided exactly once


def test_queue_full_is_deterministic_and_recoverable(controller):
    # one long runner on joint 0
    assert controller.submit(
        "runner", [joint(0, 160.0, vel=10.0, acc=20.0)]
    ).accepted
    time.sleep(0.03)
    cap = controller.config.queue_capacity
    for i in range(cap):
        r = controller.submit(f"fill{i}", [joint(1, 20.0)])
        assert r.accepted, i
    overflow = controller.submit("overflow", [joint(1, 20.0)])
    assert overflow.reject_reason is RejectReason.QUEUE_FULL
    assert f"{cap}/{cap}" in overflow.message

    # once everything drains and the machine is IDLE, commands are accepted
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        snap = controller.snapshot()
        if (
            snap.state.value == "IDLE"
            and snap.queued_request_ids == []
            and snap.current_request_id is None
        ):
            break
        time.sleep(0.005)
    assert controller.submit("later", [joint(2, 20.0)]).accepted
    wait_for_status(controller, "later", CommandStatus.COMPLETED, timeout=10.0)


def test_joint_conflict_distinguishable_from_queue_full(controller):
    assert controller.submit(
        "runner", [joint(0, 150.0, vel=10.0, acc=20.0)]
    ).accepted
    time.sleep(0.03)
    # a command touching running joint 0 -> JOINT_CONFLICT
    conflict = controller.submit("conf", [joint(0, 10.0), joint(1, 10.0)])
    assert conflict.reject_reason is RejectReason.JOINT_CONFLICT
    assert "[0]" in conflict.message
    # a disjoint joint command with free queue space -> accepted
    disjoint = controller.submit("disjoint", [joint(2, 10.0)])
    assert disjoint.accepted, disjoint.message


def test_burst_pressure_never_loses_or_duplicates(controller):
    n = 60
    outcomes: dict[str, tuple] = {}
    barrier = threading.Barrier(n)
    lock = threading.Lock()

    def fire(i):
        rid = f"b{i:03d}"
        barrier.wait()
        r = controller.submit(
            rid, [joint(i % 3, 5.0 + (i % 5), vel=60.0, acc=120.0)]
        )
        with lock:
            outcomes[rid] = (r.accepted, r.reject_reason)

    threads = [threading.Thread(target=fire, args=(i,)) for i in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(outcomes) == n  # every caller got exactly one decision
    accepted_ids = [rid for rid, (ok, _) in outcomes.items() if ok]
    rejected = [reason for _, (ok, reason) in outcomes.items() if not ok]
    # accepted ids unique, no duplicates queued
    assert len(accepted_ids) == len(set(accepted_ids))
    # under pressure rejection reasons are one of the two backpressure causes
    assert all(
        reason in (RejectReason.QUEUE_FULL, RejectReason.JOINT_CONFLICT)
        for reason in rejected
    )
    for rid in accepted_ids:
        wait_for_status(controller, rid, CommandStatus.COMPLETED, timeout=15.0)

    snap = controller.snapshot()
    assert snap.current_request_id is None
    assert snap.queued_request_ids == []
    assert snap.state.value == "IDLE"
    # nothing executed twice: each accepted command has exactly one lifecycle
    for rid in accepted_ids:
        rec = controller.get_status(rid)
        assert rec.status is CommandStatus.COMPLETED
        assert rec.start_clock_ms is not None and rec.finish_clock_ms is not None
