"""Execution engine: serialises motion commands through a single worker loop.

Concurrency model
-----------------
There is exactly one engine worker thread, so at most one motion can ever
execute. Producers (HTTP/test threads) only append a record to the waiting
deque under ``self._lock``; every state mutation of records, the actuator and
the interlock happens on the worker thread inside ``_pump``. This
single-threaded ownership is what guarantees no command is lost or observed
in an inconsistent state under pressure.

Critical sections are kept tiny: the worker does not block while holding the
lock, so ``abort_all`` (emergency stop / fault) is never delayed by the
worker. The worker paces itself on the simulated clock tick.

Queue overflow is handled at submit time (QUEUE_FULL rejection, see
controller); queue timeout is handled here: a command that has waited longer
than ``queue_timeout_ms`` is cancelled with QUEUE_TIMEOUT before it starts.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import deque

from .actuator import SimulatedActuator
from .clock import SimClock
from .config import MotionConfig
from .interlock import SafetyInterlock
from .models import CommandRecord, CommandStatus, ControllerSnapshot, MotionState

logger = logging.getLogger("robot_motion.engine")

# Real-time pacing of the worker loop (seconds). Independent of virtual time:
# just the granularity at which the engine reacts; all timing *decisions* use
# the simulated clock so behaviour stays deterministic.
_WORKER_POLL_SECONDS = 0.002


class ExecutionEngine:
    """Owns the waiting queue and drives one motion at a time."""

    def __init__(
        self,
        config: MotionConfig,
        clock: SimClock,
        actuator: SimulatedActuator,
        interlock: SafetyInterlock,
    ) -> None:
        self._config = config
        self._clock = clock
        self._actuator = actuator
        self._interlock = interlock
        self._lock = threading.RLock()
        self._waiting: deque[CommandRecord] = deque()
        self._current: CommandRecord | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------ producers
    def enqueue(self, record: CommandRecord) -> None:
        """Append an already-validated record and mark it QUEUED."""
        with self._lock:
            self._waiting.append(record)
            record.status = CommandStatus.QUEUED
            logger.info(
                "enqueue request_id=%s queue_depth=%d",
                record.request_id,
                len(self._waiting),
            )

    def queue_depth(self) -> int:
        with self._lock:
            return len(self._waiting)

    def abort_all(
        self,
        status: CommandStatus,
        trip: object = None,
    ) -> None:
        """Atomically trip the interlock, freeze and cancel every command.

        Latching the interlock and draining the queue/freezing the actuator
        happen inside the same critical section that the worker uses, so no
        worker tick can execute between "machine is tripped" and "motion is
        stopped": a running command cannot advance a single extra tick.

        ``trip`` is the interlock latching method (emergency_stop /
        external_fault); it is invoked while holding the engine lock.
        """
        logger.warning("abort_all requested status=%s", status.value)
        with self._lock:
            if trip is not None:
                trip()
            now = self._clock.now_ms()
            if self._current is not None:
                rec = self._current
                self._actuator.freeze(now)
                rec.status = status
                rec.finish_clock_ms = now
                rec.detail = f"aborted by {status.value}"
                logger.warning(
                    "running command request_id=%s -> %s",
                    rec.request_id,
                    status.value,
                )
                self._current = None
            while self._waiting:
                rec = self._waiting.popleft()
                rec.status = status
                rec.finish_clock_ms = now
                rec.detail = f"cancelled while queued by {status.value}"
                logger.warning(
                    "queued command request_id=%s -> %s",
                    rec.request_id,
                    status.value,
                )

    # ------------------------------------------------------------- snapshot
    def snapshot(self, records: dict[str, CommandRecord]) -> ControllerSnapshot:
        from .models import ControllerSnapshot

        with self._lock:
            return ControllerSnapshot(
                state=self._interlock.state,
                clock_ms=self._clock.now_ms(),
                queue_capacity=self._config.queue_capacity,
                queued_request_ids=[r.request_id for r in self._waiting],
                current_request_id=(
                    self._current.request_id if self._current else None
                ),
                joints=self._actuator.snapshot(),
                commands=dict(records),
            )

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(
                target=self._run, name="motion-engine", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                return
            self._running = False
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        with self._lock:
            self._thread = None

    # -------------------------------------------------------------- worker
    def _run(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
                self._pump()
            time.sleep(_WORKER_POLL_SECONDS)

    def _pump(self) -> None:
        """One deterministic step. Caller must hold (briefly) ``self._lock``."""
        now = self._clock.now_ms()

        # 1) Expire over-aged queued commands (deterministic cancellation).
        if self._waiting:
            kept: deque[CommandRecord] = deque()
            while self._waiting:
                rec = self._waiting.popleft()
                waited = now - rec.submit_clock_ms
                if waited > self._config.queue_timeout_ms:
                    rec.status = CommandStatus.QUEUE_TIMEOUT
                    rec.finish_clock_ms = now
                    rec.detail = (
                        f"queued for {waited}ms > limit "
                        f"{self._config.queue_timeout_ms}ms"
                    )
                    logger.warning(
                        "queue timeout request_id=%s waited_ms=%s",
                        rec.request_id,
                        waited,
                    )
                else:
                    kept.append(rec)
            self._waiting = kept

        # 2) Advance / complete the running motion.
        if self._current is not None:
            if self._interlock.state is not MotionState.RUNNING:
                # abort_all already finalised and cleared it.
                self._current = None
            else:
                moving = self._actuator.advance(now)
                if not moving:
                    rec = self._current
                    rec.status = CommandStatus.COMPLETED
                    rec.finish_clock_ms = now
                    rec.detail = "motion completed"
                    logger.info(
                        "command completed request_id=%s finish_ms=%s",
                        rec.request_id,
                        now,
                    )
                    self._current = None
                    self._interlock.end_motion()

        # 3) Start the next command only when interlock is IDLE.
        if self._current is None and self._waiting:
            if self._interlock.state is MotionState.IDLE:
                rec = self._waiting.popleft()
                if self._interlock.begin_motion():
                    duration = self._actuator.start_motion(rec.joints, now)
                    rec.status = CommandStatus.RUNNING
                    rec.start_clock_ms = now
                    rec.detail = f"running, planned_duration_ms={duration}"
                    self._current = rec
                    logger.info(
                        "command started request_id=%s start_ms=%s duration_ms=%s",
                        rec.request_id,
                        now,
                        duration,
                    )
                else:
                    # Interlock tripped between check and begin; keep it.
                    self._waiting.appendleft(rec)
            # ESTOPPED/FAULT: abort_all has emptied the queue; submit() blocks
            # any further arrivals while the interlock is tripped.
