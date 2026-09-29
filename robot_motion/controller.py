"""Motion controller: request validation, idempotency and command lifecycle.

This is the only facade application/API/tests talk to. Submission validation
order is fixed so rejection reasons stay distinguishable and explainable:

1. DUPLICATE_REQUEST   - same request_id seen before (returns prior record;
                         the command is never executed a second time)
2. INTERLOCK_ENGAGED   - state is ESTOPPED or FAULT (reset required first)
3. payload validation  - joints non-empty, well formed, unique
4. POSITION_OUT_OF_RANGE / VELOCITY_OUT_OF_RANGE /
   ACCELERATION_OUT_OF_RANGE - per-joint hard limits (checked per parameter)
5. JOINT_CONFLICT      - any joint is in the currently RUNNING command
6. QUEUE_FULL          - waiting queue at capacity

Every decision logs the request parameters and the basis for the decision.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass

from .actuator import SimulatedActuator
from .clock import SimClock
from .config import MotionConfig
from .engine import ExecutionEngine
from .interlock import SafetyInterlock
from .models import (
    CommandRecord,
    CommandStatus,
    ControllerSnapshot,
    JointMotion,
    MotionState,
    RejectReason,
)

logger = logging.getLogger("robot_motion.controller")


@dataclass
class SubmissionResult:
    """Outcome of a submit call: accepted record or rejection explanation."""

    accepted: bool
    record: CommandRecord | None
    reject_reason: RejectReason | None
    message: str

    def as_dict(self) -> dict:
        return {
            "accepted": self.accepted,
            "request_id": self.record.request_id if self.record else None,
            "status": self.record.status.value if self.record else None,
            "reject_reason": (
                self.reject_reason.value if self.reject_reason else None
            ),
            "message": self.message,
        }


class MotionController:
    """Application-facing facade implementing submit / query / safety ops."""

    def __init__(self, config: MotionConfig | None = None) -> None:
        self._config = config or MotionConfig.default()
        self._lock = threading.RLock()
        self._clock = SimClock(
            self._config.clock_tick_ms, self._config.clock_real_ms_per_tick
        )
        self._interlock = SafetyInterlock()
        self._actuator = SimulatedActuator(
            self._config.joint_limits, self._clock
        )
        self._engine = ExecutionEngine(
            self._config, self._clock, self._actuator, self._interlock
        )
        self._records: dict[str, CommandRecord] = {}
        self._clock.start()
        self._engine.start()
        logger.info(
            "MotionController initialised: joints=%d queue_capacity=%d "
            "queue_timeout_ms=%d tick_ms=%d",
            len(self._config.joint_limits),
            self._config.queue_capacity,
            self._config.queue_timeout_ms,
            self._config.clock_tick_ms,
        )

    # ============================================================= submit
    def submit(self, request_id: str, joints: list[dict]) -> SubmissionResult:
        """Validate and submit one multi-joint motion command.

        A duplicate ``request_id`` returns the stored record with
        DUPLICATE_REQUEST and is never scheduled twice, regardless of whether
        the original is queued, running, completed or rejected.
        """
        # Basic payload shape first so logs are always meaningful.
        if not isinstance(request_id, str) or not request_id:
            return self._reject(
                None,
                joints,
                RejectReason.INVALID_PAYLOAD,
                "request_id must be a non-empty string",
            )

        logger.info(
            "submit received request_id=%s joints=%s state=%s",
            request_id,
            joints,
            self._interlock.state.value,
        )

        with self._lock:
            # 1) Idempotency -------------------------------------------------
            existing = self._records.get(request_id)
            if existing is not None:
                logger.warning(
                    "submit DUPLICATE request_id=%s -> original status=%s "
                    "reason=same request_id resubmitted; not executed again",
                    request_id,
                    existing.status.value,
                )
                return SubmissionResult(
                    accepted=False,
                    record=existing,
                    reject_reason=RejectReason.DUPLICATE_REQUEST,
                    message=(
                        f"request_id {request_id!r} already submitted; "
                        f"returning original status {existing.status.value}"
                    ),
                )

            # 2) Interlock ---------------------------------------------------
            if self._interlock.state in (
                MotionState.ESTOPPED,
                MotionState.FAULT,
            ):
                return self._reject(
                    request_id,
                    joints,
                    RejectReason.INTERLOCK_ENGAGED,
                    f"interlock state={self._interlock.state.value}; reset "
                    "required before new motions",
                )

            # 3) Payload structure ------------------------------------------
            parsed, err = self._parse_joints(joints)
            if err is not None:
                return self._reject(
                    request_id, joints, RejectReason.INVALID_PAYLOAD, err
                )

            # 4) Per-joint hard limits --------------------------------------
            limit_reason, detail = self._check_limits(parsed)
            if limit_reason is not None:
                return self._reject(request_id, joints, limit_reason, detail)

            # 5) Conflict with the currently running command ----------------
            running_ids = self._running_joint_ids()
            overlap = sorted(set(j.joint_id for j in parsed) & running_ids)
            if overlap:
                return self._reject(
                    request_id,
                    joints,
                    RejectReason.JOINT_CONFLICT,
                    f"joints {overlap} are in the running command; wait for "
                    "it to finish or choose disjoint joints",
                )

            # 6) Queue capacity ---------------------------------------------
            depth = self._engine.queue_depth()
            if depth >= self._config.queue_capacity:
                return self._reject(
                    request_id,
                    joints,
                    RejectReason.QUEUE_FULL,
                    f"waiting queue full ({depth}/{self._config.queue_capacity})",
                )

            # Accepted: store then enqueue under the same lock so the record
            # is queryable immediately and cannot be lost between dict and
            # queue.
            record = CommandRecord(
                request_id=request_id,
                joints=tuple(parsed),
                status=CommandStatus.PENDING,
                submit_clock_ms=self._clock.now_ms(),
            )
            self._records[request_id] = record
            self._engine.enqueue(record)
            logger.info(
                "submit ACCEPTED request_id=%s joint_ids=%s queue_depth=%d",
                request_id,
                [j.joint_id for j in parsed],
                self._engine.queue_depth(),
            )
            return SubmissionResult(
                accepted=True,
                record=record,
                reject_reason=None,
                message="accepted and queued for serial execution",
            )

    # -------------------------------------------------------- parsing/checks
    def _parse_joints(
        self, joints: list[dict]
    ) -> tuple[list[JointMotion], str | None]:
        if not isinstance(joints, list) or not joints:
            return [], "joints must be a non-empty list"
        parsed: list[JointMotion] = []
        seen: set[int] = set()
        required = ("joint_id", "target_position", "velocity", "acceleration")
        for item in joints:
            if not isinstance(item, dict):
                return [], f"joint entry must be an object, got {item!r}"
            missing = [k for k in required if k not in item]
            if missing:
                return [], f"joint entry missing fields {missing}: {item}"
            try:
                jid = int(item["joint_id"])
                target = float(item["target_position"])
                vel = float(item["velocity"])
                acc = float(item["acceleration"])
            except (TypeError, ValueError):
                return [], f"joint entry has non-numeric fields: {item}"
            if jid in seen:
                return [], f"joint_id {jid} appears twice in one request"
            seen.add(jid)
            if self._config.limits_for(jid) is None:
                return [], (
                    f"unknown joint_id {jid}; valid ids="
                    f"{sorted(lim.joint_id for lim in self._config.joint_limits)}"
                )
            if vel <= 0:
                return [], f"joint {jid}: velocity must be > 0, got {vel}"
            if acc <= 0:
                return [], f"joint {jid}: acceleration must be > 0, got {acc}"
            parsed.append(JointMotion(jid, target, vel, acc))
        return parsed, None

    def _check_limits(
        self, motions: list[JointMotion]
    ) -> tuple[RejectReason | None, str]:
        for jm in motions:
            lim = self._config.limits_for(jm.joint_id)
            assert lim is not None
            if not (lim.min_position <= jm.target_position <= lim.max_position):
                logger.warning(
                    "limit check FAIL request joint_id=%d field=target_position "
                    "value=%.3f range=[%.3f,%.3f]",
                    jm.joint_id,
                    jm.target_position,
                    lim.min_position,
                    lim.max_position,
                )
                return (
                    RejectReason.POSITION_OUT_OF_RANGE,
                    f"joint {jm.joint_id}: target_position {jm.target_position} "
                    f"outside [{lim.min_position}, {lim.max_position}]",
                )
            if jm.velocity > lim.max_velocity:
                logger.warning(
                    "limit check FAIL request joint_id=%d field=velocity "
                    "value=%.3f max=%.3f",
                    jm.joint_id,
                    jm.velocity,
                    lim.max_velocity,
                )
                return (
                    RejectReason.VELOCITY_OUT_OF_RANGE,
                    f"joint {jm.joint_id}: velocity {jm.velocity} exceeds "
                    f"max {lim.max_velocity}",
                )
            if jm.acceleration > lim.max_acceleration:
                logger.warning(
                    "limit check FAIL request joint_id=%d field=acceleration "
                    "value=%.3f max=%.3f",
                    jm.joint_id,
                    jm.acceleration,
                    lim.max_acceleration,
                )
                return (
                    RejectReason.ACCELERATION_OUT_OF_RANGE,
                    f"joint {jm.joint_id}: acceleration {jm.acceleration} "
                    f"exceeds max {lim.max_acceleration}",
                )
        return None, ""

    def _running_joint_ids(self) -> set[int]:
        snap = self._engine.snapshot(self._records)
        if snap.current_request_id is None:
            return set()
        current = self._records[snap.current_request_id]
        return {j.joint_id for j in current.joints}

    def _reject(
        self,
        request_id: str | None,
        joints: list[dict],
        reason: RejectReason,
        detail: str,
    ) -> SubmissionResult:
        record = None
        if request_id:
            record = CommandRecord(
                request_id=request_id,
                joints=(),
                status=CommandStatus.REJECTED,
                submit_clock_ms=self._clock.now_ms(),
                reject_reason=reason,
                detail=detail,
            )
            # Rejected requests are also remembered so resubmission of the
            # same request_id is a DUPLICATE rather than a second evaluation.
            self._records[request_id] = record
        logger.warning(
            "submit REJECTED request_id=%s joints=%s reason=%s basis=%s",
            request_id,
            joints,
            reason.value,
            detail,
        )
        return SubmissionResult(False, record, reason, detail)

    # ============================================================= safety
    def emergency_stop(self) -> None:
        """Atomically latch E-stop, freeze motion and cancel the whole queue.

        No command can execute between latching and stopping: the engine
        performs both under the lock its worker tick uses.
        """
        self._engine.abort_all(
            CommandStatus.EMERGENCY_STOPPED,
            trip=self._interlock.emergency_stop,
        )
        logger.warning(
            "emergency_stop applied; no command runs until reset()"
        )

    def external_fault(self) -> None:
        """Latch an external fault; behaves like E-stop for command safety."""
        self._engine.abort_all(
            CommandStatus.FAULTED,
            trip=self._interlock.external_fault,
        )
        logger.error(
            "external_fault applied; no command runs until reset()"
        )

    def reset(self) -> None:
        """Return the machine to the stable IDLE initial state.

        Valid only from ESTOPPED/FAULT. The queue is already empty (drained at
        trip time) and the actuator holds the frozen position, which becomes
        the well-defined starting position for subsequent motions.
        """
        if self._interlock.reset():
            logger.info(
                "reset complete: state=IDLE held_positions=%s",
                [
                    (s.joint_id, round(s.position, 3))
                    for s in self._actuator.snapshot()
                ],
            )

    # ============================================================= queries
    def get_status(self, request_id: str) -> CommandRecord | None:
        with self._lock:
            return self._records.get(request_id)

    def snapshot(self) -> ControllerSnapshot:
        with self._lock:
            return self._engine.snapshot(self._records)

    @property
    def config(self) -> MotionConfig:
        return self._config

    def close(self) -> None:
        self._engine.stop()
        self._clock.stop()
