"""Domain models: enums and value objects."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class MotionState(str, Enum):
    """Controller / interlock state machine."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    ESTOPPED = "ESTOPPED"
    FAULT = "FAULT"


class CommandStatus(str, Enum):
    """Lifecycle status of a single motion command."""

    PENDING = "PENDING"
    QUEUED = "QUEUED"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    REJECTED = "REJECTED"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"
    QUEUE_TIMEOUT = "QUEUE_TIMEOUT"
    FAULTED = "FAULTED"


class RejectReason(str, Enum):
    """Distinguishable reasons for rejecting a submission."""

    POSITION_OUT_OF_RANGE = "POSITION_OUT_OF_RANGE"
    VELOCITY_OUT_OF_RANGE = "VELOCITY_OUT_OF_RANGE"
    ACCELERATION_OUT_OF_RANGE = "ACCELERATION_OUT_OF_RANGE"
    JOINT_CONFLICT = "JOINT_CONFLICT"
    QUEUE_FULL = "QUEUE_FULL"
    INTERLOCK_ENGAGED = "INTERLOCK_ENGAGED"
    DUPLICATE_REQUEST = "DUPLICATE_REQUEST"
    INVALID_PAYLOAD = "INVALID_PAYLOAD"


@dataclass(frozen=True)
class JointMotion:
    """One joint's motion target within a command."""

    joint_id: int
    target_position: float
    velocity: float
    acceleration: float


@dataclass
class CommandRecord:
    """Runtime record of a submitted command."""

    request_id: str
    joints: tuple[JointMotion, ...]
    status: CommandStatus
    submit_clock_ms: int
    reject_reason: RejectReason | None = None
    start_clock_ms: int | None = None
    finish_clock_ms: int | None = None
    detail: str = ""


@dataclass
class JointSnapshot:
    """Observable state of one simulated joint."""

    joint_id: int
    position: float
    commanded_velocity: float
    target_position: float | None
    moving: bool


@dataclass
class ControllerSnapshot:
    """Full observable controller snapshot used by status API and tests."""

    state: MotionState
    clock_ms: int
    queue_capacity: int
    queued_request_ids: list[str]
    current_request_id: str | None
    joints: list[JointSnapshot]
    commands: dict[str, CommandRecord] = field(default_factory=dict)
