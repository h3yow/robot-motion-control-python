"""命令与状态的数据模型。"""

from __future__ import annotations

from dataclasses import dataclass, field

from .enums import CommandState, RejectReason


@dataclass(frozen=True)
class MotionRequest:
    """一次多关节运动请求。

    request_id 由调用方保证（幂等键）；positions 为关节序号 -> 目标位置；
    velocity / acceleration 为标量运动学约束；blocking=False 表示排队等待。
    """

    request_id: str
    positions: dict[int, float]
    velocity: float = 5.0
    acceleration: float = 5.0
    blocking: bool = True


@dataclass
class CommandRecord:
    """一条命令在控制器中的记录（含终态与拒绝原因）。"""

    request_id: str
    state: CommandState
    positions: dict[int, float]
    velocity: float
    acceleration: float
    blocking: bool
    enqueue_time: float
    reject_reason: RejectReason | None = None
    reject_detail: str = ""
    start_time: float | None = None
    finish_time: float | None = None


@dataclass
class ControllerSnapshot:
    """控制器对外暴露的状态快照。"""

    state: str
    queue_size: int
    current_request_id: str | None
    joint_positions: dict[int, float]
    fault_message: str = ""
