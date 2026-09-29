"""状态机与拒绝原因枚举。"""

from __future__ import annotations

from enum import Enum


class ControllerState(str, Enum):
    """控制器安全状态机。"""

    IDLE = "IDLE"                      # 空闲，可接收命令
    RUNNING = "RUNNING"                # 有运动正在执行（队列可有等待项）
    EMERGENCY_STOP = "EMERGENCY_STOP"  # 急停，拒绝一切新运动，需复位
    FAULT = "FAULT"                    # 外部故障，拒绝一切新运动，需复位


class CommandState(str, Enum):
    """单条运动命令生命周期。"""

    QUEUED = "QUEUED"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    REJECTED = "REJECTED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"


class RejectReason(str, Enum):
    """命令被拒绝的可区分原因。"""

    DUPLICATE_REQUEST = "DUPLICATE_REQUEST"      # 同一请求重复提交
    POSITION_OUT_OF_RANGE = "POSITION_OUT_OF_RANGE"
    VELOCITY_OUT_OF_RANGE = "VELOCITY_OUT_OF_RANGE"
    ACCELERATION_OUT_OF_RANGE = "ACCELERATION_OUT_OF_RANGE"
    MOTION_CONFLICT = "MOTION_CONFLICT"          # 与正在执行的运动冲突
    QUEUE_FULL = "QUEUE_FULL"                    # 队列溢出
    QUEUE_TIMEOUT = "QUEUE_TIMEOUT"              # 排队超时
    NOT_RESET = "NOT_RESET"                      # 急停/故障未复位
    BAD_REQUEST = "BAD_REQUEST"                  # 参数结构错误
