"""领域异常，携带可区分的拒绝原因。"""

from __future__ import annotations

from .enums import RejectReason


class MotionControlError(Exception):
    """运动控制领域异常基类。"""

    def __init__(self, reason: RejectReason, message: str) -> None:
        super().__init__(message)
        self.reason = reason
        self.message = message


class CommandRejectedError(MotionControlError):
    """命令在校验/入队阶段被拒绝。"""


class ControllerNotResetError(MotionControlError):
    """控制器处于急停/故障态，未复位。"""
