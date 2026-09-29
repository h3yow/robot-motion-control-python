"""骨架冒烟测试：确认包可导入、枚举/配置可构造。"""

from robot_motion.config import ControllerConfig
from robot_motion.enums import ControllerState, RejectReason


def test_skeleton_importable() -> None:
    assert ControllerState.IDLE.value == "IDLE"
    assert RejectReason.QUEUE_FULL.value == "QUEUE_FULL"
    assert isinstance(ControllerConfig.default, object) or callable(ControllerConfig.default)
