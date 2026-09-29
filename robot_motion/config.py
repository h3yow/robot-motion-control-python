"""控制器可调配置项（安全限位 / 队列 / 超时）。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointLimit:
    """单关节安全限位。"""

    min_position: float
    max_position: float
    max_velocity: float
    max_acceleration: float


@dataclass(frozen=True)
class ControllerConfig:
    """控制器配置。

    - joint_limits: 关节序号 -> 限位
    - joint_count: 关节数量
    - queue_capacity: 等待队列容量（不含正在执行的那条）
    - command_timeout: 单条运动从入队到开始执行的最长等待秒数
    - default_speed: 模拟运动的参考速度（单位/秒），用于估算执行耗时
    """

    joint_limits: dict[int, JointLimit]
    joint_count: int = 0
    queue_capacity: int = 16
    command_timeout: float = 5.0
    default_speed: float = 10.0

    @staticmethod
    def default() -> "ControllerConfig":
        """6 关节默认配置：位置 [-180, 180]（度），速度/加速度安全上限。"""
        joint_limits = {
            i: JointLimit(
                min_position=-180.0,
                max_position=180.0,
                max_velocity=60.0,
                max_acceleration=120.0,
            )
            for i in range(6)
        }
        return ControllerConfig(joint_limits=joint_limits, joint_count=6)
