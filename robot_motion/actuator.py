"""本地模拟执行器：维护关节位置，执行阻塞式定时运动，支持急停冻结。"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from .config import ControllerConfig

# 单个时间片长度（秒）：越细急停响应越快，模拟精度越高
_TICK_SECONDS = 0.01


@dataclass
class ActuatorState:
    """执行器当前物理状态（模拟）。"""

    positions: dict[int, float]
    moving: bool = False


class SimulatedActuator:
    """用时间片睡眠 + 线性插值模拟运动过程的本地执行器。

    运动耗时按“最长关节位移 / velocity”估算（下限一个时间片），
    执行期间周期性检查 stop_event：被置位则就地冻结并返回 False。
    """

    def __init__(self, config: ControllerConfig) -> None:
        self._config = config
        self._lock = threading.Lock()
        self._positions: dict[int, float] = {}
        self._moving = False

    def initialize(self, home_positions: dict[int, float]) -> None:
        with self._lock:
            self._positions = dict(home_positions)
            self._moving = False

    def snapshot_positions(self) -> dict[int, float]:
        with self._lock:
            return dict(self._positions)

    def is_moving(self) -> bool:
        with self._lock:
            return self._moving

    def execute_move(
        self,
        positions: dict[int, float],
        velocity: float,
        stop_event: threading.Event,
    ) -> bool:
        """执行一段运动。

        返回 True 表示正常完成，False 表示被急停中止（位置停在中断点）。
        """
        with self._lock:
            starts = dict(self._positions)
            self._moving = True
        try:
            max_distance = max(
                abs(positions[j] - starts.get(j, positions[j]))
                for j in positions
            )
            duration = max(max_distance / max(velocity, 1e-9), _TICK_SECONDS)
            elapsed = 0.0
            while elapsed < duration:
                if stop_event.is_set():
                    return False  # 就地冻结：不更新到目标点
                if stop_event.wait(_TICK_SECONDS):
                    return False
                elapsed += _TICK_SECONDS
                ratio = min(elapsed / duration, 1.0)
                with self._lock:
                    self._positions = {
                        j: starts[j] + (positions[j] - starts[j]) * ratio
                        for j in positions
                    }
            with self._lock:
                self._positions = dict(positions)
            return True
        finally:
            with self._lock:
                self._moving = False

    def freeze(self) -> None:
        """急停：运动线程通过 stop_event 感知；此处只落下 moving 标志语义。"""
        # 真正的停止信号由控制器持有并 set；这里确保状态一致。
        with self._lock:
            self._moving = False

    def reset(self, home_positions: dict[int, float]) -> None:
        """复位到初始（零位）状态。"""
        with self._lock:
            self._positions = dict(home_positions)
            self._moving = False
