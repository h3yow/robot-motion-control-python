"""pytest 共享夹具与日志配置。

要求“单测日志里打印请求参数与判定依据”：
1. 打开 robot_motion 日志（提交/拒绝/急停等事件均会带参数与依据）；
2. 提供 fixture，失败时 caplog 内容会随 -v / --log-cli-level 输出。
"""

from __future__ import annotations

import logging
import os

import pytest

from robot_motion.config import ControllerConfig
from robot_motion.controller import MotionController

# 保证测试期间能看到领域日志（CI 中可用 --log-cli-level=INFO 查看）
logging.getLogger("robot_motion").setLevel(logging.INFO)


def fast_config(**overrides) -> ControllerConfig:
    """短运动时距的配置，便于快速测试；可用覆盖项改队列/超时。"""
    params = dict(
        joint_limits=ControllerConfig.default().joint_limits,
        joint_count=6,
        queue_capacity=4,
        command_timeout=0.5,
        default_speed=100.0,
    )
    params.update(overrides)
    return ControllerConfig(**params)


@pytest.fixture
def home():
    return {i: 0.0 for i in range(6)}


@pytest.fixture
def controller():
    ctrl = MotionController(fast_config())
    yield ctrl
    ctrl.shutdown()
