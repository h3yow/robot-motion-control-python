"""幂等性与运动冲突测试。"""

from __future__ import annotations

import logging
import threading
import time

from robot_motion.enums import RejectReason
from robot_motion.models import MotionRequest

logger = logging.getLogger("robot_motion.tests")


def test_duplicate_request_executed_once(controller) -> None:
    req = MotionRequest("dup-1", {i: 30.0 for i in range(6)}, velocity=50.0)
    first = controller.submit(req)
    second = controller.submit(
        MotionRequest("dup-1", {i: -30.0 for i in range(6)}, velocity=1.0)
    )
    logger.info(
        "重复提交 request_id=%s：首次 state=%s，二次拿到同一记录=%s，目标位置保持=%s",
        req.request_id, first.state.value, second is first, second.positions,
    )
    assert second is first                      # 同一条记录
    assert second.positions == {i: 30.0 for i in range(6)}  # 参数不被覆盖
    assert second.state.value == "COMPLETED"


def test_blocking_conflict_while_running(controller) -> None:
    # 位移 40 / 速度 40 = 1.0s，确保发冲突请求时仍在运动
    target = {i: 40.0 for i in range(6)}

    def start_long():
        controller.submit(
            MotionRequest("long", target, velocity=40.0)
        )

    t = threading.Thread(target=start_long)
    t.start()
    time.sleep(0.15)  # 确保 long 正在执行
    snap = controller.snapshot()
    assert snap.state == "RUNNING"

    conflict = controller.submit(
        MotionRequest("conflict", {i: 0.0 for i in range(6)}, velocity=10.0)
    )
    logger.info(
        "冲突请求参数 request_id=%s positions=%s velocity=%s",
        conflict.request_id, conflict.positions, conflict.velocity,
    )
    logger.info(
        "判定: state=%s reason=%s 依据=%s",
        conflict.state.value, conflict.reject_reason.value,
        conflict.reject_detail,
    )
    assert conflict.reject_reason is RejectReason.MOTION_CONFLICT
    t.join()


def test_non_blocking_queued_instead_of_conflict(controller) -> None:
    # 位移 5 / 速度 50 = 0.1s，排队项可在 0.5s 超时前开始执行
    t = threading.Thread(
        target=lambda: controller.submit(
            MotionRequest("busy2", {i: 5.0 for i in range(6)}, velocity=50.0)
        )
    )
    t.start()
    time.sleep(0.05)
    queued = controller.submit(
        MotionRequest("queued", {i: -5.0 for i in range(6)},
                      velocity=50.0, blocking=False)
    )
    logger.info("非阻塞请求受理状态=%s 依据=忙时入队等待", queued.state.value)
    assert queued.state.value == "QUEUED"
    t.join()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        final = controller.get_status("queued")
        if final.state.value == "COMPLETED":
            break
        time.sleep(0.02)
    assert final.state.value == "COMPLETED"
