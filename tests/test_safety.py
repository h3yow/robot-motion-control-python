"""安全联锁与状态流转：运动中急停、外部故障、复位回零。"""

from __future__ import annotations

import logging
import threading
import time

from robot_motion.enums import ControllerState, RejectReason
from robot_motion.models import MotionRequest

logger = logging.getLogger("robot_motion.tests")


def test_emergency_stop_during_motion(controller) -> None:
    # 位移 80 / 速度 10 = 8s 的长运动，便于在中途急停
    t = threading.Thread(
        target=lambda: controller.submit(
            MotionRequest("moving", {i: 80.0 for i in range(6)}, velocity=10.0)
        )
    )
    t.start()
    time.sleep(0.2)
    before = controller.snapshot().joint_positions[0]
    logger.info("急停前请求参数 request_id=moving 目标=80 速度=10，实际位置=%.2f", before)
    assert before > 0

    controller.emergency_stop("测试员按下急停")
    t.join()
    time.sleep(0.05)

    snap = controller.snapshot()
    rec = controller.get_status("moving")
    logger.info(
        "急停判定: 控制器状态=%s 运动终态=%s 冻结位置=%.2f",
        snap.state, rec.state.value, snap.joint_positions[0],
    )
    assert snap.state == ControllerState.EMERGENCY_STOP.value
    assert rec.state.value == "ABORTED"
    assert snap.joint_positions[0] < 80.0  # 就地冻结，未滑到目标

    # 未复位：新运动一律拒绝
    blocked = controller.submit(
        MotionRequest("blocked", {i: 0.0 for i in range(6)}, velocity=10.0)
    )
    logger.info(
        "未复位新请求参数=%s -> 判定原因=%s 依据=%s",
        blocked.positions, blocked.reject_reason.value, blocked.reject_detail,
    )
    assert blocked.reject_reason is RejectReason.NOT_RESET


def test_reset_returns_to_stable_idle(controller) -> None:
    t = threading.Thread(
        target=lambda: controller.submit(
            MotionRequest("moving2", {i: 80.0 for i in range(6)}, velocity=10.0)
        )
    )
    t.start()
    time.sleep(0.2)
    controller.emergency_stop()
    t.join()

    # 队列里放一个等待项，复位后应作废且历史清空
    queued = controller.submit(
        MotionRequest("q", {i: 1.0 for i in range(6)},
                      velocity=10.0, blocking=False)
    )
    assert queued.reject_reason is RejectReason.NOT_RESET

    snap = controller.reset()
    logger.info(
        "复位后: state=%s 位置=%s 队列=%d 历史查询=%s",
        snap.state, snap.joint_positions, snap.queue_size,
        controller.get_status("moving2"),
    )
    assert snap.state == "IDLE"
    assert snap.joint_positions == {i: 0.0 for i in range(6)}
    assert snap.queue_size == 0
    assert controller.get_status("moving2") is None
    assert controller.get_status("q") is None

    # 复位后可以正常下发
    rec = controller.submit(
        MotionRequest("ok-after-reset", {i: 20.0 for i in range(6)}, velocity=40.0)
    )
    assert rec.state.value == "COMPLETED"


def test_external_fault_blocks_commands(controller) -> None:
    controller.external_fault("驱动器过流")
    snap = controller.snapshot()
    assert snap.state == "FAULT"
    rec = controller.submit(
        MotionRequest("during-fault", {i: 0.0 for i in range(6)}, velocity=5.0)
    )
    logger.info(
        "故障态请求 -> 原因=%s 依据=%s",
        rec.reject_reason.value, rec.reject_detail,
    )
    assert rec.reject_reason is RejectReason.NOT_RESET
    after = controller.reset()
    assert after.state == "IDLE"
