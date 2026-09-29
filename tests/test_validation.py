"""越界请求测试：位置/速度/加速度/结构错误，原因必须可区分。"""

from __future__ import annotations

import logging

from robot_motion.enums import RejectReason
from robot_motion.models import MotionRequest

logger = logging.getLogger("robot_motion.tests")


def test_position_out_of_range(controller) -> None:
    req = MotionRequest(
        request_id="bad-pos",
        positions={**{i: 0.0 for i in range(6)}, 2: 250.0},
        velocity=10.0,
        acceleration=20.0,
    )
    logger.info("测试请求参数: %s", req)
    rec = controller.submit(req)
    logger.info("判定结果: state=%s reason=%s 依据=%s",
                rec.state.value, rec.reject_reason.value, rec.reject_detail)
    assert rec.state.value == "REJECTED"
    assert rec.reject_reason is RejectReason.POSITION_OUT_OF_RANGE
    assert "关节 2" in rec.reject_detail


def test_velocity_out_of_range(controller) -> None:
    req = MotionRequest(
        "bad-vel", {i: 0.0 for i in range(6)}, velocity=90.0
    )
    logger.info("测试请求参数: positions=全零 velocity=%s", req.velocity)
    rec = controller.submit(req)
    logger.info("判定结果: reason=%s 依据=%s",
                rec.reject_reason.value, rec.reject_detail)
    assert rec.reject_reason is RejectReason.VELOCITY_OUT_OF_RANGE
    assert "60" in rec.reject_detail


def test_acceleration_out_of_range(controller) -> None:
    req = MotionRequest(
        "bad-acc", {i: 10.0 for i in range(6)},
        velocity=10.0, acceleration=500.0,
    )
    logger.info("测试请求参数: positions=10 velocity=%s acceleration=%s",
                req.velocity, req.acceleration)
    rec = controller.submit(req)
    logger.info("判定结果: reason=%s 依据=%s",
                rec.reject_reason.value, rec.reject_detail)
    assert rec.reject_reason is RejectReason.ACCELERATION_OUT_OF_RANGE


def test_bad_request_missing_and_unknown_joint(controller) -> None:
    req_missing = MotionRequest("miss", {0: 1.0}, velocity=5.0)
    rec = controller.submit(req_missing)
    assert rec.reject_reason is RejectReason.BAD_REQUEST

    req_unknown = MotionRequest(
        "unknown", {i: 0.0 for i in range(6)} | {9: 0.0}, velocity=5.0
    )
    rec2 = controller.submit(req_unknown)
    assert rec2.reject_reason is RejectReason.BAD_REQUEST


def test_negative_velocity_rejected(controller) -> None:
    rec = controller.submit(
        MotionRequest("neg-vel", {i: 0.0 for i in range(6)}, velocity=-1.0)
    )
    assert rec.reject_reason is RejectReason.VELOCITY_OUT_OF_RANGE
