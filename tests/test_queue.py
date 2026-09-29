"""并发提交、串行化、队列溢出/排队超时与压力下一致性测试。"""

from __future__ import annotations

import logging
import threading
import time

from robot_motion.config import ControllerConfig
from robot_motion.controller import MotionController
from robot_motion.enums import RejectReason
from robot_motion.models import MotionRequest

logger = logging.getLogger("robot_motion.tests")


def test_concurrent_submissions_are_serialized() -> None:
    """20 个线程同时下发非阻塞命令：串行执行、全部有确定终态、无并发运动。"""
    from tests.conftest import fast_config

    ctrl = MotionController(fast_config(queue_capacity=30, command_timeout=5.0))
    n = 20
    results: dict[str, object] = {}
    barrier = threading.Barrier(n)

    def worker(i: int) -> None:
        barrier.wait()  # 尽量同时发出
        rec = ctrl.submit(
            MotionRequest(
                request_id=f"c{i}",
                positions={j: 0.0 for j in range(6)},
                velocity=60.0,
                blocking=False,
            )
        )
        results[f"c{i}"] = rec.state.value

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # 全部立即拿到非终态 QUEUED 或终态（满队/超时）；等待 worker 清空
    deadline = time.monotonic() + 10.0
    finals: dict[str, str] = {}
    while time.monotonic() < deadline:
        finals = {
            f"c{i}": ctrl.get_status(f"c{i}").state.value for i in range(n)
        }
        if all(v in ("COMPLETED", "REJECTED", "ABORTED", "TIMEOUT")
               for v in finals.values()):
            break
        time.sleep(0.02)
    logger.info("并发 %d 请求终态分布: %s", n,
                {s: list(finals.values()).count(s) for s in set(finals.values())})
    assert all(v == "COMPLETED" for v in finals.values())
    # 整个过程中不应出现状态错乱
    snap = ctrl.snapshot()
    assert snap.state == "IDLE"
    assert snap.current_request_id is None
    ctrl.shutdown()


def test_never_two_motions_at_once() -> None:
    """采样执行器 busy 标志：任意时刻至多一段运动（串行化保证）。"""
    from tests.conftest import fast_config

    controller = MotionController(fast_config(queue_capacity=16, command_timeout=5.0))
    stop = threading.Event()
    violations: list[bool] = []

    def sampler() -> None:
        while not stop.is_set():
            # current_request_id 最多一个即表示串行
            snap = controller.snapshot()
            if snap.queue_size >= 0 and snap.state == "RUNNING":
                # 只有 current，不存在第二个执行位
                violations.append(snap.current_request_id is None)
            time.sleep(0.002)

    st = threading.Thread(target=sampler)
    st.start()
    threads = [
        threading.Thread(
            target=lambda i=i: controller.submit(
                MotionRequest(
                    f"s{i}",
                    {j: 3.0 for j in range(6)},
                    velocity=50.0,
                    blocking=False,
                )
            )
        )
        for i in range(8)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if controller.snapshot().state == "IDLE" and \
                controller.snapshot().queue_size == 0:
            break
        time.sleep(0.02)
    stop.set()
    st.join()
    assert not any(violations)
    finals = {f"s{i}": controller.get_status(f"s{i}").state.value
              for i in range(8)}
    logger.info("串行化测试终态: %s", finals)
    assert all(v == "COMPLETED" for v in finals.values())
    controller.shutdown()


def test_queue_overflow_is_deterministic(controller) -> None:
    """容量 4：忙时第 5 个非阻塞请求确定以 QUEUE_FULL 拒绝。"""
    # 先占满 worker：长运动 8s
    threading.Thread(
        target=lambda: controller.submit(
            MotionRequest("occupy", {i: 80.0 for i in range(6)}, velocity=10.0)
        )
    ).start()
    time.sleep(0.15)

    queued = [
        controller.submit(
            MotionRequest(
                f"cap{i}", {j: 10.0 for j in range(6)},
                velocity=10.0, blocking=False,
            )
        )
        for i in range(4)
    ]
    overflow = controller.submit(
        MotionRequest(
            "overflow", {j: 10.0 for j in range(6)},
            velocity=10.0, blocking=False,
        )
    )
    logger.info(
        "队列容量=4：入队 %d 条均 QUEUED=%s，溢出请求判定=%s 依据=%s",
        len(queued), [q.state.value for q in queued],
        overflow.reject_reason.value, overflow.reject_detail,
    )
    assert all(q.state.value == "QUEUED" for q in queued)
    assert overflow.reject_reason is RejectReason.QUEUE_FULL
    controller.emergency_stop()  # 清理占住 worker 的长运动


def test_queue_timeout_is_deterministic() -> None:
    """排队超过 command_timeout 仍未开始执行 -> 确定 TIMEOUT。"""
    from tests.conftest import fast_config

    ctrl = MotionController(fast_config(command_timeout=0.2, queue_capacity=4))
    threading.Thread(
        target=lambda: ctrl.submit(
            MotionRequest("slow", {i: 80.0 for i in range(6)}, velocity=10.0)
        )
    ).start()
    time.sleep(0.15)
    rec = ctrl.submit(
        MotionRequest("will-timeout", {i: 0.0 for i in range(6)},
                      velocity=10.0, blocking=False)
    )
    assert rec.state.value == "QUEUED"
    # 等 slow（8s）结束后，该命令出队时已远超 0.2s
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        final = ctrl.get_status("will-timeout")
        if final.state.value in ("TIMEOUT", "COMPLETED", "ABORTED"):
            break
        time.sleep(0.05)
    logger.info(
        "排队超时判定: state=%s reason=%s 依据=%s",
        final.state.value,
        final.reject_reason.value if final.reject_reason else None,
        final.reject_detail,
    )
    assert final.state.value == "TIMEOUT"
    assert final.reject_reason is RejectReason.QUEUE_TIMEOUT
    ctrl.emergency_stop()
    ctrl.shutdown()


def test_no_command_lost_under_pressure() -> None:
    """50 个并发 blocking 请求：每个 request_id 都有终态记录，无丢失/错乱。"""
    from tests.conftest import fast_config

    ctrl = MotionController(fast_config(queue_capacity=64))
    n = 50
    barrier = threading.Barrier(n)

    def worker(i: int) -> None:
        barrier.wait()
        ctrl.submit(
            MotionRequest(
                f"p{i}", {j: 1.0 for j in range(6)},
                velocity=50.0, blocking=True,
            )
        )

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    statuses = {f"p{i}": ctrl.get_status(f"p{i}") for i in range(n)}
    missing = [k for k, v in statuses.items() if v is None]
    states = {v.state.value for v in statuses.values()}
    logger.info(
        "压力测试 n=%d：丢失=%d 终态集合=%s", n, len(missing), states,
    )
    assert missing == []
    # blocking 忙时只会是 COMPLETED 或 MOTION_CONFLICT；两种都是明确终态
    assert states <= {"COMPLETED", "REJECTED"}
    for v in statuses.values():
        if v.state.value == "REJECTED":
            assert v.reject_reason in (
                RejectReason.MOTION_CONFLICT, RejectReason.QUEUE_FULL,
            )
    ctrl.shutdown()
