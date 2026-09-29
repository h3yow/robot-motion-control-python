"""本地一键演示：并发下发 / 运动冲突 / 急停 / 越界 / 队列满 / 排队超时 / 幂等。

运行：uv run python examples/demo.py
不依赖任何外部硬件或服务。
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_motion.config import ControllerConfig
from robot_motion.controller import MotionController
from robot_motion.enums import RejectReason
from robot_motion.models import MotionRequest

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger("demo")

HOME = {i: 0.0 for i in range(6)}


def section(title: str) -> None:
    logger.info("=" * 70)
    logger.info(title)
    logger.info("=" * 70)


def show(rec, tag: str) -> None:
    reason = rec.reject_reason.value if rec.reject_reason else "-"
    logger.info(
        "[%s] id=%s state=%s reason=%s detail=%s",
        tag, rec.request_id, rec.state.value, reason, rec.reject_detail or "-",
    )


def main() -> None:
    config = ControllerConfig(
        joint_limits=ControllerConfig.default().joint_limits,
        joint_count=6,
        queue_capacity=3,
        command_timeout=0.4,
    )
    ctrl = MotionController(config)

    # 1. 正常 blocking 运动
    section("1. 正常多关节运动")
    show(ctrl.submit(MotionRequest("ok", {i: 30.0 for i in HOME}, velocity=30.0)),
         "正常")

    # 2. 越界（位置/速度/加速度）
    section("2. 越界请求：原因可区分")
    show(ctrl.submit(MotionRequest("bad-pos", {**HOME, 2: 999.0}, velocity=10.0)),
         "位置越界")
    show(ctrl.submit(MotionRequest("bad-vel", HOME, velocity=999.0)), "速度越界")
    show(ctrl.submit(MotionRequest("bad-acc", HOME, velocity=10.0, acceleration=999.0)),
         "加速度越界")

    # 3. 幂等
    section("3. 同一 request_id 重复提交只执行一次")
    r1 = ctrl.submit(MotionRequest("dup", {i: 10.0 for i in HOME}, velocity=30.0))
    r2 = ctrl.submit(MotionRequest("dup", {i: -99.0 for i in HOME}, velocity=1.0))
    show(r1, "首次")
    logger.info("[幂等] 二次提交返回同一记录=%s", r2 is r1)

    # 4. 运动中冲突拒绝 + 非阻塞排队
    section("4. 运动冲突（blocking 拒绝）与排队（non-blocking 等待）")
    t = threading.Thread(
        target=lambda: show(
            ctrl.submit(MotionRequest("long", {i: 20.0 for i in HOME}, velocity=10.0)),
            "长运动",
        )
    )
    t.start()
    time.sleep(0.15)
    show(ctrl.submit(MotionRequest("conflict", HOME, velocity=10.0)), "冲突拒绝")
    show(ctrl.submit(MotionRequest("queued", {i: -10.0 for i in HOME},
                                  velocity=30.0, blocking=False)), "排队等待")
    t.join()

    # 5. 队列满
    section("5. 队列溢出（容量 3）")
    t = threading.Thread(
        target=lambda: ctrl.submit(
            MotionRequest("occupy", {i: 80.0 for i in HOME}, velocity=10.0)
        )
    )
    t.start()
    time.sleep(0.15)
    for i in range(3):
        show(ctrl.submit(MotionRequest(f"cap{i}", HOME, velocity=10.0,
                                       blocking=False)), f"入队{i}")
    show(ctrl.submit(MotionRequest("overflow", HOME, velocity=10.0,
                                   blocking=False)), "溢出")

    # 6. 急停：运动就地中止 + 未复位拒绝 + 复位回零
    section("6. 急停 / 联锁 / 复位")
    time.sleep(0.1)
    moving_snapshot = ctrl.snapshot()
    logger.info("急停前位置=%.2f 状态=%s",
                moving_snapshot.joint_positions[0], moving_snapshot.state)
    ctrl.emergency_stop("演示急停")
    t.join()
    time.sleep(0.05)
    estop_snapshot = ctrl.snapshot()
    logger.info("急停后位置=%.2f 状态=%s（就地冻结）",
                estop_snapshot.joint_positions[0], estop_snapshot.state)
    show(ctrl.submit(MotionRequest("blocked", HOME, velocity=10.0)), "未复位拒绝")
    reset_snapshot = ctrl.reset()
    logger.info("复位后状态=%s 位置=%s（回零、队列/历史清空）",
                reset_snapshot.state, reset_snapshot.joint_positions)

    # 7. 排队超时（出队时已等待超过 0.4s）
    section("7. 排队超时：确定 TIMEOUT 终态")
    # 忙碌运动：位移 10 / 速度 10 = 1.0s；排队项必然等待 > 0.4s
    t = threading.Thread(
        target=lambda: ctrl.submit(
            MotionRequest("slow", {i: 10.0 for i in HOME}, velocity=10.0)
        )
    )
    t.start()
    time.sleep(0.15)
    late = ctrl.submit(
        MotionRequest("late", HOME, velocity=10.0, blocking=False)
    )
    show(late, "已入队")
    t.join()  # busy 完成后 late 出队 -> TIMEOUT
    show(ctrl.get_status("late"), "排队超时")

    # 8. 并发压力
    section("8. 并发压力：30 线程同时下发，全部有确定终态")
    barrier = threading.Barrier(30)
    results: dict[str, str] = {}

    def worker(i: int) -> None:
        barrier.wait()
        rec_i = ctrl.submit(
            MotionRequest(f"p{i}", {j: 1.0 for j in HOME},
                          velocity=60.0, blocking=True)
        )
        results[rec_i.request_id] = (
            rec_i.state.value + (":" + rec_i.reject_reason.value
                                 if rec_i.reject_reason else "")
        )

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(30)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    distribution: dict[str, int] = {}
    for v in results.values():
        distribution[v] = distribution.get(v, 0) + 1
    logger.info("30 并发终态分布=%s，无丢失=%s",
                distribution, len(results) == 30)

    final = ctrl.snapshot()
    logger.info("最终控制器状态=%s current=%s queue=%d",
                final.state, final.current_request_id, final.queue_size)
    ctrl.shutdown()


if __name__ == "__main__":
    main()
