"""核心控制器：状态机、串行队列、幂等、急停/故障/复位。

并发模型（关键不变量：同一时刻至多一段运动在执行）：
- 一把 Condition 保护全部共享状态（状态机/队列/记录/当前命令）。
- 实际运动在锁外执行，但“谁获得执行权”在锁内决定：
  * 控制器空闲时到达的第一条命令，在提交者线程内联执行；
  * 忙时到达的非阻塞命令入队，由单条 worker 线程在空闲后串行取出执行。
因此并发 submit 下不会出现两段运动，blocking 命令也不会经历排队超时。
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

from .actuator import SimulatedActuator
from .config import ControllerConfig
from .enums import CommandState, ControllerState, RejectReason
from .models import CommandRecord, ControllerSnapshot, MotionRequest
from .validator import validate_request

logger = logging.getLogger("robot_motion")


class MotionController:
    """线程安全的本地运动控制器。"""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self._config = config or ControllerConfig.default()
        self._home = {i: 0.0 for i in range(self._config.joint_count)}
        self._actuator = SimulatedActuator(self._config)
        self._actuator.initialize(self._home)

        self._cond = threading.Condition()
        self._state: ControllerState = ControllerState.IDLE
        self._fault_message = ""
        self._queue: deque[CommandRecord] = deque()
        self._records: dict[str, CommandRecord] = {}
        self._current: CommandRecord | None = None
        self._stop_event: threading.Event | None = None
        self._shutdown = False

        self._worker = threading.Thread(
            target=self._run_worker, name="motion-worker", daemon=True
        )
        self._worker.start()

    # ------------------------------------------------------------------ #
    # 对外 API
    # ------------------------------------------------------------------ #
    def submit(self, request: MotionRequest) -> CommandRecord:
        """提交一条运动命令。

        - blocking=True：空闲时在当前线程执行到结束；忙时直接以
          MOTION_CONFLICT 拒绝（不排队、不超时）。
        - blocking=False：空闲时在当前线程执行到结束并返回终态；忙时入队
          并立即返回 QUEUED。队列满 -> QUEUE_FULL；出队时已超过
          command_timeout -> TIMEOUT。
        - 重复 request_id：原样返回首次记录，绝不执行第二次。
        """
        with self._cond:
            existing = self._records.get(request.request_id)
            if existing is not None:
                logger.info(
                    "拒绝重复提交 request_id=%s 原因=%s（返回首次记录状态=%s）",
                    request.request_id,
                    RejectReason.DUPLICATE_REQUEST.value,
                    existing.state.value,
                )
                return existing

            record = CommandRecord(
                request_id=request.request_id,
                state=CommandState.QUEUED,
                positions=dict(request.positions),
                velocity=request.velocity,
                acceleration=request.acceleration,
                blocking=request.blocking,
                enqueue_time=time.monotonic(),
            )
            self._records[request.request_id] = record

            # 1) 安全联锁优先
            if self._state in (
                ControllerState.EMERGENCY_STOP,
                ControllerState.FAULT,
            ):
                self._reject_locked(
                    record,
                    RejectReason.NOT_RESET,
                    f"控制器处于 {self._state.value}，需先复位（{self._fault_message}）",
                )
                return record

            # 2) 参数校验（位置/速度/加速度，顺序由 validator 固定）
            verdict = validate_request(request, self._config)
            if verdict is not None:
                reason, detail = verdict
                self._reject_locked(record, reason, detail)
                logger.info(
                    "拒绝命令 request_id=%s 参数=%s 原因=%s 依据=%s",
                    request.request_id,
                    self._fmt_params(request),
                    reason.value,
                    detail,
                )
                return record

            # 3) 忙时决策
            if self._current is not None:
                if request.blocking:
                    detail = (
                        f"当前有运动 {self._current.request_id} 正在执行，"
                        "blocking 请求不允许排队"
                    )
                    self._reject_locked(record, RejectReason.MOTION_CONFLICT, detail)
                    logger.info(
                        "拒绝命令 request_id=%s 参数=%s 原因=%s 依据=%s",
                        request.request_id,
                        self._fmt_params(request),
                        RejectReason.MOTION_CONFLICT.value,
                        detail,
                    )
                    return record
                if len(self._queue) >= self._config.queue_capacity:
                    detail = f"等待队列已满（容量 {self._config.queue_capacity}）"
                    self._reject_locked(record, RejectReason.QUEUE_FULL, detail)
                    logger.info(
                        "拒绝命令 request_id=%s 参数=%s 原因=%s 依据=%s",
                        request.request_id,
                        self._fmt_params(request),
                        RejectReason.QUEUE_FULL.value,
                        detail,
                    )
                    return record
                self._queue.append(record)
                logger.info(
                    "命令入队 request_id=%s 参数=%s 队列长度=%d",
                    request.request_id,
                    self._fmt_params(request),
                    len(self._queue),
                )
                self._cond.notify_all()
                return record

            # 4) 空闲：在提交者线程内联获得执行权（消除“空闲竞态”）
            self._claim_execution_locked(record)
            stop_event = self._stop_event
            positions = record.positions
            velocity = record.velocity
            logger.info(
                "命令受理（内联执行）request_id=%s 参数=%s",
                request.request_id,
                self._fmt_params(request),
            )

        completed = self._actuator.execute_move(positions, velocity, stop_event)
        return self._finish_execution(record, completed)

    def get_status(self, request_id: str) -> CommandRecord | None:
        """按幂等键查询单条命令；复位后历史清空，返回 None。"""
        with self._cond:
            return self._records.get(request_id)

    def snapshot(self) -> ControllerSnapshot:
        """查询控制器整体状态。"""
        with self._cond:
            return self._snapshot_locked()

    def emergency_stop(self, message: str = "manual emergency stop") -> None:
        """急停：当前运动就地中止，等待队列全部作废；复位前拒绝新命令。"""
        with self._cond:
            logger.warning("触发急停：%s", message)
            self._state = ControllerState.EMERGENCY_STOP
            self._fault_message = message
            self._drain_queue_locked("紧急停止，等待队列被清空")
            if self._stop_event is not None:
                self._stop_event.set()
            self._cond.notify_all()

    def external_fault(self, message: str) -> None:
        """外部故障：与急停同样的联锁动作，状态记为 FAULT。"""
        with self._cond:
            logger.warning("收到外部故障：%s", message)
            self._state = ControllerState.FAULT
            self._fault_message = message
            self._drain_queue_locked("外部故障，等待队列被清空")
            if self._stop_event is not None:
                self._stop_event.set()
            self._cond.notify_all()

    def reset(self) -> ControllerSnapshot:
        """复位到明确稳定的初始状态：IDLE、回零、队列与历史清空。

        RUNNING 时不允许复位（必须先急停或等待完成），调用被忽略并返回当前快照。
        """
        with self._cond:
            if self._state == ControllerState.RUNNING:
                logger.warning("运动执行中忽略复位请求")
                return self._snapshot_locked()
            if self._state in (
                ControllerState.EMERGENCY_STOP,
                ControllerState.FAULT,
            ):
                logger.info("复位：清空队列与命令历史，执行器回零")
                self._drain_queue_locked("复位清空等待队列")
                self._records.clear()
                self._actuator.reset(self._home)
            self._state = ControllerState.IDLE
            self._fault_message = ""
            self._cond.notify_all()
            return self._snapshot_locked()

    def shutdown(self) -> None:
        """停止 worker（若正在运动也会中断），用于进程退出清理。"""
        with self._cond:
            self._shutdown = True
            if self._stop_event is not None:
                self._stop_event.set()
            self._cond.notify_all()
        self._worker.join(timeout=2.0)

    # ------------------------------------------------------------------ #
    # 内部实现
    # ------------------------------------------------------------------ #
    def _claim_execution_locked(self, record: CommandRecord) -> None:
        """在持锁状态下取得执行权：调用方必须已确认 current is None。"""
        self._current = record
        self._state = ControllerState.RUNNING
        record.state = CommandState.EXECUTING
        record.start_time = time.monotonic()
        self._stop_event = threading.Event()
        logger.info(
            "开始执行 request_id=%s 目标=%s 速度=%s",
            record.request_id,
            record.positions,
            record.velocity,
        )
        self._cond.notify_all()

    def _finish_execution(
        self, record: CommandRecord, completed: bool
    ) -> CommandRecord:
        """运动结束后在锁内落终态并交还执行权；随后唤醒 worker。"""
        with self._cond:
            record.finish_time = time.monotonic()
            if completed:
                record.state = CommandState.COMPLETED
                logger.info("运动完成 request_id=%s", record.request_id)
            else:
                record.state = CommandState.ABORTED
                logger.warning("运动中止 request_id=%s", record.request_id)
            self._current = None
            self._stop_event = None
            # 仅当没有并发的急停/故障时才回到 IDLE
            if self._state == ControllerState.RUNNING:
                self._state = ControllerState.IDLE
            self._cond.notify_all()
        return record

    def _run_worker(self) -> None:
        """仅执行“忙时入队”的非阻塞命令，串行、单条。"""
        while True:
            with self._cond:
                while not self._shutdown and (
                    self._state != ControllerState.IDLE or not self._queue
                ):
                    self._cond.wait()
                if self._shutdown:
                    return

                record = self._queue.popleft()

                # 排队超时：到达队首时已过截止时间 -> 确定性 TIMEOUT
                waited = time.monotonic() - record.enqueue_time
                if waited > self._config.command_timeout:
                    record.state = CommandState.TIMEOUT
                    record.finish_time = time.monotonic()
                    record.reject_reason = RejectReason.QUEUE_TIMEOUT
                    record.reject_detail = (
                        f"排队等待 {waited:.3f}s 超过命令超时 "
                        f"{self._config.command_timeout:.3f}s"
                    )
                    logger.info(
                        "命令排队超时 request_id=%s 依据=%s",
                        record.request_id,
                        record.reject_detail,
                    )
                    self._cond.notify_all()
                    continue

                self._claim_execution_locked(record)
                stop_event = self._stop_event
                positions = dict(record.positions)
                velocity = record.velocity

            # 锁外执行阻塞式运动
            completed = self._actuator.execute_move(positions, velocity, stop_event)
            self._finish_execution(record, completed)

    def _reject_locked(
        self, record: CommandRecord, reason: RejectReason, detail: str
    ) -> None:
        record.state = CommandState.REJECTED
        record.reject_reason = reason
        record.reject_detail = detail
        record.finish_time = time.monotonic()
        self._cond.notify_all()

    def _drain_queue_locked(self, detail: str) -> None:
        """把等待中的命令全部置为 ABORTED 并清空队列。"""
        while self._queue:
            record = self._queue.popleft()
            record.state = CommandState.ABORTED
            record.reject_detail = detail
            record.finish_time = time.monotonic()

    def _snapshot_locked(self) -> ControllerSnapshot:
        return ControllerSnapshot(
            state=self._state.value,
            queue_size=len(self._queue),
            current_request_id=(
                self._current.request_id if self._current else None
            ),
            joint_positions=self._actuator.snapshot_positions(),
            fault_message=self._fault_message,
        )

    @staticmethod
    def _fmt_params(request: MotionRequest) -> str:
        return (
            f"positions={request.positions}, velocity={request.velocity}, "
            f"acceleration={request.acceleration}, blocking={request.blocking}"
        )
