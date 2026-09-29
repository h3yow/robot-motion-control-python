"""FastAPI HTTP 接口：命令提交/查询、急停/故障/复位/状态。

约定：命令被领域规则拒绝时 HTTP 仍返回 200，响应体 state=REJECTED 并带
reject_reason / reject_detail，便于调用方按可区分原因处理；资源不存在用 404。
"""

from __future__ import annotations

from typing import Literal

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from .controller import MotionController
from .enums import CommandState
from .models import MotionRequest


class MotionCommandIn(BaseModel):
    request_id: str = Field(..., min_length=1)
    positions: dict[int, float]
    velocity: float = 5.0
    acceleration: float = 5.0
    blocking: bool = True


class CommandOut(BaseModel):
    request_id: str
    state: str
    reject_reason: str | None = None
    reject_detail: str = ""
    enqueue_time: float
    start_time: float | None = None
    finish_time: float | None = None
    positions: dict[int, float]
    velocity: float
    acceleration: float
    blocking: bool


class SnapshotOut(BaseModel):
    state: str
    queue_size: int
    current_request_id: str | None = None
    joint_positions: dict[int, float]
    fault_message: str = ""


class SafetyMessageIn(BaseModel):
    message: str = ""


def _to_command_out(record) -> CommandOut:
    return CommandOut(
        request_id=record.request_id,
        state=record.state.value,
        reject_reason=(
            record.reject_reason.value if record.reject_reason else None
        ),
        reject_detail=record.reject_detail,
        enqueue_time=record.enqueue_time,
        start_time=record.start_time,
        finish_time=record.finish_time,
        positions=record.positions,
        velocity=record.velocity,
        acceleration=record.acceleration,
        blocking=record.blocking,
    )


def create_app(controller: MotionController | None = None) -> FastAPI:
    app = FastAPI(title="Local Robot Motion Control", version="0.1.0")
    app.state.controller = controller or MotionController()

    @app.post("/commands", response_model=CommandOut)
    def submit_command(payload: MotionCommandIn) -> CommandOut:
        request = MotionRequest(
            request_id=payload.request_id,
            positions=payload.positions,
            velocity=payload.velocity,
            acceleration=payload.acceleration,
            blocking=payload.blocking,
        )
        record = app.state.controller.submit(request)
        return _to_command_out(record)

    @app.get("/commands/{request_id}", response_model=CommandOut)
    def get_command(request_id: str) -> CommandOut:
        record = app.state.controller.get_status(request_id)
        if record is None:
            raise HTTPException(status_code=404, detail="未知 request_id（可能已复位清空）")
        return _to_command_out(record)

    @app.get("/status", response_model=SnapshotOut)
    def get_status() -> SnapshotOut:
        snap = app.state.controller.snapshot()
        return SnapshotOut(**snap.__dict__)

    @app.post("/safety/emergency-stop", response_model=SnapshotOut)
    def emergency_stop(payload: SafetyMessageIn) -> SnapshotOut:
        app.state.controller.emergency_stop(
            payload.message or "manual emergency stop"
        )
        return SnapshotOut(**app.state.controller.snapshot().__dict__)

    @app.post("/safety/external-fault", response_model=SnapshotOut)
    def external_fault(payload: SafetyMessageIn) -> SnapshotOut:
        app.state.controller.external_fault(payload.message or "unspecified fault")
        return SnapshotOut(**app.state.controller.snapshot().__dict__)

    @app.post("/safety/reset", response_model=SnapshotOut)
    def reset() -> SnapshotOut:
        return SnapshotOut(**app.state.controller.reset().__dict__)

    return app


app = create_app()
