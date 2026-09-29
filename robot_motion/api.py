"""FastAPI HTTP surface for the motion controller.

Endpoints
---------
GET  /health                -> liveness + current state
POST /commands              -> submit a multi-joint motion command
GET  /commands/{request_id} -> single command lifecycle status
GET  /status                -> full controller snapshot
POST /safety/estop          -> engage emergency stop
POST /safety/fault          -> latch an external fault
POST /safety/reset          -> clear ESTOPPED/FAULT back to IDLE
GET  /config                -> effective limits and queue/clock config

Rejected submissions return HTTP 409 with a structured body containing the
distinguishable ``reject_reason``; malformed payloads return 422/400.
"""
from __future__ import annotations

import logging
from typing import Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from .controller import MotionController

logger = logging.getLogger("robot_motion.api")


class JointRequest(BaseModel):
    joint_id: int
    target_position: float
    velocity: float
    acceleration: float


class CommandRequest(BaseModel):
    request_id: str = Field(min_length=1)
    joints: list[JointRequest] = Field(min_length=1)


def _record_dict(rec: Any) -> dict:
    if rec is None:
        return {}
    return {
        "request_id": rec.request_id,
        "status": rec.status.value,
        "reject_reason": rec.reject_reason.value if rec.reject_reason else None,
        "submit_clock_ms": rec.submit_clock_ms,
        "start_clock_ms": rec.start_clock_ms,
        "finish_clock_ms": rec.finish_clock_ms,
        "detail": rec.detail,
        "joints": [
            {
                "joint_id": j.joint_id,
                "target_position": j.target_position,
                "velocity": j.velocity,
                "acceleration": j.acceleration,
            }
            for j in rec.joints
        ],
    }


def create_app(controller: MotionController | None = None) -> FastAPI:
    owns_controller = controller is None
    ctrl = controller or MotionController()
    app = FastAPI(
        title="Local Simulated Robot Motion Control",
        version="0.1.0",
    )

    @app.get("/health")
    def health() -> dict:
        snap = ctrl.snapshot()
        return {"status": "ok", "interlock_state": snap.state.value}

    @app.get("/config")
    def get_config() -> dict:
        cfg = ctrl.config
        return {
            "joint_limits": [
                {
                    "joint_id": lim.joint_id,
                    "min_position": lim.min_position,
                    "max_position": lim.max_position,
                    "max_velocity": lim.max_velocity,
                    "max_acceleration": lim.max_acceleration,
                }
                for lim in cfg.joint_limits
            ],
            "queue_capacity": cfg.queue_capacity,
            "queue_timeout_ms": cfg.queue_timeout_ms,
            "clock_tick_ms": cfg.clock_tick_ms,
            "clock_real_ms_per_tick": cfg.clock_real_ms_per_tick,
        }

    @app.post("/commands", status_code=202)
    async def submit_command(request: Request) -> JSONResponse:
        # Parse manually so the controller's own INVALID_PAYLOAD explanations
        # (duplicate joint, unknown joint, non-positive v/a) are preserved.
        try:
            body = await request.json()
        except Exception:
            raise HTTPException(status_code=400, detail="body must be JSON")
        if not isinstance(body, dict) or "request_id" not in body:
            raise HTTPException(
                status_code=422, detail="request_id is required"
            )
        result = ctrl.submit(body.get("request_id"), body.get("joints", []))
        payload = {
            **result.as_dict(),
            "command": _record_dict(result.record),
        }
        code = 202 if result.accepted else 409
        logger.info("POST /commands -> %s body=%s", code, body)
        return JSONResponse(status_code=code, content=payload)

    @app.get("/commands/{request_id}")
    def get_command(request_id: str) -> dict:
        rec = ctrl.get_status(request_id)
        if rec is None:
            raise HTTPException(
                status_code=404, detail=f"unknown request_id {request_id!r}"
            )
        return _record_dict(rec)

    @app.get("/status")
    def get_status() -> dict:
        snap = ctrl.snapshot()
        return {
            "state": snap.state.value,
            "clock_ms": snap.clock_ms,
            "queue_capacity": snap.queue_capacity,
            "queued_request_ids": snap.queued_request_ids,
            "current_request_id": snap.current_request_id,
            "joints": [
                {
                    "joint_id": j.joint_id,
                    "position": j.position,
                    "commanded_velocity": j.commanded_velocity,
                    "target_position": j.target_position,
                    "moving": j.moving,
                }
                for j in snap.joints
            ],
            "commands": {
                rid: _record_dict(rec) for rid, rec in snap.commands.items()
            },
        }

    @app.post("/safety/estop")
    def estop() -> dict:
        ctrl.emergency_stop()
        return {"state": ctrl.snapshot().state.value}

    @app.post("/safety/fault")
    def fault() -> dict:
        ctrl.external_fault()
        return {"state": ctrl.snapshot().state.value}

    @app.post("/safety/reset")
    def reset() -> dict:
        ctrl.reset()
        return {"state": ctrl.snapshot().state.value}

    if owns_controller:
        @app.on_event("shutdown")
        def _shutdown() -> None:  # pragma: no cover
            ctrl.close()

    return app
