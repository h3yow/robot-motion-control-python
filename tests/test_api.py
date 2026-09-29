"""HTTP API 端到端测试：提交、查询、拒绝原因、急停、复位、404。"""

from __future__ import annotations

import logging

import pytest
from fastapi.testclient import TestClient

from robot_motion.api import create_app
from robot_motion.controller import MotionController

logger = logging.getLogger("robot_motion.tests")


@pytest.fixture
def client():
    from tests.conftest import fast_config

    ctrl = MotionController(fast_config(command_timeout=2.0))
    clt = TestClient(create_app(ctrl))
    yield clt
    ctrl.shutdown()


def test_submit_and_query(client) -> None:
    body = {
        "request_id": "api-1",
        "positions": {str(i): 12.0 for i in range(6)},
        "velocity": 30.0,
        "acceleration": 60.0,
    }
    r = client.post("/commands", json=body)
    assert r.status_code == 200
    data = r.json()
    logger.info("HTTP 提交请求=%s 响应 state=%s", body, data["state"])
    assert data["state"] == "COMPLETED"

    r2 = client.get("/commands/api-1")
    assert r2.json()["state"] == "COMPLETED"


def test_rejection_reason_distinguishable(client) -> None:
    r1 = client.post("/commands", json={
        "request_id": "p",
        "positions": {str(i): 999.0 for i in range(6)},
        "velocity": 10.0,
    })
    r2 = client.post("/commands", json={
        "request_id": "v",
        "positions": {str(i): 0.0 for i in range(6)},
        "velocity": 999.0,
    })
    logger.info(
        "越界 HTTP 响应: 位置=%s/%s 速度=%s/%s",
        r1.json()["state"], r1.json()["reject_reason"],
        r2.json()["state"], r2.json()["reject_reason"],
    )
    assert r1.json()["reject_reason"] == "POSITION_OUT_OF_RANGE"
    assert r2.json()["reject_reason"] == "VELOCITY_OUT_OF_RANGE"


def test_duplicate_returns_first_record(client) -> None:
    body = {
        "request_id": "dup-api",
        "positions": {str(i): 5.0 for i in range(6)},
        "velocity": 20.0,
    }
    first = client.post("/commands", json=body).json()
    second = client.post("/commands", json={
        "request_id": "dup-api",
        "positions": {str(i): -999.0 for i in range(6)},
        "velocity": 1.0,
    }).json()
    assert second["state"] == first["state"] == "COMPLETED"
    assert second["positions"]["0"] == 5.0  # 未被二次参数覆盖


def test_estop_reset_flow(client) -> None:
    assert client.get("/status").json()["state"] == "IDLE"
    r = client.post("/safety/emergency-stop", json={"message": "api estop"})
    assert r.json()["state"] == "EMERGENCY_STOP"

    blocked = client.post("/commands", json={
        "request_id": "blocked-api",
        "positions": {str(i): 0.0 for i in range(6)},
        "velocity": 10.0,
    }).json()
    assert blocked["reject_reason"] == "NOT_RESET"

    reset = client.post("/safety/reset").json()
    assert reset["state"] == "IDLE"
    assert reset["joint_positions"]["0"] == 0.0


def test_unknown_request_404(client) -> None:
    r = client.get("/commands/nope")
    assert r.status_code == 404


def test_external_fault_endpoint(client) -> None:
    r = client.post("/safety/external-fault", json={"message": "overcurrent"})
    assert r.json()["state"] == "FAULT"
    assert "overcurrent" in r.json()["fault_message"]
