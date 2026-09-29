"""HTTP end-to-end flows through the FastAPI app with the fast test config."""
from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from robot_motion.api import create_app
from robot_motion.controller import MotionController
from robot_motion.models import CommandStatus

from .conftest import fast_config, joint


@pytest.fixture
def client():
    ctrl = MotionController(fast_config())
    app = create_app(ctrl)
    with TestClient(app) as c:
        yield c, ctrl
    ctrl.close()


def _wait(client, rid, status, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        body = client.get(f"/commands/{rid}").json()
        if body["status"] == status:
            return body
        time.sleep(0.005)
    raise AssertionError(f"{rid} never reached {status}")


def test_health_config_and_submit_flow(client):
    http, _ = client
    assert http.get("/health").json()["interlock_state"] == "IDLE"
    cfg = http.get("/config").json()
    assert [j["joint_id"] for j in cfg["joint_limits"]] == [0, 1, 2]
    assert cfg["queue_capacity"] == 4

    r = http.post(
        "/commands",
        json={"request_id": "api-1", "joints": [joint(0, 90.0)]},
    )
    assert r.status_code == 202
    body = r.json()
    assert body["accepted"] is True
    assert body["status"] in ("QUEUED", "RUNNING")

    done = _wait(http, "api-1", "COMPLETED")
    assert done["finish_clock_ms"] is not None
    status = http.get("/status").json()
    assert status["state"] == "IDLE"
    j0 = next(j for j in status["joints"] if j["joint_id"] == 0)
    assert abs(j0["position"] - 90.0) < 1e-9


def test_rejections_carry_distinguishable_reasons(client):
    http, _ = client
    cases = [
        ("p", [joint(0, 999.0)], "POSITION_OUT_OF_RANGE"),
        ("v", [joint(0, 10.0, vel=999.0)], "VELOCITY_OUT_OF_RANGE"),
        ("a", [joint(0, 10.0, acc=999.0)], "ACCELERATION_OUT_OF_RANGE"),
    ]
    for rid, joints, reason in cases:
        r = http.post("/commands", json={"request_id": rid, "joints": joints})
        assert r.status_code == 409
        assert r.json()["reject_reason"] == reason

    # malformed JSON / missing request id
    r = http.post("/commands", content=b"not-json", headers={})
    assert r.status_code == 400
    r = http.post("/commands", json={"joints": []})
    assert r.status_code == 422


def test_duplicate_request_over_http_not_executed_twice(client):
    http, _ = client
    payload = {"request_id": "dup", "joints": [joint(1, 30.0)]}
    assert http.post("/commands", json=payload).status_code == 202
    r = http.post("/commands", json=payload)
    assert r.status_code == 409
    assert r.json()["reject_reason"] == "DUPLICATE_REQUEST"
    _wait(http, "dup", "COMPLETED")


def test_estop_and_reset_over_http(client):
    http, _ = client
    r = http.post(
        "/commands",
        json={
            "request_id": "api-moving",
            "joints": [joint(0, 150.0, vel=10.0, acc=20.0)],
        },
    )
    assert r.status_code == 202
    _wait(http, "api-moving", "RUNNING")
    time.sleep(0.03)

    assert http.post("/safety/estop").json()["state"] == "ESTOPPED"
    rec = http.get("/commands/api-moving").json()
    assert rec["status"] == "EMERGENCY_STOPPED"

    blocked = http.post(
        "/commands",
        json={"request_id": "blocked", "joints": [joint(1, 10.0)]},
    )
    assert blocked.json()["reject_reason"] == "INTERLOCK_ENGAGED"

    assert http.post("/safety/reset").json()["state"] == "IDLE"
    r = http.post(
        "/commands",
        json={"request_id": "after", "joints": [joint(1, 10.0)]},
    )
    assert r.status_code == 202
    _wait(http, "after", "COMPLETED")


def test_unknown_request_404(client):
    http, _ = client
    assert http.get("/commands/nope").status_code == 404
