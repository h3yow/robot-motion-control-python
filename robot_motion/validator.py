"""请求参数校验：位置 / 速度 / 加速度越界判定。"""

from __future__ import annotations

import math

from .config import ControllerConfig
from .enums import RejectReason
from .models import MotionRequest

# (拒绝原因, 判定依据)；None 表示校验通过
ValidationResult = tuple[RejectReason, str] | None


def validate_request(
    request: MotionRequest, config: ControllerConfig
) -> ValidationResult:
    """返回拒绝原因与判定依据；None 表示通过。

    校验顺序固定（保证原因可区分、可解释）：
    0. 结构/数值合法性：空目标、未知关节、缺失关节、NaN/Inf -> BAD_REQUEST
    1. 位置越界：任一关节目标位置超出 [min, max]
    2. 速度越界：必须 > 0 且不超过涉及关节的最小 max_velocity
    3. 加速度越界：必须 > 0 且不超过涉及关节的最小 max_acceleration
    """
    positions = request.positions

    # 0. 结构合法性
    if not isinstance(positions, dict) or not positions:
        return (
            RejectReason.BAD_REQUEST,
            "positions 为空或不是关节号->位置的映射",
        )
    unknown = sorted(set(positions) - set(config.joint_limits))
    if unknown:
        return (
            RejectReason.BAD_REQUEST,
            f"存在未知关节 {unknown}，合法关节为 {sorted(config.joint_limits)}",
        )
    missing = sorted(set(config.joint_limits) - set(positions))
    if missing:
        return (
            RejectReason.BAD_REQUEST,
            f"缺少关节 {missing} 的目标位置，多关节运动必须覆盖全部关节",
        )
    numeric_values = [*positions.values(), request.velocity, request.acceleration]
    if any(not math.isfinite(v) for v in numeric_values):
        return (
            RejectReason.BAD_REQUEST,
            "位置/速度/加速度中存在 NaN 或 Inf",
        )

    # 1. 位置越界
    for joint in sorted(positions):
        target = positions[joint]
        limit = config.joint_limits[joint]
        if target < limit.min_position or target > limit.max_position:
            return (
                RejectReason.POSITION_OUT_OF_RANGE,
                f"关节 {joint} 目标位置 {target} 超出限位 "
                f"[{limit.min_position}, {limit.max_position}]",
            )

    involved = [config.joint_limits[j] for j in sorted(positions)]

    # 2. 速度越界
    vel_cap = min(lim.max_velocity for lim in involved)
    if request.velocity <= 0:
        return (
            RejectReason.VELOCITY_OUT_OF_RANGE,
            f"速度 {request.velocity} 必须为正数",
        )
    if request.velocity > vel_cap:
        return (
            RejectReason.VELOCITY_OUT_OF_RANGE,
            f"速度 {request.velocity} 超过涉及关节最小速度上限 {vel_cap}",
        )

    # 3. 加速度越界
    acc_cap = min(lim.max_acceleration for lim in involved)
    if request.acceleration <= 0:
        return (
            RejectReason.ACCELERATION_OUT_OF_RANGE,
            f"加速度 {request.acceleration} 必须为正数",
        )
    if request.acceleration > acc_cap:
        return (
            RejectReason.ACCELERATION_OUT_OF_RANGE,
            f"加速度 {request.acceleration} 超过涉及关节最小加速度上限 {acc_cap}",
        )

    return None
