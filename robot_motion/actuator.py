"""Simulated actuator state: per-joint positions driven by trapezoid profiles.

Each command moves one or more joints. Every joint gets its own symmetric
trapezoidal velocity profile (accelerate -> cruise -> decelerate); all joints
in a command start together and the command duration is the longest profile,
with shorter joints padding a cruise/hold so they arrive together at
``target_position`` exactly at command end.

Positions are *simulated* local state only; no hardware or I/O is involved.
"""
from __future__ import annotations

from dataclasses import dataclass

from .clock import SimClock
from .config import JointLimits
from .models import JointMotion, JointSnapshot


@dataclass
class _Plan:
    start: float
    start_pos: float
    target: float
    direction: float
    accel_dist: float
    cruise_start: float
    cruise_end: float
    duration: float


class SimulatedActuator:
    """Holds per-joint positions and advances the active motion profile."""

    def __init__(self, limits: tuple[JointLimits, ...], clock: SimClock) -> None:
        self._limits = {lim.joint_id: lim for lim in limits}
        self._clock = clock
        self._positions: dict[int, float] = {
            lim.joint_id: 0.0 for lim in limits
        }
        self._plans: dict[int, _Plan] = {}
        self._commanded_velocity: dict[int, float] = {
            lim.joint_id: 0.0 for lim in limits
        }
        self._targets: dict[int, float | None] = {
            lim.joint_id: None for lim in limits
        }
        self._moving = False

    # ------------------------------------------------------------------ plan
    def start_motion(
        self, joints: tuple[JointMotion, ...], start_ms: int
    ) -> int:
        """Plan trapezoid profiles for every joint; return duration in ms."""
        plans: dict[int, _Plan] = {}
        duration_ms = 0.0
        for jm in joints:
            start_pos = self._positions[jm.joint_id]
            delta = jm.target_position - start_pos
            direction = 1.0 if delta >= 0 else -1.0
            dist = abs(delta)
            v = jm.velocity
            a = jm.acceleration
            # symmetric ramp: distance needed to accelerate then stop.
            accel_dist = v * v / a
            if dist <= 0:
                accel_dur = 0.0
                cruise_dur = 0.0
                accel_dist = 0.0
            elif accel_dist >= dist:
                # short move: triangle profile at peak feasible velocity.
                peak_v = (a * dist) ** 0.5
                accel_dur = peak_v / a
                cruise_dur = 0.0
                accel_dist = dist / 2.0
                v = peak_v
            else:
                accel_dur = v / a
                cruise_dur = (dist - 2.0 * accel_dist) / v
            joint_dur_ms = (2.0 * accel_dur + cruise_dur) * 1000.0
            duration_ms = max(duration_ms, joint_dur_ms)
            cruise_start = start_ms + accel_dur * 1000.0
            cruise_end = start_ms + (accel_dur + cruise_dur) * 1000.0
            plans[jm.joint_id] = _Plan(
                start=start_ms,
                start_pos=start_pos,
                target=jm.target_position,
                direction=direction,
                accel_dist=accel_dist,
                cruise_start=cruise_start,
                cruise_end=cruise_end,
                duration=joint_dur_ms,
            )
            self._targets[jm.joint_id] = jm.target_position
            self._commanded_velocity[jm.joint_id] = jm.velocity
        self._plans = plans
        self._moving = True
        return int(duration_ms + 0.9999)

    # --------------------------------------------------------------- advance
    def advance(self, now_ms: int) -> bool:
        """Advance every joint to ``now_ms``.

        Returns True while the command is still moving, False once every
        joint has arrived. Positions snap exactly to target at completion.
        """
        if not self._plans:
            self._moving = False
            return False
        still = False
        for joint_id, plan in self._plans.items():
            t = now_ms - plan.start
            if t >= plan.duration:
                self._positions[joint_id] = plan.target
                continue
            still = True
            self._positions[joint_id] = self._position_at(plan, t)
        if not still:
            self._positions.update({jid: p.target for jid, p in self._plans.items()})
            for jid in self._plans:
                self._commanded_velocity[jid] = 0.0
            self._plans = {}
            self._moving = False
            return False
        self._moving = True
        return True

    @staticmethod
    def _position_at(plan: _Plan, t_ms: float) -> float:
        """Distance traveled along the path at t ms (unsigned distance)."""
        t = t_ms / 1000.0
        accel_t = (plan.cruise_start - plan.start) / 1000.0
        if accel_t <= 0:
            return plan.start_pos
        a = None
        # recover acceleration from ramp distance/time: a = d_acc / (0.5*t^2)
        a = 2.0 * plan.accel_dist / (accel_t * accel_t)
        if t_ms < plan.cruise_start - plan.start:
            distance = 0.5 * a * t * t
        elif t_ms < plan.cruise_end - plan.start:
            v = a * accel_t
            distance = plan.accel_dist + v * (t - accel_t)
        else:
            v = a * accel_t
            decel_t = t - (plan.cruise_end - plan.start) / 1000.0
            # v*dt - 0.5*a*dt^2 counted from cruise end
            distance = (
                plan.accel_dist
                + v * ((plan.cruise_end - plan.cruise_start) / 1000.0)
                + v * decel_t
                - 0.5 * a * decel_t * decel_t
            )
        return plan.start_pos + plan.direction * distance

    def freeze(self, now_ms: int) -> None:
        """Emergency stop: hold the *last commanded* position immediately.

        The profile is not advanced to ``now_ms``: an E-stop means "stop now",
        so joints keep the position reached at the previous tick, velocity is
        zeroed and targets cleared. The next accepted command plans from that
        held position. ``now_ms`` is recorded only as the stop timestamp.
        """
        del now_ms
        if self._plans:
            for jid in self._plans:
                self._commanded_velocity[jid] = 0.0
            self._plans = {}
        self._moving = False
        for jid in self._targets:
            self._targets[jid] = None

    # -------------------------------------------------------------- snapshot
    def snapshot(self) -> list[JointSnapshot]:
        from .models import JointSnapshot

        return [
            JointSnapshot(
                joint_id=jid,
                position=self._positions[jid],
                commanded_velocity=self._commanded_velocity[jid],
                target_position=self._targets[jid],
                moving=jid in self._plans,
            )
            for jid in sorted(self._positions)
        ]
