"""Safety interlock: state machine with emergency stop / fault / reset.

States
------
IDLE      : initial, stable, accepting commands
RUNNING   : exactly one motion is executing
ESTOPPED  : emergency stop engaged; every new command is refused
FAULT     : external fault latched; every new command is refused

Transitions
-----------
IDLE    --begin-->     RUNNING
RUNNING --end-->       IDLE
*       --estop-->     ESTOPPED      (running motion is aborted)
*       --fault-->     FAULT         (running motion is aborted)
ESTOPPED/FAULT --reset--> IDLE       (explicit recovery only)

Reset from RUNNING is refused: the machine must be stopped first, so
recovery always returns to a known-stable IDLE with no motion in flight.
"""
from __future__ import annotations

import logging
import threading

from .models import MotionState

logger = logging.getLogger("robot_motion.interlock")


class SafetyInterlock:
    """Thread-safe IDLE/RUNNING/ESTOPPED/FAULT state holder."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = MotionState.IDLE

    @property
    def state(self) -> MotionState:
        with self._lock:
            return self._state

    def begin_motion(self) -> bool:
        """IDLE -> RUNNING. Fails in every other state."""
        with self._lock:
            if self._state is not MotionState.IDLE:
                logger.warning(
                    "begin_motion refused: interlock state=%s", self._state.value
                )
                return False
            self._set(MotionState.RUNNING)
            return True

    def end_motion(self) -> None:
        """RUNNING -> IDLE; never clears ESTOPPED/FAULT."""
        with self._lock:
            if self._state is MotionState.RUNNING:
                self._set(MotionState.IDLE)

    def emergency_stop(self) -> bool:
        """Engage emergency stop. Returns True on the latching transition."""
        with self._lock:
            if self._state is MotionState.ESTOPPED:
                return False
            previous = self._state
            self._set(MotionState.ESTOPPED)
            logger.warning("EMERGENCY STOP latched (from %s)", previous.value)
            return True

    def external_fault(self) -> bool:
        """Latch an external fault. Returns True on the latching transition."""
        with self._lock:
            if self._state is MotionState.FAULT:
                return False
            previous = self._state
            self._set(MotionState.FAULT)
            logger.error("EXTERNAL FAULT latched (from %s)", previous.value)
            return True

    def reset(self) -> bool:
        """ESTOPPED/FAULT -> IDLE. Returns False if not in a tripped state."""
        with self._lock:
            if self._state not in (MotionState.ESTOPPED, MotionState.FAULT):
                logger.warning(
                    "reset refused: interlock state=%s is not tripped",
                    self._state.value,
                )
                return False
            previous = self._state
            self._set(MotionState.IDLE)
            logger.info("interlock reset to IDLE (from %s)", previous.value)
            return True

    def _set(self, new_state: MotionState) -> None:
        # caller holds the lock
        logger.info("interlock %s -> %s", self._state.value, new_state.value)
        self._state = new_state
