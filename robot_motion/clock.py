"""Simulated clock: virtual time advanced in deterministic ticks.

The clock has no relationship to wall-clock time except through
``real_ms_per_tick`` (how long the ticker thread sleeps between ticks). This
makes every motion duration, queue timeout and ordering decision a pure
function of the number of ticks elapsed, so tests are deterministic and fast:
e.g. 100 virtual ms with a 10 ms tick and 0.2 real-ms/tick takes ~2 real ms.
"""
from __future__ import annotations

import threading
import time


class SimClock:
    """Monotonic virtual clock advanced by a single background ticker."""

    def __init__(self, tick_ms: int, real_ms_per_tick: float) -> None:
        if tick_ms <= 0:
            raise ValueError("tick_ms must be positive")
        self._tick_ms = tick_ms
        self._real_sleep = max(real_ms_per_tick, 0.0)
        self._cond = threading.Condition()
        self._now_ms = 0
        self._running = False
        self._thread: threading.Thread | None = None

    def now_ms(self) -> int:
        """Current virtual time in milliseconds."""
        with self._cond:
            return self._now_ms

    def start(self) -> None:
        with self._cond:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(
                target=self._run, name="sim-clock", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        with self._cond:
            if not self._running:
                return
            self._running = False
            self._cond.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        with self._cond:
            self._thread = None

    def wait_until(self, target_ms: int, timeout: float | None = None) -> bool:
        """Block until virtual time >= ``target_ms``.

        Returns True if the target was reached, False on real-time ``timeout``
        (seconds) or clock shutdown.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cond:
            while self._now_ms < target_ms and self._running:
                remaining = None
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return False
                self._cond.wait(timeout=remaining)
            return self._now_ms >= target_ms

    def _run(self) -> None:
        while True:
            # time.sleep takes seconds; config value is milliseconds.
            time.sleep(self._real_sleep / 1000.0)
            with self._cond:
                if not self._running:
                    return
                self._now_ms += self._tick_ms
                self._cond.notify_all()
