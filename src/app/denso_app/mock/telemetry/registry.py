"""Raw joint values, and when each last arrived. Nothing else.

This class deliberately derives *nothing*. It does not know what a
temperature is, what counts as hot, or when a joint should be considered
stale. It stores the numbers ros2_control published, under the interface
names ros2_control used, plus the instant each arrived — and hands that to
the browser, which does the interpreting.

That is the whole point: a C++ reimplementation of this server has to
serialise a map and stamp it, and no more.

The one thing kept here beyond storage is ``group_has_fault`` — the safety
interlock behind HTTP 409. That is not display logic and must not live in a
page the operator can edit from DevTools.
"""

from __future__ import annotations

import threading
import time
from collections import deque

#: Damiao status nibbles at or above this are faults. The only piece of
#: motor semantics the server is allowed to know, and only because refusing
#: a dangerous enable cannot be delegated to the browser.
FAULT_NIBBLE_MIN = 0x8


class JointRegistry:
    def __init__(self, log_size: int = 200):
        self._joints: dict[str, dict[str, float]] = {}
        self._stamp: dict[str, float] = {}
        self._log: deque[dict] = deque(maxlen=log_size)
        self._rates: dict[str, float] = {}
        self._lock = threading.Lock()

    # ---------- written from the ROS thread ----------

    def update(self, joint: str, values: dict[str, float], stamp: float) -> None:
        """Store one joint's interfaces verbatim. No renaming, no filtering."""
        with self._lock:
            self._joints.setdefault(joint, {}).update(values)
            self._stamp[joint] = stamp

    def note_rate(self, name: str, hz: float) -> None:
        with self._lock:
            self._rates[name] = hz

    def push_log(self, stamp: float, level: int, name: str, msg: str) -> None:
        with self._lock:
            self._log.appendleft(
                {'stamp': stamp, 'level': level, 'name': name, 'msg': msg})

    # ---------- read from the asyncio thread ----------

    def snapshot(self) -> dict:
        """One frame, as it goes on the wire.

        ``t`` and every ``stamp`` come from the same monotonic clock, so the
        browser can subtract them without involving its own clock at all.
        """
        with self._lock:
            return {
                't': time.monotonic(),
                'wall': time.time(),
                'rates': dict(self._rates),
                'joints': {
                    name: {'stamp': self._stamp.get(name, 0.0), **values}
                    for name, values in self._joints.items()
                },
                'log': list(self._log)[:40],
            }

    # ---------- safety, not display ----------

    def group_has_fault(self, joints: list[str]) -> bool:
        with self._lock:
            for j in joints:
                status = self._joints.get(j, {}).get('status')
                if status is not None and int(status) & 0x0F >= FAULT_NIBBLE_MIN:
                    return True
            return False
