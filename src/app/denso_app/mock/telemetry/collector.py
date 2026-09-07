"""ROS in. Copies topics into the registry and interprets nothing.

Reads ``/dynamic_joint_states`` rather than ``/joint_states``: the latter is
fixed at position, velocity and effort, while the former carries *every*
state interface the hardware plugin exported, by name. Those names are
forwarded to the browser untouched, so the day ``openarm_hardware`` starts
exporting ``temperature_mos`` it appears on the page with nothing here or in
the server changing.

There is no interface whitelist on purpose. A name this file has never heard
of is still a number the robot published, and dropping it would be a
decision this layer has no business making.
"""

from __future__ import annotations

import time

from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.msg import DynamicJointState

from mock.telemetry.registry import JointRegistry


class TelemetryCollector(Node):
    def __init__(self, registry: JointRegistry, watch_loggers: list[str]):
        super().__init__('denso_web_telemetry')
        self._reg = registry
        self._watch = tuple(watch_loggers)
        self._frames = 0
        self._last_rate_at = time.monotonic()

        # BEST_EFFORT on the subscriber side: a RELIABLE publisher still
        # matches it, whereas a RELIABLE subscriber would not match a
        # BEST_EFFORT publisher. This is the permissive half of the handshake.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._on_joints, qos)

        # Every RCLCPP_ERROR in the hardware plugins already lands on /rosout,
        # so the fault feed works today with no robot-side change.
        self.create_subscription(Log, '/rosout', self._on_log, QoSProfile(depth=100))

        self.create_timer(1.0, self._tick_rate)

    def _on_joints(self, msg: DynamicJointState) -> None:
        # Arrival time, not the header stamp: a node that died mid-publish
        # leaves a perfectly plausible timestamp behind, and staleness is
        # exactly the case we need to catch.
        now = time.monotonic()
        self._frames += 1
        for name, iv in zip(msg.joint_names, msg.interface_values):
            self._reg.update(name, dict(zip(iv.interface_names, iv.values)), now)

    def _on_log(self, msg: Log) -> None:
        if msg.level < Log.WARN:
            return
        if self._watch and not msg.name.startswith(self._watch):
            return
        # Level goes out as the raw rcl integer; the page decides what colour
        # a 40 is.
        self._reg.push_log(time.time(), msg.level, msg.name, msg.msg)

    def _tick_rate(self) -> None:
        now = time.monotonic()
        dt = now - self._last_rate_at
        if dt > 0:
            self._reg.note_rate('dynamic_joint_states', self._frames / dt)
        self._frames = 0
        self._last_rate_at = now
