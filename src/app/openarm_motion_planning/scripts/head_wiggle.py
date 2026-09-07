#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drive the OpenArm head a little at a time.

Publishes position commands to the head forward position controller. Every
`--period` seconds it steps to the next pose in a small sweep, so the head
visibly moves a bit each time (pan the neck, nod the head).

The controller claims the joints in the order listed in its YAML
(head_forward_position_controller -> [head_joint_vertical, head_joint_horizontal]),
so the command array is [vertical, horizontal].

Run (after sourcing the workspace and launching with the head enabled):
    python3 scripts/head_wiggle.py
    python3 scripts/head_wiggle.py --period 5.0 --amp 0.4
"""

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class HeadWiggle(Node):
    def __init__(self, topic, period, amp):
        super().__init__("head_wiggle")
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)

        # Sweep sequence as (vertical, horizontal) in radians. Small motions so
        # the head just nudges each step. Starts/ends centered.
        self.poses = [
            (0.0, 0.0),      # center
            (0.0, amp),      # pan neck one way
            (0.0, -amp),     # pan neck the other way
            (0.0, 0.0),      # re-center
            (amp, 0.0),      # nod head up
            (-amp, 0.0),     # nod head down
        ]
        self.idx = 0
        self.topic = topic

        self.get_logger().info(
            f"publishing to '{topic}' every {period:.1f}s, amplitude {amp:.2f} rad "
            f"(order: [vertical, horizontal])"
        )
        # Fire once right away, then every `period` seconds.
        self._tick()
        self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        vertical, horizontal = self.poses[self.idx]
        msg = Float64MultiArray()
        msg.data = [float(vertical), float(horizontal)]
        self.pub.publish(msg)
        self.get_logger().info(
            f"[{self.idx + 1}/{len(self.poses)}] "
            f"vertical={vertical:+.2f}  horizontal={horizontal:+.2f}"
        )
        self.idx = (self.idx + 1) % len(self.poses)


def main():
    parser = argparse.ArgumentParser(description="Wiggle the OpenArm head.")
    parser.add_argument(
        "--topic",
        default="/head_forward_position_controller/commands",
        help="Float64MultiArray command topic of the head forward controller.",
    )
    parser.add_argument(
        "--period", type=float, default=2.0,
        help="Seconds between each step (default: 5.0).",
    )
    parser.add_argument(
        "--amp", type=float, default=1.4,
        help="Motion amplitude in radians (default: 0.4).",
    )
    args = parser.parse_args()

    rclpy.init()
    node = HeadWiggle(args.topic, args.period, args.amp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
