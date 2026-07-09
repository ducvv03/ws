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

"""Dora-to-ROS2 Bridge Node.

This node acts as a state holder and message translator between the Dora dataflow
and ROS 2 control interfaces. It subscribes to the right arm's joint position
input from the Dora graph and forwards it, merged with a fixed left-arm pose,
into a single sensor_msgs/JointState on the /joint_command topic. The left arm
is not driven by the Dora graph in this configuration — only the right arm/
gripper are teleoperated. It also forwards the VR controller's A/B/X/Y button
states as a sensor_msgs/Joy on the /vr_buttons topic.
"""

import time

import dora
import numpy as np
import pyarrow as pa


def main() -> None:
    """Run the Dora-to-ROS2 bridge node."""
    # --- 1. ROS 2 Setup ---
    context = dora.Ros2Context()
    options = dora.Ros2NodeOptions(rosout=True)
    node = context.new_node("dora_to_ros2", "/openarm", options)

    qos_arm = dora.Ros2QosPolicies(reliable=True)
    # Best-effort: the VR headset re-sends button state on every tick (up to
    # ~500 Hz) whether or not it changed. Reliable QoS made the writer block
    # trying to redeliver that firehose to /vr_buttons whenever nothing was
    # subscribed/keeping up, eventually erroring out with a publish timeout.
    qos_buttons = dora.Ros2QosPolicies(reliable=False)

    # --- 2. Define Publishers ---
    p_joint_cmd = node.create_publisher(
        node.create_topic(
            "/joint_command",
            "sensor_msgs/JointState",
            qos_arm,
        )
    )
    p_buttons = node.create_publisher(
        node.create_topic(
            "/vr_buttons",
            "sensor_msgs/Joy",
            qos_buttons,
        )
    )

    # --- 3. Pre-defined Constants ---
    EMPTY_F64 = np.array([], dtype=np.float64)
    EMPTY_F32 = np.array([], dtype=np.float32)

    # Left arm is not teleoperated in this configuration: hold a fixed pose
    # (all joints 0 except joint4 = 1.5 rad) and a closed/neutral gripper.
    LEFT_FIXED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # Merged joint order: left/right arm joints interleaved, then each side's
    # primary gripper joint. openarm_left_finger_joint2, openarm_left_hand,
    # openarm_right_finger_joint2, openarm_right_hand, openarm_left_ee_tcp_joint,
    # and openarm_right_ee_tcp_joint are intentionally omitted — the Dora
    # right_position input only carries one gripper scalar.
    JOINT_NAMES = []
    for i in range(7):
        JOINT_NAMES.append(f"openarm_left_joint{i + 1}")
        JOINT_NAMES.append(f"openarm_right_joint{i + 1}")
    JOINT_NAMES.append("openarm_left_finger_joint1")
    JOINT_NAMES.append("openarm_right_finger_joint1")

    # buttons[0..3] = a, b, x, y — index into BUTTON_CACHE below.
    BUTTON_INDEX = {"button_a": 0, "button_b": 1, "button_x": 2, "button_y": 3}

    # --- 4. Helpers ---
    def now_stamp() -> dict:
        """Return the current time as a ROS2 stamp dict."""
        t = time.time()
        return {"sec": np.int32(int(t)), "nanosec": np.uint32(int((t % 1.0) * 1e9))}

    def make_joint_command_msg(stamp: dict, left: np.ndarray, right: np.ndarray) -> dict:
        """Build a merged JointState message from the latest left/right arrays."""
        positions = []
        for i in range(7):
            positions.append(left[i])
            positions.append(right[i])
        positions.append(left[7])
        positions.append(right[7])

        return {
            "header": {"stamp": stamp, "frame_id": ""},
            "name": JOINT_NAMES,
            "position": positions,
            "velocity": EMPTY_F64,
            "effort": EMPTY_F64,
        }

    def make_joy_msg(stamp: dict, buttons: np.ndarray) -> dict:
        """Build a sensor_msgs/Joy message from the latest button states."""
        return {
            "header": {"stamp": stamp, "frame_id": ""},
            "axes": EMPTY_F32,
            "buttons": buttons,
        }

    # --- 5. Dora Loop ---
    dora_node = dora.Node()

    # Latest known [7 arm joints, 1 gripper] for the right side. The left side
    # is not teleoperated, so it always publishes LEFT_FIXED.
    right_cache = np.zeros(8, dtype=np.float64)
    button_cache = np.zeros(4, dtype=np.int32)

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        value = event["value"]

        if eid in BUTTON_INDEX:
            idx = BUTTON_INDEX[eid]
            new_val = int(bool(value[0].as_py()))
            if new_val == button_cache[idx]:
                continue
            button_cache[idx] = new_val
            stamp = now_stamp()
            msg = make_joy_msg(stamp, button_cache)
            p_buttons.publish(pa.array([msg]))
            continue

        if eid != "right_position":
            continue

        vals = value.to_numpy().astype(np.float64)
        n = min(len(vals), 8)
        right_cache[:n] = vals[:n]

        stamp = now_stamp()
        msg = make_joint_command_msg(stamp, LEFT_FIXED, right_cache)
        p_joint_cmd.publish(pa.array([msg]))


if __name__ == "__main__":
    main()
