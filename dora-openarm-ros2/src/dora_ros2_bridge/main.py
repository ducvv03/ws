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
to the robot. The left arm is not driven by the Dora graph in this
configuration — only the right arm/gripper are teleoperated. It also forwards
the VR controller's A/B/X/Y button states as a sensor_msgs/Joy on the
/vr_buttons topic.

--mode sim (default): publishes a single merged sensor_msgs/JointState on
/joint_command.

--mode real: publishes separate trajectory_msgs/JointTrajectory per arm to
/left_joint_trajectory_controller/joint_trajectory and
/right_joint_trajectory_controller/joint_trajectory. Targets are ramped
toward (MAX_STEP per cycle) rather than applied directly, and resynced from
the controllers' real /*_joint_trajectory_controller/controller_state
feedback whenever they've drifted more than SYNC_THRESHOLD from what we last
commanded — real hardware shouldn't be commanded to jump instantly to a new
target the way simulation can be.
"""

import argparse
import threading
import time

import dora
import numpy as np
import pyarrow as pa
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node as RclpyNode


class RobotStateSubscriber(RclpyNode):
    """Tracks real physical joint feedback, used by --mode real's safety ramp."""

    def __init__(self, physical_state: dict) -> None:
        """Subscribe to both arms' controller_state feedback topics."""
        super().__init__("dora_bridge_state_subscriber")
        self._physical_state = physical_state
        self.create_subscription(
            JointTrajectoryControllerState,
            "/left_joint_trajectory_controller/controller_state",
            self._left_cb,
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/right_joint_trajectory_controller/controller_state",
            self._right_cb,
            10,
        )

    def _left_cb(self, msg: JointTrajectoryControllerState) -> None:
        self._physical_state["left"][:] = msg.feedback.positions[:7]
        self._physical_state["left_ready"] = True

    def _right_cb(self, msg: JointTrajectoryControllerState) -> None:
        self._physical_state["right"][:] = msg.feedback.positions[:7]
        self._physical_state["right_ready"] = True


def _spin_robot_state_subscriber(physical_state: dict) -> None:
    rclpy.init()
    node = RobotStateSubscriber(physical_state)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Parse CLI args and run the Dora-to-ROS2 bridge node."""
    parser = argparse.ArgumentParser(description="Dora-to-ROS2 bridge node")
    parser.add_argument(
        "--mode",
        choices=["sim", "real"],
        default="sim",
        help="sim: merged JointState on /joint_command (default). real: "
        "separate JointTrajectory per arm, ramped toward target with real "
        "physical-feedback resync for hardware safety.",
    )
    args = parser.parse_args()
    _run(args)


def _run(args: argparse.Namespace) -> None:
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
    # (all joints 0) and a closed/neutral gripper.
    LEFT_FIXED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # buttons[0..3] = a, b, x, y — index into BUTTON_CACHE below.
    BUTTON_INDEX = {"button_a": 0, "button_b": 1, "button_x": 2, "button_y": 3}

    # --- 4. Helpers ---
    def now_stamp() -> dict:
        """Return the current time as a ROS2 stamp dict."""
        t = time.time()
        return {"sec": np.int32(int(t)), "nanosec": np.uint32(int((t % 1.0) * 1e9))}

    def make_joy_msg(stamp: dict, buttons: np.ndarray) -> dict:
        """Build a sensor_msgs/Joy message from the latest button states."""
        return {
            "header": {"stamp": stamp, "frame_id": ""},
            "axes": EMPTY_F32,
            "buttons": buttons,
        }

    if args.mode == "sim":
        p_joint_cmd = node.create_publisher(
            node.create_topic(
                "/joint_command",
                "sensor_msgs/JointState",
                qos_arm,
            )
        )

        # Merged joint order: left/right arm joints interleaved, then each
        # side's primary gripper joint. openarm_left_finger_joint2,
        # openarm_left_hand, openarm_right_finger_joint2, openarm_right_hand,
        # openarm_left_ee_tcp_joint, and openarm_right_ee_tcp_joint are
        # intentionally omitted — the Dora right_position input only carries
        # one gripper scalar.
        JOINT_NAMES = []
        for i in range(7):
            JOINT_NAMES.append(f"openarm_left_joint{i + 1}")
            JOINT_NAMES.append(f"openarm_right_joint{i + 1}")
        JOINT_NAMES.append("openarm_left_finger_joint1")
        JOINT_NAMES.append("openarm_right_finger_joint1")

        def make_joint_command_msg(
            stamp: dict, left: np.ndarray, right: np.ndarray
        ) -> dict:
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

    else:  # real
        p_l_arm = node.create_publisher(
            node.create_topic(
                "/left_joint_trajectory_controller/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
                qos_arm,
            )
        )
        p_r_arm = node.create_publisher(
            node.create_topic(
                "/right_joint_trajectory_controller/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
                qos_arm,
            )
        )

        STAMP_ZERO = {"sec": np.int32(0), "nanosec": np.uint32(0)}
        NAMES_L_ARM = [f"openarm_left_joint{i + 1}" for i in range(7)]
        NAMES_R_ARM = [f"openarm_right_joint{i + 1}" for i in range(7)]

        MAX_STEP = 0.02
        SYNC_THRESHOLD = 0.1

        internal_target_l = np.zeros(7, dtype=np.float64)
        internal_target_r = np.zeros(7, dtype=np.float64)

        physical_state = {
            "left": np.zeros(7, dtype=np.float64),
            "right": np.zeros(7, dtype=np.float64),
            "left_ready": False,
            "right_ready": False,
        }
        threading.Thread(
            target=_spin_robot_state_subscriber,
            args=(physical_state,),
            daemon=True,
        ).start()

        def make_joint_msg(names: list, positions: list) -> dict:
            """Build a single-point JointTrajectory message for immediate execution."""
            return {
                "header": {"stamp": STAMP_ZERO, "frame_id": ""},
                "joint_names": names,
                "points": [
                    {
                        "positions": positions,
                        "velocities": EMPTY_F64,
                        "accelerations": EMPTY_F64,
                        "effort": EMPTY_F64,
                        "time_from_start": {"sec": np.int32(0), "nanosec": np.uint32(0)},
                    }
                ],
            }

        def get_safe_position(
            target: np.ndarray,
            internal_target: np.ndarray,
            physical_pos: np.ndarray,
            is_ready: bool,
        ) -> list:
            """Ramp internal_target toward target by at most MAX_STEP per call.

            Resyncs internal_target to the real physical position first if it
            has drifted more than SYNC_THRESHOLD away from it (e.g. the arm
            was moved independently of this node), so the ramp always starts
            from where the arm actually is rather than compounding on a stale
            internal estimate.
            """
            if not is_ready:
                return internal_target.tolist()

            max_error = np.max(np.abs(internal_target - physical_pos))
            if max_error > SYNC_THRESHOLD:
                internal_target[:] = physical_pos

            step = np.clip(target - internal_target, -MAX_STEP, MAX_STEP)
            internal_target += step
            return internal_target.tolist()

    # --- 5. Dora Loop ---
    dora_node = dora.Node()

    # Latest known [7 arm joints, 1 gripper] for the right side. The left side
    # is not teleoperated, so it always targets LEFT_FIXED.
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

        if args.mode == "sim":
            stamp = now_stamp()
            msg = make_joint_command_msg(stamp, LEFT_FIXED, right_cache)
            p_joint_cmd.publish(pa.array([msg]))
        else:
            safe_r = get_safe_position(
                right_cache[:7],
                internal_target_r,
                physical_state["right"],
                physical_state["right_ready"],
            )
            p_r_arm.publish(pa.array([make_joint_msg(NAMES_R_ARM, safe_r)]))

            safe_l = get_safe_position(
                LEFT_FIXED[:7],
                internal_target_l,
                physical_state["left"],
                physical_state["left_ready"],
            )
            p_l_arm.publish(pa.array([make_joint_msg(NAMES_L_ARM, safe_l)]))


if __name__ == "__main__":
    main()
