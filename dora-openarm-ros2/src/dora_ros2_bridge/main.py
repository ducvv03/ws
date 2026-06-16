# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import time
import threading
import dora
import numpy as np
import pyarrow as pa

import rclpy
from rclpy.node import Node as RclpyNode
from control_msgs.msg import JointTrajectoryControllerState

# ==========================================
# 1. BACKGROUND STATE SUBSCRIBER
# ==========================================
robot_physical_state = {
    "left": np.zeros(7, dtype=np.float64),
    "right": np.zeros(7, dtype=np.float64),
    "left_ready": False,
    "right_ready": False
}


class RobotStateSubscriber(RclpyNode):
    def __init__(self):
        super().__init__('dora_bridge_state_subscriber')
        self.sub_l = self.create_subscription(
            JointTrajectoryControllerState,
            '/left_joint_trajectory_controller/controller_state',
            self.left_cb,
            10)
        self.sub_r = self.create_subscription(
            JointTrajectoryControllerState,
            '/right_joint_trajectory_controller/controller_state',
            self.right_cb,
            10)

    def left_cb(self, msg):
        robot_physical_state["left"][:] = msg.feedback.positions[:7]
        robot_physical_state["left_ready"] = True

    def right_cb(self, msg):
        robot_physical_state["right"][:] = msg.feedback.positions[:7]
        robot_physical_state["right_ready"] = True


def spin_ros_subscriber():
    rclpy.init()
    node = RobotStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# =============
# 2. DORA NODE
# =============
def main() -> None:
    # --- Initialize ROS 2 State ---
    t = threading.Thread(target=spin_ros_subscriber, daemon=True)
    t.start()

    # --- Setup Dora ROS 2 Context ---
    context = dora.Ros2Context()
    options = dora.Ros2NodeOptions(rosout=True)
    node = context.new_node("dora_to_ros2", "/openarm", options)

    qos_arm = dora.Ros2QosPolicies(reliable=True)
    qos_cam = dora.Ros2QosPolicies(reliable=False)

    # Publishers
    p_l_arm = node.create_publisher(
        node.create_topic("/left_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory",
                          qos_arm))
    p_r_arm = node.create_publisher(
        node.create_topic("/right_joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory",
                          qos_arm))

    CAMERA_TOPICS = {
        "camera_wrist_right": "/camera/wrist_right/image_raw/compressed",
        "camera_wrist_left": "/camera/wrist_left/image_raw/compressed",
        "camera_head_left": "/camera/head_left/image_raw/compressed",
        "camera_head_right": "/camera/head_right/image_raw/compressed",
    }
    camera_publishers = {
        eid: node.create_publisher(node.create_topic(topic, "sensor_msgs/CompressedImage", qos_cam))
        for eid, topic in CAMERA_TOPICS.items()
    }

    EMPTY_F64 = np.array([], dtype=np.float64)
    STAMP_ZERO = {"sec": np.int32(0), "nanosec": np.uint32(0)}

    NAMES_L_ARM = [f"openarm_left_joint{i + 1}" for i in range(7)]
    NAMES_R_ARM = [f"openarm_right_joint{i + 1}" for i in range(7)]

    internal_target_l = np.zeros(7, dtype=np.float64)
    internal_target_r = np.zeros(7, dtype=np.float64)

    MAX_STEP = 0.02
    SYNC_THRESHOLD = 0.1

    def now_stamp() -> dict:
        t = time.time()
        return {"sec": np.int32(int(t)), "nanosec": np.uint32(int((t % 1.0) * 1e9))}

    def make_joint_msg(names: list, positions: list) -> dict:
        return {
            "header": {"stamp": STAMP_ZERO, "frame_id": ""},
            "joint_names": names,
            "points": [{
                "positions": positions,
                "velocities": EMPTY_F64,
                "accelerations": EMPTY_F64,
                "effort": EMPTY_F64,
                "time_from_start": {"sec": np.int32(0), "nanosec": np.uint32(0)},
            }],
        }

    def get_safe_position(joystick_cmd, internal_target, physical_pos, is_ready):
        if not is_ready:
            return internal_target.tolist()

        max_error = np.max(np.abs(internal_target - physical_pos))
        if max_error > SYNC_THRESHOLD:
            internal_target[:] = physical_pos[:]

        diff = joystick_cmd - internal_target
        step = np.clip(diff, -MAX_STEP, MAX_STEP)
        internal_target += step

        return internal_target.tolist()

    # --- DORA LOOP ---
    dora_node = dora.Node()

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        value = event["value"]

        if eid in CAMERA_TOPICS:
            stamp = now_stamp()
            msg = {"header": {"stamp": stamp, "frame_id": "world"}, "format": "jpeg",
                   "data": value.to_numpy().astype(np.uint8)}
            camera_publishers[eid].publish(pa.array([msg]))
            continue

        if eid in ("left_position", "right_position"):
            joystick_target = value.to_numpy().astype(np.float64)[:7]

            if eid == "left_position":
                safe_pos = get_safe_position(
                    joystick_target,
                    internal_target_l,
                    robot_physical_state["left"],
                    robot_physical_state["left_ready"]
                )
                p_l_arm.publish(pa.array([make_joint_msg(NAMES_L_ARM, safe_pos)]))

            else:  # right_position
                safe_pos = get_safe_position(
                    joystick_target,
                    internal_target_r,
                    robot_physical_state["right"],
                    robot_physical_state["right_ready"]
                )
                p_r_arm.publish(pa.array([make_joint_msg(NAMES_R_ARM, safe_pos)]))


if __name__ == "__main__":
    main()
