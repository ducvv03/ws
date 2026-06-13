#!/usr/bin/env python3

import sys
import math
import copy
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveDualArmIK(Node):

    def __init__(self):
        super().__init__("move_dual_arm_ik")

        self.cb_group = ReentrantCallbackGroup()

        # 1. Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk", callback_group=self.cb_group)

        # 2. State & Publishers
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)

        # 3. Targets
        self.left_targets, self.right_targets = self.define_targets()

    def define_targets(self):
        def create_pose(x, y, z, qx, qy, qz, qw):
            p = Pose()
            p.position.x = x;
            p.position.y = y;
            p.position.z = z
            p.orientation.x = qx;
            p.orientation.y = qy;
            p.orientation.z = qz;
            p.orientation.w = qw
            return p

        left = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.30, 0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.33, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, 0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, 0.25, 0.40, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        right = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.30, -0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.33, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, -0.0, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, -0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        return left, right

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def parse_target(self, target):
        if isinstance(target, tuple) and len(target) == 2 and target[1] == "HOLD":
            return target[0], True
        return target, False

    def control_hands(self, close=True):
        msg_right = JointTrajectory()
        msg_left = JointTrajectory()

        msg_right.joint_names = [
            "right_thumb_proximal_joint", "right_thumb_metacarpal_joint",
            "right_index_proximal_joint", "right_middle_proximal_joint",
            "right_ring_proximal_joint", "right_pinky_proximal_joint"
        ]
        msg_left.joint_names = [
            "left_thumb_proximal_joint", "left_thumb_metacarpal_joint",
            "left_index_proximal_joint", "left_middle_proximal_joint",
            "left_ring_proximal_joint", "left_pinky_proximal_joint"
        ]

        point = JointTrajectoryPoint()
        if close:
            self.get_logger().info("Closing both hands...")
            point.positions = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
        else:
            self.get_logger().info("Opening both hands...")
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        point.time_from_start.sec = 1
        msg_right.points.append(point)
        msg_left.points.append(point)

        self.right_hand_pub.publish(msg_right)
        self.left_hand_pub.publish(msg_left)

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        if future.done():
            return future.result()
        return None

    # ============================================================
    # SYNCHRONOUS FUNCTIONS
    # ============================================================
    def compute_ik_sync(self, group_name, ik_link_name, target_pose, seed_state=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True

        if seed_state:
            req.ik_request.robot_state = seed_state

        future = self.ik_client.call_async(req)
        res = self._wait_for_future(future)

        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        prefix = "openarm_left" if "left" in group_name else "openarm_right"
        joints = []
        for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position):
            if prefix in name and "finger" not in name:
                joints.append(pos)
        return joints

    def compute_fk_sync(self, joint_names, joint_positions):
        req = GetPositionFK.Request()
        req.header.frame_id = "openarm_body_link0"
        req.fk_link_names = ["openarm_left_tcp", "openarm_right_tcp"]

        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = joint_positions
        req.robot_state = rs

        future = self.fk_client.call_async(req)
        res = self._wait_for_future(future)

        if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
            p1 = res.pose_stamped[0].pose.position
            p2 = res.pose_stamped[1].pose.position
            return math.dist((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))
        return 999.0

    def plan_segment_sync(self, start_joints_l, start_joints_r, end_joints_l, end_joints_r):
        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = True

        rs = RobotState()
        rs.is_diff = True
        rs.joint_state.name = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                              [f"openarm_right_joint{i + 1}" for i in range(7)]
        rs.joint_state.position = list(start_joints_l) + list(start_joints_r)
        goal.request.start_state = rs

        c = Constraints()
        for i, pos in enumerate(end_joints_l):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_left_joint{i + 1}", position=pos, tolerance_above=0.001,
                                tolerance_below=0.001, weight=1.0))
        for i, pos in enumerate(end_joints_r):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_right_joint{i + 1}", position=pos, tolerance_above=0.001,
                                tolerance_below=0.001, weight=1.0))
        goal.request.goal_constraints.append(c)

        future = self.move_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future)

        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        result_wrapper = self._wait_for_future(result_future)

        if result_wrapper is None:
            return None

        result = result_wrapper.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        return result.planned_trajectory

    def execute_trajectory_sync(self, full_trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = full_trajectory
        future = self.exec_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future)

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Master trajectory rejected!")
            return False

        result_future = goal_handle.get_result_async()
        result_wrapper = self._wait_for_future(result_future)

        if result_wrapper is None:
            return False

        res = result_wrapper.result
        return res.error_code.val == MoveItErrorCodes.SUCCESS

    def append_trajectory(self, master_traj, new_traj):
        if not master_traj.joint_trajectory.points:
            master_traj.joint_trajectory.joint_names = new_traj.joint_trajectory.joint_names
            master_traj.joint_trajectory.points = new_traj.joint_trajectory.points
            return

        last_pt = master_traj.joint_trajectory.points[-1]
        base_sec = last_pt.time_from_start.sec
        base_nano = last_pt.time_from_start.nanosec

        for i, pt in enumerate(new_traj.joint_trajectory.points):
            if i == 0: continue

            new_pt = JointTrajectoryPoint()
            new_pt.positions = pt.positions
            new_pt.velocities = pt.velocities
            new_pt.accelerations = pt.accelerations

            total_nano = base_nano + pt.time_from_start.nanosec
            add_sec = total_nano // int(1e9)
            rem_nano = total_nano % int(1e9)

            new_pt.time_from_start.sec = base_sec + pt.time_from_start.sec + add_sec
            new_pt.time_from_start.nanosec = rem_nano

            master_traj.joint_trajectory.points.append(new_pt)

    def planning_thread(self):
        self.get_logger().info("Waiting for MoveIt Actions/Services...")
        self.move_client.wait_for_server()
        self.exec_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.fk_client.wait_for_service()

        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        self.get_logger().info("--- STARTING GLOBAL PLANNING ---")
        master_trajectory = RobotTrajectory()

        curr_l = [pos for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position) if
                  "openarm_left_joint" in name]
        curr_r = [pos for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position) if
                  "openarm_right_joint" in name]

        if len(curr_l) != 7 or len(curr_r) != 7:
            curr_l = [0.0] * 7;
            curr_r = [0.0] * 7

        for i in range(len(self.left_targets)):
            if not rclpy.ok():
                break

            self.get_logger().info(f"Resolving & Planning segment {i} -> {i + 1}...")

            t_left, hold_l = self.parse_target(self.left_targets[i])
            t_right, hold_r = self.parse_target(self.right_targets[i])
            is_hold = hold_l or hold_r

            end_l = t_left if isinstance(t_left, list) else self.compute_ik_sync("left_arm", "openarm_left_tcp", t_left)
            end_r = t_right if isinstance(t_right, list) else self.compute_ik_sync("right_arm", "openarm_right_tcp",
                                                                                   t_right)

            if end_l is None or end_r is None:
                self.get_logger().error(f"IK Failed at waypoint {i}. Aborting global plan.")
                return

            attempts = 0
            valid_segment_traj = None

            while attempts < 5 and rclpy.ok():
                attempts += 1
                segment_traj = self.plan_segment_sync(curr_l, curr_r, end_l, end_r)

                if not segment_traj or not segment_traj.joint_trajectory.points:
                    continue

                if is_hold:
                    points = segment_traj.joint_trajectory.points
                    indices_to_check = [0, len(points) // 4, len(points) // 2, 3 * len(points) // 4, len(points) - 1]
                    valid = True
                    for idx in indices_to_check:
                        dist = self.compute_fk_sync(segment_traj.joint_trajectory.joint_names, points[idx].positions)
                        if dist > 0.25:
                            valid = False
                            break
                    if not valid:
                        self.get_logger().warn(f"Segment {i} violated HOLD distance. Replanning...")
                        continue

                valid_segment_traj = segment_traj
                break

            if not valid_segment_traj:
                self.get_logger().error(f"Failed to plan valid segment {i} after 5 attempts.")
                return

            curr_l = end_l
            curr_r = end_r

            self.append_trajectory(master_trajectory, valid_segment_traj)

        self.get_logger().info("--- GLOBAL PLANNING SUCCESS! Executing full continuous trajectory... ---")

        self.control_hands(close=True)
        time.sleep(1.0)

        success = self.execute_trajectory_sync(master_trajectory)

        if success:
            self.get_logger().info("Done executing massive trajectory.")
            self.control_hands(close=False)
        else:
            self.get_logger().error("Failed to execute master trajectory.")


def main():
    rclpy.init()
    node = MoveDualArmIK()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    planning_thread = threading.Thread(target=node.planning_thread, daemon=True)
    planning_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()