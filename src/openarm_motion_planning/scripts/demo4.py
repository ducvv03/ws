#!/usr/bin/env python3

import sys
import math
import threading
import time
import copy
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotState,
    AttachedCollisionObject, CollisionObject, RobotTrajectory
)
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene


class MoveDualArmHybrid(Node):
    def __init__(self):
        super().__init__("move_dual_arm_hybrid")

        # ==========================================
        # PLAN or LOAD mode
        # ==========================================
        # self.RUN_MODE = "PLAN"
        self.RUN_MODE = "LOAD"
        self.CSV_PATH = "40dg.csv"
        # ==========================================

        self.cb_group = ReentrantCallbackGroup()

        # 1. Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                               callback_group=self.cb_group)

        # 2. State & Publishers
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        # Topic for hands
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_revo2_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_revo2_hand_controller/joint_trajectory", 10)

        # Topic Streaming for arms
        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory",
                                                  10)
        self.right_arm_pub = self.create_publisher(JointTrajectory,
                                                   "/right_joint_trajectory_controller/joint_trajectory", 10)

        # 3. Targets
        self.left_targets = self.define_targets()

    # ============================================================
    # DEFINE POSE
    # ============================================================
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

        qx, qy, qz, qw = 0.71, 0.0, 0.71, 0.0
        # qx2, qy2, qz2, qw2 = 0.965926, 0.0, 0.258819, 0.0
        qx2, qy2, qz2, qw2 = 0.939693, 0.0, 0.342020, 0.0

        left_forward = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 0. Home
            (create_pose(0.014604, 0.1535, 0.35, qx, qy, qz, qw)), # 1. Raise hand
            (create_pose(0.30, 0.23, 0.35, qx2, qy2, qz2, qw2), "CLOSE_HAND"),         # 2. Pre-pick
            (create_pose(0.30, 0.16, 0.35, qx2, qy2, qz2, qw2)),  # 3. Pick
            (create_pose(0.30, 0.16, 0.40, qx2, qy2, qz2, qw2), "WAIT_ENTER"), # 4. Lift
        ]
        return left_forward

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def parse_target(self, target):
        if isinstance(target, tuple) and len(target) == 2:
            return target[0], target[1]
        return target, None

    def control_hands(self, close=True):
        msg_right = JointTrajectory()
        msg_left = JointTrajectory()

        msg_right.joint_names = [f"right_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_right.joint_names.insert(1, "right_thumb_metacarpal_joint")

        msg_left.joint_names = [f"left_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_left.joint_names.insert(1, "left_thumb_metacarpal_joint")

        point = JointTrajectoryPoint()
        if close:
            self.get_logger().info("Closing hands ...")
            point.positions = [1.1, 0.0, 0.4, 0.7, 0.9, 1.0]
            # point.positions = [0.0, 0.0, 0.5, 0.6, 0.6, 0.5]
        else:
            self.get_logger().info("Opening hands ...")
            point.positions = [1.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            # point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        point.time_from_start.sec = 1
        msg_right.points.append(point)
        msg_left.points.append(point)

        self.right_hand_pub.publish(msg_right)
        self.left_hand_pub.publish(msg_left)

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result() if future.done() else None

    # ============================================================
    # SAVE & LOAD CSV
    # ============================================================
    def save_to_csv(self, filename, trajectories):
        try:
            with open(filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                header = ["segment_idx"] + \
                         [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                         [f"openarm_right_joint{i + 1}" for i in range(7)]
                writer.writerow(header)

                for seg_idx, traj in enumerate(trajectories):
                    for pt in traj.joint_trajectory.points:
                        row = [seg_idx] + list(pt.positions)
                        writer.writerow(row)

            self.get_logger().info(f"Saved {len(trajectories)} trajectories into: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error when save CSV: {e}")

    def load_from_csv(self, filename):
        trajectories = []
        if not os.path.exists(filename):
            self.get_logger().error(f"File {filename} doesnt exist!")
            return None

        try:
            current_seg_idx = -1
            current_traj = None
            joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                          [f"openarm_right_joint{i + 1}" for i in range(7)]

            with open(filename, mode='r') as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:
                    if not row: continue
                    seg_idx = int(row[0])
                    positions = [float(x) for x in row[1:]]

                    if seg_idx != current_seg_idx:
                        if current_traj is not None:
                            trajectories.append(current_traj)
                        current_traj = RobotTrajectory()
                        current_traj.joint_trajectory.joint_names = joint_names
                        current_seg_idx = seg_idx

                    pt = JointTrajectoryPoint()
                    pt.positions = positions
                    current_traj.joint_trajectory.points.append(pt)

                if current_traj is not None:
                    trajectories.append(current_traj)

            self.get_logger().info(f"Loaded {len(trajectories)} trajectories from: {filename}")
            return trajectories
        except Exception as e:
            self.get_logger().error(f"Error when read CSV: {e}")
            return None

    # ============================================================
    # COMPUTE IK
    # ============================================================
    def compute_ik_left_sync(self, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "left_arm"
        req.ik_request.ik_link_name = "openarm_left_tcp"
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True

        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state

        res = self._wait_for_future(self.ik_client.call_async(req))
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        joints = [pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position)
                  if "openarm_left" in name and "finger" not in name]
        return joints

    # ========
    # PLANNING
    # ========
    def plan_left_segment_sync(self, end_l):
        goal = MoveGroup.Goal()
        goal.request.group_name = "left_arm"
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = True

        c = Constraints()
        for i, pos in enumerate(end_l):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_left_joint{i + 1}", position=pos, tolerance_above=0.01,
                                tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)

        goal_handle = self._wait_for_future(self.move_client.send_goal_async(goal))
        if not goal_handle or not goal_handle.accepted:
            return None

        result_wrapper = self._wait_for_future(goal_handle.get_result_async())
        if result_wrapper is None: return None

        result = result_wrapper.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        return result.planned_trajectory

    # ============================================================
    # MIRROR MOTION FOR RIGHT ARM
    # ============================================================
    def mirror_left_trajectory_to_dual(self, left_traj):
        dual_traj = RobotTrajectory()
        dual_traj.joint_trajectory.joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                                                 [f"openarm_right_joint{i + 1}" for i in range(7)]

        for pt in left_traj.joint_trajectory.points:
            left_pos = pt.positions
            if len(left_pos) != 7: continue

            right_pos = [-left_pos[0], -left_pos[1], -left_pos[2], left_pos[3], -left_pos[4], -left_pos[5],
                         -left_pos[6]]

            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(left_pos) + right_pos
            dual_traj.joint_trajectory.points.append(new_pt)

        return dual_traj

    # ============================================================
    # REVERSE MOTION
    # ============================================================
    def reverse_trajectory(self, traj: RobotTrajectory):
        rev_traj = RobotTrajectory()
        rev_traj.joint_trajectory.header = traj.joint_trajectory.header
        rev_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names

        reversed_points = list(reversed(traj.joint_trajectory.points))
        for pt in reversed_points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            rev_traj.joint_trajectory.points.append(new_pt)

        return rev_traj

    # ============================================================
    # TRANSFORM TRAJECTORY AND SEND TOPIC (Streaming)
    # ============================================================
    def execute_by_streaming(self, dual_trajectory):
        if not dual_trajectory.joint_trajectory.points:
            return

        names = dual_trajectory.joint_trajectory.joint_names
        idx_l = [i for i, n in enumerate(names) if "left" in n]
        idx_r = [i for i, n in enumerate(names) if "right" in n]
        names_l = [names[i] for i in idx_l]
        names_r = [names[i] for i in idx_r]

        all_pts = [pt.positions for pt in dual_trajectory.joint_trajectory.points]
        stream_pts = []
        last_p = all_pts[0]
        stream_pts.append(last_p)

        STEP_RAD = 0.02

        for p in all_pts[1:]:
            while True:
                diff = [a - b for a, b in zip(p, last_p)]
                max_diff = max(abs(d) for d in diff)
                if max_diff < STEP_RAD:
                    break

                ratio = STEP_RAD / max_diff
                interp_p = [last_p[i] + diff[i] * ratio for i in range(len(p))]
                stream_pts.append(interp_p)
                last_p = interp_p

        stream_pts.append(all_pts[-1])

        self.get_logger().info(f"==> Streaming {len(stream_pts)} dual-points ...")

        for p in stream_pts:
            if not rclpy.ok():
                break

            msg_l = JointTrajectory()
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint()
            pt_l.positions = [p[i] for i in idx_l]
            pt_l.time_from_start.sec = 0;
            pt_l.time_from_start.nanosec = 0
            msg_l.points.append(pt_l)

            msg_r = JointTrajectory()
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint()
            pt_r.positions = [p[i] for i in idx_r]
            pt_r.time_from_start.sec = 0;
            pt_r.time_from_start.nanosec = 0
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)

            time.sleep(0.02)

    # ============================================================
    # BACKGROUND THREAD
    # ============================================================
    def planning_thread(self):
        if self.RUN_MODE == "PLAN":
            self.get_logger().info("Waiting MoveIt Service/Action ...")
            self.move_client.wait_for_server()
            self.ik_client.wait_for_service()
            self.scene_client.wait_for_service()

        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        self.control_hands(close=False)
        time.sleep(2.0)

        self.get_logger().info(f"--- STAGE 1: PLANNING (MODE: {self.RUN_MODE}) ---")

        saved_trajectories = []

        if self.RUN_MODE == "LOAD":
            loaded = self.load_from_csv(self.CSV_PATH)
            if loaded is None:
                return
            saved_trajectories = loaded

        for i in range(len(self.left_targets)):
            if not rclpy.ok(): break
            self.get_logger().info(f"\n--- Processing segment {i} -> {i + 1} ---")

            t_left, tag = self.parse_target(self.left_targets[i])

            if self.RUN_MODE == "PLAN":
                end_l = t_left if isinstance(t_left, list) else self.compute_ik_left_sync(t_left)
                if end_l is None:
                    self.get_logger().error(f"Error IK Left Arm at step {i}.")
                    return

                left_trajectory = self.plan_left_segment_sync(end_l)
                if not left_trajectory:
                    self.get_logger().error(f"Error Planning Left Arm at step {i}.")
                    return

                dual_trajectory = self.mirror_left_trajectory_to_dual(left_trajectory)
                saved_trajectories.append(dual_trajectory)
            else:
                if i < len(saved_trajectories):
                    dual_trajectory = saved_trajectories[i]
                else:
                    self.get_logger().error("File CSV doesnt enough (segment)!")
                    return

            self.execute_by_streaming(dual_trajectory)

            if tag == "HOLD" or tag == "CLOSE_HAND":
                self.control_hands(close=True)
                time.sleep(2.0)

            elif tag == "WAIT_ENTER":
                if self.RUN_MODE == "PLAN":
                    self.save_to_csv(self.CSV_PATH, saved_trajectories)

                print("\n\033[93m" + "=" * 50)
                print(" Press Enter! ")
                print("=" * 50 + "\033[0m\n")
                input()
                break


        self.get_logger().info("--- STAGE 2: REVERSE MOTION ---")

        print("\033[92m---> Place object at Pick position...\033[0m")
        traj_lift_to_pick = self.reverse_trajectory(saved_trajectories[-1])
        self.execute_by_streaming(traj_lift_to_pick)

        # 2.(DROP)


        # 3. Move to home
        print("\033[92m---> Reverse arm to come Home...\033[0m")
        for idx in range(len(saved_trajectories) - 2, -1, -1):
            self.control_hands(close=False)
            # time.sleep(2.0)
            rev_traj = self.reverse_trajectory(saved_trajectories[idx])
            self.execute_by_streaming(rev_traj)

        self.get_logger().info("--- FINISH! ---")


def main():
    rclpy.init()
    node = MoveDualArmHybrid()

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