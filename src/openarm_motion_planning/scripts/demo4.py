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
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveDualArmIK(Node):

    def __init__(self):
        super().__init__("move_dual_arm_ik")

        self.cb_group = ReentrantCallbackGroup()

        # 1. Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk", callback_group=self.cb_group)

        # 2. State & Publishers
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)

        # CÁC TOPIC ARM NÀY COPY Y HỆT TỪ CODE B ĐỂ STREAMING
        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory",
                                                  10)
        self.right_arm_pub = self.create_publisher(JointTrajectory,
                                                   "/right_joint_trajectory_controller/joint_trajectory", 10)

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
        # ... logic kẹp tay giữ nguyên ...
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
            point.positions = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
        else:
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

    def append_trajectory(self, master_traj, new_traj):
        if not master_traj.joint_trajectory.points:
            master_traj.joint_trajectory.joint_names = new_traj.joint_trajectory.joint_names
            master_traj.joint_trajectory.points = new_traj.joint_trajectory.points
            return

        for pt in new_traj.joint_trajectory.points[1:]:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = pt.positions
            master_traj.joint_trajectory.points.append(new_pt)

    # ĐÂY LÀ HÀM QUAN TRỌNG NHẤT: THAY THẾ ACTION SERVER BẰNG STREAMING
    def execute_by_streaming(self, master_traj):
        names = master_traj.joint_trajectory.joint_names
        idx_l = [i for i, n in enumerate(names) if "left" in n]
        idx_r = [i for i, n in enumerate(names) if "right" in n]
        names_l = [names[i] for i in idx_l]
        names_r = [names[i] for i in idx_r]

        # Lấy tất cả các tọa độ MoveIt sinh ra
        all_pts = [pt.positions for pt in master_traj.joint_trajectory.points]

        # CHIA LẠI QUỸ ĐẠO THÀNH CÁC ĐIỂM CÁCH ĐỀU NHAU (Gỡ bỏ gia tốc/giảm tốc của MoveIt)
        stream_pts = []
        last_p = all_pts[0]
        stream_pts.append(last_p)

        STEP_RAD = 0.02  # Góc thay đổi lớn nhất trong 1 lần tick. Thay đổi số này để tăng/giảm tốc độ tổng.

        for p in all_pts[1:]:
            while True:
                diff = [a - b for a, b in zip(p, last_p)]
                max_diff = max(abs(d) for d in diff)
                if max_diff < STEP_RAD:
                    break  # Điểm MoveIt sinh ra chưa đủ xa, bỏ qua để làm mượt

                # Nội suy ra 1 điểm cách đúng STEP_RAD
                ratio = STEP_RAD / max_diff
                interp_p = [last_p[i] + diff[i] * ratio for i in range(len(p))]
                stream_pts.append(interp_p)
                last_p = interp_p

        stream_pts.append(all_pts[-1])

        self.get_logger().info(f"Streaming {len(stream_pts)} points like Teleop...")

        # Bắn liên tục xuống ROS y hệt như Code B
        for p in stream_pts:
            if not rclpy.ok():
                break

            # --- LEFT ---
            msg_l = JointTrajectory()
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint()
            pt_l.positions = [p[i] for i in idx_l]
            pt_l.time_from_start.sec = 0  # BẮT BUỘC = 0 giống Code B
            pt_l.time_from_start.nanosec = 0  # BẮT BUỘC = 0 giống Code B
            msg_l.points.append(pt_l)

            # --- RIGHT ---
            msg_r = JointTrajectory()
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint()
            pt_r.positions = [p[i] for i in idx_r]
            pt_r.time_from_start.sec = 0
            pt_r.time_from_start.nanosec = 0
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)

            # Sleep 0.02s => tương đương 50Hz (y hệt teleop frequency)
            time.sleep(0.02)

    def planning_thread(self):
        self.get_logger().info("Waiting for MoveIt Actions/Services...")
        self.move_client.wait_for_server()
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
                if segment_traj and segment_traj.joint_trajectory.points:
                    valid_segment_traj = segment_traj
                    break

            if not valid_segment_traj:
                self.get_logger().error(f"Failed to plan valid segment {i} after 5 attempts.")
                return

            curr_l = end_l
            curr_r = end_r

            self.append_trajectory(master_trajectory, valid_segment_traj)

        self.get_logger().info("--- PLANNING SUCCESS! Executing via Teleop Streaming... ---")

        self.control_hands(close=True)
        time.sleep(1.0)

        # GỌI HÀM STREAMING Y HỆT TELEOP THAY VÌ GỌI ACTION SERVER
        self.execute_by_streaming(master_trajectory)

        self.get_logger().info("Done executing massive trajectory.")
        self.control_hands(close=False)


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

# #!/usr/bin/env python3

#
# import sys
# import math
# import copy
# import threading
# import time
#
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
#
# import numpy as np
# from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState
# from moveit_msgs.action import MoveGroup, ExecuteTrajectory
# from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory, RobotState
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#
#
# class MoveDualArmIK(Node):
#
#     def __init__(self):
#         super().__init__("move_dual_arm_ik")
#
#         self.cb_group = ReentrantCallbackGroup()
#
#         # 1. Clients
#         self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
#         self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.cb_group)
#         self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
#         self.fk_client = self.create_client(GetPositionFK, "/compute_fk", callback_group=self.cb_group)
#
#         # 2. State & Publishers
#         self.current_joint_state = None
#         self.js_sub = self.create_subscription(
#             JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)
#
#         self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
#         self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)
#
#         # 3. Targets
#         self.left_targets, self.right_targets = self.define_targets()
#
#     def define_targets(self):
#         def create_pose(x, y, z, qx, qy, qz, qw):
#             p = Pose()
#             p.position.x = x;
#             p.position.y = y;
#             p.position.z = z
#             p.orientation.x = qx;
#             p.orientation.y = qy;
#             p.orientation.z = qz;
#             p.orientation.w = qw
#             return p
#
#         left = [
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#             create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.30, 0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.33, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.25, 0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.25, 0.25, 0.40, 0.71, 0.0, 0.71, 0.0),
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#         ]
#
#         right = [
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#             create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.30, -0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.33, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.25, -0.0, 0.40, 0.71, 0.0, 0.71, 0.0),
#             create_pose(0.25, -0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#         ]
#         return left, right
#
#     def joint_state_callback(self, msg):
#         self.current_joint_state = msg
#
#     def parse_target(self, target):
#         if isinstance(target, tuple) and len(target) == 2 and target[1] == "HOLD":
#             return target[0], True
#         return target, False
#
#     def control_hands(self, close=True):
#         msg_right = JointTrajectory()
#         msg_left = JointTrajectory()
#
#         msg_right.joint_names = [
#             "right_thumb_proximal_joint", "right_thumb_metacarpal_joint",
#             "right_index_proximal_joint", "right_middle_proximal_joint",
#             "right_ring_proximal_joint", "right_pinky_proximal_joint"
#         ]
#         msg_left.joint_names = [
#             "left_thumb_proximal_joint", "left_thumb_metacarpal_joint",
#             "left_index_proximal_joint", "left_middle_proximal_joint",
#             "left_ring_proximal_joint", "left_pinky_proximal_joint"
#         ]
#
#         point = JointTrajectoryPoint()
#         if close:
#             self.get_logger().info("Closing both hands...")
#             point.positions = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
#         else:
#             self.get_logger().info("Opening both hands...")
#             point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#
#         point.time_from_start.sec = 1
#         msg_right.points.append(point)
#         msg_left.points.append(point)
#
#         self.right_hand_pub.publish(msg_right)
#         self.left_hand_pub.publish(msg_left)
#
#     def _wait_for_future(self, future):
#         while rclpy.ok() and not future.done():
#             time.sleep(0.01)
#         if future.done():
#             return future.result()
#         return None
#
#     # ============================================================
#     # SYNCHRONOUS FUNCTIONS
#     # ============================================================
#     def compute_ik_sync(self, group_name, ik_link_name, target_pose, seed_state=None):
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = group_name
#         req.ik_request.ik_link_name = ik_link_name
#         req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
#         req.ik_request.pose_stamped.pose = target_pose
#         req.ik_request.avoid_collisions = True
#
#         if seed_state:
#             req.ik_request.robot_state = seed_state
#
#         future = self.ik_client.call_async(req)
#         res = self._wait_for_future(future)
#
#         if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
#             return None
#
#         prefix = "openarm_left" if "left" in group_name else "openarm_right"
#         joints = []
#         for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position):
#             if prefix in name and "finger" not in name:
#                 joints.append(pos)
#         return joints
#
#     def compute_fk_sync(self, joint_names, joint_positions):
#         req = GetPositionFK.Request()
#         req.header.frame_id = "openarm_body_link0"
#         req.fk_link_names = ["openarm_left_tcp", "openarm_right_tcp"]
#
#         rs = RobotState()
#         rs.joint_state.name = joint_names
#         rs.joint_state.position = joint_positions
#         req.robot_state = rs
#
#         future = self.fk_client.call_async(req)
#         res = self._wait_for_future(future)
#
#         if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
#             p1 = res.pose_stamped[0].pose.position
#             p2 = res.pose_stamped[1].pose.position
#             return math.dist((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))
#         return 999.0
#
#     def plan_segment_sync(self, start_joints_l, start_joints_r, end_joints_l, end_joints_r):
#         goal = MoveGroup.Goal()
#         goal.request.group_name = "both_arms"
#         goal.request.allowed_planning_time = 5.0
#         goal.planning_options.plan_only = True
#
#         rs = RobotState()
#         rs.is_diff = True
#         rs.joint_state.name = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
#                               [f"openarm_right_joint{i + 1}" for i in range(7)]
#         rs.joint_state.position = list(start_joints_l) + list(start_joints_r)
#         goal.request.start_state = rs
#
#         c = Constraints()
#         for i, pos in enumerate(end_joints_l):
#             c.joint_constraints.append(
#                 JointConstraint(joint_name=f"openarm_left_joint{i + 1}", position=pos, tolerance_above=0.001,
#                                 tolerance_below=0.001, weight=1.0))
#         for i, pos in enumerate(end_joints_r):
#             c.joint_constraints.append(
#                 JointConstraint(joint_name=f"openarm_right_joint{i + 1}", position=pos, tolerance_above=0.001,
#                                 tolerance_below=0.001, weight=1.0))
#         goal.request.goal_constraints.append(c)
#
#         future = self.move_client.send_goal_async(goal)
#         goal_handle = self._wait_for_future(future)
#
#         if goal_handle is None or not goal_handle.accepted:
#             return None
#
#         result_future = goal_handle.get_result_async()
#         result_wrapper = self._wait_for_future(result_future)
#
#         if result_wrapper is None:
#             return None
#
#         result = result_wrapper.result
#         if result.error_code.val != MoveItErrorCodes.SUCCESS:
#             return None
#
#         return result.planned_trajectory
#
#     def retime_and_smooth_trajectory(self, master_traj):
#         """
#         Lọc bỏ hiện tượng Stop-and-Go của MoveIt bằng cách tính lại thời gian
#         và xóa bỏ các ràng buộc vận tốc = 0 ở các điểm nối.
#         """
#         new_traj = RobotTrajectory()
#         new_traj.joint_trajectory.header = master_traj.joint_trajectory.header
#         new_traj.joint_trajectory.joint_names = master_traj.joint_trajectory.joint_names
#
#         current_time = 0.0
#         last_pos = None
#
#         # Tốc độ di chuyển trung bình (rad/s). Bạn có thể tăng/giảm tùy ý (Vd: 0.5 là chậm mượt, 1.0 là nhanh)
#         SPEED_RAD_PER_SEC = 1.0
#
#         for pt in master_traj.joint_trajectory.points:
#             pos = pt.positions
#
#             if last_pos is not None:
#                 # Tìm khớp phải xoay góc lớn nhất
#                 max_diff = max(abs(p - lp) for p, lp in zip(pos, last_pos))
#
#                 # Bỏ qua các điểm trùng lặp hoặc quá sát nhau ở chỗ nối segment
#                 if max_diff < 0.001:
#                     continue
#
#                 # Thời gian = Quãng đường / Vận tốc
#                 dt = max_diff / SPEED_RAD_PER_SEC
#                 current_time += dt
#             else:
#                 current_time = 1.0  # Khởi động sau 1 giây
#
#             new_pt = JointTrajectoryPoint()
#             new_pt.positions = pos
#
#             # QUAN TRỌNG NHẤT: Ép rỗng Vận tốc và Gia tốc (Giống hệt Code B)
#             # ROS 2 Spline sẽ tự động làm trơn quỹ đạo đi qua các điểm này
#             new_pt.velocities = []
#             new_pt.accelerations = []
#             new_pt.effort = []
#
#             # Tính lại giây và nano giây chuẩn
#             sec = int(current_time)
#             nanosec = int((current_time - sec) * 1e9)
#             new_pt.time_from_start.sec = sec
#             new_pt.time_from_start.nanosec = nanosec
#
#             new_traj.joint_trajectory.points.append(new_pt)
#             last_pos = pos
#
#         return new_traj
#
#     def execute_trajectory_sync(self, full_trajectory):
#         goal = ExecuteTrajectory.Goal()
#         goal.trajectory = full_trajectory
#         future = self.exec_client.send_goal_async(goal)
#         goal_handle = self._wait_for_future(future)
#
#         if goal_handle is None or not goal_handle.accepted:
#             self.get_logger().error("Master trajectory rejected!")
#             return False
#
#         result_future = goal_handle.get_result_async()
#         result_wrapper = self._wait_for_future(result_future)
#
#         if result_wrapper is None:
#             return False
#
#         res = result_wrapper.result
#         return res.error_code.val == MoveItErrorCodes.SUCCESS
#
#     def append_trajectory(self, master_traj, new_traj):
#         if not master_traj.joint_trajectory.points:
#             master_traj.joint_trajectory.joint_names = new_traj.joint_trajectory.joint_names
#             master_traj.joint_trajectory.points = new_traj.joint_trajectory.points
#             return
#
#         last_pt = master_traj.joint_trajectory.points[-1]
#         base_sec = last_pt.time_from_start.sec
#         base_nano = last_pt.time_from_start.nanosec
#
#         for i, pt in enumerate(new_traj.joint_trajectory.points):
#             if i == 0: continue
#
#             new_pt = JointTrajectoryPoint()
#             new_pt.positions = pt.positions
#             new_pt.velocities = pt.velocities
#             new_pt.accelerations = pt.accelerations
#
#             total_nano = base_nano + pt.time_from_start.nanosec
#             add_sec = total_nano // int(1e9)
#             rem_nano = total_nano % int(1e9)
#
#             new_pt.time_from_start.sec = base_sec + pt.time_from_start.sec + add_sec
#             new_pt.time_from_start.nanosec = rem_nano
#
#             master_traj.joint_trajectory.points.append(new_pt)
#
#     def planning_thread(self):
#         self.get_logger().info("Waiting for MoveIt Actions/Services...")
#         self.move_client.wait_for_server()
#         self.exec_client.wait_for_server()
#         self.ik_client.wait_for_service()
#         self.fk_client.wait_for_service()
#
#         while self.current_joint_state is None and rclpy.ok():
#             time.sleep(0.1)
#
#         self.get_logger().info("--- STARTING GLOBAL PLANNING ---")
#         master_trajectory = RobotTrajectory()
#
#         curr_l = [pos for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position) if
#                   "openarm_left_joint" in name]
#         curr_r = [pos for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position) if
#                   "openarm_right_joint" in name]
#
#         if len(curr_l) != 7 or len(curr_r) != 7:
#             curr_l = [0.0] * 7;
#             curr_r = [0.0] * 7
#
#         for i in range(len(self.left_targets)):
#             if not rclpy.ok():
#                 break
#
#             self.get_logger().info(f"Resolving & Planning segment {i} -> {i + 1}...")
#
#             t_left, hold_l = self.parse_target(self.left_targets[i])
#             t_right, hold_r = self.parse_target(self.right_targets[i])
#             is_hold = hold_l or hold_r
#
#             end_l = t_left if isinstance(t_left, list) else self.compute_ik_sync("left_arm", "openarm_left_tcp", t_left)
#             end_r = t_right if isinstance(t_right, list) else self.compute_ik_sync("right_arm", "openarm_right_tcp",
#                                                                                    t_right)
#
#             if end_l is None or end_r is None:
#                 self.get_logger().error(f"IK Failed at waypoint {i}. Aborting global plan.")
#                 return
#
#             attempts = 0
#             valid_segment_traj = None
#
#             while attempts < 5 and rclpy.ok():
#                 attempts += 1
#                 segment_traj = self.plan_segment_sync(curr_l, curr_r, end_l, end_r)
#
#                 if not segment_traj or not segment_traj.joint_trajectory.points:
#                     continue
#
#                 if is_hold:
#                     points = segment_traj.joint_trajectory.points
#                     indices_to_check = [0, len(points) // 4, len(points) // 2, 3 * len(points) // 4, len(points) - 1]
#                     valid = True
#                     for idx in indices_to_check:
#                         dist = self.compute_fk_sync(segment_traj.joint_trajectory.joint_names, points[idx].positions)
#                         if dist > 0.25:
#                             valid = False
#                             break
#                     if not valid:
#                         self.get_logger().warn(f"Segment {i} violated HOLD distance. Replanning...")
#                         continue
#
#                 valid_segment_traj = segment_traj
#                 break
#
#             if not valid_segment_traj:
#                 self.get_logger().error(f"Failed to plan valid segment {i} after 5 attempts.")
#                 return
#
#             curr_l = end_l
#             curr_r = end_r
#
#             self.append_trajectory(master_trajectory, valid_segment_traj)
#
#         self.get_logger().info("--- GLOBAL PLANNING SUCCESS! Retiming for smooth execution... ---")
#
#         # 1. Chạy hàm làm mượt để đập bỏ cái "Đi-và-Dừng" của MoveIt
#         smooth_master_trajectory = self.retime_and_smooth_trajectory(master_trajectory)
#
#         self.control_hands(close=True)
#         time.sleep(1.0)
#
#         # Nếu bạn có thêm hàm publish_right_arm_trajectory đã làm ở trước đó
#         # self.publish_right_arm_trajectory(smooth_master_trajectory)
#
#         # 2. Đưa quỹ đạo đã mượt vào chạy
#         success = self.execute_trajectory_sync(smooth_master_trajectory)
#
#         if success:
#             self.get_logger().info("Done executing massive trajectory.")
#             self.control_hands(close=False)
#         else:
#             self.get_logger().error("Failed to execute master trajectory.")
#
#         # self.get_logger().info("--- GLOBAL PLANNING SUCCESS! Executing full continuous trajectory... ---")
#         #
#         # self.control_hands(close=True)
#         # time.sleep(1.0)
#         #
#         # success = self.execute_trajectory_sync(master_trajectory)
#         #
#         # if success:
#         #     self.get_logger().info("Done executing massive trajectory.")
#         #     self.control_hands(close=False)
#         # else:
#         #     self.get_logger().error("Failed to execute master trajectory.")
#
#
# def main():
#     rclpy.init()
#     node = MoveDualArmIK()
#
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
#
#     planning_thread = threading.Thread(target=node.planning_thread, daemon=True)
#     planning_thread.start()
#
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.try_shutdown()
#
#
# if __name__ == "__main__":
#     main()