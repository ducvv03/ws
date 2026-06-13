#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveDualArmIK(Node):

    def __init__(self):
        super().__init__("move_dual_arm_ik")

        # 1. Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")  # Client mới dùng để kiểm tra khoảng cách

        # 2. Variables
        self.left_joints = None
        self.right_joints = None
        self.left_targets = []
        self.right_targets = []
        self.current_index = 0

        self.saved_trajectories = {}

        # Biến phục vụ việc lập kế hoạch lại (Replanning)
        self.is_hold = False
        self.plan_attempts = 0
        self.MAX_ATTEMPTS = 5
        self.current_checking_traj = None
        self.fk_checks_remaining = 0
        self.fk_distance_valid = True

        # 3. Hand Publishers
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)

        # NEW: Tạo publisher để phát bản tin gộp JointState của 2 tay
        self.combined_joints_pub = self.create_publisher(JointState, "/target_combined_joints", 10)

    # ============================================================
    # HAND CONTROL
    # ============================================================
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

    # ============================================================
    # HELPER FUNCTIONS
    # ============================================================
    def floats_to_joint_state(self, floats, arm_prefix):
        if len(floats) != 7:
            self.get_logger().error(f"Input list must have 7 joints! Got {len(floats)}")
            sys.exit(1)
        js = JointState()
        js.name = [f"openarm_{arm_prefix}_joint{i + 1}" for i in range(7)]
        js.position = floats
        return js

    def parse_target(self, target):
        """Kiểm tra xem target có đính kèm tag 'HOLD' không"""
        if isinstance(target, tuple) and len(target) == 2 and target[1] == "HOLD":
            return target[0], True
        return target, False

    def reverse_robot_trajectory(self, trajectory):
        """Toán học lật ngược trục thời gian và tọa độ để lùi"""
        rev_traj = RobotTrajectory()
        rev_traj.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if not points: return rev_traj

        total_time = points[-1].time_from_start.sec + (points[-1].time_from_start.nanosec * 1e-9)
        for pt in reversed(points):
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            new_pt.velocities = [-v for v in pt.velocities] if pt.velocities else []
            new_pt.accelerations = [-a for a in pt.accelerations] if pt.accelerations else []

            t = pt.time_from_start.sec + (pt.time_from_start.nanosec * 1e-9)
            new_t = max(0.0, total_time - t)

            new_pt.time_from_start.sec = int(new_t)
            new_pt.time_from_start.nanosec = int((new_t % 1.0) * 1e9)
            rev_traj.joint_trajectory.points.append(new_pt)
        return rev_traj

    # ============================================================
    # START PROCESS
    # ============================================================
    def start_process(self):
        def create_pose(x, y, z, qx, qy, qz, qw):
            p = Pose()
            p.position.x = x;
            p.position.y = y;
            p.position.z = z
            p.orientation.x = qx;
            p.orientation.y = qy
            p.orientation.z = qz;
            p.orientation.w = qw
            return p

        # =========================================================
        # TARGET LIST (CÓ THỂ GẮN TAG "HOLD" VÀO DẠNG TUPLE)
        # =========================================================
        self.left_targets = [
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0),

            # Gắn tag HOLD vào điểm này
            [-0.244, -0.036, 0.297, 1.420, 0.081, 0.288, 0.080],
            (create_pose(0.41, 0.09, 0.56, 0.537, 0.547, 0.455, 0.451), "HOLD"),
            [-0.244, -0.036, 0.297, 1.420, 0.081, 0.288, 0.080],
            # "REVERSE_PREV",
            (create_pose(0.25, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0), "HOLD"),
            create_pose(0.25, 0.25, 0.50, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        # self.left_targets = [
        #     [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     [-0.3, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     [-0.3, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0],
        #     [-0.3, -0.3, -0.3, -0.3, 0.0, 0.0, 0.0],
        #     [-0.3, -0.3, -0.3, -0.3, -0.3, 0.0, 0.0],
        #     [-0.3, -0.3, -0.3, -0.3, -0.3, -0.3, 0.0],
        #     [-0.3, -0.3, -0.3, -0.3, -0.3, -0.3, -0.3],
        #
        # ]

        self.right_targets = [
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0),

            [0.244, 0.036, -0.297, 1.420, 0.081, -0.288, -0.080],
            (create_pose(0.414, 0.045, 0.67, 0.193, 0.716, 0.56, 0.369), "HOLD"),
            [0.244, 0.036, -0.297, 1.420, 0.081, -0.288, -0.080],
            # "REVERSE_PREV",
            (create_pose(0.25, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0), "HOLD"),
            create_pose(0.25, -0.05, 0.50, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]




        self.current_index = 0
        self.process_current_target()

    # ============================================================
    # BƯỚC 1: XỬ LÝ TARGET & TÍNH TOÁN ĐỘNG HỌC (IK)
    # ============================================================
    def process_current_target(self):
        if self.current_index >= len(self.left_targets):
            self.get_logger().info("All target points completed.")
            sys.exit(0)

        raw_left = self.left_targets[self.current_index]
        raw_right = self.right_targets[self.current_index]

        target_left, hold_l = self.parse_target(raw_left)
        target_right, hold_r = self.parse_target(raw_right)

        # Nếu 1 trong 2 tay đánh dấu HOLD, bật chế độ kiểm duyệt khắt khe
        self.is_hold = hold_l or hold_r
        self.target_right_cache = target_right

        self.get_logger().info(
            f"========== Processing target {self.current_index + 1} (HOLD: {self.is_hold}) ==========")

        if target_left == "REVERSE_PREV":
            self.execute_reverse_trajectory()
            return

        # Tính IK cho tay trái
        if isinstance(target_left, list) or isinstance(target_left, tuple):
            self.left_joints = self.floats_to_joint_state(target_left, "left")
            self.process_right_target()
        elif isinstance(target_left, Pose):
            self.compute_ik("left_arm", "openarm_left_tcp", target_left, self.on_left_ik_done)

    def compute_ik(self, group_name, ik_link_name, target_pose, callback_func):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        self.ik_client.call_async(req).add_done_callback(callback_func)

    def on_left_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Left arm IK failed.")
            sys.exit(1)
        self.left_joints = res.solution.joint_state
        self.process_right_target()

    def process_right_target(self):
        target_right = self.target_right_cache
        if isinstance(target_right, list) or isinstance(target_right, tuple):
            self.right_joints = self.floats_to_joint_state(target_right, "right")
            self.trigger_planning_loop()
        elif isinstance(target_right, Pose):
            self.compute_ik("right_arm", "openarm_right_tcp", target_right, self.on_right_ik_done)

    def on_right_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Right arm IK failed.")
            sys.exit(1)
        self.right_joints = res.solution.joint_state
        self.trigger_planning_loop()

    # ============================================================
    # BƯỚC 2: LẬP KẾ HOẠCH BẰNG VÒNG LẶP (PLANNING LOOP)
    # ============================================================
    def trigger_planning_loop(self):
        """Bắt đầu vòng lặp lập kế hoạch (tối đa 5 lần)"""
        self.plan_attempts = 0
        self.plan_current_step()

    def plan_current_step(self):
        self.plan_attempts += 1
        if self.plan_attempts > self.MAX_ATTEMPTS:
            self.get_logger().error("Failed to find valid trajectory after 5 attempts!")
            sys.exit(1)

        self.get_logger().info(f"--- Planning Attempt {self.plan_attempts}/{self.MAX_ATTEMPTS} ---")

        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = True

        combined_constraint = Constraints()

        def add_joints(joint_state, prefix):
            for name, pos in zip(joint_state.name, joint_state.position):
                if prefix in name and "finger" not in name:
                    jc = JointConstraint(joint_name=name, position=pos, tolerance_above=0.001, tolerance_below=0.001,
                                         weight=1.0)
                    combined_constraint.joint_constraints.append(jc)

        add_joints(self.left_joints, "openarm_left_joint")
        add_joints(self.right_joints, "openarm_right_joint")
        goal.request.goal_constraints.append(combined_constraint)

        self.move_client.send_goal_async(goal).add_done_callback(self.plan_goal_response_callback)

    def plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Plan rejected.")
            self.plan_current_step()
            return
        goal_handle.get_result_async().add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn("Planning algorithm failed. Retrying...")
            self.plan_current_step()
            return

        traj = result.planned_trajectory

        # Nếu có tag HOLD, đưa vào phòng kiểm duyệt khoảng cách
        if self.is_hold:
            self.validate_trajectory_distance(traj)
        else:
            # Không có HOLD, xuất thẳng lệnh cho chạy
            self.execute_trajectory(traj)

    # ============================================================
    # BƯỚC 3: KIỂM DUYỆT KHOẢNG CÁCH (FORWARD KINEMATICS VALIDATION)
    # ============================================================
    def validate_trajectory_distance(self, traj):
        points = traj.joint_trajectory.points
        if len(points) < 2:
            self.execute_trajectory(traj)
            return

        # Rút trích 5 điểm đại diện trên toàn bộ quỹ đạo (0%, 25%, 50%, 75%, 100%)
        indices = [0, len(points) // 4, len(points) // 2, 3 * len(points) // 4, len(points) - 1]

        self.fk_checks_remaining = len(indices)
        self.fk_distance_valid = True
        self.current_checking_traj = traj

        for idx in indices:
            pt = points[idx]
            req = GetPositionFK.Request()
            req.header.frame_id = "openarm_body_link0"
            req.fk_link_names = ["openarm_left_tcp", "openarm_right_tcp"]

            rs = RobotState()
            rs.joint_state.name = traj.joint_trajectory.joint_names
            rs.joint_state.position = pt.positions
            req.robot_state = rs

            self.fk_client.call_async(req).add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        res = future.result()
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            p1 = res.pose_stamped[0].pose.position
            p2 = res.pose_stamped[1].pose.position
            # Tính khoảng cách Euclidean 3D
            dist = math.dist((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))

            if dist > 0.25:  # Nếu > 25cm, đánh dấu là Vi Phạm
                self.fk_distance_valid = False

        self.fk_checks_remaining -= 1

        # Khi tất cả 5 điểm đã được check xong
        if self.fk_checks_remaining == 0:
            if self.fk_distance_valid:
                self.get_logger().info("✅ Trajectory Validated! (Distance <= 25cm). Executing...")
                self.execute_trajectory(self.current_checking_traj)
            else:
                self.get_logger().warn("❌ Trajectory violated HOLD distance (>25cm). Throwing away and Replanning...")
                self.plan_current_step()  # Bỏ quỹ đạo này, tìm quỹ đạo khác

    # ============================================================
    # BƯỚC 4: THỰC THI QUỸ ĐẠO BẰNG EXECUTE_TRAJECTORY ACTION
    # ============================================================
    def execute_reverse_trajectory(self):
        self.get_logger().info(">>> Keyword 'REVERSE_PREV' detected! Executing backwards...")
        prev_traj = self.saved_trajectories.get(self.current_index - 1)
        if not prev_traj:
            self.get_logger().error("No previous trajectory saved. Cannot reverse!")
            sys.exit(1)

        rev_traj = self.reverse_robot_trajectory(prev_traj)
        self.execute_trajectory(rev_traj, is_reverse=True)

    def execute_trajectory(self, trajectory, is_reverse=False):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        # Nếu không phải là quỹ đạo đi lùi, lưu nó lại vào từ điển để dùng cho sau này
        if not is_reverse:
            self.saved_trajectories[self.current_index] = trajectory

        # ============================================================
        # NEW: Publish mảng góc khớp cuối cùng lên topic /target_combined_joints
        # ============================================================
        if trajectory.joint_trajectory.points:
            combined_js_msg = JointState()
            combined_js_msg.header.stamp = self.get_clock().now().to_msg()
            combined_js_msg.header.frame_id = "openarm_body_link0"

            # Gán tên của 14 khớp
            combined_js_msg.name = trajectory.joint_trajectory.joint_names
            # Lấy mảng góc quay của điểm Đích (điểm cuối cùng trong quỹ đạo)
            combined_js_msg.position = trajectory.joint_trajectory.points[-1].positions

            self.combined_joints_pub.publish(combined_js_msg)
            self.get_logger().info("Đã publish tọa độ góc khớp ĐÍCH ĐẾN lên topic /target_combined_joints")
        # ============================================================

        self.exec_client.send_goal_async(goal).add_done_callback(self.exec_goal_response_callback)

    def exec_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution rejected.")
            sys.exit(1)
        goal_handle.get_result_async().add_done_callback(self.exec_result_callback)

    def exec_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.handle_post_step()
        else:
            self.get_logger().error(f"Execution failed: {result.error_code.val}")
            sys.exit(1)

    # ============================================================
    # POST-STEP LOGIC
    # ============================================================
    def handle_post_step(self):
        if self.current_index == 0:
            self.control_hands(close=True)
        if self.current_index == 7:
            self.control_hands(close=False)

        self.current_index += 1
        self.process_current_target()


def main():
    rclpy.init()
    node = MoveDualArmIK()

    node.get_logger().info("Waiting for MoveIt Actions (/move_action and /execute_trajectory)...")
    node.move_client.wait_for_server()
    node.exec_client.wait_for_server()

    node.get_logger().info("Waiting for Kinematics Services (/compute_ik and /compute_fk)...")
    node.ik_client.wait_for_service()
    node.fk_client.wait_for_service()

    def start():
        timer.cancel()
        node.start_process()

    timer = node.create_timer(0.5, start)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()