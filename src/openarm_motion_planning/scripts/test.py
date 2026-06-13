#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState  # <--- Bổ sung import này
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveDualArmIK(Node):

    def __init__(self):
        super().__init__("move_dual_arm_ik")

        # MoveIt action client
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        # IK service client
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.left_joints = None
        self.right_joints = None

        self.left_targets = []
        self.right_targets = []
        self.current_index = 0

        # ============================================================
        # HAND PUBLISHERS
        # ============================================================
        self.right_hand_pub = self.create_publisher(
            JointTrajectory,
            "/right_hand_controller/joint_trajectory",
            10
        )

        self.left_hand_pub = self.create_publisher(
            JointTrajectory,
            "/left_hand_controller/joint_trajectory",
            10
        )

    # ============================================================
    # HAND CONTROL
    # ============================================================
    def control_hands(self, close=True):

        msg_right = JointTrajectory()
        msg_left = JointTrajectory()

        msg_right.joint_names = [
            "right_thumb_proximal_joint",
            "right_thumb_metacarpal_joint",
            "right_index_proximal_joint",
            "right_middle_proximal_joint",
            "right_ring_proximal_joint",
            "right_pinky_proximal_joint"
        ]

        msg_left.joint_names = [
            "left_thumb_proximal_joint",
            "left_thumb_metacarpal_joint",
            "left_index_proximal_joint",
            "left_middle_proximal_joint",
            "left_ring_proximal_joint",
            "left_pinky_proximal_joint"
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
    # HELPER: CHUYỂN LIST THÀNH JOINT STATE
    # ============================================================
    def floats_to_joint_state(self, floats, arm_prefix):
        """Tạo đối tượng JointState giả lập từ mảng số để đồng nhất dữ liệu"""
        if len(floats) != 7:
            self.get_logger().error(f"Input list must have exactly 7 joint values! Got {len(floats)}")
            sys.exit(1)

        js = JointState()
        js.name = [f"openarm_{arm_prefix}_joint{i + 1}" for i in range(7)]
        js.position = floats
        return js

    # ============================================================
    # IK SERVICE
    # ============================================================
    def compute_ik(self, group_name, ik_link_name, target_pose, callback_func):

        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1

        self.get_logger().info(f"Solving IK for {group_name}...")
        future = self.ik_client.call_async(req)
        future.add_done_callback(callback_func)

    # ============================================================
    # START PROCESS
    # ============================================================
    def start_process(self):

        def create_pose(x, y, z, qx, qy, qz, qw):
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            return p

        # =========================================================
        # BẠN CÓ THỂ ĐIỀN POSE HOẶC MẢNG LIST VÀO ĐÂY TÙY Ý
        # =========================================================

        # self.left_targets = [
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.30, 0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.41, 0.09, 0.56, 0.537, 0.547, 0.455, 0.451),
        #     create_pose(0.30, 0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.25, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.25, 0.25, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(-3.4694e-18, 0.1535, 0.222, 1.0, 0.0, 0.0, 0.0),
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # ]
        # #
        # self.right_targets = [
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.30, -0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.414, 0.045, 0.67, 0.193, 0.716, 0.56, 0.369),
        #     create_pose(0.30, -0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.25, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(0.25, -0.05, 0.50, 0.71, 0.0, 0.71, 0.0),
        #     create_pose(-3.4694e-18, -0.1535, 0.222, 1.0, 0.0, 0.0, 0.0),
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # ]

        self.left_targets = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
            [-0.24444147062471414,
             -0.03618045296908994,
             0.2970632622193026,
             1.4201010067876565,
             0.08194388462026085,
             0.28857222078463923,
             0.08098920862892871],
            create_pose(0.41, 0.09, 0.56, 0.537, 0.547, 0.455, 0.451),
            [-0.24444147062471414,
             -0.03618045296908994,
             0.2970632622193026,
             1.4201010067876565,
             0.08194388462026085,
             0.28857222078463923,
             0.08098920862892871],
            create_pose(0.25, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, 0.25, 0.50, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        #
        self.right_targets = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0),
            [0.24444147062471414,
             0.03618045296908994,
             -0.2970632622193026,
             1.4201010067876565,
             0.08194388462026085,
             -0.28857222078463923,
             -0.08098920862892871],
            create_pose(0.414, 0.045, 0.67, 0.193, 0.716, 0.56, 0.369),
            [0.24444147062471414,
             0.03618045296908994,
             -0.2970632622193026,
             1.4201010067876565,
             0.08194388462026085,
             -0.28857222078463923,
             -0.08098920862892871],
            create_pose(0.25, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.25, -0.05, 0.50, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        self.current_index = 0
        self.process_current_target()

    # ============================================================
    # QUY TRÌNH XỬ LÝ (TÁCH BIỆT LOGIC THEO LOẠI DỮ LIỆU)
    # ============================================================
    def process_current_target(self):

        if self.current_index >= len(self.left_targets):
            self.get_logger().info("All target points completed.")
            sys.exit(0)

        target_left = self.left_targets[self.current_index]
        self.target_right = self.right_targets[self.current_index]  # Lưu lại để chạy sau

        self.get_logger().info(
            "==============================\n"
            f"Processing target {self.current_index + 1}\n"
            "=============================="
        )

        # 1. XỬ LÝ TAY TRÁI TRƯỚC
        if isinstance(target_left, list) or isinstance(target_left, tuple):
            self.get_logger().info("Left target is Joint list. Skipping IK.")
            self.left_joints = self.floats_to_joint_state(target_left, "left")
            self.process_right_target(self.target_right)  # Chuyển sang tay phải luôn
        elif isinstance(target_left, Pose):
            self.compute_ik("left_arm", "openarm_left_tcp", target_left, self.on_left_ik_done)
        else:
            self.get_logger().error("Left target format unknown!")
            sys.exit(1)

    def on_left_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Left arm IK failed.")
            sys.exit(1)

        self.left_joints = res.solution.joint_state
        self.process_right_target(self.target_right)

    # 2. XỬ LÝ TAY PHẢI SAU ĐÓ
    def process_right_target(self, target_right):
        if isinstance(target_right, list) or isinstance(target_right, tuple):
            self.get_logger().info("Right target is Joint list. Skipping IK.")
            self.right_joints = self.floats_to_joint_state(target_right, "right")
            self.execute_dual_arm_joints()  # Cả 2 tay đã có Joints -> Chạy
        elif isinstance(target_right, Pose):
            self.compute_ik("right_arm", "openarm_right_tcp", target_right, self.on_right_ik_done)
        else:
            self.get_logger().error("Right target format unknown!")
            sys.exit(1)

    def on_right_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Right arm IK failed.")
            sys.exit(1)

        self.right_joints = res.solution.joint_state
        self.execute_dual_arm_joints()

    # ============================================================
    # EXECUTE MOTION
    # ============================================================
    def execute_dual_arm_joints(self):

        self.get_logger().info("Planning motion for both arms...")

        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5

        combined_constraint = Constraints()

        def add_joints_to_constraint(joint_state, arm_prefix):
            for name, position in zip(joint_state.name, joint_state.position):
                # Đảm bảo chỉ bắt góc tay, không bắt góc ngón tay (finger)
                if arm_prefix in name and "finger" not in name:
                    jc = JointConstraint()
                    jc.joint_name = name
                    jc.position = position
                    jc.tolerance_above = 0.001
                    jc.tolerance_below = 0.001
                    jc.weight = 1.0
                    combined_constraint.joint_constraints.append(jc)

        add_joints_to_constraint(self.left_joints, "openarm_left_joint")
        add_joints_to_constraint(self.right_joints, "openarm_right_joint")

        goal.request.goal_constraints.append(combined_constraint)

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # ============================================================
    # MOVE RESULT
    # ============================================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the motion plan.")
            sys.exit(1)

        self.get_logger().info("Executing motion...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Target reached successfully.")

            # ===== HAND LOGIC =====
            if self.current_index == 0:
                self.control_hands(close=True)

            if self.current_index == 7:
                self.control_hands(close=False)

            self.current_index += 1
            self.process_current_target()

        else:
            self.get_logger().error(f"Motion failed: {error_code}")
            sys.exit(1)


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()
    node = MoveDualArmIK()

    node.get_logger().info("Waiting for MoveIt...")
    node.move_client.wait_for_server()

    node.get_logger().info("Waiting for IK service...")
    node.ik_client.wait_for_service()

    node.get_logger().info("Ready.")

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