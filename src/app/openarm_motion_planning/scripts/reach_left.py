#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive


class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")
        self.client = ActionClient(self, MoveGroup, "/move_action")

    # =========================================================================
    # HÀM ẨN: Tự động chuyển đổi Điểm (Pose) thành Constraint cho MoveIt hiểu.
    # =========================================================================
    def go_to_pose(self, target_pose, group_name="left_arm", end_effector="openarm_left_tcp"):
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5

        constraint = Constraints()

        # 1. Ràng buộc về vị trí
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "openarm_body_link0"
        pos_constraint.link_name = end_effector
        pos_constraint.weight = 1.0

        # Tạo sai số cực nhỏ (0.1 mm) để ép robot đi đến ĐÚNG ĐIỂM
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.0001])
        bv = BoundingVolume(primitives=[sphere], primitive_poses=[target_pose])
        pos_constraint.constraint_region = bv

        # 2. Ràng buộc về góc xoay
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "openarm_body_link0"
        ori_constraint.link_name = end_effector
        ori_constraint.orientation = target_pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.001
        ori_constraint.absolute_y_axis_tolerance = 0.001
        ori_constraint.absolute_z_axis_tolerance = 0.001
        ori_constraint.weight = 1.0

        # Đóng gói constraint
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraint)

        self.get_logger().info(f"Đang ra lệnh cho {end_effector} đi đến điểm chính xác...")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # =========================================================================
    # PHẦN CODE CHÍNH CỦA BẠN: CHỈ CẦN NHẬP ĐIỂM VÀ ĐI TỚI ĐÓ
    # =========================================================================
    def send_goal(self):
        target = Pose()
        # 0.39299;
        # 0.13797;
        # 0.59005
        #
        # 0.63647;
        # -0.10274;
        # 0.7644;
        # -0.0057326
        # Nhập tọa độ XYZ
        target.position.x = 0.366
        target.position.y = 0.20
        target.position.z = 0.60

        # target.orientation.w = 0.0
        # target.orientation.x = 1.0
        # target.orientation.y = 0.0
        # target.orientation.z = 0.0

        target.orientation.w = 0.0
        target.orientation.x = 0.92
        target.orientation.y = -0.38
        target.orientation.z = 0.0

        # Gọi hàm đi tới điểm chính xác
        # self.go_to_pose(target, group_name="right_arm", end_effector="openarm_right_tcp")
        self.go_to_pose(target, group_name="left_arm", end_effector="openarm_left_tcp")

    # Các hàm Callback xử lý kết quả
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt từ chối mục tiêu! Hãy kiểm tra log của terminal chạy MoveIt.")
            sys.exit(1)

        self.get_logger().info("Đã nhận điểm. Đang lên kế hoạch...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("THÀNH CÔNG! Tay máy đã đạt đến điểm chính xác.")
        else:
            self.get_logger().error(f"THẤT BẠI! Mã lỗi MoveIt: {error_code}")
            if error_code == -1:
                self.get_logger().info(
                    "Gợi ý: Mã -1 (Planning Failed) -> Có thể điểm này quá gần một vật cản hoặc bị giới hạn bởi joint_limits.")
            elif error_code == -4:
                self.get_logger().info(
                    "Gợi ý: Mã -4 (Kinematics Failed) -> Tọa độ nằm ngoài tầm với của tay robot, hoặc góc xoay không khả thi.")

        sys.exit(0)


def main():
    rclpy.init()
    node = MoveArm()

    node.get_logger().info("Đang chờ MoveIt Action Server...")
    node.client.wait_for_server()
    node.get_logger().info("Đã kết nối!")

    # Chạy lệnh 1 lần duy nhất sau 0.5s
    def start_moving():
        timer.cancel()
        node.send_goal()

    timer = node.create_timer(0.5, start_moving)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()