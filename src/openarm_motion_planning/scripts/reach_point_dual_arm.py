#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK


class MoveDualArmIK(Node):
    def __init__(self):
        super().__init__("move_dual_arm_ik")
        # Client gửi lệnh chạy robot
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        # Client gọi giải động học ngược (Inverse Kinematics)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.left_joints = None
        self.right_joints = None

        self.target_left = Pose()
        self.target_right = Pose()

    # =========================================================================
    # BƯỚC 1 & 2: GỌI SERVICE GIẢI ĐỘNG HỌC NGƯỢC (IK) CHO TỪNG TAY
    # =========================================================================
    def compute_ik(self, group_name, ik_link_name, target_pose, callback_func):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True  # Ưu tiên giải nghiệm không va chạm
        req.ik_request.timeout.sec = 1

        self.get_logger().info(f"Đang giải mã tọa độ thành góc khớp cho {group_name}...")
        future = self.ik_client.call_async(req)
        future.add_done_callback(callback_func)

    # =========================================================================
    # QUY TRÌNH CHẠY CHÍNH ĐƯỢC CHIA THÀNH CÁC BƯỚC NỐI TIẾP NHAU
    # =========================================================================
    def start_process(self):
        # KHAI BÁO TỌA ĐỘ
        # 1. Tay trái
        # self.target_left.position.x = 0.37724
        # self.target_left.position.y = 0.15115
        # self.target_left.position.z = 0.571
        # self.target_left.orientation.w = -0.016981
        # self.target_left.orientation.x = 0.6646
        # self.target_left.orientation.y = -0.14217
        # self.target_left.orientation.z = 0.73335
        #
        # # 2. Tay phải
        # self.target_right.position.x = 0.37724
        # self.target_right.position.y = -0.15115
        # self.target_right.position.z = 0.571
        # self.target_right.orientation.w = -0.016981
        # self.target_right.orientation.x = 0.6646
        # self.target_right.orientation.y = 0.14217
        # self.target_right.orientation.z = 0.73335

        self.target_left.position.x = 0.27724
        self.target_left.position.y = 0.1115
        self.target_left.position.z = 0.4
        # self.target_left.orientation.w = 0.0
        # self.target_left.orientation.x = 0.71
        # self.target_left.orientation.y = 0.0
        # self.target_left.orientation.z = 0.71

        self.target_left.orientation.w = 0.0
        self.target_left.orientation.x = 1.0
        self.target_left.orientation.y = 0.0
        self.target_left.orientation.z = 0.0

        # 2. Tay phải
        self.target_right.position.x = 0.27724
        self.target_right.position.y = -0.25115
        self.target_right.position.z = 0.4
        # self.target_right.orientation.w = 0.0
        # self.target_right.orientation.x = 0.71
        # self.target_right.orientation.y = 0.0
        # self.target_right.orientation.z = 0.71

        self.target_right.orientation.w = 0.0
        self.target_right.orientation.x = 1.0
        self.target_right.orientation.y = 0.0
        self.target_right.orientation.z = 0.0


        # BẮT ĐẦU: Gọi IK cho tay trái trước
        self.compute_ik("left_arm", "openarm_left_tcp", self.target_left, self.on_left_ik_done)

    def on_left_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Lỗi: Không thể tìm được nghiệm góc khớp (IK) cho TAY TRÁI!")
            sys.exit(1)

        self.left_joints = res.solution.joint_state
        # Có góc trái rồi, tiếp tục gọi IK cho tay phải
        self.compute_ik("right_arm", "openarm_right_tcp", self.target_right, self.on_right_ik_done)

    def on_right_ik_done(self, future):
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Lỗi: Không thể tìm được nghiệm góc khớp (IK) cho TAY PHẢI!")
            sys.exit(1)

        self.right_joints = res.solution.joint_state
        # Cả 2 tay đã có góc khớp -> Chuyển sang bước kết hợp và chạy
        self.execute_dual_arm_joints()

    # =========================================================================
    # BƯỚC 3: GỘP 14 GÓC KHỚP VÀ GỬI CHO NHÓM BOTH_ARMS ĐỂ CHẠY CÙNG LÚC
    # =========================================================================
    def execute_dual_arm_joints(self):
        self.get_logger().info("Đã có nghiệm cho cả 2 tay. Bắt đầu tính toán quỹ đạo đồng thời...")
        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5

        combined_constraint = Constraints()

        # Hàm tiện ích để chuyển đổi JointState thành JointConstraint
        def add_joints_to_constraint(joint_state, arm_prefix):
            for name, position in zip(joint_state.name, joint_state.position):
                if arm_prefix in name and "finger" not in name:  # Lọc ra các khớp tay (bỏ qua kẹp)
                    jc = JointConstraint()
                    jc.joint_name = name
                    jc.position = position
                    jc.tolerance_above = 0.001  # Sai số góc cực nhỏ
                    jc.tolerance_below = 0.001
                    jc.weight = 1.0
                    combined_constraint.joint_constraints.append(jc)

        # Trích xuất 7 khớp tay trái và 7 khớp tay phải
        add_joints_to_constraint(self.left_joints, "openarm_left_joint")
        add_joints_to_constraint(self.right_joints, "openarm_right_joint")

        goal.request.goal_constraints.append(combined_constraint)

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # =========================================================================
    # XỬ LÝ KẾT QUẢ CỦA LỆNH MOVE
    # =========================================================================
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt từ chối quỹ đạo both_arms! Có thể 2 tay va chạm nhau.")
            sys.exit(1)

        self.get_logger().info("Đã lên kế hoạch thành công! Bắt đầu di chuyển 2 tay cùng lúc...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("THÀNH CÔNG RỰC RỠ! Cả 2 tay máy đã đạt đến điểm chính xác cùng lúc.")
        else:
            self.get_logger().error(f"THẤT BẠI khi di chuyển! Mã lỗi: {error_code}")

        sys.exit(0)


def main():
    rclpy.init()
    node = MoveDualArmIK()

    node.get_logger().info("Đang chờ MoveIt Action Server (/move_action)...")
    node.move_client.wait_for_server()

    node.get_logger().info("Đang chờ MoveIt IK Service (/compute_ik)...")
    node.ik_client.wait_for_service()

    node.get_logger().info("Đã kết nối đủ các thành phần!")

    # Bắt đầu tự động chạy sau 0.5s
    def start_moving():
        timer.cancel()
        node.start_process()

    timer = node.create_timer(0.5, start_moving)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()