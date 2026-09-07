#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class CommandMergerNode(Node):
    def __init__(self):
        super().__init__('command_merger_node')

        # Lưu trữ tạm thời giá trị của tay và ngón tay
        self.left_arm_positions = None
        self.right_arm_positions = None
        self.left_hand_positions = None
        self.right_hand_positions = None

        # 1. Danh sách tên khớp của cánh tay (7 khớp mỗi bên)
        self.left_arm_names = [f'openarmx_left_joint{i}' for i in range(1, 8)]
        self.right_arm_names = [f'openarmx_right_joint{i}' for i in range(1, 8)]

        # 2. Danh sách tên khớp của bàn tay Dex3 (Đã bỏ ngón cái/thumb - còn 4 khớp mỗi bên)
        self.left_hand_names = [
            'left_hand_index_0_joint',
            'left_hand_middle_0_joint',
            'left_hand_index_1_joint',
            'left_hand_middle_1_joint',
            'left_hand_thumb_2_joint'
        ]

        self.right_hand_names = [
            'right_hand_index_0_joint',
            'right_hand_middle_0_joint',
            'right_hand_index_1_joint',
            'right_hand_middle_1_joint',
            'right_hand_thumb_2_joint'
        ]

        # Hệ số nhân để scale góc quay cho ngón tay.
        # Ở node OpenArmXTeleopVRNode, khi bóp hết cò, giá trị gửi đi là 0.04
        # Để ngón tay gập khoảng 90 độ (1.57 rad), ta cần nhân lên: 1.57 / 0.04 = 39.25
        self.finger_multiplier = 39.25

        # Subscribers
        self.left_sub = self.create_subscription(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            self.left_callback,
            10
        )
        self.right_sub = self.create_subscription(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            self.right_callback,
            10
        )

        # Publisher
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_command',
            10
        )

        self.get_logger().info("Đã khởi động node gộp lệnh (Chỉ ngón trỏ và ngón giữa).")

    def left_callback(self, msg: Float64MultiArray):
        # Có ít nhất 8 phần tử (7 cánh tay + 1 lệnh cò súng)
        if len(msg.data) >= 8:
            # 7 phần tử đầu là của cánh tay
            self.left_arm_positions = msg.data[:7]

            # Phần tử số 8 (index 7) là lệnh đóng mở
            trigger_cmd = msg.data[7]

            # Tính góc quay cho ngón tay và áp dụng chung cho 4 khớp ngón tay
            finger_angle = -trigger_cmd * self.finger_multiplier
            self.left_hand_positions = [finger_angle] * len(self.left_hand_names)

            self.publish_joint_state()

    def right_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.right_arm_positions = msg.data[:7]

            trigger_cmd = msg.data[7]

            finger_angle = trigger_cmd * self.finger_multiplier
            self.right_hand_positions = [finger_angle] * len(self.right_hand_names)

            self.publish_joint_state()

    def publish_joint_state(self):
        # Đợi có đủ data của cả 2 bên mới publish
        if self.left_arm_positions is None or self.right_arm_positions is None:
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Nối tất cả mảng tên khớp lại với nhau (Tổng 22 khớp)
        joint_state_msg.name = (
                self.left_arm_names +
                self.right_arm_names +
                self.left_hand_names +
                self.right_hand_names
        )

        # Nối tất cả mảng góc vị trí lại với nhau
        joint_state_msg.position = (
                list(self.left_arm_positions) +
                list(self.right_arm_positions) +
                list(self.left_hand_positions) +
                list(self.right_hand_positions)
        )

        self.joint_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()



# #!/usr/bin/env python3


#
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
#
#
# class CommandMergerNode(Node):
#     def __init__(self):
#         super().__init__('command_merger_node')
#
#         # Lưu trữ tạm thời giá trị của tay trái và phải
#         # Để an toàn, chúng ta sẽ chờ nhận được data từ cả 2 tay rồi mới publish
#         self.left_positions = None
#         self.right_positions = None
#
#         # Khởi tạo danh sách tên khớp (joints) bao gồm cả 7 khớp tay + 1 khớp ngón tay
#         self.left_joint_names = [f'openarmx_left_joint{i}' for i in range(1, 8)] + ['openarmx_left_finger_joint1']
#         self.right_joint_names = [f'openarmx_right_joint{i}' for i in range(1, 8)] + ['openarmx_right_finger_joint1']
#
#         # Subscribers
#         self.left_sub = self.create_subscription(
#             Float64MultiArray,
#             '/left_forward_position_controller/commands',
#             self.left_callback,
#             10
#         )
#         self.right_sub = self.create_subscription(
#             Float64MultiArray,
#             '/right_forward_position_controller/commands',
#             self.right_callback,
#             10
#         )
#
#         # Publisher
#         self.joint_pub = self.create_publisher(
#             JointState,
#             '/joint_command',
#             10
#         )
#
#         self.get_logger().info("Đã khởi động node gộp lệnh (Command Merger Node). Đang chờ dữ liệu...")
#
#     def left_callback(self, msg: Float64MultiArray):
#         # Kiểm tra xem có đủ ít nhất 8 phần tử (7 joint cánh tay + 1 gripper) không
#         if len(msg.data) >= 8:
#             # Lấy 8 giá trị đầu tiên (index 0 đến 7)
#             self.left_positions = msg.data[:8]
#             self.publish_joint_state()
#
#     def right_callback(self, msg: Float64MultiArray):
#         # Kiểm tra xem có đủ ít nhất 8 phần tử không
#         if len(msg.data) >= 8:
#             # Lấy 8 giá trị đầu tiên (index 0 đến 7)
#             self.right_positions = msg.data[:8]
#             self.publish_joint_state()
#
#     def publish_joint_state(self):
#         # Chỉ publish khi đã nhận được dữ liệu của cả 2 cánh tay ít nhất 1 lần
#         if self.left_positions is None or self.right_positions is None:
#             return
#
#         # Khởi tạo tin nhắn JointState
#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#
#         # Gộp tên joints và vị trí (positions)
#         joint_state_msg.name = self.left_joint_names + self.right_joint_names
#         joint_state_msg.position = list(self.left_positions) + list(self.right_positions)
#
#         # Phát message ra topic /joint_command
#         self.joint_pub.publish(joint_state_msg)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = CommandMergerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.try_shutdown()
#
#
# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3

#
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
#
#
# class CommandMergerNode(Node):
#     def __init__(self):
#         super().__init__('command_merger_node')
#
#         # Lưu trữ tạm thời giá trị của tay trái và phải
#         # Để an toàn, chúng ta sẽ chờ nhận được data từ cả 2 tay rồi mới publish
#         self.left_positions = None
#         self.right_positions = None
#
#         # Khởi tạo danh sách tên khớp (joints)
#         self.left_joint_names = [f'openarmx_left_joint{i}' for i in range(1, 8)]
#         self.right_joint_names = [f'openarmx_right_joint{i}' for i in range(1, 8)]
#
#         # Subscribers
#         self.left_sub = self.create_subscription(
#             Float64MultiArray,
#             '/left_forward_position_controller/commands',
#             self.left_callback,
#             10
#         )
#         self.right_sub = self.create_subscription(
#             Float64MultiArray,
#             '/right_forward_position_controller/commands',
#             self.right_callback,
#             10
#         )
#
#         # Publisher
#         self.joint_pub = self.create_publisher(
#             JointState,
#             '/joint_command',
#             10
#         )
#
#         self.get_logger().info("Đã khởi động node gộp lệnh (Command Merger Node). Đang chờ dữ liệu...")
#
#     def left_callback(self, msg: Float64MultiArray):
#         # Kiểm tra xem có đủ ít nhất 7 phần tử không
#         if len(msg.data) >= 7:
#             # Lấy 7 giá trị đầu tiên (từ index 0 đến 6), bỏ qua giá trị cuối (gripper)
#             self.left_positions = msg.data[:7]
#             self.publish_joint_state()
#
#     def right_callback(self, msg: Float64MultiArray):
#         # Kiểm tra xem có đủ ít nhất 7 phần tử không
#         if len(msg.data) >= 7:
#             # Lấy 7 giá trị đầu tiên (từ index 0 đến 6), bỏ qua giá trị cuối (gripper)
#             self.right_positions = msg.data[:7]
#             self.publish_joint_state()
#
#     def publish_joint_state(self):
#         # Chỉ publish khi đã nhận được dữ liệu của cả 2 cánh tay ít nhất 1 lần
#         # Điều này tránh việc publish giá trị [0.0] cho cánh tay kia gây giật cục (jump)
#         if self.left_positions is None or self.right_positions is None:
#             return
#
#         # Khởi tạo tin nhắn JointState
#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#
#         # Gộp tên joints và vị trí (positions)
#         joint_state_msg.name = self.left_joint_names + self.right_joint_names
#         joint_state_msg.position = list(self.left_positions) + list(self.right_positions)
#
#         # Phát message ra topic /joint_command
#         self.joint_pub.publish(joint_state_msg)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = CommandMergerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.try_shutdown()
#
#
# if __name__ == '__main__':
#     main()