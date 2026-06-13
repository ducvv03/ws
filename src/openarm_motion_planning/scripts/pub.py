#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class OpenArmBridge(Node):
    def __init__(self):
        super().__init__('openarm_bridge')

        #
        # ===== Joint names cho /joint_command =====
        #
        self.left_arm_names = [
            f'openarm_left_joint{i}' for i in range(1, 8)
        ]

        self.right_arm_names = [
            f'openarm_right_joint{i}' for i in range(1, 8)
        ]

        #
        # ===== Buffer trajectory =====
        #
        self.left_arm_positions = None
        self.right_arm_positions = None

        #
        # ===== Subscribers =====
        #

        # /temp_joint_states -> /joint_states
        self.temp_joint_state_sub = self.create_subscription(
            JointState,
            '/temp_joint_states',
            self.temp_joint_state_callback,
            10
        )

        # trajectory -> joint_command
        self.left_traj_sub = self.create_subscription(
            JointTrajectory,
            '/left_joint_trajectory_controller/joint_trajectory',
            self.left_callback,
            10
        )

        self.right_traj_sub = self.create_subscription(
            JointTrajectory,
            '/right_joint_trajectory_controller/joint_trajectory',
            self.right_callback,
            10
        )

        #
        # ===== Publishers =====
        #

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.joint_command_pub = self.create_publisher(
            JointState,
            '/joint_command',
            10
        )

        self.get_logger().info('OpenArm Bridge started')
        self.get_logger().info(
            '/temp_joint_states -> /joint_states'
        )
        self.get_logger().info(
            '/left_joint_trajectory_controller/joint_trajectory + '
            '/right_joint_trajectory_controller/joint_trajectory '
            '-> /joint_command'
        )

    # ==================================================
    # /temp_joint_states -> /joint_states
    # ==================================================

    def temp_joint_state_callback(self, msg: JointState):

        new_msg = JointState()

        new_msg.header = msg.header

        #
        # openarm_left_joint1
        # ->
        # openarmx_left_joint1
        #
        new_msg.name = [
            name.replace('openarm_', 'openarmx_', 1)
            if name.startswith('openarm_')
            else name
            for name in msg.name
        ]

        new_msg.position = list(msg.position)

        if len(msg.velocity) > 0:
            new_msg.velocity = list(msg.velocity)

        if len(msg.effort) > 0:
            new_msg.effort = list(msg.effort)

        self.joint_state_pub.publish(new_msg)

    # ==================================================
    # trajectory -> joint_command
    # ==================================================

    def left_callback(self, msg: JointTrajectory):

        if len(msg.points) == 0:
            return

        positions = msg.points[-1].positions

        if len(positions) < 7:
            self.get_logger().warn(
                'Left trajectory contains less than 7 joints'
            )
            return

        self.left_arm_positions = list(positions[:7])

        self.publish_joint_command()

    def right_callback(self, msg: JointTrajectory):

        if len(msg.points) == 0:
            return

        positions = msg.points[-1].positions

        if len(positions) < 7:
            self.get_logger().warn(
                'Right trajectory contains less than 7 joints'
            )
            return

        self.right_arm_positions = list(positions[:7])

        self.publish_joint_command()

    def publish_joint_command(self):

        if self.left_arm_positions is None:
            return

        if self.right_arm_positions is None:
            return

        joint_msg = JointState()

        joint_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        joint_msg.name = (
            self.left_arm_names +
            self.right_arm_names
        )

        joint_msg.position = (
            self.left_arm_positions +
            self.right_arm_positions
        )

        self.joint_command_pub.publish(joint_msg)

    # ==================================================

def main(args=None):
    rclpy.init(args=args)

    node = OpenArmBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#
# #!/usr/bin/env python3
#
# import rclpy
# from rclpy.node import Node
#
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
#
#
# class OpenArmBridge(Node):
#     def __init__(self):
#         super().__init__('openarm_bridge')
#
#         #
#         # ===== Joint names cho /joint_command =====
#         #
#         self.left_arm_names = [
#             f'openarm_left_joint{i}' for i in range(1, 8)
#         ]
#
#         self.right_arm_names = [
#             f'openarm_right_joint{i}' for i in range(1, 8)
#         ]
#
#         self.left_arm_positions = None
#         self.right_arm_positions = None
#
#         #
#         # ===== Subscribers =====
#         #
#
#         # temp_joint_states -> joint_states
#         self.temp_joint_state_sub = self.create_subscription(
#             JointState,
#             '/temp_joint_states',
#             self.temp_joint_state_callback,
#             10
#         )
#
#         # commands -> joint_command
#         self.left_cmd_sub = self.create_subscription(
#             Float64MultiArray,
#             '/left_forward_position_controller/commands',
#             self.left_callback,
#             10
#         )
#
#         self.right_cmd_sub = self.create_subscription(
#             Float64MultiArray,
#             '/right_forward_position_controller/commands',
#             self.right_callback,
#             10
#         )
#
#         #
#         # ===== Publishers =====
#         #
#
#         # renamed joint states
#         self.joint_state_pub = self.create_publisher(
#             JointState,
#             '/joint_states',
#             10
#         )
#
#         # merged command
#         self.joint_command_pub = self.create_publisher(
#             JointState,
#             '/joint_command',
#             10
#         )
#
#         self.get_logger().info(
#             'OpenArm Bridge started'
#         )
#         self.get_logger().info(
#             '/temp_joint_states -> /joint_states'
#         )
#         self.get_logger().info(
#             '/left_forward_position_controller/commands + '
#             '/right_forward_position_controller/commands -> /joint_command'
#         )
#
#     # ==================================================
#     # temp_joint_states -> joint_states
#     # ==================================================
#
#     def temp_joint_state_callback(self, msg: JointState):
#
#         new_msg = JointState()
#
#         new_msg.header = msg.header
#
#
#         new_names = []
#
#         for name in msg.name:
#             if name.startswith('openarm_'):
#                 name = name.replace(
#                     'openarm_',
#                     'openarmx_',
#                     1
#                 )
#
#             new_names.append(name)
#
#         new_msg.name = new_names
#         new_msg.position = list(msg.position)
#
#         if len(msg.velocity) > 0:
#             new_msg.velocity = list(msg.velocity)
#
#         if len(msg.effort) > 0:
#             new_msg.effort = list(msg.effort)
#
#         self.joint_state_pub.publish(new_msg)
#
#     # ==================================================
#     # commands -> joint_command
#     # ==================================================
#
#     def left_callback(self, msg: Float64MultiArray):
#
#         if len(msg.data) < 7:
#             self.get_logger().warn(
#                 'Left command contains less than 7 joints'
#             )
#             return
#
#         self.left_arm_positions = list(msg.data[:7])
#
#         self.publish_joint_command()
#
#     def right_callback(self, msg: Float64MultiArray):
#
#         if len(msg.data) < 7:
#             self.get_logger().warn(
#                 'Right command contains less than 7 joints'
#             )
#             return
#
#         self.right_arm_positions = list(msg.data[:7])
#
#         self.publish_joint_command()
#
#     def publish_joint_command(self):
#
#         if self.left_arm_positions is None:
#             return
#
#         if self.right_arm_positions is None:
#             return
#
#         msg = JointState()
#
#         msg.header.stamp = (
#             self.get_clock().now().to_msg()
#         )
#
#         msg.name = (
#             self.left_arm_names +
#             self.right_arm_names
#         )
#
#         msg.position = (
#             self.left_arm_positions +
#             self.right_arm_positions
#         )
#
#         self.joint_command_pub.publish(msg)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#
#     node = OpenArmBridge()
#
#     try:
#         rclpy.spin(node)
#
#     except KeyboardInterrupt:
#         pass
#
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()