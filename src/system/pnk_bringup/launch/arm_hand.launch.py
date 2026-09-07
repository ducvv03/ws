#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # ================= 1. DECLARE LAUNCH ARGUMENTS =================
    declared_arguments = [
        DeclareLaunchArgument("arm_type", default_value="v10", description="OpenArm Type"),
        # ĐẶT FALSE ĐỂ CHẠY ĐỒ THẬT
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware"),
        DeclareLaunchArgument("right_can_interface", default_value="can0", description="Right Arm CAN"),
        DeclareLaunchArgument("left_can_interface", default_value="can1", description="Left Arm CAN"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz2"),
    ]

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    right_can = LaunchConfiguration("right_can_interface")
    left_can = LaunchConfiguration("left_can_interface")
    arm_type = LaunchConfiguration("arm_type")
    use_rviz = LaunchConfiguration("use_rviz")

    # ================= 2. PATHS TO CONFIG FILES =================
    # Đường dẫn Modbus của bàn tay BrainCo
    left_protocol_config = PathJoinSubstitution([FindPackageShare("brainco_hand_driver"), "config", "protocol_modbus_left.yaml"])
    right_protocol_config = PathJoinSubstitution([FindPackageShare("brainco_hand_driver"), "config", "protocol_modbus_right.yaml"])
    initial_positions = PathJoinSubstitution([FindPackageShare("brainco_hand_driver"), "config", "dual_revo2_initial_positions.yaml"])

    # THAY ĐỔI 'openarm_revo2_moveit_config' THÀNH TÊN PACKAGE CHỨA FILE URDF TỔNG HỢP CỦA BẠN
    unified_urdf_path = PathJoinSubstitution([FindPackageShare("openarm_revo2_moveit_config"), "config", "openarm_revo2.urdf.xacro"])
    unified_controllers_path = PathJoinSubstitution([FindPackageShare("openarm_revo2_moveit_config"), "config", "ros2_controllers.yaml"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare("openarm_description"), "rviz", "bimanual.rviz"])

    # ================= 3. XACRO COMMAND (TRUYỀN PARAM CHO CẢ ARM VÀ HAND) =================
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", unified_urdf_path, " ",
        "arm_type:=", arm_type, " ",
        "use_fake_hardware:=", use_fake_hardware, " ",
        "right_can_interface:=", right_can, " ",
        "left_can_interface:=", left_can, " ",
        "left_protocol_config_file:=", left_protocol_config, " ",
        "right_protocol_config_file:=", right_protocol_config, " ",
        "initial_positions_file:=", initial_positions
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ================= 4. ROS2 NODES =================
    # Controller Manager (Quản lý cả CAN của Arm và Modbus của Hand)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, unified_controllers_path],
        output="both",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    # ================= 5. CONTROLLER SPAWNERS =================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Arm Controllers
    arm_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "right_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # Hand Controllers
    hand_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_revo2_hand_controller", "right_revo2_hand_controller", "-c", "/controller_manager"],
    )

    # ================= 6. EVENT HANDLERS (TRÌNH TỰ KHỞI ĐỘNG) =================
    # Đợi joint_state_broadcaster chạy xong mới bật các controllers điều khiển
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controllers_spawner, hand_controllers_spawner],
        )
    )

    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_pub_node,
        TimerAction(period=1.5, actions=[joint_state_broadcaster_spawner]), # Chờ control_node sẵn sàng
        delay_controllers,
        rviz_node
    ])