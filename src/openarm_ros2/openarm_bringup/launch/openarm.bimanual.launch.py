# Copyright 2025 Enactic, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# All accepted arm_type values
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    if arm_type_str not in VALID_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type_str}'. "
            f"Please specify openarm_v1.0 or openarm_v2.0."
        )
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


def namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip('/')
    return None


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, use_fake_hand, right_can_interface, left_can_interface,
                               use_teleop, hand_protocol, right_hand_can_interface, left_hand_can_interface, hands,
                               use_head_vertical, use_head_horizontal,
                               head_vertical_can_interface, head_horizontal_can_interface,
                               head_vertical_can_id, head_horizontal_can_id):
    """Generate robot description using xacro processing."""
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    use_fake_hand_str = context.perform_substitution(use_fake_hand)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    use_teleop_str = context.perform_substitution(use_teleop)
    hand_protocol_str = context.perform_substitution(hand_protocol)
    right_hand_can_str = context.perform_substitution(right_hand_can_interface)
    left_hand_can_str = context.perform_substitution(left_hand_can_interface)
    hands_str = context.perform_substitution(hands)
    use_head_vertical_str = context.perform_substitution(use_head_vertical)
    use_head_horizontal_str = context.perform_substitution(use_head_horizontal)
    head_vertical_can_interface_str = context.perform_substitution(head_vertical_can_interface)
    head_horizontal_can_interface_str = context.perform_substitution(head_horizontal_can_interface)
    head_vertical_can_id_str = context.perform_substitution(head_vertical_can_id)
    head_horizontal_can_id_str = context.perform_substitution(head_horizontal_can_id)

    folder_name, file_name = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "assets", "robot", folder_name, "urdf", file_name
    )

    # --- [ĐÃ SỬA] TRÍCH XUẤT ĐƯỜNG DẪN CỦA HAND CONFIG ---
    try:
        brainco_driver_path = get_package_share_directory("brainco_hand_driver")
        brainco_moveit_path = get_package_share_directory("brainco_moveit_config")
    except Exception:
        brainco_driver_path = ""
        brainco_moveit_path = ""

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "use_fake_hand": use_fake_hand_str,
            "use_teleop": use_teleop_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
            "use_head_vertical": use_head_vertical_str,
            "use_head_horizontal": use_head_horizontal_str,
            "head_vertical_can_interface": head_vertical_can_interface_str,
            "head_horizontal_can_interface": head_horizontal_can_interface_str,
            "head_vertical_can_id": head_vertical_can_id_str,
            "head_horizontal_can_id": head_horizontal_can_id_str,
            # --- [ĐÃ SỬA] TRUYỀN ĐƯỜNG DẪN VÀO MAPPINGS ĐỂ XACRO KHÔNG BỊ LỖI ---
            # Selected by the hand_protocol launch argument: modbus | canfd | socketcan.
            "left_protocol_config_file": os.path.join(
                brainco_driver_path, "config", f"protocol_{hand_protocol_str}_left.yaml"),
            "right_protocol_config_file": os.path.join(
                brainco_driver_path, "config", f"protocol_{hand_protocol_str}_right.yaml"),
            "initial_positions_file": os.path.join(brainco_moveit_path, "config", "dual_revo2_initial_positions.yaml"),
            # Empty keeps whatever the protocol config file specifies.
            "right_hand_can_interface": right_hand_can_str,
            "left_hand_can_interface": left_hand_can_str,
            "hands": hands_str,
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, use_fake_hand, controllers_file,
                        right_can_interface, left_can_interface, arm_prefix, use_teleop,
                        hand_protocol, right_hand_can_interface, left_hand_can_interface,
                        hands, use_head_vertical, use_head_horizontal,
                        head_vertical_can_interface, head_horizontal_can_interface,
                        head_vertical_can_id, head_horizontal_can_id):
    """Spawn both robot state publisher and control nodes with shared robot description."""
    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type,
        use_fake_hardware, use_fake_hand, right_can_interface, left_can_interface,
        use_teleop, hand_protocol, right_hand_can_interface, left_hand_can_interface, hands,
        use_head_vertical, use_head_horizontal,
        head_vertical_can_interface, head_horizontal_can_interface,
        head_vertical_can_id, head_horizontal_can_id,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    if namespace:
        controllers_file_str = controllers_file_str.replace(
            "openarm_bimanual_controllers.yaml",
            "openarm_bimanual_controllers_namespaced.yaml"
        )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    # The head_forward_position_controller lives in the same controllers file as
    # the arms (openarm_bimanual_controllers.yaml), so it is already loaded here;
    # the launch only needs to spawn it when a head joint is enabled (see
    # head_controller_spawner below).
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn controller based on robot_controller argument."""
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[robot_controller_left, robot_controller_right,
                       "-c", controller_manager_ref],
        )
    ]


def arm_pid_controller_spawner(context: LaunchContext, use_payload_compensation, arm_prefix):
    """Spawn the arm PID controllers used for payload / static-droop compensation.

    These claim the *effort* command interface only, so they coexist with
    whichever position controller is active -- no interface conflict. Their
    output lands in tau_commands_, which OpenArmHW::write() adds to the gravity
    torques and sends as the MIT frame's t_ff term.

    Off by default: the controller injects torque, and it only behaves once
    something publishes control_msgs/MultiDOFCommand on
    <arm>_arm_pid_controller/reference tracking the same setpoint the position
    controller is following. Activated without that reference it holds the pose
    it saw at activation, and its integral will wind up fighting any commanded
    motion (bounded by u_clamp, but still wrong).
    """
    use_payload_compensation_str = context.perform_substitution(use_payload_compensation)

    if use_payload_compensation_str.lower() not in ("true", "1"):
        print("[openarm.bimanual.launch] payload compensation disabled "
              f"(use_payload_compensation:={use_payload_compensation_str}), "
              "arm PID controllers not spawned")
        return []

    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    print("[openarm.bimanual.launch] payload compensation enabled, spawning "
          f"left/right_arm_pid_controller on {controller_manager_ref}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["left_arm_pid_controller", "right_arm_pid_controller",
                       "-c", controller_manager_ref],
        )
    ]


def generate_launch_description():
    """Generate launch description for OpenArm bimanual configuration."""

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="openarm_v1.0",
            description="Arm type. Accepts: v1.0, v10, openarm_v1.0, v2.0, v20, openarm_v2.0, etc.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "use_fake_hand",
            default_value="true",
            description="Use fake hand instead of real hand.",
        ),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="false",
            description="Use the teleop-to-sim hardware interface (OpenArmHWTeleOp) for the arms.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "hand_protocol",
            default_value="modbus",
            choices=["modbus", "canfd", "socketcan"],
            description=(
                "Transport for the Revo2 hands. Selects "
                "brainco_hand_driver/config/protocol_<value>_{left,right}.yaml. "
                "'socketcan' expects the hand interfaces to be up already "
                "(the arms hold can0/can1, so the hands default to can2/can3); "
                "'canfd' needs the driver rebuilt with -DENABLE_CANFD=ON."
            ),
        ),
        DeclareLaunchArgument(
            "hands",
            default_value="both",
            choices=["both", "left", "right"],
            description=(
                "Which Revo2 hands to bring up. A single side instantiates only that "
                "hand's ros2_control system and spawns only its controller, so a robot "
                "with one hand wired does not fail on the missing one."
            ),
        ),
        DeclareLaunchArgument(
            "right_hand_can_interface",
            default_value="",
            description=(
                "SocketCAN interface for the right Revo2 hand. Empty keeps the value "
                "from the protocol config file (can2). Set to the right arm's bus "
                "(can0) to share one line down the arm."
            ),
        ),
        DeclareLaunchArgument(
            "left_hand_can_interface",
            default_value="",
            description=(
                "SocketCAN interface for the left Revo2 hand. Empty keeps the value "
                "from the protocol config file (can3). Set to the left arm's bus "
                "(can1) to share one line down the arm."
            ),
        ),
        DeclareLaunchArgument(
            "use_head_vertical",
            default_value="false",
            choices=["true", "false"],
            description="Enable the vertical head joint (head_hardware/HeadHw).",
        ),
        DeclareLaunchArgument(
            "use_head_horizontal",
            default_value="false",
            choices=["true", "false"],
            description="Enable the horizontal head joint (head_hardware/HeadHw).",
        ),
        DeclareLaunchArgument(
            "head_vertical_can_interface",
            default_value="can1",
            description="CAN interface for the vertical head joint (defaults to the left arm bus).",
        ),
        DeclareLaunchArgument(
            "head_horizontal_can_interface",
            default_value="can0",
            description="CAN interface for the horizontal head joint (defaults to the right arm bus).",
        ),
        DeclareLaunchArgument(
            "head_vertical_can_id",
            default_value="0x22",
            description="CAN id of the vertical head motor (decimal or 0x-hex).",
        ),
        DeclareLaunchArgument(
            "head_horizontal_can_id",
            default_value="0x23",
            description="CAN id of the horizontal head motor (decimal or 0x-hex).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_bimanual_controllers.yaml",
            description="Controllers file to use.",
        ),
        DeclareLaunchArgument(
            "use_payload_compensation",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Spawn left/right_arm_pid_controller, which add an integral "
                "torque term on top of gravity compensation to cancel the "
                "static droop caused by a grasped payload. Requires a "
                "control_msgs/MultiDOFCommand publisher on "
                "<arm>_arm_pid_controller/reference carrying the same setpoint "
                "the position controller is tracking -- without it the "
                "integral will fight commanded motion."
            ),
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_hand = LaunchConfiguration("use_fake_hand")
    use_teleop = LaunchConfiguration("use_teleop")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    use_payload_compensation = LaunchConfiguration("use_payload_compensation")
    hand_protocol = LaunchConfiguration("hand_protocol")
    right_hand_can_interface = LaunchConfiguration("right_hand_can_interface")
    left_hand_can_interface = LaunchConfiguration("left_hand_can_interface")
    hands = LaunchConfiguration("hands")
    use_head_vertical = LaunchConfiguration("use_head_vertical")
    use_head_horizontal = LaunchConfiguration("use_head_horizontal")
    head_vertical_can_interface = LaunchConfiguration("head_vertical_can_interface")
    head_horizontal_can_interface = LaunchConfiguration("head_horizontal_can_interface")
    head_vertical_can_id = LaunchConfiguration("head_vertical_can_id")
    head_horizontal_can_id = LaunchConfiguration("head_horizontal_can_id")

    try:
        camera_pkg_share = get_package_share_directory("openarm_bringup")
        camera_yaml_file = os.path.join(camera_pkg_share, "config", "calibration", "camera_tf.yaml")

        with open(camera_yaml_file, "r") as f:
            camera_cfg = yaml.safe_load(f)

        cp = camera_cfg["camera_tf"]["ros__parameters"]

        camera_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_static_tf",
            arguments=[
                "--x", str(cp["x"]),
                "--y", str(cp["y"]),
                "--z", str(cp["z"]),
                "--qx", str(cp["qx"]),
                "--qy", str(cp["qy"]),
                "--qz", str(cp["qz"]),
                "--qw", str(cp["qw"]),
                "--frame-id", cp["parent_frame"],
                "--child-frame-id", cp["child_frame"],
            ],
        )
    except Exception as e:
        print(f"Warning: Could not load camera_tf.yaml. Error: {e}")
        camera_tf_node = None

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, use_fake_hand, controllers_file,
              right_can_interface, left_can_interface, arm_prefix, use_teleop,
              hand_protocol, right_hand_can_interface, left_hand_can_interface, hands,
              use_head_vertical, use_head_horizontal,
              head_vertical_can_interface, head_horizontal_can_interface,
              head_vertical_can_id, head_horizontal_can_id]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "bimanual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                if namespace_from_context(context, arm_prefix)
                else "/controller_manager"
            ],
        )]
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[robot_controller, arm_prefix]
    )

    def head_controller_spawner(context: LaunchContext):
        """Spawn the head forward controller only when a head joint is enabled."""
        v = context.perform_substitution(use_head_vertical).lower() in ("true", "1")
        h = context.perform_substitution(use_head_horizontal).lower() in ("true", "1")
        if not (v or h):
            return []
        namespace = namespace_from_context(context, arm_prefix)
        controller_manager_ref = (
            f"/{namespace}/controller_manager" if namespace else "/controller_manager"
        )
        return [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["head_forward_position_controller",
                       "-c", controller_manager_ref],
        )]

    head_controller_spawner_func = OpaqueFunction(function=head_controller_spawner)

    def hand_controller_spawner_fn(context: LaunchContext):
        """Spawn one controller per hand that was actually instantiated.

        The spawner activates its controllers in order and aborts on the first
        failure, so listing a hand whose hardware never configured would also
        keep the working hand's controller from ever coming up.
        """
        hands_str = context.perform_substitution(hands)
        controllers = []
        if hands_str in ("both", "left"):
            controllers.append("left_revo2_hand_controller")
        if hands_str in ("both", "right"):
            controllers.append("right_revo2_hand_controller")

        namespace = namespace_from_context(context, arm_prefix)
        controller_manager_ref = (
            f"/{namespace}/controller_manager" if namespace else "/controller_manager"
        )

        # One spawner per controller: a failure on one hand then cannot take the
        # other one down with it.
        return [
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[controller, "-c", controller_manager_ref],
            )
            for controller in controllers
        ]

    hand_controller_spawner = OpaqueFunction(function=hand_controller_spawner_fn)

    arm_pid_controller_spawner_func = OpaqueFunction(
        function=arm_pid_controller_spawner,
        args=[use_payload_compensation, arm_prefix]
    )

    LAUNCH_DELAY_SECONDS = 1.0
    # Spawn the PID controllers after the position controllers are up. They
    # claim a different command interface so there is no conflict, but letting
    # position control settle first keeps the integral from starting against a
    # moving target.
    PID_SPAWN_DELAY_SECONDS = 2.0

    launch_actions = declared_arguments + [
        robot_nodes_spawner_func,
        rviz_node,
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[controller_spawner_func]),
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[hand_controller_spawner]),
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[head_controller_spawner_func]),
        TimerAction(period=PID_SPAWN_DELAY_SECONDS, actions=[arm_pid_controller_spawner_func]),
    ]

    if camera_tf_node:
        launch_actions.append(camera_tf_node)

    return LaunchDescription(launch_actions)
