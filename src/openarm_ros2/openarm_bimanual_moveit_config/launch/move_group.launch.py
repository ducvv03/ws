# Copyright 2025 Enactic, Inc.
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
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def move_group_spawner(context: LaunchContext, arm_type, use_fake_hardware):
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)

    # Trích xuất đường dẫn tuyệt đối của các file cấu hình YAML
    try:
        brainco_driver_path = get_package_share_directory("brainco_hand_driver")
        brainco_moveit_path = get_package_share_directory("brainco_moveit_config")
    except Exception as e:
        brainco_driver_path = ""
        brainco_moveit_path = ""

    # Truyền đủ các tham số (mappings) cho Xacro để không bị lỗi "Is a directory"
    xacro_mappings = {
        "arm_type": arm_type_str,
        "use_fake_hardware": use_fake_hardware_str,
        "left_protocol_config_file": os.path.join(brainco_driver_path, "config", "protocol_modbus_left.yaml"),
        "right_protocol_config_file": os.path.join(brainco_driver_path, "config", "protocol_modbus_right.yaml"),
        "initial_positions_file": os.path.join(brainco_moveit_path, "config", "dual_revo2_initial_positions.yaml"),
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(mappings=xacro_mappings)
        .robot_description_semantic(file_path=f"config/{arm_type_str}/openarm_bimanual.srdf")
        .joint_limits(file_path=f"config/{arm_type_str}/joint_limits.yaml")
        .robot_description_kinematics(file_path=f"config/{arm_type_str}/kinematics.yaml")
        .trajectory_execution(file_path=f"config/{arm_type_str}/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path=f"config/{arm_type_str}/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_type", default_value="openarm_v1.0"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use fake hardware flag"),
        OpaqueFunction(function=move_group_spawner, args=[
                       LaunchConfiguration("arm_type"),
                       LaunchConfiguration("use_fake_hardware")])
    ])