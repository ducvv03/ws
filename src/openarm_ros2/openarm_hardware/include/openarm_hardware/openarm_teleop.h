// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "openarm_hardware/openarm_simple_hardware.hpp"
#include <controller/control.hpp>
#include  <yamlloader.hpp>
using namespace openarm_hardware;
namespace openarm_hardware_teleop {

/**
 * @brief Teleoperation variant of the OpenArm hardware interface.
 *
 * Reuses all of OpenArmHW's lifecycle/setup logic and only customizes the
 * read()/write() loop for the teleop use case.
 */
class OpenArmHWTeleOp : public openarm_hardware::OpenArmHW {
 public:
  OpenArmHWTeleOp() {
    //control_  = std::make_unique<Control> () ;
  }

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Customized for teleop — rewritten, not delegated.
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::unique_ptr<Control> control_;
    //std::vector<JointState> joint_arm_states(OpenArmHW::ARM_DOF) ;
};


}  // namespace openarm_hardware_teleop
