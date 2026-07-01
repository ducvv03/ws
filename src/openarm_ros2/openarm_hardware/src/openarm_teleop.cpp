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

#include "openarm_hardware/openarm_teleop.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware_teleop {

namespace {
constexpr const char* kLogger = "OpenArmHWTeleOp";
}

OpenArmHWTeleOp::OpenArmHWTeleOp() = default;

hardware_interface::CallbackReturn OpenArmHWTeleOp::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp on_init");
  return OpenArmHW::on_init(info);
}

hardware_interface::CallbackReturn OpenArmHWTeleOp::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp on_configure");
  return OpenArmHW::on_configure(previous_state);
}

std::vector<hardware_interface::StateInterface>
OpenArmHWTeleOp::export_state_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp export_state_interfaces");
  return OpenArmHW::export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface>
OpenArmHWTeleOp::export_command_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp export_command_interfaces");
  return OpenArmHW::export_command_interfaces();
}

hardware_interface::CallbackReturn OpenArmHWTeleOp::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp on_activate");
  return OpenArmHW::on_activate(previous_state);
}

hardware_interface::CallbackReturn OpenArmHWTeleOp::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "teleOp on_deactivate");
  return OpenArmHW::on_deactivate(previous_state);
}

// ---------------------------------------------------------------------------
// TODO(teleop): customize read()/write() for teleoperation.
// For now they delegate to the base implementation so the plugin builds and
// behaves identically to OpenArmHW. Protected members of OpenArmHW (openarm_,
// pos_states_, tau_commands_, kp_, kd_, ...) are accessible here.
// ---------------------------------------------------------------------------

hardware_interface::return_type OpenArmHWTeleOp::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  return OpenArmHW::read(time, period);
}

hardware_interface::return_type OpenArmHWTeleOp::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  return OpenArmHW::write(time, period);
}

}  // namespace openarm_hardware_teleop

PLUGINLIB_EXPORT_CLASS(openarm_hardware_teleop::OpenArmHWTeleOp,
                       hardware_interface::SystemInterface)
