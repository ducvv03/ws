// Copyright 2026 Enactic, Inc.
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

#include "head_hardware/head_hardware.h"

#include <stdexcept>
#include <string>

namespace head_hardware {

namespace {
// Parameter keys as declared in the <ros2_control> xacro <param name="...">.
constexpr const char* PARAM_CAN_INTERFACE = "can_interface";
constexpr const char* PARAM_DIRECTION = "direction";
constexpr const char* PARAM_CAN_ID = "can_id";
constexpr const char* PARAM_MOTOR_TYPE = "motor_type";

// Defaults used when a param is absent.
constexpr const char* DEFAULT_CAN_INTERFACE = "can0";
constexpr HeadHw::CanId DEFAULT_CAN_ID = 0x21;
constexpr openarm::damiao_motor::MotorType DEFAULT_MOTOR_TYPE =
    openarm::damiao_motor::MotorType::DM4340

// Look up a hardware parameter, returning `fallback` if it is not present.
std::string get_param(const hardware_interface::HardwareInfo& info,
                      const std::string& key, const std::string& fallback) {
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return it->second;
}
}  // namespace

hardware_interface::CallbackReturn HeadHw::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(rclcpp::get_logger("HeadHw"),
              "on_init: initializing '%s' (%zu joints declared)",
              info.name.c_str(), info.joints.size());

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("HeadHw"),
                 "on_init: base SystemInterface::on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HeadHw"), "on_init: success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeadHw::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("HeadHw"),
              "on_configure: configuring (previous state: '%s')",
              previous_state.label().c_str());

  RCLCPP_INFO(rclcpp::get_logger("HeadHw"), "on_configure: success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HeadHw::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HeadHw::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces;
}

hardware_interface::CallbackReturn HeadHw::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeadHw::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HeadHw::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeadHw::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

void HeadHw::parse_config(const hardware_interface::HardwareInfo& info) {
  const auto logger = rclcpp::get_logger("HeadHw");

  // --- joint name: this hardware component owns exactly one joint ----------
  if (info.joints.empty()) {
    RCLCPP_ERROR(logger,
                 "parse_config: no <joint> declared in ros2_control block '%s'",
                 info.name.c_str());
    joint_name_.clear();
  } else {
    joint_name_ = info.joints.front().name;
    if (info.joints.size() > 1) {
      RCLCPP_WARN(logger,
                  "parse_config: %zu joints declared, HeadHw handles only the "
                  "first ('%s')",
                  info.joints.size(), joint_name_.c_str());
    }
  }

  // --- CAN interface -------------------------------------------------------
  can_interface_ = get_param(info, PARAM_CAN_INTERFACE, DEFAULT_CAN_INTERFACE);
  if (info.hardware_parameters.count(PARAM_CAN_INTERFACE) == 0) {
    RCLCPP_WARN(logger, "parse_config: '%s' not set, using default '%s'",
                PARAM_CAN_INTERFACE, can_interface_.c_str());
  }

  // --- direction (vertical / horizontal) -----------------------------------
  const std::string direction = get_param(info, PARAM_DIRECTION, "vertical");
  if (direction == "vertical") {
    dir_ = Direction::VERTICAL;
  } else if (direction == "horizontal") {
    dir_ = Direction::HORIZONTAL;
  } else {
    RCLCPP_WARN(logger,
                "parse_config: unknown '%s'='%s', defaulting to vertical",
                PARAM_DIRECTION, direction.c_str());
    dir_ = Direction::VERTICAL;
  }

  // --- CAN id (optional, accepts decimal or 0x-hex) ------------------------
  const std::string can_id_str = get_param(info, PARAM_CAN_ID, "");
  if (can_id_str.empty()) {
    can_id_ = DEFAULT_CAN_ID;
    RCLCPP_WARN(logger, "parse_config: '%s' not set, using default 0x%02X",
                PARAM_CAN_ID, can_id_);
  } else {
    try {
      can_id_ = static_cast<CanId>(std::stoul(can_id_str, nullptr, 0));
    } catch (const std::exception& e) {
      can_id_ = DEFAULT_CAN_ID;
      RCLCPP_ERROR(logger,
                   "parse_config: invalid '%s'='%s' (%s), using default 0x%02X",
                   PARAM_CAN_ID, can_id_str.c_str(), e.what(), can_id_);
    }
  }

  // --- motor type (optional) -----------------------------------------------
  const std::string motor_type_str = get_param(info, PARAM_MOTOR_TYPE, "");
  if (motor_type_str.empty()) {
    motor_type_ = DEFAULT_MOTOR_TYPE;
  } else if (motor_type_str == "DM4310") {
    motor_type_ = openarm::damiao_motor::MotorType::DM4310;
  } else if (motor_type_str == "DM4340") {
    motor_type_ = openarm::damiao_motor::MotorType::DM4340;
  } else if (motor_type_str == "DM8009") {
    motor_type_ = openarm::damiao_motor::MotorType::DM8009;
  } else {
    motor_type_ = DEFAULT_MOTOR_TYPE;
    RCLCPP_WARN(logger,
                "parse_config: unknown '%s'='%s', using default DM4310",
                PARAM_MOTOR_TYPE, motor_type_str.c_str());
  }

  RCLCPP_INFO(logger,
              "parse_config: joint='%s' can='%s' direction='%s' can_id=0x%02X",
              joint_name_.c_str(), can_interface_.c_str(), direction.c_str(),
              can_id_);
}

int HeadHw::return_to_zero() { return 0; }

}  // namespace head_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(head_hardware::HeadHw,
                       hardware_interface::SystemInterface)
