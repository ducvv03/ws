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

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace head_hardware {

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
  // Parse the <param> values from the ros2_control block into members.
  if (!parse_config(info)) {
    RCLCPP_ERROR(rclcpp::get_logger("HeadHw"),
                 "on_init: failed to parse hardware configuration");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Bring up the CAN socket and register the single head motor. The receive
  // CAN id follows the openarm_hardware convention (send + 0x10).
  RCLCPP_INFO(rclcpp::get_logger("HeadHw"),
              "on_init: opening CAN socket on '%s' (CAN-FD %s) for motor 0x%02X",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled", can_id_);
  try {
    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(can_interface_,
                                                               can_fd_);
    // Register the motor directly in POS_VEL mode: write() drives it with
    // posvel_control_all, which the library rejects unless the motor's control
    // mode is POS_VEL (default is MIT).
    openarm_->init_arm_motors({motor_type_}, {can_id_},
                              {can_id_ + RECV_CAN_ID_OFFSET},
                              {openarm::damiao_motor::ControlMode::POS_VEL});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HeadHw"),
                 "on_init: failed to initialize CAN on '%s': %s",
                 can_interface_.c_str(), e.what());
    openarm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HeadHw"), "on_init: success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeadHw::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto logger = rclcpp::get_logger("HeadHw");
  RCLCPP_INFO(logger, "on_configure: configuring (previous state: '%s')",
              previous_state.label().c_str());

  if (!openarm_) {
    RCLCPP_ERROR(logger, "on_configure: CAN not initialized (on_init failed?)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure only: put the motor in STATE-reporting mode and read its current
  // state once (without energizing it) so we can verify the bus is alive. The
  // motor is NOT enabled here -- that belongs in on_activate.
  try {
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    openarm_->refresh_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();

    const auto motors = openarm_->get_arm().get_motors();
    if (motors.empty()) {
      RCLCPP_ERROR(logger, "on_configure: no motor registered on '%s'",
                   can_interface_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    pos_states_ = motors.front().get_position();
    vel_states_ = motors.front().get_velocity();
    tau_states_ = motors.front().get_torque();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "on_configure: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
              "on_configure: motor 0x%02X initial state pos=%.4f vel=%.4f "
              "tau=%.4f",
              can_id_, pos_states_, vel_states_, tau_states_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HeadHw::export_state_interfaces() {
  // Expose the motor's position/velocity/effort as read-only state handles that
  // controllers subscribe to. Each handle points at a member updated by read().
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint_name_,
                                hardware_interface::HW_IF_POSITION,
                                &pos_states_);
  state_interfaces.emplace_back(joint_name_,
                                hardware_interface::HW_IF_VELOCITY,
                                &vel_states_);
  state_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT,
                                &tau_states_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HeadHw::export_command_interfaces() {
  // Expose position/velocity/effort as writable command handles that a
  // controller claims. write() pushes these members out to the motor over CAN.
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint_name_,
                                  hardware_interface::HW_IF_POSITION,
                                  &pos_cmd_);
  command_interfaces.emplace_back(joint_name_,
                                  hardware_interface::HW_IF_VELOCITY,
                                  &vel_cmd_);
  command_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT,
                                  &tau_cmd_);
  return command_interfaces;
}

hardware_interface::CallbackReturn HeadHw::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const auto logger = rclcpp::get_logger("HeadHw");
  RCLCPP_INFO(logger, "on_activate: enabling motor 0x%02X", can_id_);

  if (!openarm_) {
    RCLCPP_ERROR(logger, "on_activate: CAN not initialized (on_init failed?)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Energize the motor so it starts reporting state and accepts commands.
  try {
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    // Push the POS_VEL control mode down to the motor before enabling it, so the
    // posvel frames from write() are accepted (motor defaults to MIT).
    openarm_->get_arm().set_control_mode_all(
        openarm::damiao_motor::ControlMode::POS_VEL);
    openarm_->enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();

    const auto motors = openarm_->get_arm().get_motors();
    if (motors.empty()) {
      RCLCPP_ERROR(logger, "on_activate: no motor registered on '%s'",
                   can_interface_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Seed both state and command from the motor's real position so the first
    // write() does not jerk the joint away from where it currently sits.
    pos_states_ = motors.front().get_position();
    vel_states_ = motors.front().get_velocity();
    tau_states_ = motors.front().get_torque();
    pos_cmd_ = pos_states_;
    vel_cmd_ = 0.0;
    tau_cmd_ = 0.0;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "on_activate: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
              "on_activate: motor 0x%02X enabled at pos=%.4f", can_id_,
              pos_states_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeadHw::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const auto logger = rclcpp::get_logger("HeadHw");
  RCLCPP_INFO(logger, "on_deactivate: di=))) sabling motor 0x%02X", can_id_);

  if (!openarm_) {
    RCLCPP_WARN(logger, "on_deactivate: CAN not initialized, nothing to do");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // De-energize the motor. Retry a few times because a single disable frame can
  // be missed on a busy bus (mirrors the arm hardware exit path).
  try {
    for (int i = 0; i < 3; ++i) {
      openarm_->disable_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      openarm_->recv_all();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "on_deactivate: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "on_deactivate: motor 0x%02X disabled", can_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HeadHw::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!openarm_) {
    return hardware_interface::return_type::ERROR;
  }

  // Poll the motor for its latest state and copy it into the state handles that
  // controllers read. No sleep here -- this runs in the controller_manager RT
  // loop, so recv_all() relies on its own short timeout.
  try {
    openarm_->refresh_all();
    openarm_->recv_all();

    const auto& motors = openarm_->get_arm().get_motors();
    if (motors.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("HeadHw"),
                   "read: no motor registered on '%s'", can_interface_.c_str());
      return hardware_interface::return_type::ERROR;
    }
    pos_states_ = motors.front().get_position();
    vel_states_ = motors.front().get_velocity();
    tau_states_ = motors.front().get_torque();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HeadHw"), "read: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeadHw::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!openarm_) {
    return hardware_interface::return_type::ERROR;
  }

  // Push the latest command out to the motor using POS_VEL control: give it the
  // target position plus a travel velocity and let the motor's internal
  // controller drive the trajectory (no MIT gain tuning / manual interpolation).
  // The forward position controller streams position only, so vel_cmd_ is 0 --
  // fall back to a fixed profile velocity so the joint actually moves; if a
  // controller does command velocity, honour it. No sleep -- RT loop.
  try {
    const double travel_vel =
        (std::abs(vel_cmd_) > 1e-6) ? std::abs(vel_cmd_) : PROFILE_VELOCITY;
    const openarm::damiao_motor::PosVelParam param{pos_cmd_, travel_vel};
    openarm_->get_arm().posvel_control_all({param});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HeadHw"), "write: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool HeadHw::parse_config(const hardware_interface::HardwareInfo& info) {
  const auto logger = rclcpp::get_logger("HeadHw");

  // --- joint name: this hardware component owns exactly one joint ----------
  if (info.joints.empty()) {
    RCLCPP_ERROR(logger,
                 "parse_config: no <joint> declared in ros2_control block '%s'",
                 info.name.c_str());
    joint_name_.clear();
    return false;
  }
  joint_name_ = info.joints.front().name;
  if (info.joints.size() > 1) {
    RCLCPP_WARN(logger,
                "parse_config: %zu joints declared, HeadHw handles only the "
                "first ('%s')",
                info.joints.size(), joint_name_.c_str());
  }

  // --- CAN interface -------------------------------------------------------
  can_interface_ = get_param(info, PARAM_CAN_INTERFACE, DEFAULT_CAN_INTERFACE);
  if (info.hardware_parameters.count(PARAM_CAN_INTERFACE) == 0) {
    RCLCPP_WARN(logger, "parse_config: '%s' not set, using default '%s'",
                PARAM_CAN_INTERFACE, can_interface_.c_str());
  }

  // --- CAN-FD (optional, defaults to enabled) ------------------------------
  const std::string can_fd_str = get_param(info, PARAM_CAN_FD, "true");
  can_fd_ = (can_fd_str == "true" || can_fd_str == "True" || can_fd_str == "1");

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
  return true;
}

int HeadHw::return_to_zero() {
  const auto logger = rclcpp::get_logger("HeadHw");
  if (!openarm_) {
    return -1;
  }

  // POS_VEL control: hand the motor the target position (ZERO_POINT) and a fixed
  // travel speed, and let its internal controller drive the trajectory itself --
  // no manual interpolation like MIT needs. Runs in on_activate (not RT).
  constexpr double RETURN_VELOCITY = 0.5;  // rad/s travel speed toward zero
  try {
    const openarm::damiao_motor::PosVelParam param{ZERO_POINT, RETURN_VELOCITY};
    openarm_->get_arm().posvel_control_all({param});
    openarm_->recv_all();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "return_to_zero: CAN error on '%s': %s",
                 can_interface_.c_str(), e.what());
    return -1;
  }

  RCLCPP_INFO(logger, "return_to_zero: motor 0x%02X commanded to %.4f at %.2f rad/s",
              can_id_, ZERO_POINT, RETURN_VELOCITY);
  return 0;
}

}  // namespace head_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(head_hardware::HeadHw,
                       hardware_interface::SystemInterface)
