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

#include "openarm_hardware/openarm_simple_hardware.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace openarm_hardware {

OpenArmHW::OpenArmHW() = default;

OpenArmHW::~OpenArmHW() {
  // Stop the parameter spin thread before members it touches are destroyed.
  param_spin_running_.store(false, std::memory_order_relaxed);
  if (param_spin_thread_.joinable()) {
    param_spin_thread_.join();
  }
}

void OpenArmHW::setup_gain_parameters() {
  if (telemetry_node_ == nullptr) {                        // Rule 3: null guard
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "telemetry_node_ is null, cannot expose gain parameters");
    return;
  }

  // Declare kp1..kp7 / kd1..kd7 seeded from the current (xacro-loaded) values,
  // so `ros2 param get` reflects what MIT mode is actually using at startup.
  for (size_t i = 0; i < ARM_DOF; ++i) {
    telemetry_node_->declare_parameter("kp" + std::to_string(i + 1), kp_[i]);
    telemetry_node_->declare_parameter("kd" + std::to_string(i + 1), kd_[i]);
  }

  // Validate-and-apply callback. Runs on the spin thread; takes gains_mutex_ so
  // write() never reads a half-updated pair.
  param_cb_handle_ = telemetry_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
          const std::string& name = param.get_name();
          const bool is_kp = name.rfind("kp", 0) == 0;
          const bool is_kd = name.rfind("kd", 0) == 0;
          if (!is_kp && !is_kd) {
            continue;  // not one of ours; accept without touching gains
          }

          // Parse the 1-based joint index out of "kp<N>" / "kd<N>".
          int joint = 0;
          try {
            joint = std::stoi(name.substr(2));
          } catch (const std::exception&) {
            continue;  // e.g. "kp_hand" -- handled elsewhere, not here
          }
          if (joint < 1 || joint > static_cast<int>(ARM_DOF)) {  // Rule 3: bounds
            result.successful = false;
            result.reason = name + ": joint index out of [1, " +
                            std::to_string(ARM_DOF) + "]";
            RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"), "%s", result.reason.c_str());
            return result;
          }

          const double value = param.as_double();
          const double lo = is_kp ? KP_MIN : KD_MIN;
          const double hi = is_kp ? KP_MAX : KD_MAX;
          if (std::isnan(value) || value < lo || value > hi) {   // Rule 3: range
            result.successful = false;
            result.reason = name + ": " + std::to_string(value) + " outside [" +
                            std::to_string(lo) + ", " + std::to_string(hi) + "]";
            RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"), "%s", result.reason.c_str());
            return result;  // reject the whole set, change nothing
          }

          {
            std::lock_guard<std::mutex> lock(gains_mutex_);
            (is_kp ? kp_ : kd_)[joint - 1] = value;
          }
          RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "%s -> %.3f", name.c_str(), value);
        }
        return result;
      });

  // telemetry_node_ is not spun anywhere else, so without this the parameter
  // services would never answer. One background thread services them.
  param_spin_running_.store(true, std::memory_order_relaxed);
  param_spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(telemetry_node_);
    while (rclcpp::ok() && param_spin_running_.load(std::memory_order_relaxed)) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ~100 Hz service
    }
  });

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "kp1..kp7 / kd1..kd7 exposed as ROS parameters on '%s' "
              "(kp in [%.0f, %.0f], kd in [%.0f, %.0f])",
              telemetry_node_->get_name(), KP_MIN, KP_MAX, KD_MIN, KD_MAX);
}

bool OpenArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }
  // Parse ee_type (default: parallel_link for v10)
  it = info.hardware_parameters.find("ee_type");
  ee_type_ =
      (it != info.hardware_parameters.end()) ? it->second : "parallel_link";
  if (hand_) {
    it = info.hardware_parameters.find("kp_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kp_ = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kd_ = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return true;
}

void OpenArmHW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Create internal ROS Node to publish topics telemetry
  std::string node_name = "openarm_hw_telemetry_" + (arm_prefix_.empty() ? "single" : arm_prefix_);
  if (!node_name.empty() && node_name.back() == '_') {
    node_name.pop_back(); //
  }
  telemetry_node_ = std::make_shared<rclcpp::Node>(node_name);

  // Create ros2 publisher
  pub_states_up_ = telemetry_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/openarm_hardware/" + arm_prefix_ + "states_up", 10);
  pub_cmds_down_ = telemetry_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/openarm_hardware/" + arm_prefix_ + "commands_down", 10);

  // Reference for the arm PID controller (payload / static-droop compensation).
  // arm_prefix_ is "left_" or "right_", giving the topic names the controllers
  // are configured under in openarm_bimanual_controllers.yaml.
  const std::string pid_reference_topic =
      "/" + arm_prefix_ + "arm_pid_controller/reference";
  pub_pid_reference_ =
      telemetry_node_->create_publisher<control_msgs::msg::MultiDOFCommand>(
          pid_reference_topic, 10);
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Publishing PID reference on %s (every %zu control cycles)",
              pid_reference_topic.c_str(), PID_REFERENCE_DECIMATION);

  // Expose kp1..kp7 / kd1..kd7 as runtime-settable ROS parameters. Done here,
  // after kp_/kd_ have been seeded from the xacro hardware_parameters, so the
  // declared defaults match what MIT mode starts with.
  setup_gain_parameters();

  //Gravity
  auto it_urdf = info.hardware_parameters.find("urdf_path");
  if (it_urdf != info.hardware_parameters.end()) {
      urdf_path_ = it_urdf->second;
  } else {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Invalid 'urdf_path' parameter in ros2_control xacro!");
      return CallbackReturn::ERROR;
  }

  std::string root_link = "openarm_body_link0";
  std::string leaf_link = "openarm_right_tcp";
  if (arm_prefix_ == "left_") {
      leaf_link = "openarm_left_tcp";
  } else if (arm_prefix_ == "right_") {
      leaf_link = "openarm_right_tcp";
  }
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing Dynamics with URDF: %s", urdf_path_.c_str());

  arm_dynamics_ = std::make_unique<Dynamics>(urdf_path_, root_link, leaf_link);
  if (!arm_dynamics_->Init()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Failed to initialize Dynamics KDL. Check URDF path!");
      return CallbackReturn::ERROR;
  }

  grav_torques_.resize(ARM_DOF, 0.0);

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  // Build the PID reference message once so write() only has to overwrite the
  // values. Arm joints only -- the gripper is not part of that controller.
  // values_dot stays empty: the controllers are configured with
  // reference_and_state_interfaces = [position], so a velocity reference would
  // be ignored and only risks a size mismatch.
  pid_reference_msg_.dof_names.assign(joint_names_.begin(),
                                      joint_names_.begin() + ARM_DOF);
  pid_reference_msg_.values.assign(ARM_DOF, 0.0);
  pid_reference_msg_.values_dot.clear();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Return to zero position
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  for (int i = 0; i < 3; ++i) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {

  //Gravity
  arm_dynamics_->GetGravity(pos_states_.data(), grav_torques_.data());

  // Snapshot the gains once under the lock into the preallocated buffers, then
  // use those -- a param-set on the spin thread cannot change kp/kd mid-loop,
  // and the RT loop holds the lock only for an ARM_DOF-double copy.
  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    for (size_t i = 0; i < ARM_DOF; ++i) {
      kp_now_[i] = kp_[i];
      kd_now_[i] = kd_[i];
    }
  }

  // One throttled line, all 7 joints hardcoded (single call site => the 10 s
  // throttle covers the whole line). kp_now_/kd_now_ is the snapshot above, so
  // this reflects live `ros2 param set kp<N>/kd<N>` changes.
  constexpr int GAIN_LOG_THROTTLE_MS = 10000;
  RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("OpenArmHW"), *telemetry_node_->get_clock(),
      GAIN_LOG_THROTTLE_MS,
      "[%s] j1[kp=%.2f kd=%.2f] j2[kp=%.2f kd=%.2f] j3[kp=%.2f kd=%.2f] "
      "j4[kp=%.2f kd=%.2f] j5[kp=%.2f kd=%.2f] j6[kp=%.2f kd=%.2f] "
      "j7[kp=%.2f kd=%.2f]",
      arm_prefix_.empty() ? "single" : arm_prefix_.c_str(),
      kp_now_[0], kd_now_[0], kp_now_[1], kd_now_[1], kp_now_[2], kd_now_[2],
      kp_now_[3], kd_now_[3], kp_now_[4], kd_now_[4], kp_now_[5], kd_now_[5],
      kp_now_[6], kd_now_[6]);

  std::vector<openarm::damiao_motor::MITParam> arm_params;

  std::vector<double> actual_tau_sent;

  for (size_t i = 0; i < ARM_DOF; ++i) {
    double total_tau_feedforward = tau_commands_[i] + grav_torques_[i];
    arm_params.push_back({
        kp_now_[i],
        kd_now_[i],
        pos_commands_[i],
        vel_commands_[i],
        total_tau_feedforward
    });
    actual_tau_sent.push_back(total_tau_feedforward);
  }

  openarm_->get_arm().mit_control_all(arm_params);

  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, motor_command, 0.0, 0.0}});

    actual_tau_sent.push_back(0.0);
  }
  openarm_->recv_all(100);

  publish_pid_reference();

  return hardware_interface::return_type::OK;
}

void OpenArmHW::publish_pid_reference() {
  // Decimate: the integral term does not need the full control rate, and this
  // runs inside write().
  if (++pid_reference_counter_ < PID_REFERENCE_DECIMATION) {
    return;
  }
  pid_reference_counter_ = 0;

  if (pub_pid_reference_ == nullptr) {                     // Rule 3: null guard
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArmHW"), *telemetry_node_->get_clock(),
                          PID_REFERENCE_LOG_THROTTLE_MS,
                          "[%s] PID reference publisher is null, not publishing",
                          arm_prefix_.empty() ? "single" : arm_prefix_.c_str());
    return;
  }

  if (pos_commands_.size() < ARM_DOF ||
      pid_reference_msg_.values.size() != ARM_DOF) {       // Rule 3: size guard
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArmHW"), *telemetry_node_->get_clock(),
                          PID_REFERENCE_LOG_THROTTLE_MS,
                          "[%s] %zu commands and %zu reference slots for %zu arm joints, "
                          "not publishing a PID reference",
                          arm_prefix_.empty() ? "single" : arm_prefix_.c_str(),
                          pos_commands_.size(), pid_reference_msg_.values.size(),
                          ARM_DOF);
    return;
  }

  for (size_t i = 0; i < ARM_DOF; ++i) {
    pid_reference_msg_.values[i] = pos_commands_[i];
  }
  pub_pid_reference_->publish(pid_reference_msg_);
}

void OpenArmHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero position...");

  openarm_->refresh_all();
  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
  const auto& arm_motors = openarm_->get_arm().get_motors();

  std::vector<double> start_pos(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    start_pos[i] = arm_motors[i].get_position();
  }

  const int steps = 200;
  const int step_ms = 10;

  for (int step = 0; step <= steps; ++step) {
    double t = static_cast<double>(step) / steps;  // 0.0 → 1.0

    std::vector<openarm::damiao_motor::MITParam> arm_params;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      double target = start_pos[i] + t * (ZERO_POSITION[i] - start_pos[i]);
      arm_params.push_back({kp_[i], kd_[i], target, 0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(arm_params);

    if (hand_) {
      openarm_->get_gripper().mit_control_all(
          {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
    }

    openarm_->recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Reached zero position");
}

double OpenArmHW::joint_to_motor_radians(double joint_value) {
  if (ee_type_ == "pinch_gripper") {
    // revolute: joint 0-1.5708 rad -> motor 0-1.5708
    return joint_value;
  } else {
    // parallel_link (prismatic): 0-0.044m -> 0 to -1.0472 rad
    return (joint_value / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS;
  }
}

double OpenArmHW::motor_radians_to_joint(double motor_radians) {
  if (ee_type_ == "pinch_gripper") {
    // revolute:
    return motor_radians;
  } else {
    // parallel_link (prismatic)
    return GRIPPER_JOINT_0_POSITION * (motor_radians / GRIPPER_MOTOR_1_RADIANS);
  }
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)

// // Copyright 2025 Enactic, Inc.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.
//
// #include "openarm_hardware/openarm_simple_hardware.hpp"
//
// #include <algorithm>
// #include <cctype>
// #include <chrono>
// #include <thread>
// #include <vector>
//
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/logging.hpp"
// #include "rclcpp/rclcpp.hpp"
//
// namespace openarm_hardware {
//
// OpenArmHW::OpenArmHW() = default;
//
// bool OpenArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
//   // Parse CAN interface (default: can0)
//   auto it = info.hardware_parameters.find("can_interface");
//   can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";
//
//   // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
//   // bimanual)
//   it = info.hardware_parameters.find("arm_prefix");
//   arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";
//
//   // Parse gripper enable (default: true for V10)
//   it = info.hardware_parameters.find("hand");
//   if (it == info.hardware_parameters.end()) {
//     hand_ = true;  // Default to true for V10
//   } else {
//     // Handle both "true"/"True" and "false"/"False"
//     std::string value = it->second;
//     std::transform(value.begin(), value.end(), value.begin(), ::tolower);
//     hand_ = (value == "true");
//   }
//
//   // Parse CAN-FD enable (default: true for V10)
//   it = info.hardware_parameters.find("can_fd");
//   if (it == info.hardware_parameters.end()) {
//     can_fd_ = true;  // Default to true for V10
//   } else {
//     // Handle both "true"/"True" and "false"/"False"
//     std::string value = it->second;
//     std::transform(value.begin(), value.end(), value.begin(), ::tolower);
//     can_fd_ = (value == "true");
//   }
//
//   // Parse control gains
//   for (size_t i = 1; i <= ARM_DOF; ++i) {
//     it = info.hardware_parameters.find("kp" + std::to_string(i));
//     if (it != info.hardware_parameters.end()) {
//       kp_[i - 1] = std::stod(it->second);
//     }
//     it = info.hardware_parameters.find("kd" + std::to_string(i));
//     if (it != info.hardware_parameters.end()) {
//       kd_[i - 1] = std::stod(it->second);
//     }
//   }
//   // Parse ee_type (default: parallel_link for v10)
//   it = info.hardware_parameters.find("ee_type");
//   ee_type_ =
//       (it != info.hardware_parameters.end()) ? it->second : "parallel_link";
//   if (hand_) {
//     it = info.hardware_parameters.find("kp_hand");
//     if (it != info.hardware_parameters.end()) {
//       gripper_kp_ = std::stod(it->second);
//     }
//     it = info.hardware_parameters.find("kd_hand");
//     if (it != info.hardware_parameters.end()) {
//       gripper_kd_ = std::stod(it->second);
//     }
//   }
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
//               "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
//               can_interface_.c_str(), arm_prefix_.c_str(),
//               hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
//   return true;
// }
//
// void OpenArmHW::generate_joint_names() {
//   joint_names_.clear();
//   // TODO: read from urdf properly and sort in the future.
//   // Currently, the joint names are hardcoded for order consistency to align
//   // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
//   for (size_t i = 1; i <= ARM_DOF; ++i) {
//     std::string joint_name =
//         "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
//     joint_names_.push_back(joint_name);
//   }
//
//   // Generate gripper joint name if enabled
//   if (hand_) {
//     std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
//     joint_names_.push_back(gripper_joint_name);
//     RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Added gripper joint: %s",
//                 gripper_joint_name.c_str());
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
//                 "Gripper joint NOT added because hand_=false");
//   }
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
//               "Generated %zu joint names for arm prefix '%s'",
//               joint_names_.size(), arm_prefix_.c_str());
// }
//
// hardware_interface::CallbackReturn OpenArmHW::on_init(
//     const hardware_interface::HardwareInfo& info) {
//   if (hardware_interface::SystemInterface::on_init(info) !=
//       CallbackReturn::SUCCESS) {
//     return CallbackReturn::ERROR;
//   }
//   // Parse configuration
//   if (!parse_config(info)) {
//     return CallbackReturn::ERROR;
//   }
//
//   // Generate joint names based on arm prefix
//   generate_joint_names();
//
//   //Gravity
//   auto it_urdf = info.hardware_parameters.find("urdf_path");
//   if (it_urdf != info.hardware_parameters.end()) {
//       urdf_path_ = it_urdf->second;
//   } else {
//       RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Invalid 'urdf_path' parameter in ros2_control xacro!");
//       return CallbackReturn::ERROR;
//   }
//
//   std::string root_link = "openarm_body_link0";
//   std::string leaf_link = "openarm_right_hand";
//   if (arm_prefix_ == "left_") {
//       leaf_link = "openarm_left_hand";
//   } else if (arm_prefix_ == "right_") {
//       leaf_link = "openarm_right_hand";
//   }
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing Dynamics with URDF: %s", urdf_path_.c_str());
//
//   arm_dynamics_ = std::make_unique<Dynamics>(urdf_path_, root_link, leaf_link);
//   if (!arm_dynamics_->Init()) {
//       RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Failed to initialize Dynamics KDL. Check URDF path!");
//       return CallbackReturn::ERROR;
//   }
//
//   grav_torques_.resize(ARM_DOF, 0.0);
//
//   // Validate joint count (7 arm joints + optional gripper)
//   size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
//   if (joint_names_.size() != expected_joints) {
//     RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
//                  "Generated %zu joint names, expected %zu", joint_names_.size(),
//                  expected_joints);
//     return CallbackReturn::ERROR;
//   }
//
//   // Initialize OpenArm with configurable CAN-FD setting
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
//               "Initializing OpenArm on %s with CAN-FD %s...",
//               can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
//   openarm_ =
//       std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);
//
//   // Initialize arm motors with V10 defaults
//   openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
//                             DEFAULT_RECV_CAN_IDS);
//
//   // Initialize gripper if enabled
//   if (hand_) {
//     RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing gripper...");
//     openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
//                                  DEFAULT_GRIPPER_SEND_CAN_ID,
//                                  DEFAULT_GRIPPER_RECV_CAN_ID);
//   }
//
//   // Initialize state and command vectors based on generated joint count
//   const size_t total_joints = joint_names_.size();
//   pos_commands_.resize(total_joints, 0.0);
//   vel_commands_.resize(total_joints, 0.0);
//   tau_commands_.resize(total_joints, 0.0);
//   pos_states_.resize(total_joints, 0.0);
//   vel_states_.resize(total_joints, 0.0);
//   tau_states_.resize(total_joints, 0.0);
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
//               "OpenArm V10 Simple HW initialized successfully");
//
//   return CallbackReturn::SUCCESS;
// }
//
// hardware_interface::CallbackReturn OpenArmHW::on_configure(
//     const rclcpp_lifecycle::State& /*previous_state*/) {
//   // Set callback mode to ignore during configuration
//   openarm_->refresh_all();
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   openarm_->recv_all();
//
//   return CallbackReturn::SUCCESS;
// }
//
// std::vector<hardware_interface::StateInterface>
// OpenArmHW::export_state_interfaces() {
//   std::vector<hardware_interface::StateInterface> state_interfaces;
//   for (size_t i = 0; i < joint_names_.size(); ++i) {
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//         joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//         joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//         joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
//   }
//
//   return state_interfaces;
// }
//
// std::vector<hardware_interface::CommandInterface>
// OpenArmHW::export_command_interfaces() {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;
//   // TODO: consider exposing only needed interfaces to avoid undefined behavior.
//   for (size_t i = 0; i < joint_names_.size(); ++i) {
//     command_interfaces.emplace_back(hardware_interface::CommandInterface(
//         joint_names_[i], hardware_interface::HW_IF_POSITION,
//         &pos_commands_[i]));
//     command_interfaces.emplace_back(hardware_interface::CommandInterface(
//         joint_names_[i], hardware_interface::HW_IF_VELOCITY,
//         &vel_commands_[i]));
//     command_interfaces.emplace_back(hardware_interface::CommandInterface(
//         joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
//   }
//
//   return command_interfaces;
// }
//
// hardware_interface::CallbackReturn OpenArmHW::on_activate(
//     const rclcpp_lifecycle::State& /*previous_state*/) {
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Activating OpenArm V10...");
//   openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
//   openarm_->enable_all();
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   openarm_->recv_all();
//
//   // Return to zero position
//   return_to_zero();
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 activated");
//   return CallbackReturn::SUCCESS;
// }
//
// hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
//     const rclcpp_lifecycle::State& /*previous_state*/) {
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Deactivating OpenArm V10...");
//
//   // Disable all motors (like full_arm.cpp exit)
//   for (int i = 0; i < 3; ++i) {
//     openarm_->disable_all();
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     openarm_->recv_all();
//   }
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 deactivated");
//   return CallbackReturn::SUCCESS;
// }
//
// hardware_interface::return_type OpenArmHW::read(
//     const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
//   // Receive all motor states
//   openarm_->refresh_all();
//   openarm_->recv_all();
//
//   // Read arm joint states
//   const auto& arm_motors = openarm_->get_arm().get_motors();
//   for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
//     pos_states_[i] = arm_motors[i].get_position();
//     vel_states_[i] = arm_motors[i].get_velocity();
//     tau_states_[i] = arm_motors[i].get_torque();
//   }
//
//   // Read gripper state if enabled
//   if (hand_ && joint_names_.size() > ARM_DOF) {
//     const auto& gripper_motors = openarm_->get_gripper().get_motors();
//     if (!gripper_motors.empty()) {
//       // TODO the mappings are approximates
//       // Convert motor position (radians) to joint value (0-0.044m)
//       double motor_pos = gripper_motors[0].get_position();
//       pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);
//
//       // Unimplemented: Velocity and torque mapping
//       vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
//       tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
//     }
//   }
//
//   return hardware_interface::return_type::OK;
// }
//
// hardware_interface::return_type OpenArmHW::write(
//     const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
//
//   //Gravity
//   arm_dynamics_->GetGravity(pos_states_.data(), grav_torques_.data());
//
//   std::vector<openarm::damiao_motor::MITParam> arm_params;
//   for (size_t i = 0; i < ARM_DOF; ++i) {
//     double total_tau_feedforward = tau_commands_[i] + grav_torques_[i];
//     arm_params.push_back({
//         kp_[i],
//         kd_[i],
//         pos_commands_[i],
//         vel_commands_[i],
//         total_tau_feedforward
//     });
//   }
//
//   // Control arm motors with MIT control
// //   std::vector<openarm::damiao_motor::MITParam> arm_params;
// //   for (size_t i = 0; i < ARM_DOF; ++i) {
// //     arm_params.push_back(
// //         {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]});
// //   }
//
//   openarm_->get_arm().mit_control_all(arm_params);
//   // Control gripper if enabled
//   if (hand_ && joint_names_.size() > ARM_DOF) {
//     // TODO the true mappings are unimplemented.
//     double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
//     openarm_->get_gripper().mit_control_all(
//         {{gripper_kp_, gripper_kd_, motor_command, 0.0, 0.0}});
//   }
//   openarm_->recv_all(100);
//   return hardware_interface::return_type::OK;
// }
//
// void OpenArmHW::return_to_zero() {
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero position...");
//
//   openarm_->refresh_all();
//   // Return arm to zero with MIT control
//   std::vector<openarm::damiao_motor::MITParam> arm_params;
//   for (size_t i = 0; i < ARM_DOF; ++i) {
//     arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
//   }
//   openarm_->get_arm().mit_control_all(arm_params);
//
//   // Return gripper to zero if enabled
//   if (hand_) {
//     openarm_->get_gripper().mit_control_all(
//         {{gripper_kp_, gripper_kd_, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
//   }
//   std::this_thread::sleep_for(std::chrono::microseconds(1000));
//   openarm_->recv_all();
//   const auto& arm_motors = openarm_->get_arm().get_motors();
//
//   std::vector<double> start_pos(ARM_DOF, 0.0);
//   for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
//     start_pos[i] = arm_motors[i].get_position();
//   }
//
//   const int steps = 200;
//   const int step_ms = 10;
//
//   for (int step = 0; step <= steps; ++step) {
//     double t = static_cast<double>(step) / steps;  // 0.0 → 1.0
//
//     std::vector<openarm::damiao_motor::MITParam> arm_params;
//     for (size_t i = 0; i < ARM_DOF; ++i) {
//       double target = start_pos[i] + t * (ZERO_POSITION[i] - start_pos[i]);
//       arm_params.push_back({kp_[i], kd_[i], target, 0.0, 0.0});
//     }
//     openarm_->get_arm().mit_control_all(arm_params);
//
//     if (hand_) {
//       openarm_->get_gripper().mit_control_all(
//           {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
//     }
//
//     openarm_->recv_all();
//     std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
//   }
//
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Reached zero position");
// }
//
// // void OpenArmHW::return_to_zero() {
// //   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero
// //   position...");
//
// //   // Return arm to zero with MIT control
// //   std::vector<openarm::damiao_motor::MITParam> arm_params;
// //   for (size_t i = 0; i < ARM_DOF; ++i) {
// //     arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
// //   }
// //   openarm_->get_arm().mit_control_all(arm_params);
//
// //   // Return gripper to zero if enabled
// //   if (hand_) {
// //     openarm_->get_gripper().mit_control_all(
// //         {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
// //   }
// //   std::this_thread::sleep_for(std::chrono::microseconds(1000));
// //   openarm_->recv_all();
// // }
//
// double OpenArmHW::joint_to_motor_radians(double joint_value) {
//   if (ee_type_ == "pinch_gripper") {
//     // revolute: joint 0-1.5708 rad -> motor 0-1.5708
//     return joint_value;
//   } else {
//     // parallel_link (prismatic): 0-0.044m -> 0 to -1.0472 rad
//     return (joint_value / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS;
//   }
// }
//
// double OpenArmHW::motor_radians_to_joint(double motor_radians) {
//   if (ee_type_ == "pinch_gripper") {
//     // revolute:
//     return motor_radians;
//   } else {
//     // parallel_link (prismatic)
//     return GRIPPER_JOINT_0_POSITION * (motor_radians / GRIPPER_MOTOR_1_RADIANS);
//   }
// }
//
// // // Gripper mapping helper functions
// // double OpenArmHW::joint_to_motor_radians(double joint_value) {
// //   // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
// //   return (joint_value / GRIPPER_JOINT_0_POSITION) *
// //          GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
// // }
//
// // double OpenArmHW::motor_radians_to_joint(double motor_radians) {
// //   // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
// //   return GRIPPER_JOINT_0_POSITION *
// //          (motor_radians /
// //           GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
// // }
//
// }  // namespace openarm_hardware
//
// #include "pluginlib/class_list_macros.hpp"
//
// PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
//                        hardware_interface::SystemInterface)
