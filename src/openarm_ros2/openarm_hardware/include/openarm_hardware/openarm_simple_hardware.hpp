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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <controller/dynamics.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArmHW : public hardware_interface::SystemInterface {
 public:
  OpenArmHW();
  ~OpenArmHW() override;

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

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:

  rclcpp::Node::SharedPtr telemetry_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_states_up_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmds_down_;

  // --- Arm PID controller reference ------------------------------------------
  // pos_commands_ is the setpoint whichever controller is currently active just
  // wrote, so mirroring it from write() gives left/right_arm_pid_controller a
  // reference that cannot disagree with what MIT mode is actually tracking --
  // no separate publisher in the teleop bridge or UDP server to keep in step,
  // and it works the same whether the arm is driven over a topic or an action.
  //
  // Published on every control cycle (decimation 1 => the full 750 Hz). The
  // message is built once in on_init() so the hot path only overwrites `values`
  // and publishes -- no allocation. Raise this if write() starts overrunning:
  // 2 gives 375 Hz, 3 gives 250 Hz, and the integral term does not need the
  // full rate. It matters more if the d gain is ever raised, since d
  // differentiates the reference and a decimated one arrives as a staircase.
  static constexpr size_t PID_REFERENCE_DECIMATION = 1;  // 750 Hz
  static constexpr int PID_REFERENCE_LOG_THROTTLE_MS = 2000;
  size_t pid_reference_counter_ = 0;
  control_msgs::msg::MultiDOFCommand pid_reference_msg_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr pub_pid_reference_;

  /// Mirrors pos_commands_ onto the arm PID controller's reference topic.
  /// Called from write(); decimated by PID_REFERENCE_DECIMATION.
  void publish_pid_reference();
  
  // Gravity
  std::unique_ptr<Dynamics> arm_dynamics_;
  std::vector<double> grav_torques_;
  std::string urdf_path_;

  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  // --- MIT-mode PD gains, runtime-tunable ------------------------------------
  // These are the kp/kd the firmware PD loop uses:
  //     tau_motor = kp*(q_d - q) + kd*(qd_d - qd) + t_ff
  // Seeded from the xacro hardware_parameters (kp1..kp7 / kd1..kd7) in on_init,
  // then exposed as ROS parameters on telemetry_node_ so they can be overwritten
  // live:
  //     ros2 param set /openarm_hw_telemetry[_<prefix>] kp2 90.0
  // A param-set runs on the telemetry node's spin thread; write() runs on the
  // controller_manager RT thread. gains_mutex_ guards the handoff -- the critical
  // section is a copy of ARM_DOF doubles, so the lock is uncontended and cheap
  // at 750 Hz, and only ever contended for the microseconds of a param update.
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

  // Damiao MIT protocol packs kp into 12 bits over [0, 500] and kd over [0, 5];
  // values outside these ranges are rejected rather than silently clipped.
  static constexpr double KP_MIN = 0.0;
  static constexpr double KP_MAX = 500.0;
  static constexpr double KD_MIN = 0.0;
  static constexpr double KD_MAX = 5.0;

  std::mutex gains_mutex_;                       // guards kp_/kd_
  // Preallocated snapshot buffers write() copies the gains into under the lock,
  // so the RT loop uses a stable set for the whole cycle without touching the
  // stack each call.
  std::array<double, ARM_DOF> kp_now_{};
  std::array<double, ARM_DOF> kd_now_{};
  std::thread param_spin_thread_;                // services telemetry_node_ params
  std::atomic<bool> param_spin_running_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  /// Declares kp1..kp7 / kd1..kd7 on telemetry_node_ (seeded from kp_/kd_),
  /// installs the validating on-set callback, and starts the spin thread.
  void setup_gain_parameters();

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_KP = 5.0;
  const double GRIPPER_KD = 0.1;

  double gripper_kp_ = GRIPPER_KP;
  double gripper_kd_ = GRIPPER_KD;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  std::string ee_type_;
  bool hand_;
  bool can_fd_;

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  static constexpr std::array<double, ARM_DOF> ZERO_POSITION = {
      0.0,  // joint1
      0.0,  // joint2
      0.0,  // joint3
      0.0,  // joint4
      0.0,  // joint5
      0.0,  // joint6
      0.0,  // joint7
  };

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
