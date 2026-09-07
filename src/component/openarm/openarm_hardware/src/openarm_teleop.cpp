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
#include <string>
#include <yamlloader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <execinfo.h>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace openarm_hardware_teleop ;

namespace {
constexpr const char* kLogger = "OpenArmHWTeleOp";

// Signal handler: dump a backtrace to stderr when the process crashes, then
// re-raise with the default handler so the OS still produces a core / exit code.
// Uses only async-signal-safe calls (backtrace / backtrace_symbols_fd / write).
void crash_handler(int sig) {
    void* frames[64];
    int n = backtrace(frames, 64);

    const char* msg = "\n=== [OpenArmHWTeleOp] caught fatal signal, backtrace: ===\n";
    ssize_t rc = write(STDERR_FILENO, msg, strlen(msg));
    (void)rc;

    backtrace_symbols_fd(frames, n, STDERR_FILENO);

    signal(sig, SIG_DFL);
    raise(sig);
}

void install_crash_handler() {
    static bool installed = false;
    if (installed) return;
    installed = true;
    // Unbuffer stdout/stderr so std::cout / printf appear in the terminal
    // immediately (ros2 launch pipes are block-buffered; output is otherwise
    // delayed and lost when the process crashes).
    // setvbuf(stdout, nullptr, _IONBF, 0);
    // setvbuf(stderr, nullptr, _IONBF, 0);
    signal(SIGSEGV, crash_handler);  // invalid memory access
    signal(SIGABRT, crash_handler);  // abort() / uncaught exception / assert
    signal(SIGFPE, crash_handler);   // div by zero, etc.
    signal(SIGBUS, crash_handler);   // bad memory alignment / mapping
}
}

hardware_interface::CallbackReturn OpenArmHWTeleOp::on_init(const hardware_interface::HardwareInfo& info)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    install_crash_handler();
    if(OpenArmHW::on_init(info) !=  CallbackReturn::SUCCESS )
    {
        return CallbackReturn::ERROR;
    }
    std::string leader_urdf_path = urdf_path_ ;
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "[ROS_2][HARDWARE_INTERFACE][OPA_TELEOP] teleOp on_init");
            // Leader parameters
    // Resolve the config path from the installed package share dir instead of a
    // relative path (the node's working directory is not the package folder).
    const std::string leader_config =
        ament_index_cpp::get_package_share_directory("openarm_hardware") + "/config/leader.yaml";
    YamlLoader leader_loader(leader_config);
    std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
    std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
    std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
    std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
    std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
    std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");
    std::string root_link = "openarm_body_link0";
    std::string leaf_link = (arm_prefix_ == "left_") ? "openarm_left_link7" : "openarm_right_link7";
    size_t leader_arm_motor_num = openarm_->get_arm().get_motors().size();
    size_t leader_hand_motor_num = openarm_->get_gripper().get_motors().size();
    std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
    std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
    Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
    // Must call Init() to actually parse the URDF and build the KDL chain +
    // solver. Without it the chain has 0 joints and solver is null -> GetGravity
    // dereferences a null solver and crashes (SIGSEGV).
    if (!leader_arm_dynamics->Init()) {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                     "Failed to init Dynamics (urdf=%s, chain %s -> %s)",
                     leader_urdf_path.c_str(), root_link.c_str(), leaf_link.c_str());
        return CallbackReturn::ERROR;
    }
    std::shared_ptr<RobotSystemState> leader_state =
            std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

    control_ = std::make_unique<Control>(
        openarm_.get(), leader_arm_dynamics, leader_arm_dynamics, leader_state,
        1.0 / 500.0, ROLE_LEADER, arm_prefix_,
        leader_arm_motor_num, leader_hand_motor_num,
        pos_states_, vel_states_, tau_states_);

    control_->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv, leader_Fo);

    return CallbackReturn::SUCCESS ;
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
//   if (hand_ && joint_names_.size() > ARM_DOF) {
//     const auto& gripper_motors = openarm_->get_gripper().get_motors();
//     if (!gripper_motors.empty()) {
//       // TODO the mappings are approximates
//       // Convert motor position (radians) to joint value (0-0.044m)
//       double motor_pos = gripper_motors[0].get_position();
//       pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

//       // Unimplemented: Velocity and torque mapping
//       vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
//       tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
//     }
//  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHWTeleOp::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
        //control_->toSim_step();
        control_->toSim_unilateral_step();
  return hardware_interface::return_type::OK;
}

  // namespace openarm_hardware_teleop

PLUGINLIB_EXPORT_CLASS(openarm_hardware_teleop::OpenArmHWTeleOp,
                       hardware_interface::SystemInterface)
