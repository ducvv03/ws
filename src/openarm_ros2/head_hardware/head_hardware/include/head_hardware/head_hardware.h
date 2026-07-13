#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace head_hardware {

class HeadHw : public hardware_interface::SystemInterface {
 public:
  using CanId = std::uint32_t;

  HeadHw() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  enum Direction { VERTICAL, HORIZONTAL };

  // Parameter keys as declared in the <ros2_control> xacro <param name="...">.
  static constexpr const char* PARAM_CAN_INTERFACE = "can_interface";
  static constexpr const char* PARAM_DIRECTION = "direction";
  static constexpr const char* PARAM_CAN_ID = "can_id";
  static constexpr const char* PARAM_MOTOR_TYPE = "motor_type";
  static constexpr const char* PARAM_CAN_FD = "can_fd";

  // Offset added to the send CAN id to obtain the motor's receive CAN id,
  // matching the openarm_hardware convention (send 0x0N -> recv 0x1N).
  static constexpr CanId RECV_CAN_ID_OFFSET = 0x10;

  // Hardcoded MIT position-control gains for the head's DM4340 motor. Fixed here
  // (not exposed as params) because the head runs a single motor type in P mode.
  // Kept intentionally soft for bring-up -- raise if the joint feels too weak.
  static constexpr double DM4340_KP = 20.0;
  static constexpr double DM4340_KD = 0.5;

  // Travel (profile) velocity for POS_VEL control in write(), in rad/s. The
  // forward position controller streams position only, so vel_cmd_ stays 0 and
  // a POS_VEL frame with velocity 0 would never move -- fall back to this speed.
  // If a controller actually commands velocity, that value is used instead.
  static constexpr double PROFILE_VELOCITY = 0.5;

  // Defaults used when a param is absent.
  static constexpr const char* DEFAULT_CAN_INTERFACE = "can0";
  static constexpr CanId DEFAULT_CAN_ID = 0x21;
  static constexpr openarm::damiao_motor::MotorType DEFAULT_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4340;

  static constexpr double ZERO_POINT = 0.0;

  // Look up a hardware parameter, returning `fallback` if it is not present.
  static std::string get_param(const hardware_interface::HardwareInfo& info,
                               const std::string& key,
                               const std::string& fallback) {
    const auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) {
      return fallback;
    }
    return it->second;
  }

  bool parse_config(const hardware_interface::HardwareInfo& info);
  int return_to_zero();

  openarm::damiao_motor::MotorType motor_type_;
  CanId can_id_;
  std::string joint_name_;
  std::string can_interface_;
  bool can_fd_ = true;
  Direction dir_;
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  double pos_states_ = 0.0;
  double vel_states_ = 0.0;
  double tau_states_ = 0.0;
  double pos_cmd_ = 0.0;
  double vel_cmd_ = 0.0;
  double tau_cmd_ = 0.0;
};

}  // namespace head_hardware
