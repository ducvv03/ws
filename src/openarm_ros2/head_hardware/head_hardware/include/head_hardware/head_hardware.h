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

  void parse_config(const hardware_interface::HardwareInfo& info);
  int return_to_zero();

  static constexpr double ZERO_POINT = 0.0;

  openarm::damiao_motor::MotorType motor_type_;
  CanId can_id_;
  std::string joint_name_;
  std::string can_interface_;
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
