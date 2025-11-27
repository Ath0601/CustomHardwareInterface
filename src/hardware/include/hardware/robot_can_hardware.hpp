#pragma once

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"

#include "fakecan/fakecan.hpp"

namespace robot_can_hardware
{

class RobotCANHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotCANHardware)

  RobotCANHardware();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_;

  // DOF and joint data
  size_t dof_;
  std::vector<std::string> joint_names_;

  // state
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // commands
  std::vector<double> hw_commands_;

  // FakeCAN backend
  std::shared_ptr<fakecan::FakeCANInterface> fake_can_;
};

} // namespace robot_can_hardware
