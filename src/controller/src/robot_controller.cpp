#include "controller/robot_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_controller
{

controller_interface::CallbackReturn RobotController::on_init()
{
  num_joints_ = 7;
  cmd_positions_.resize(num_joints_, 0.0);
  state_positions_.resize(num_joints_, 0.0);
  state_velocities_.resize(num_joints_, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(cmd_positions_[i]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cfg.names.clear();
  for (size_t i = 0; i < num_joints_; ++i) {
    cfg.names.push_back("joint_" + std::to_string(i+1) + "/position");
  }

  return cfg;
}

controller_interface::InterfaceConfiguration
RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cfg.names.clear();
  for (size_t i = 0; i < num_joints_; ++i) {
    cfg.names.push_back("joint_" + std::to_string(i+1) + "/position");
    cfg.names.push_back("joint_" + std::to_string(i+1) + "/velocity");
  }

  return cfg;
}

}  // namespace robot_controller

PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController, controller_interface::ControllerInterface)
