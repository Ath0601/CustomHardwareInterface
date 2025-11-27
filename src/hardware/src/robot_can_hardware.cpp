#include "hardware/robot_can_hardware.hpp"

namespace robot_can_hardware
{

RobotCANHardware::RobotCANHardware()
: logger_(rclcpp::get_logger("RobotCANHardware"))
{
}

hardware_interface::CallbackReturn RobotCANHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  dof_ = info.joints.size();
  joint_names_.reserve(dof_);

  hw_positions_.assign(dof_, 0.0);
  hw_velocities_.assign(dof_, 0.0);
  hw_commands_.assign(dof_, 0.0);

  for (const auto & joint : info.joints)
    joint_names_.push_back(joint.name);

  // initialize fake CAN backend
  fake_can_ = std::make_shared<fakecan::FakeCANInterface>();
  if (!fake_can_->init(static_cast<int>(dof_))) {
    RCLCPP_ERROR(logger_, "Failed to init FakeCAN backend");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initialized FakeCAN with %zu joints", dof_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotCANHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotCANHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");

  // Read current positions from fakecan and set initial commands = positions
  // This avoids large jumps when controllers start.
  for (size_t i = 0; i < dof_; ++i) {
    double pos = 0.0;
    double vel = 0.0;
    // If reading fails, keep default zero but warn
    if (!fake_can_->read_position(static_cast<int>(i), pos)) {
      RCLCPP_WARN(logger_, "FakeCAN read_position failed for joint %zu", i);
    }
    if (!fake_can_->read_velocity(static_cast<int>(i), vel)) {
      // not fatal, just leave zero velocity
    }

    hw_positions_[i] = pos;
    hw_velocities_[i] = vel;
    hw_commands_[i] = pos; // initialize command equal to current position
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotCANHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotCANHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < dof_; ++i)
  {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotCANHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < dof_; ++i)
  {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  return interfaces;
}

hardware_interface::return_type RobotCANHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < dof_; ++i)
  {
    double pos = 0.0;
    double vel = 0.0;

    fake_can_->read_position(static_cast<int>(i), pos);
    fake_can_->read_velocity(static_cast<int>(i), vel);

    hw_positions_[i] = pos;
    hw_velocities_[i] = vel;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type RobotCANHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0)
    dt = 0.001;

  for (size_t i = 0; i < dof_; ++i)
  {
    fake_can_->write_command(static_cast<int>(i), hw_commands_[i]);
  }

  fake_can_->update(dt);

  return hardware_interface::return_type::OK;
}


} // namespace robot_can_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_can_hardware::RobotCANHardware, hardware_interface::SystemInterface)
