#include "touch_controllers/impedance_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace touch_controllers
{
namespace
{

Eigen::Vector3d to_eigen(const std::array<double, 3> & value)
{
  return Eigen::Vector3d(value[0], value[1], value[2]);
}

std::array<double, 3> to_array(const Eigen::Vector3d & value)
{
  return {value.x(), value.y(), value.z()};
}

}  // namespace

void ImpedanceController::reset(const touch_hardware::DeviceState & state)
{
  target_position_ = to_eigen(state.position_m);
  target_initialized_ = true;
}

touch_hardware::ForceCommand ImpedanceController::compute_force(
  const touch_hardware::DeviceState & state,
  const touch_hardware::CommandState & command,
  double)
{
  if (command.target_pose_valid) {
    target_position_ = to_eigen(command.target_position_m);
    target_initialized_ = true;
  }

  if (!target_initialized_) {
    reset(state);
  }

  const Eigen::Vector3d position = to_eigen(state.position_m);
  const Eigen::Vector3d velocity = to_eigen(state.linear_velocity_m_s);
  const Eigen::Vector3d stiffness = to_eigen(command.impedance_stiffness);
  const Eigen::Vector3d damping = to_eigen(command.impedance_damping);

  const Eigen::Vector3d force =
    stiffness.asDiagonal() * (target_position_ - position) - damping.asDiagonal() * velocity;

  touch_hardware::ForceCommand output;
  output.force_n = to_array(force);
  return output;
}

}  // namespace touch_controllers

PLUGINLIB_EXPORT_CLASS(touch_controllers::ImpedanceController, touch_hardware::TouchController)
