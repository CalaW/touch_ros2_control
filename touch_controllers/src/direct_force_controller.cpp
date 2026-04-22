#include "touch_controllers/direct_force_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace touch_controllers
{

void DirectForceController::reset(const touch_hardware::RawDeviceState &)
{
}

touch_hardware::ForceCommand DirectForceController::compute_force(
  const touch_hardware::RawDeviceState &,
  const touch_hardware::CommandState & command,
  double)
{
  touch_hardware::ForceCommand output;
  output.device_force_n = command.device_force_n;
  return output;
}

}  // namespace touch_controllers

PLUGINLIB_EXPORT_CLASS(touch_controllers::DirectForceController, touch_hardware::TouchController)
