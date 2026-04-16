#include "touch_controllers/null_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace touch_controllers
{

void NullController::reset(const touch_hardware::DeviceState &)
{
}

touch_hardware::ForceCommand NullController::compute_force(
  const touch_hardware::DeviceState &,
  const touch_hardware::CommandState &,
  double)
{
  return {};
}

}  // namespace touch_controllers

PLUGINLIB_EXPORT_CLASS(touch_controllers::NullController, touch_hardware::TouchController)
