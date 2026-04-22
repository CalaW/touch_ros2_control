#ifndef TOUCH_CONTROLLERS__DIRECT_FORCE_CONTROLLER_HPP_
#define TOUCH_CONTROLLERS__DIRECT_FORCE_CONTROLLER_HPP_

#include "touch_hardware/controller_base.hpp"

namespace touch_controllers
{

class DirectForceController : public touch_hardware::TouchController
{
public:
  void reset(const touch_hardware::RawDeviceState & state) override;

  touch_hardware::ForceCommand compute_force(
    const touch_hardware::RawDeviceState & state,
    const touch_hardware::CommandState & command,
    double dt) override;
};

}  // namespace touch_controllers

#endif  // TOUCH_CONTROLLERS__DIRECT_FORCE_CONTROLLER_HPP_
