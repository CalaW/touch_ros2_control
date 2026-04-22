#ifndef TOUCH_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
#define TOUCH_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_

#include "touch_hardware/controller_base.hpp"

#include <Eigen/Core>

namespace touch_controllers
{

class ImpedanceController : public touch_hardware::TouchController
{
public:
  void reset(const touch_hardware::RawDeviceState & state) override;

  touch_hardware::ForceCommand compute_force(
    const touch_hardware::RawDeviceState & state,
    const touch_hardware::CommandState & command,
    double dt) override;

private:
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  bool target_initialized_{false};
};

}  // namespace touch_controllers

#endif  // TOUCH_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
