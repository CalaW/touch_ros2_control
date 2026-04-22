#ifndef TOUCH_HARDWARE__CONTROLLER_BASE_HPP_
#define TOUCH_HARDWARE__CONTROLLER_BASE_HPP_

#include <array>

namespace touch_hardware
{

struct RawDeviceState
{
  std::array<double, 3> velocity_mm_s{0.0, 0.0, 0.0};
  std::array<double, 16> transform{};
  std::array<double, 3> arm_joint_angles_rad{0.0, 0.0, 0.0};
  std::array<double, 3> gimbal_angles_rad{0.0, 0.0, 0.0};
};

struct DeviceState
{
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity_m_s{0.0, 0.0, 0.0};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 6> joint_positions_rad{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> joint_velocities_rad_s{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct CommandState
{
  std::array<double, 3> device_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> target_position_m{0.0, 0.0, 0.0};
  std::array<double, 3> impedance_stiffness{45.0, 45.0, 45.0};
  std::array<double, 3> impedance_damping{2.5, 2.5, 2.5};
  bool target_pose_valid{false};
};

struct ForceCommand
{
  std::array<double, 3> device_force_n{0.0, 0.0, 0.0};
};

class TouchController
{
public:
  virtual ~TouchController();

  virtual void reset(const RawDeviceState & state) = 0;

  virtual ForceCommand compute_force(
    const RawDeviceState & state, const CommandState & command, double dt) = 0;
};

}  // namespace touch_hardware

#endif  // TOUCH_HARDWARE__CONTROLLER_BASE_HPP_
