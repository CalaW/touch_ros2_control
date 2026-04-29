#ifndef TOUCH_HARDWARE__IMPEDANCE_MATH_HPP_
#define TOUCH_HARDWARE__IMPEDANCE_MATH_HPP_

#include "touch_hardware/types.hpp"

#include <array>
#include <cstddef>

namespace touch_hardware
{

constexpr double kMillimetersToMeters = 1e-3;

inline std::array<double, 3> raw_position_m(const RawDeviceState & state)
{
  return {
    state.transform[12] * kMillimetersToMeters,
    state.transform[13] * kMillimetersToMeters,
    state.transform[14] * kMillimetersToMeters,
  };
}

inline std::array<double, 3> raw_velocity_m_s(const RawDeviceState & state)
{
  return {
    state.velocity_mm_s[0] * kMillimetersToMeters,
    state.velocity_mm_s[1] * kMillimetersToMeters,
    state.velocity_mm_s[2] * kMillimetersToMeters,
  };
}

inline ForceCommand compute_impedance_force_device(
  const RawDeviceState & state, const CommandState & command)
{
  const auto position = raw_position_m(state);
  const auto velocity = raw_velocity_m_s(state);

  ForceCommand output;
  for (std::size_t i = 0; i < output.device_force_n.size(); ++i) {
    output.device_force_n[i] =
      command.device_force_n[i] +
      command.impedance_stiffness[i] * (command.target_position_m[i] - position[i]) -
      command.impedance_damping[i] * velocity[i];
  }

  return output;
}

}  // namespace touch_hardware

#endif  // TOUCH_HARDWARE__IMPEDANCE_MATH_HPP_
