#ifndef TOUCH_HARDWARE__DEVICE_TRANSFORMS_HPP_
#define TOUCH_HARDWARE__DEVICE_TRANSFORMS_HPP_

#include "touch_hardware/controller_base.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cstddef>

namespace touch_hardware
{

namespace detail
{

constexpr double kMillimetersToMeters = 1e-3;

inline const Eigen::Matrix3d & device_to_ros_basis()
{
  // clang-format off
  static const Eigen::Matrix3d basis = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0).finished();
  // clang-format on
  return basis;
}

}  // namespace detail

inline std::array<double, 3> transform_vector_device_to_ros(const std::array<double, 3> & value)
{
  const Eigen::Vector3d device(value[0], value[1], value[2]);
  const Eigen::Vector3d ros = detail::device_to_ros_basis() * device;
  return {ros.x(), ros.y(), ros.z()};
}

inline std::array<double, 3> transform_vector_ros_to_device(const std::array<double, 3> & value)
{
  const Eigen::Vector3d ros(value[0], value[1], value[2]);
  const Eigen::Vector3d device = detail::device_to_ros_basis().transpose() * ros;
  return {device.x(), device.y(), device.z()};
}

inline void device_transform_to_ros_pose(
  const std::array<double, 16> & transform, std::array<double, 3> & position_m,
  std::array<double, 4> & orientation_xyzw)
{
  const Eigen::Vector3d device_position_mm(transform[12], transform[13], transform[14]);
  const Eigen::Vector3d ros_position_m =
    detail::device_to_ros_basis() * device_position_mm * detail::kMillimetersToMeters;

  // clang-format off
  Eigen::Matrix3d rotation_device;
  rotation_device <<
    transform[0], transform[1], transform[2],
    transform[4], transform[5], transform[6],
    transform[8], transform[9], transform[10];
  // clang-format on

  const Eigen::Matrix3d rotation_ros =
    detail::device_to_ros_basis() * rotation_device.transpose() *
    detail::device_to_ros_basis().transpose();
  Eigen::Quaterniond ros_orientation(rotation_ros);
  ros_orientation.normalize();

  position_m = {ros_position_m.x(), ros_position_m.y(), ros_position_m.z()};
  orientation_xyzw = {
    ros_orientation.x(),
    ros_orientation.y(),
    ros_orientation.z(),
    ros_orientation.w(),
  };
}

inline std::array<double, 6> calculate_joint_positions_rad(const RawDeviceState & state)
{
  return {
    state.arm_joint_angles_rad[0],
    state.arm_joint_angles_rad[1],
    state.arm_joint_angles_rad[2] - state.arm_joint_angles_rad[1],
    state.gimbal_angles_rad[0],
    state.gimbal_angles_rad[1],
    state.gimbal_angles_rad[2],
  };
}

inline std::array<double, 6> calculate_joint_velocities_rad_s(
  const std::array<double, 6> & current_joint_positions,
  const std::array<double, 6> & previous_joint_positions, double dt)
{
  std::array<double, 6> joint_velocities{};
  if (dt <= 0.0) {
    return joint_velocities;
  }

  for (std::size_t i = 0; i < joint_velocities.size(); ++i) {
    joint_velocities[i] = (current_joint_positions[i] - previous_joint_positions[i]) / dt;
  }

  return joint_velocities;
}

inline DeviceState transform_raw_state_to_ros(
  const RawDeviceState & state, const std::array<double, 6> * previous_joint_positions,
  double dt)
{
  DeviceState ros_state;
  device_transform_to_ros_pose(state.transform, ros_state.position_m, ros_state.orientation_xyzw);
  ros_state.linear_velocity_m_s = transform_vector_device_to_ros(state.velocity_mm_s);
  for (double & value : ros_state.linear_velocity_m_s) {
    value *= detail::kMillimetersToMeters;
  }

  ros_state.joint_positions_rad = calculate_joint_positions_rad(state);
  if (previous_joint_positions != nullptr) {
    ros_state.joint_velocities_rad_s = calculate_joint_velocities_rad_s(
      ros_state.joint_positions_rad, *previous_joint_positions, dt);
  }

  return ros_state;
}

}  // namespace touch_hardware

#endif  // TOUCH_HARDWARE__DEVICE_TRANSFORMS_HPP_
