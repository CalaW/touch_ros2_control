#include <gtest/gtest.h>

#include "touch_hardware/device_transforms.hpp"
#include "touch_controllers/direct_force_controller.hpp"
#include "touch_controllers/impedance_controller.hpp"
#include "touch_controllers/null_controller.hpp"

namespace
{

touch_hardware::RawDeviceState make_raw_state(
  const std::array<double, 3> & position_m, const std::array<double, 3> & velocity_m_s)
{
  touch_hardware::RawDeviceState state;
  state.transform = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  };

  const auto device_position_mm = touch_hardware::transform_vector_ros_to_device(
    {position_m[0] * 1000.0, position_m[1] * 1000.0, position_m[2] * 1000.0});
  state.transform[12] = device_position_mm[0];
  state.transform[13] = device_position_mm[1];
  state.transform[14] = device_position_mm[2];

  const auto device_velocity_m_s = touch_hardware::transform_vector_ros_to_device(velocity_m_s);
  state.velocity_mm_s = {
    device_velocity_m_s[0] * 1000.0,
    device_velocity_m_s[1] * 1000.0,
    device_velocity_m_s[2] * 1000.0,
  };

  return state;
}

TEST(NullController, ReturnsZeroForce)
{
  touch_controllers::NullController controller;
  touch_hardware::RawDeviceState state;
  touch_hardware::CommandState command;

  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.device_force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[2], 0.0);
}

TEST(DirectForceController, ForwardsCommand)
{
  touch_controllers::DirectForceController controller;
  touch_hardware::RawDeviceState state;
  touch_hardware::CommandState command;
  command.device_force_n = {1.0, -2.0, 3.5};

  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.device_force_n[0], 1.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[1], -2.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[2], 3.5);
}

TEST(ImpedanceController, HoldsResetPositionWithoutExternalTarget)
{
  touch_controllers::ImpedanceController controller;
  const auto state = make_raw_state({0.1, -0.2, 0.3}, {0.0, 0.0, 0.0});
  touch_hardware::CommandState command;
  command.impedance_stiffness = {10.0, 20.0, 30.0};
  command.impedance_damping = {1.0, 2.0, 3.0};

  controller.reset(state);
  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.device_force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(force.device_force_n[2], 0.0);
}

TEST(ImpedanceController, ComputesImpedanceForceFromCommandedTarget)
{
  touch_controllers::ImpedanceController controller;
  const auto initial_state = make_raw_state({0.1, 0.2, 0.3}, {0.0, 0.0, 0.0});
  controller.reset(initial_state);

  const auto state = make_raw_state({0.1, 0.2, 0.3}, {0.4, -0.5, 0.6});

  touch_hardware::CommandState command;
  command.target_pose_valid = true;
  command.target_position_m = {0.2, 0.1, 0.5};
  command.impedance_stiffness = {10.0, 20.0, 30.0};
  command.impedance_damping = {1.0, 2.0, 3.0};

  const auto force = controller.compute_force(state, command, 0.001);
  const auto ros_force = touch_hardware::transform_vector_device_to_ros(force.device_force_n);
  EXPECT_NEAR(ros_force[0], 0.6, 1e-9);
  EXPECT_NEAR(ros_force[1], -1.0, 1e-9);
  EXPECT_NEAR(ros_force[2], 4.2, 1e-9);
}

}  // namespace
