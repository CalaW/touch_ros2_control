#include <gtest/gtest.h>

#include "touch_controllers/direct_force_controller.hpp"
#include "touch_controllers/impedance_controller.hpp"
#include "touch_controllers/null_controller.hpp"

namespace
{

TEST(NullController, ReturnsZeroForce)
{
  touch_controllers::NullController controller;
  touch_hardware::DeviceState state;
  touch_hardware::CommandState command;

  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(force.force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(force.force_n[2], 0.0);
}

TEST(DirectForceController, ForwardsCommand)
{
  touch_controllers::DirectForceController controller;
  touch_hardware::DeviceState state;
  touch_hardware::CommandState command;
  command.direct_force_n = {1.0, -2.0, 3.5};

  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.force_n[0], 1.0);
  EXPECT_DOUBLE_EQ(force.force_n[1], -2.0);
  EXPECT_DOUBLE_EQ(force.force_n[2], 3.5);
}

TEST(ImpedanceController, HoldsResetPositionWithoutExternalTarget)
{
  touch_controllers::ImpedanceController controller;
  touch_hardware::DeviceState state;
  state.position_m = {0.1, -0.2, 0.3};
  state.linear_velocity_m_s = {0.0, 0.0, 0.0};
  touch_hardware::CommandState command;
  command.impedance_stiffness = {10.0, 20.0, 30.0};
  command.impedance_damping = {1.0, 2.0, 3.0};

  controller.reset(state);
  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_DOUBLE_EQ(force.force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(force.force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(force.force_n[2], 0.0);
}

TEST(ImpedanceController, ComputesImpedanceForceFromCommandedTarget)
{
  touch_controllers::ImpedanceController controller;
  touch_hardware::DeviceState initial_state;
  initial_state.position_m = {0.1, 0.2, 0.3};
  controller.reset(initial_state);

  touch_hardware::DeviceState state;
  state.position_m = {0.1, 0.2, 0.3};
  state.linear_velocity_m_s = {0.4, -0.5, 0.6};

  touch_hardware::CommandState command;
  command.target_pose_valid = true;
  command.target_position_m = {0.2, 0.1, 0.5};
  command.impedance_stiffness = {10.0, 20.0, 30.0};
  command.impedance_damping = {1.0, 2.0, 3.0};

  const auto force = controller.compute_force(state, command, 0.001);
  EXPECT_NEAR(force.force_n[0], 0.6, 1e-9);
  EXPECT_NEAR(force.force_n[1], -1.0, 1e-9);
  EXPECT_NEAR(force.force_n[2], 4.2, 1e-9);
}

}  // namespace
