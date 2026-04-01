#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

TEST(TouchSystemHardwareTest, plugin_loads)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "touch_hardware", "hardware_interface::SystemInterface");

  std::shared_ptr<hardware_interface::SystemInterface> instance =
    loader.createSharedInstance("touch_hardware/TouchSystemHardware");

  ASSERT_NE(instance, nullptr);
}

}  // namespace
