#ifndef TOUCH_HARDWARE__TOUCH_SYSTEM_HPP_
#define TOUCH_HARDWARE__TOUCH_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace touch_hardware
{

class TouchSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TouchSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool validate_hardware_info_() const;
  void set_zero_command_();
  hardware_interface::CallbackReturn cleanup_device_();

  std::string device_name_;
  std::vector<std::string> joint_names_;
  std::array<double, 6> previous_joint_positions_{};
  bool previous_joint_positions_valid_{false};
  bool configured_{false};
  bool active_{false};

  class TouchDevice;
  std::unique_ptr<TouchDevice> touch_device_;
};

}  // namespace touch_hardware

#endif  // TOUCH_HARDWARE__TOUCH_SYSTEM_HPP_
