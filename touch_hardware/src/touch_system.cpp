#include "touch_hardware/touch_system.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace touch_hardware
{

namespace
{

struct RawSnapshot
{
  std::array<double, 3> position_mm{0.0, 0.0, 0.0};
  std::array<double, 3> velocity_mm_s{0.0, 0.0, 0.0};
};

using Vec3 = std::array<double, 3>;

constexpr double kMillimetersToMeters = 1e-3;

Vec3 raw_to_ros(const Vec3 & raw)
{
  // Device frame: +X right, +Y up, +Z toward user
  // ROS frame:    +X forward, +Y left, +Z up
  return {raw[2], -raw[0], raw[1]};
}

Vec3 ros_to_raw(const Vec3 & ros)
{
  return {-ros[1], ros[2], ros[0]};
}

void throw_on_hd_error(const std::string & message)
{
  const HDErrorInfo error = hdGetError();
  if (error.errorCode != HD_SUCCESS)
  {
    throw std::runtime_error(message + ": " + hdGetErrorString(error.errorCode));
  }
}

}  // namespace

class TouchSystemHardware::Impl
{
public:
  void open(const std::string & device_name)
  {
    if (opened_)
    {
      return;
    }

    handle_ = hdInitDevice(device_name.c_str());
    throw_on_hd_error("Failed to initialize haptic device");
    hdMakeCurrentDevice(handle_);
    throw_on_hd_error("Failed to make device current");
    opened_ = true;
  }

  void close()
  {
    if (!opened_)
    {
      return;
    }

    stop();
    hdDisableDevice(handle_);
    handle_ = HD_INVALID_HANDLE;
    opened_ = false;
  }

  void start()
  {
    if (!opened_ || running_)
    {
      return;
    }

    hdMakeCurrentDevice(handle_);
    throw_on_hd_error("Failed to make device current");

    if (!hdIsEnabled(HD_FORCE_OUTPUT))
    {
      hdEnable(HD_FORCE_OUTPUT);
      throw_on_hd_error("Failed to enable force output");
    }

    callback_handle_ = hdScheduleAsynchronous(
      &Impl::scheduler_callback, this, HD_DEFAULT_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to schedule device callback");

    hdStartScheduler();
    throw_on_hd_error("Failed to start scheduler");
    running_ = true;
  }

  void stop()
  {
    if (!opened_ || !running_)
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      raw_force_command_ = {0.0, 0.0, 0.0};
    }

    hdStopScheduler();
    if (callback_handle_ != HD_INVALID_HANDLE)
    {
      hdUnschedule(callback_handle_);
      callback_handle_ = HD_INVALID_HANDLE;
    }
    hdDisable(HD_FORCE_OUTPUT);
    running_ = false;
  }

  void set_force_command(const Vec3 & force)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_force_command_ = force;
  }

  RawSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  static HDCallbackCode HDCALLBACK scheduler_callback(void * data)
  {
    auto * self = static_cast<Impl *>(data);

    hdBeginFrame(hdGetCurrentDevice());

    RawSnapshot local_snapshot;
    hdGetDoublev(HD_CURRENT_POSITION, local_snapshot.position_mm.data());
    hdGetDoublev(HD_CURRENT_VELOCITY, local_snapshot.velocity_mm_s.data());

    Vec3 local_force_command{0.0, 0.0, 0.0};
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->snapshot_ = local_snapshot;
      local_force_command = self->raw_force_command_;
    }

    hdSetDoublev(HD_CURRENT_FORCE, local_force_command.data());
    hdEndFrame(hdGetCurrentDevice());

    const HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error))
    {
      hduPrintError(stderr, &error, "Touch scheduler callback error");
      if (hduIsSchedulerError(&error))
      {
        return HD_CALLBACK_DONE;
      }
    }

    return HD_CALLBACK_CONTINUE;
  }

  mutable std::mutex mutex_;
  HHD handle_{HD_INVALID_HANDLE};
  HDSchedulerHandle callback_handle_{HD_INVALID_HANDLE};
  bool opened_{false};
  bool running_{false};
  RawSnapshot snapshot_{};
  Vec3 raw_force_command_{0.0, 0.0, 0.0};
};

hardware_interface::CallbackReturn TouchSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_hardware_info_())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto it = info_.hardware_parameters.find("device_name");
  if (it == info_.hardware_parameters.end() || it->second.empty())
  {
    RCLCPP_ERROR(get_logger(), "Missing required hardware parameter 'device_name'.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_name_ = it->second;
  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  impl_ = std::make_unique<Impl>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool TouchSystemHardware::validate_hardware_info_() const
{
  if (info_.joints.size() != 3)
  {
    RCLCPP_ERROR(get_logger(), "Expected exactly 3 joints, got %zu.", info_.joints.size());
    return false;
  }

  const std::array<std::string, 3> expected_joint_names{"touch_x", "touch_y", "touch_z"};
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    if (joint.name != expected_joint_names[i])
    {
      RCLCPP_ERROR(
        get_logger(), "Joint %zu must be named '%s', got '%s'.", i,
        expected_joint_names[i].c_str(), joint.name.c_str());
      return false;
    }

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose exactly 1 command interface.", joint.name.c_str());
      return false;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' command interface must be '%s'.", joint.name.c_str(),
        hardware_interface::HW_IF_EFFORT);
      return false;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose exactly 2 state interfaces.", joint.name.c_str());
      return false;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' state interfaces must be 'position' and 'velocity'.",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

hardware_interface::CallbackReturn TouchSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    impl_->open(device_name_);
    configured_ = true;
    active_ = false;
    set_zero_command_();
    for (const auto & joint_name : joint_names_)
    {
      set_state(joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
      set_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
      set_command(joint_name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure hardware: %s", e.what());
    configured_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn TouchSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return cleanup_device_();
}

hardware_interface::CallbackReturn TouchSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return cleanup_device_();
}

hardware_interface::CallbackReturn TouchSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!configured_)
  {
    RCLCPP_ERROR(get_logger(), "Cannot activate before configure.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    set_zero_command_();
    impl_->start();
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to activate hardware: %s", e.what());
    active_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn TouchSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_zero_command_();
  if (impl_)
  {
    impl_->stop();
  }
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TouchSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_zero_command_();
  return cleanup_device_();
}

hardware_interface::return_type TouchSystemHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!configured_ || !impl_)
  {
    return hardware_interface::return_type::OK;
  }

  const RawSnapshot snapshot = impl_->snapshot();
  const Vec3 ros_position_m = raw_to_ros(snapshot.position_mm);
  const Vec3 ros_velocity_m_s = raw_to_ros(snapshot.velocity_mm_s);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    set_state(
      joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION,
      ros_position_m[i] * kMillimetersToMeters);
    set_state(
      joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY,
      ros_velocity_m_s[i] * kMillimetersToMeters);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TouchSystemHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!configured_ || !impl_)
  {
    return hardware_interface::return_type::OK;
  }

  Vec3 ros_force{
    get_command(joint_names_[0] + "/" + hardware_interface::HW_IF_EFFORT),
    get_command(joint_names_[1] + "/" + hardware_interface::HW_IF_EFFORT),
    get_command(joint_names_[2] + "/" + hardware_interface::HW_IF_EFFORT),
  };

  for (double & value : ros_force)
  {
    if (!std::isfinite(value))
    {
      value = 0.0;
    }
  }

  if (!active_)
  {
    ros_force = {0.0, 0.0, 0.0};
  }

  impl_->set_force_command(ros_to_raw(ros_force));
  return hardware_interface::return_type::OK;
}

void TouchSystemHardware::set_zero_command_()
{
  for (const auto & joint_name : joint_names_)
  {
    set_command(joint_name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
  }

  if (impl_)
  {
    impl_->set_force_command({0.0, 0.0, 0.0});
  }
}

hardware_interface::CallbackReturn TouchSystemHardware::cleanup_device_()
{
  if (impl_)
  {
    impl_->set_force_command({0.0, 0.0, 0.0});
    impl_->close();
  }
  configured_ = false;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace touch_hardware

PLUGINLIB_EXPORT_CLASS(touch_hardware::TouchSystemHardware, hardware_interface::SystemInterface)
