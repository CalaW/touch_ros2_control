#include "touch_hardware/touch_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <Eigen/Geometry>

#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace touch_hardware
{

namespace
{

struct RawSnapshot
{
  std::array<double, 3> velocity_mm_s{0.0, 0.0, 0.0};
  std::array<double, 16> transform{};
  std::array<double, 6> joint_angles{};
};

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

constexpr double kMillimetersToMeters = 1e-3;
constexpr const char * kForceGroupName = "force";
constexpr const char * kPoseName = "tcp_pose";

const Eigen::Matrix3d & raw_to_ros_basis()
{
  // clang-format off
  static const Eigen::Matrix3d B = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0
  ).finished();
  // clang-format on
  return B;
}

void raw_transform_to_ros_pose(
  const std::array<double, 16> & transform, Vec3 & position, Quat & orientation)
{
  static const Eigen::Matrix3d basis = raw_to_ros_basis();

  Eigen::Vector3d raw_position(transform[12], transform[13], transform[14]);
  Eigen::Vector3d ros_position = basis * raw_position;

  Eigen::Matrix3d r_raw;
  // clang-format off
  r_raw <<
    transform[0], transform[1], transform[2],
    transform[4], transform[5], transform[6],
    transform[8], transform[9], transform[10];
  // clang-format on
  const Eigen::Matrix3d r_ros = basis * r_raw.transpose() * basis.transpose();
  Eigen::Quaterniond q_ros(r_ros);
  q_ros.normalize();

  position = {ros_position.x(), ros_position.y(), ros_position.z()};
  orientation = {q_ros.x(), q_ros.y(), q_ros.z(), q_ros.w()};
}

Vec3 force_ros_to_raw(const Vec3 & force_ros)
{
  const Eigen::Vector3d f_ros(force_ros[0], force_ros[1], force_ros[2]);
  const Eigen::Vector3d f_raw = raw_to_ros_basis().transpose() * f_ros;
  return {f_raw.x(), f_raw.y(), f_raw.z()};
}

void throw_on_hd_error(const std::string & message)
{
  const HDErrorInfo error = hdGetError();
  if (error.errorCode != HD_SUCCESS) {
    throw std::runtime_error(message + ": " + hdGetErrorString(error.errorCode));
  }
}

}  // namespace

class TouchSystemHardware::TouchDevice
{
public:
  void open(const std::string & device_name)
  {
    if (opened_) {
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
    if (!opened_) return;

    stop();
    hdDisableDevice(handle_);
    handle_ = HD_INVALID_HANDLE;
    opened_ = false;
  }

  void start()
  {
    if (!opened_ || running_) return;

    hdMakeCurrentDevice(handle_);
    throw_on_hd_error("Failed to make device current");

    callback_handle_ =
      hdScheduleAsynchronous(&TouchDevice::scheduler_callback, this, HD_DEFAULT_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to schedule device callback");

    hdStartScheduler();
    throw_on_hd_error("Failed to start scheduler");
    hdScheduleSynchronous(&TouchDevice::enable_force_output_callback, this, HD_MAX_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to enable force output");
    running_ = true;
  }

  void stop()
  {
    if (!opened_ || !running_) return;

    set_force_command({0.0, 0.0, 0.0});
    hdScheduleSynchronous(
      &TouchDevice::disable_force_output_callback, this, HD_MAX_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to disable force output");

    hdStopScheduler();
    if (callback_handle_ != HD_INVALID_HANDLE) {
      hdUnschedule(callback_handle_);
      callback_handle_ = HD_INVALID_HANDLE;
    }
    running_ = false;
  }

  void set_force_command(const Vec3 & force)
  {
    if (!running_) {
      raw_force_command_ = force;
      return;
    }

    CommandRequest request{this, force};
    hdScheduleSynchronous(
      &TouchDevice::set_force_command_callback, &request, HD_MAX_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to update force command");
  }

  RawSnapshot snapshot() const
  {
    if (!running_) {
      return snapshot_;
    }

    SnapshotRequest request{this, {}};
    hdScheduleSynchronous(&TouchDevice::copy_snapshot_callback, &request, HD_MIN_SCHEDULER_PRIORITY);
    return request.snapshot;
  }

  bool calibration_ok() const
  {
    hdMakeCurrentDevice(handle_);
    throw_on_hd_error("Failed to make device current");
    return hdCheckCalibration() == HD_CALIBRATION_OK;
  }

private:
  struct CommandRequest
  {
    TouchDevice * device;
    Vec3 force;
  };

  struct SnapshotRequest
  {
    const TouchDevice * device;
    RawSnapshot snapshot;
  };

  static HDCallbackCode HDCALLBACK scheduler_callback(void * data)
  {
    auto * self = static_cast<TouchDevice *>(data);

    hdBeginFrame(self->handle_);

    RawSnapshot local_snapshot;
    hdGetDoublev(HD_CURRENT_VELOCITY, local_snapshot.velocity_mm_s.data());
    hdGetDoublev(HD_CURRENT_TRANSFORM, local_snapshot.transform.data());

    std::array<double, 3> arm_joint_angles{};
    std::array<double, 3> gimbal_angles{};
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, arm_joint_angles.data());
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles.data());

    local_snapshot.joint_angles[0] = arm_joint_angles[0];
    local_snapshot.joint_angles[1] = arm_joint_angles[1];
    local_snapshot.joint_angles[2] = arm_joint_angles[2] - arm_joint_angles[1];
    local_snapshot.joint_angles[3] = gimbal_angles[0];
    local_snapshot.joint_angles[4] = gimbal_angles[1];
    local_snapshot.joint_angles[5] = gimbal_angles[2];

    self->snapshot_ = local_snapshot;

    hdSetDoublev(HD_CURRENT_FORCE, self->raw_force_command_.data());
    hdEndFrame(self->handle_);

    const HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      hduPrintError(stderr, &error, "Touch scheduler callback error");
      if (hduIsSchedulerError(&error)) {
        return HD_CALLBACK_DONE;
      }
    }

    return HD_CALLBACK_CONTINUE;
  }

  static HDCallbackCode HDCALLBACK set_force_command_callback(void * data)
  {
    auto * request = static_cast<CommandRequest *>(data);
    request->device->raw_force_command_ = request->force;
    return HD_CALLBACK_DONE;
  }

  static HDCallbackCode HDCALLBACK copy_snapshot_callback(void * data)
  {
    auto * request = static_cast<SnapshotRequest *>(data);
    request->snapshot = request->device->snapshot_;
    return HD_CALLBACK_DONE;
  }

  static HDCallbackCode HDCALLBACK enable_force_output_callback(void * data)
  {
    auto * self = static_cast<TouchDevice *>(data);
    hdMakeCurrentDevice(self->handle_);
    if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
      hdEnable(HD_FORCE_OUTPUT);
    }
    return HD_CALLBACK_DONE;
  }

  static HDCallbackCode HDCALLBACK disable_force_output_callback(void * data)
  {
    auto * self = static_cast<TouchDevice *>(data);
    hdMakeCurrentDevice(self->handle_);
    if (hdIsEnabled(HD_FORCE_OUTPUT)) {
      hdDisable(HD_FORCE_OUTPUT);
    }
    return HD_CALLBACK_DONE;
  }

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
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_hardware_info_()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto it = info_.hardware_parameters.find("device_name");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required hardware parameter 'device_name'.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_name_ = it->second;
  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  touch_device_ = std::make_unique<TouchDevice>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool TouchSystemHardware::validate_hardware_info_() const
{
  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(get_logger(), "Expected exactly 6 joints, got %zu.", info_.joints.size());
    return false;
  }

  const std::array<std::string, 6> expected_joint_names{"waist", "shoulder", "elbow",
                                                        "yaw",   "pitch",    "roll"};
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    if (joint.name != expected_joint_names[i]) {
      RCLCPP_ERROR(
        get_logger(), "Joint %zu must be named '%s', got '%s'.", i, expected_joint_names[i].c_str(),
        joint.name.c_str());
      return false;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose exactly 2 state interfaces.", joint.name.c_str());
      return false;
    }

    if (
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' state interfaces must be 'position' and 'velocity'.",
        joint.name.c_str());
      return false;
    }
  }

  if (info_.gpios.size() != 1 || info_.gpios[0].name != kForceGroupName) {
    RCLCPP_ERROR(get_logger(), "Expected one gpio named '%s'.", kForceGroupName);
    return false;
  }

  const auto & force_gpio = info_.gpios[0];
  const std::array<std::string, 3> expected_force_interfaces{"force.x", "force.y", "force.z"};
  if (force_gpio.command_interfaces.size() != expected_force_interfaces.size()) {
    RCLCPP_ERROR(get_logger(), "Force gpio must expose 3 command interfaces.");
    return false;
  }
  for (size_t i = 0; i < expected_force_interfaces.size(); ++i) {
    if (force_gpio.command_interfaces[i].name != expected_force_interfaces[i]) {
      RCLCPP_ERROR(
        get_logger(), "Force command interface %zu must be '%s', got '%s'.", i,
        expected_force_interfaces[i].c_str(), force_gpio.command_interfaces[i].name.c_str());
      return false;
    }
  }

  if (info_.sensors.size() != 1 || info_.sensors[0].name != kPoseName) {
    RCLCPP_ERROR(get_logger(), "Expected one sensor named '%s'.", kPoseName);
    return false;
  }

  const auto & pose_sensor = info_.sensors[0];
  const std::array<std::string, 7> expected_pose_interfaces{
    "position.x",    "position.y",    "position.z",   "orientation.x",
    "orientation.y", "orientation.z", "orientation.w"};
  if (pose_sensor.state_interfaces.size() != expected_pose_interfaces.size()) {
    RCLCPP_ERROR(get_logger(), "TCP pose sensor must expose 7 state interfaces.");
    return false;
  }
  for (size_t i = 0; i < expected_pose_interfaces.size(); ++i) {
    if (pose_sensor.state_interfaces[i].name != expected_pose_interfaces[i]) {
      RCLCPP_ERROR(
        get_logger(), "TCP pose interface %zu must be '%s', got '%s'.", i,
        expected_pose_interfaces[i].c_str(), pose_sensor.state_interfaces[i].name.c_str());
      return false;
    }
  }

  return true;
}

hardware_interface::CallbackReturn TouchSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    touch_device_->open(device_name_);
    configured_ = true;
    active_ = false;
    set_zero_command_();
    for (const auto & joint_name : joint_names_) {
      set_state(joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
      set_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }
    set_state(std::string(kPoseName) + "/position.x", 0.0);
    set_state(std::string(kPoseName) + "/position.y", 0.0);
    set_state(std::string(kPoseName) + "/position.z", 0.0);
    set_state(std::string(kPoseName) + "/orientation.x", 0.0);
    set_state(std::string(kPoseName) + "/orientation.y", 0.0);
    set_state(std::string(kPoseName) + "/orientation.z", 0.0);
    set_state(std::string(kPoseName) + "/orientation.w", 1.0);
    previous_joint_positions_.fill(0.0);
    previous_joint_positions_valid_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
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
  if (!configured_) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before configure.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    set_zero_command_();
    if (!touch_device_->calibration_ok()) {
      RCLCPP_ERROR(
        get_logger(),
        "Haptic device calibration is not OK. Force output will not behave correctly until the "
        "device is calibrated with the vendor/OpenHaptics calibration tool.");
      active_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    }
    touch_device_->start();
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate hardware: %s", e.what());
    active_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn TouchSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_zero_command_();
  if (touch_device_) {
    touch_device_->stop();
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!configured_ || !touch_device_) {
    return hardware_interface::return_type::OK;
  }

  const RawSnapshot snapshot = touch_device_->snapshot();
  std::array<double, 6> current_joint_positions = snapshot.joint_angles;
  std::array<double, 6> current_joint_velocities{};
  if (previous_joint_positions_valid_ && period.seconds() > 0.0) {
    for (size_t i = 0; i < current_joint_positions.size(); ++i) {
      current_joint_velocities[i] =
        (current_joint_positions[i] - previous_joint_positions_[i]) / period.seconds();
    }
  }

  previous_joint_positions_ = current_joint_positions;
  previous_joint_positions_valid_ = true;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    set_state(
      joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, current_joint_positions[i]);
    set_state(
      joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, current_joint_velocities[i]);
  }

  Vec3 ros_position_mm{};
  Quat ros_orientation{};
  raw_transform_to_ros_pose(snapshot.transform, ros_position_mm, ros_orientation);
  set_state(std::string(kPoseName) + "/position.x", ros_position_mm[0] * kMillimetersToMeters);
  set_state(std::string(kPoseName) + "/position.y", ros_position_mm[1] * kMillimetersToMeters);
  set_state(std::string(kPoseName) + "/position.z", ros_position_mm[2] * kMillimetersToMeters);
  set_state(std::string(kPoseName) + "/orientation.x", ros_orientation[0]);
  set_state(std::string(kPoseName) + "/orientation.y", ros_orientation[1]);
  set_state(std::string(kPoseName) + "/orientation.z", ros_orientation[2]);
  set_state(std::string(kPoseName) + "/orientation.w", ros_orientation[3]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TouchSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!configured_ || !touch_device_) {
    return hardware_interface::return_type::OK;
  }

  Vec3 ros_force{
    get_command(std::string(kForceGroupName) + "/force.x"),
    get_command(std::string(kForceGroupName) + "/force.y"),
    get_command(std::string(kForceGroupName) + "/force.z"),
  };

  for (double & value : ros_force) {
    if (!std::isfinite(value)) {
      value = 0.0;
    }
  }

  if (!active_) {
    ros_force = {0.0, 0.0, 0.0};
  }

  set_state(std::string(kForceGroupName) + "/force.x", ros_force[0]);
  set_state(std::string(kForceGroupName) + "/force.y", ros_force[1]);
  set_state(std::string(kForceGroupName) + "/force.z", ros_force[2]);
  touch_device_->set_force_command(force_ros_to_raw(ros_force));
  return hardware_interface::return_type::OK;
}

void TouchSystemHardware::set_zero_command_()
{
  set_command(std::string(kForceGroupName) + "/force.x", 0.0);
  set_command(std::string(kForceGroupName) + "/force.y", 0.0);
  set_command(std::string(kForceGroupName) + "/force.z", 0.0);
  set_state(std::string(kForceGroupName) + "/force.x", 0.0);
  set_state(std::string(kForceGroupName) + "/force.y", 0.0);
  set_state(std::string(kForceGroupName) + "/force.z", 0.0);

  if (touch_device_) {
    touch_device_->set_force_command({0.0, 0.0, 0.0});
  }
}

hardware_interface::CallbackReturn TouchSystemHardware::cleanup_device_()
{
  if (touch_device_) {
    touch_device_->set_force_command({0.0, 0.0, 0.0});
    touch_device_->close();
  }
  configured_ = false;
  active_ = false;
  previous_joint_positions_valid_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace touch_hardware

PLUGINLIB_EXPORT_CLASS(touch_hardware::TouchSystemHardware, hardware_interface::SystemInterface)
