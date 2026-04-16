#include "touch_hardware/controller_base.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace touch_hardware
{
namespace
{

using Clock = std::chrono::steady_clock;
constexpr double kMillimetersToMeters = 1e-3;

struct StateSnapshot
{
  DeviceState state;
  std::array<double, 3> commanded_force_n{0.0, 0.0, 0.0};
};

struct RawSnapshot
{
  std::array<double, 3> velocity_mm_s{0.0, 0.0, 0.0};
  std::array<double, 16> transform{};
  std::array<double, 6> joint_angles{};
};

template<typename T>
class DoubleBuffer
{
public:
  void write(const T & value)
  {
    const int next_index = 1 - index_.load(std::memory_order_relaxed);
    values_[next_index] = value;
    index_.store(next_index, std::memory_order_release);
  }

  T read() const
  {
    return values_[index_.load(std::memory_order_acquire)];
  }

private:
  std::array<T, 2> values_{};
  std::atomic<int> index_{0};
};

const Eigen::Matrix3d & raw_to_ros_basis()
{
  static const Eigen::Matrix3d basis = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0).finished();
  return basis;
}

std::array<double, 3> transform_vector_raw_to_ros(const std::array<double, 3> & value)
{
  const Eigen::Vector3d raw(value[0], value[1], value[2]);
  const Eigen::Vector3d ros = raw_to_ros_basis() * raw;
  return {ros.x(), ros.y(), ros.z()};
}

std::array<double, 3> transform_vector_ros_to_raw(const std::array<double, 3> & value)
{
  const Eigen::Vector3d ros(value[0], value[1], value[2]);
  const Eigen::Vector3d raw = raw_to_ros_basis().transpose() * ros;
  return {raw.x(), raw.y(), raw.z()};
}

void raw_transform_to_ros_pose(
  const std::array<double, 16> & transform,
  std::array<double, 3> & position_m,
  std::array<double, 4> & orientation_xyzw)
{
  const Eigen::Vector3d raw_position_mm(transform[12], transform[13], transform[14]);
  const Eigen::Vector3d ros_position_m = raw_to_ros_basis() * raw_position_mm * kMillimetersToMeters;

  Eigen::Matrix3d rotation_raw;
  rotation_raw <<
    transform[0], transform[1], transform[2],
    transform[4], transform[5], transform[6],
    transform[8], transform[9], transform[10];

  const Eigen::Matrix3d rotation_ros =
    raw_to_ros_basis() * rotation_raw.transpose() * raw_to_ros_basis().transpose();
  Eigen::Quaterniond q_ros(rotation_ros);
  q_ros.normalize();

  position_m = {ros_position_m.x(), ros_position_m.y(), ros_position_m.z()};
  orientation_xyzw = {q_ros.x(), q_ros.y(), q_ros.z(), q_ros.w()};
}

std::array<double, 6> raw_joint_angles_to_ros(const std::array<double, 6> & joint_angles)
{
  return {
    joint_angles[0],
    joint_angles[1],
    joint_angles[2],
    joint_angles[3],
    joint_angles[4],
    joint_angles[5],
  };
}

void throw_on_hd_error(const std::string & message)
{
  const HDErrorInfo error = hdGetError();
  if (error.errorCode != HD_SUCCESS) {
    throw std::runtime_error(message + ": " + hdGetErrorString(error.errorCode));
  }
}

class ServoCore
{
public:
  ServoCore(std::shared_ptr<TouchController> controller, double max_force)
  : controller_(std::move(controller)), max_force_(max_force)
  {}

  void set_command(const CommandState & command)
  {
    command_buffer_.write(command);
  }

  StateSnapshot latest_snapshot() const
  {
    return state_buffer_.read();
  }

  static HDCallbackCode HDCALLBACK scheduler_callback(void * data)
  {
    return static_cast<ServoCore *>(data)->tick();
  }

private:
  HDCallbackCode tick()
  {
    const auto now = Clock::now();
    double dt = 0.001;
    if (last_tick_time_.time_since_epoch().count() != 0) {
      dt = std::chrono::duration<double>(now - last_tick_time_).count();
      if (dt <= 0.0) {
        dt = 0.001;
      }
    }
    last_tick_time_ = now;

    const HHD device = hdGetCurrentDevice();
    hdBeginFrame(device);

    RawSnapshot raw_snapshot;
    hdGetDoublev(HD_CURRENT_VELOCITY, raw_snapshot.velocity_mm_s.data());
    hdGetDoublev(HD_CURRENT_TRANSFORM, raw_snapshot.transform.data());

    std::array<double, 3> arm_joint_angles{};
    std::array<double, 3> gimbal_angles{};
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, arm_joint_angles.data());
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles.data());

    raw_snapshot.joint_angles[0] = arm_joint_angles[0];
    raw_snapshot.joint_angles[1] = arm_joint_angles[1];
    raw_snapshot.joint_angles[2] = arm_joint_angles[2] - arm_joint_angles[1];
    raw_snapshot.joint_angles[3] = gimbal_angles[0];
    raw_snapshot.joint_angles[4] = gimbal_angles[1];
    raw_snapshot.joint_angles[5] = gimbal_angles[2];

    DeviceState state;
    raw_transform_to_ros_pose(raw_snapshot.transform, state.position_m, state.orientation_xyzw);
    state.linear_velocity_m_s = transform_vector_raw_to_ros(raw_snapshot.velocity_mm_s);
    for (double & value : state.linear_velocity_m_s) {
      value *= kMillimetersToMeters;
    }
    state.joint_positions_rad = raw_joint_angles_to_ros(raw_snapshot.joint_angles);

    if (have_previous_joint_positions_) {
      for (size_t i = 0; i < state.joint_positions_rad.size(); ++i) {
        state.joint_velocities_rad_s[i] =
          (state.joint_positions_rad[i] - previous_joint_positions_[i]) / dt;
      }
    }
    previous_joint_positions_ = state.joint_positions_rad;
    have_previous_joint_positions_ = true;

    if (!controller_initialized_) {
      controller_->reset(state);
      controller_initialized_ = true;
    }

    const CommandState command = command_buffer_.read();
    ForceCommand ros_force_command = controller_->compute_force(state, command, dt);
    sanitize_force(ros_force_command.force_n);
    clamp_force(ros_force_command.force_n);

    const std::array<double, 3> raw_force_command = transform_vector_ros_to_raw(ros_force_command.force_n);
    hdSetDoublev(HD_CURRENT_FORCE, raw_force_command.data());
    hdEndFrame(device);

    const HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      hduPrintError(stderr, &error, "Touch servo callback error");
      if (hduIsSchedulerError(&error)) {
        return HD_CALLBACK_DONE;
      }
    }

    StateSnapshot snapshot;
    snapshot.state = state;
    snapshot.commanded_force_n = ros_force_command.force_n;
    state_buffer_.write(snapshot);

    return HD_CALLBACK_CONTINUE;
  }

  void sanitize_force(std::array<double, 3> & force) const
  {
    for (double & value : force) {
      if (!std::isfinite(value)) {
        value = 0.0;
      }
    }
  }

  void clamp_force(std::array<double, 3> & force) const
  {
    const double magnitude = std::sqrt(
      force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
    if (magnitude <= max_force_ || magnitude <= 0.0) {
      return;
    }

    const double scale = max_force_ / magnitude;
    for (double & value : force) {
      value *= scale;
    }
  }

  std::shared_ptr<TouchController> controller_;
  double max_force_;
  DoubleBuffer<CommandState> command_buffer_;
  DoubleBuffer<StateSnapshot> state_buffer_;
  Clock::time_point last_tick_time_{};
  std::array<double, 6> previous_joint_positions_{};
  bool have_previous_joint_positions_{false};
  bool controller_initialized_{false};
};

class HdapiDevice
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
    hdEnable(HD_FORCE_OUTPUT);
    throw_on_hd_error("Failed to enable force output");
    opened_ = true;
  }

  void start(ServoCore * servo_core)
  {
    if (!opened_ || running_) {
      return;
    }

    hdMakeCurrentDevice(handle_);
    throw_on_hd_error("Failed to make device current");
    callback_handle_ = hdScheduleAsynchronous(
      &ServoCore::scheduler_callback, servo_core, HD_DEFAULT_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to schedule servo callback");
    hdStartScheduler();
    throw_on_hd_error("Failed to start scheduler");
    running_ = true;
  }

  void stop()
  {
    if (!opened_ || !running_) {
      return;
    }

    hdStopScheduler();
    hdMakeCurrentDevice(handle_);
    if (hdIsEnabled(HD_FORCE_OUTPUT)) {
      hdDisable(HD_FORCE_OUTPUT);
    }
    running_ = false;
    callback_handle_ = HD_INVALID_HANDLE;
  }

  void close()
  {
    if (!opened_) {
      return;
    }

    stop();
    hdMakeCurrentDevice(handle_);
    hdDisableDevice(handle_);
    handle_ = HD_INVALID_HANDLE;
    opened_ = false;
  }

  ~HdapiDevice()
  {
    close();
  }

private:
  HHD handle_{HD_INVALID_HANDLE};
  HDSchedulerHandle callback_handle_{HD_INVALID_HANDLE};
  bool opened_{false};
  bool running_{false};
};

class TouchNode : public rclcpp::Node
{
public:
  TouchNode()
  : Node("touch_driver"),
    controller_loader_("touch_hardware", "touch_hardware::TouchController")
  {
    declare_parameter<std::string>("device_name", "Default Device");
    declare_parameter<double>("publish_rate_hz", 250.0);
    declare_parameter<std::string>("controller_plugin", "touch_controllers/NullController");
    declare_parameter<double>("max_force", 8.0);
    declare_parameter<std::vector<double>>("impedance_stiffness", {45.0, 45.0, 45.0});
    declare_parameter<std::vector<double>>("impedance_damping", {2.5, 2.5, 2.5});

    const std::string device_name = get_parameter("device_name").as_string();
    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    const std::string controller_plugin = get_parameter("controller_plugin").as_string();
    const double max_force = get_parameter("max_force").as_double();

    CommandState initial_command;
    initial_command.impedance_stiffness = read_vec3_parameter("impedance_stiffness");
    initial_command.impedance_damping = read_vec3_parameter("impedance_damping");

    controller_ = controller_loader_.createSharedInstance(controller_plugin);
    servo_core_ = std::make_unique<ServoCore>(controller_, max_force);
    servo_core_->set_command(initial_command);
    command_state_ = initial_command;

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/touch/state/pose", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/touch/state/twist", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/touch/state/wrench_commanded", 10);

    direct_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/touch/command/direct_wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        command_state_.direct_force_n = {
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z,
        };
        servo_core_->set_command(command_state_);
      });

    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/touch/command/target_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        command_state_.target_position_m = {
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z,
        };
        command_state_.target_pose_valid = true;
        servo_core_->set_command(command_state_);
        RCLCPP_INFO(
          get_logger(), "Received target pose: [%.6f, %.6f, %.6f]",
          command_state_.target_position_m[0],
          command_state_.target_position_m[1],
          command_state_.target_position_m[2]);
      });

    const auto publish_period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz, 1.0));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&TouchNode::publish_state, this));

    hdapi_device_.open(device_name);
    hdapi_device_.start(servo_core_.get());
  }

  ~TouchNode() override
  {
    try {
      CommandState zero_command = command_state_;
      zero_command.direct_force_n = {0.0, 0.0, 0.0};
      servo_core_->set_command(zero_command);
      hdapi_device_.stop();
      hdapi_device_.close();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Shutdown failed: %s", error.what());
    }
  }

private:
  std::array<double, 3> read_vec3_parameter(const std::string & name)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error("Parameter '" + name + "' must have exactly 3 elements.");
    }
    return {values[0], values[1], values[2]};
  }

  void publish_state()
  {
    const StateSnapshot snapshot = servo_core_->latest_snapshot();
    const auto stamp = now();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};
    joint_state.position.assign(
      snapshot.state.joint_positions_rad.begin(), snapshot.state.joint_positions_rad.end());
    joint_state.velocity.assign(
      snapshot.state.joint_velocities_rad_s.begin(), snapshot.state.joint_velocities_rad_s.end());
    joint_state_pub_->publish(joint_state);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "touch_base";
    pose.pose.position.x = snapshot.state.position_m[0];
    pose.pose.position.y = snapshot.state.position_m[1];
    pose.pose.position.z = snapshot.state.position_m[2];
    pose.pose.orientation.x = snapshot.state.orientation_xyzw[0];
    pose.pose.orientation.y = snapshot.state.orientation_xyzw[1];
    pose.pose.orientation.z = snapshot.state.orientation_xyzw[2];
    pose.pose.orientation.w = snapshot.state.orientation_xyzw[3];
    pose_pub_->publish(pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = "touch_base";
    twist.twist.linear.x = snapshot.state.linear_velocity_m_s[0];
    twist.twist.linear.y = snapshot.state.linear_velocity_m_s[1];
    twist.twist.linear.z = snapshot.state.linear_velocity_m_s[2];
    twist_pub_->publish(twist);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = stamp;
    wrench.header.frame_id = "touch_base";
    wrench.wrench.force.x = snapshot.commanded_force_n[0];
    wrench.wrench.force.y = snapshot.commanded_force_n[1];
    wrench.wrench.force.z = snapshot.commanded_force_n[2];
    wrench_pub_->publish(wrench);
  }

  pluginlib::ClassLoader<TouchController> controller_loader_;
  std::shared_ptr<TouchController> controller_;
  std::unique_ptr<ServoCore> servo_core_;
  HdapiDevice hdapi_device_;
  CommandState command_state_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr direct_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace
}  // namespace touch_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<touch_hardware::TouchNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    fprintf(stderr, "touch_driver failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
