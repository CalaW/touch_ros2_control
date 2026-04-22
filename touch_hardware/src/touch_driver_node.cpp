#include "touch_hardware/controller_base.hpp"
#include "touch_hardware/device_transforms.hpp"

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

struct StateSnapshot
{
  RawDeviceState raw_state;
  RawDeviceState previous_raw_state;
  std::array<double, 3> commanded_device_force_n{0.0, 0.0, 0.0};
  double sample_period_s{0.0};
  bool have_previous_raw_state{false};
};

template <typename T>
class DoubleBuffer
{
public:
  void write(const T & value)
  {
    const int next_index = 1 - index_.load(std::memory_order_relaxed);
    values_[next_index] = value;
    index_.store(next_index, std::memory_order_release);
  }

  T read() const { return values_[index_.load(std::memory_order_acquire)]; }

private:
  std::array<T, 2> values_{};
  std::atomic<int> index_{0};
};

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
  {
  }

  void set_command(const CommandState & command) { command_buffer_.write(command); }

  StateSnapshot latest_snapshot() const { return state_buffer_.read(); }

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

    RawDeviceState raw_state;
    hdGetDoublev(HD_CURRENT_VELOCITY, raw_state.velocity_mm_s.data());
    hdGetDoublev(HD_CURRENT_TRANSFORM, raw_state.transform.data());
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, raw_state.arm_joint_angles_rad.data());
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, raw_state.gimbal_angles_rad.data());

    if (!controller_initialized_) {
      controller_->reset(raw_state);
      controller_initialized_ = true;
    }

    const CommandState command = command_buffer_.read();
    ForceCommand force_command = controller_->compute_force(raw_state, command, dt);
    sanitize_force(force_command.device_force_n);
    clamp_force(force_command.device_force_n);

    hdSetDoublev(HD_CURRENT_FORCE, force_command.device_force_n.data());
    hdEndFrame(device);

    const HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      hduPrintError(stderr, &error, "Touch servo callback error");
      if (hduIsSchedulerError(&error)) {
        return HD_CALLBACK_DONE;
      }
    }

    StateSnapshot snapshot;
    snapshot.raw_state = raw_state;
    snapshot.previous_raw_state = previous_raw_state_;
    snapshot.commanded_device_force_n = force_command.device_force_n;
    snapshot.sample_period_s = dt;
    snapshot.have_previous_raw_state = have_previous_raw_state_;
    state_buffer_.write(snapshot);
    previous_raw_state_ = raw_state;
    have_previous_raw_state_ = true;

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
    const double magnitude =
      std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
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
  RawDeviceState previous_raw_state_{};
  bool have_previous_raw_state_{false};
  bool controller_initialized_{false};
};

class HdapiDevice
{
public:
  void open(const std::string & device_name, ServoCore * servo_core)
  {
    if (is_open_) {
      return;
    }
    if (device_name.empty()) {
      throw std::runtime_error("Device name cannot be empty.");
    }

    device_handle_ = hdInitDevice(device_name.c_str());
    throw_on_hd_error("Failed to initialize haptic device");

    if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
      hdEnable(HD_FORCE_OUTPUT);
    } else {
      throw std::runtime_error("Failed to enable force output.");
    }

    hdScheduleAsynchronous(
      &ServoCore::scheduler_callback, servo_core, HD_DEFAULT_SCHEDULER_PRIORITY);
    throw_on_hd_error("Failed to schedule servo callback");

    hdStartScheduler();
    throw_on_hd_error("Failed to start scheduler");
    is_open_ = true;
  }

  void close()
  {
    if (!is_open_) {
      return;
    }

    hdStopScheduler();
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(device_handle_);
    device_handle_ = HD_INVALID_HANDLE;
    is_open_ = false;
  }

  ~HdapiDevice() { close(); }

private:
  HHD device_handle_{HD_INVALID_HANDLE};
  bool is_open_{false};
};

class TouchNode : public rclcpp::Node
{
public:
  TouchNode()
  : Node("touch_driver"), controller_loader_("touch_hardware", "touch_hardware::TouchController")
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
    wrench_pub_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>("/touch/state/wrench_commanded", 10);

    direct_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/touch/command/direct_wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        const std::array<double, 3> ros_force = {
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z,
        };
        command_state_.device_force_n = transform_vector_ros_to_device(ros_force);
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
          command_state_.target_position_m[0], command_state_.target_position_m[1],
          command_state_.target_position_m[2]);
      });

    const auto publish_period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz, 1.0));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&TouchNode::publish_state, this));

    hdapi_device_.open(device_name, servo_core_.get());
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
    std::array<double, 6> previous_joint_positions{};
    const std::array<double, 6> * previous_joint_positions_ptr = nullptr;
    if (snapshot.have_previous_raw_state) {
      previous_joint_positions = calculate_joint_positions_rad(snapshot.previous_raw_state);
      previous_joint_positions_ptr = &previous_joint_positions;
    }

    const DeviceState state = transform_raw_state_to_ros(
      snapshot.raw_state, previous_joint_positions_ptr, snapshot.sample_period_s);
    const auto commanded_force_ros =
      transform_vector_device_to_ros(snapshot.commanded_device_force_n);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};
    joint_state.position.assign(state.joint_positions_rad.begin(), state.joint_positions_rad.end());
    joint_state.velocity.assign(
      state.joint_velocities_rad_s.begin(), state.joint_velocities_rad_s.end());
    joint_state_pub_->publish(joint_state);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "touch_base";
    pose.pose.position.x = state.position_m[0];
    pose.pose.position.y = state.position_m[1];
    pose.pose.position.z = state.position_m[2];
    pose.pose.orientation.x = state.orientation_xyzw[0];
    pose.pose.orientation.y = state.orientation_xyzw[1];
    pose.pose.orientation.z = state.orientation_xyzw[2];
    pose.pose.orientation.w = state.orientation_xyzw[3];
    pose_pub_->publish(pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = "touch_base";
    twist.twist.linear.x = state.linear_velocity_m_s[0];
    twist.twist.linear.y = state.linear_velocity_m_s[1];
    twist.twist.linear.z = state.linear_velocity_m_s[2];
    twist_pub_->publish(twist);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = stamp;
    wrench.header.frame_id = "touch_base";
    wrench.wrench.force.x = commanded_force_ros[0];
    wrench.wrench.force.y = commanded_force_ros[1];
    wrench.wrench.force.z = commanded_force_ros[2];
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
