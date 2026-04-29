#include <eigen3/Eigen/Dense>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "touch_hardware/device.hpp"
#include "touch_hardware/touch_driver_parameters.hpp"

namespace touch_hardware {
class DeviceDriver : public rclcpp::Node {
protected:
  static constexpr double MM2M = 1.e-3;
  static constexpr uint8_t DOF = 6;
  static constexpr const char *FRAME_ID = "touch_base";
  static constexpr const char *CHILD_FRAME_ID = "touch_ee";

public:
  struct DeviceState {
    sensor_msgs::msg::JointState joint_state;
    geometry_msgs::msg::TwistStamped twist_stamped;
  };

public:
  DeviceDriver(const rclcpp::NodeOptions &options)
      : Node("touch_driver", options), q_prev_set_(false) {
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
    this->validate_params_();
    this->init_msgs_();
    this->init_pubs_(); // cheers ;)
    this->init_subs_();
    ctrl_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1.e3 / params_.update_rate)),
        std::bind(&DeviceDriver::on_update_, this));
    RCLCPP_INFO(this->get_logger(), "Instantiating device %s...", params_.device_name.c_str());
    device_ = std::make_unique<Device>();
    this->init_device_params_();
    device_->open(params_.device_name);
    RCLCPP_INFO(this->get_logger(), "Done. haptic callbacks observed: %llu",
                static_cast<unsigned long long>(
                    device_->get_state().callback_count.load(std::memory_order_relaxed)));
  };

  ~DeviceDriver() override {
    // Stop ROS callbacks before closing the haptic device so no timer or
    // subscription can touch Device::State while the HDAPI scheduler is being
    // stopped and the device handle is being released.
    if (ctrl_timer_) {
      ctrl_timer_->cancel();
      ctrl_timer_.reset();
    }
    w_sub_.reset();
    if (device_) {
      RCLCPP_INFO(this->get_logger(), "Closing haptic device...");
      device_->close();
      device_.reset();
      RCLCPP_INFO(this->get_logger(), "Haptic device closed.");
    }
  }

protected:
  // Published states
  geometry_msgs::msg::TransformStamped tf_stamped_;
  sensor_msgs::msg::JointState js_;
  bool q_prev_set_;
  Eigen::Matrix<double, DOF, 1> q_, q_prev_, dq_;
  geometry_msgs::msg::TwistStamped ts_;

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Device state
  DeviceState state_;

  // Publishers
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ts_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr w_sub_;

  // Device
  std::unique_ptr<Device> device_;

protected:
  static std::array<double, 3> to_array3_(const std::vector<double> &values) {
    return {values[0], values[1], values[2]};
  }

  void validate_params_() {
    if (params_.device_name.empty()) {
      std::string err = "No device_name given, shutting down.";
      RCLCPP_ERROR(this->get_logger(), err.c_str());
      throw std::runtime_error(err);
    }
    RCLCPP_INFO(this->get_logger(), "*** Parameters");
    RCLCPP_INFO(this->get_logger(), "*   update_rate: %ld Hz", params_.update_rate);
    RCLCPP_INFO(this->get_logger(), "*   frame_id: %s", FRAME_ID);
    RCLCPP_INFO(this->get_logger(), "*   child_frame_id: %s", CHILD_FRAME_ID);
    RCLCPP_INFO(this->get_logger(), "*   controller: builtin impedance");
  };
  void init_msgs_() {
    // Joint states
    js_.effort.resize(DOF, 0.);
    js_.position.resize(DOF, 0.);
    js_.velocity.resize(DOF, 0.);
    js_.name = {"waist",  "shoulder",  "elbow",
                "yaw", "pitch", "roll"};
  }
  void init_pubs_() {
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                   rclcpp::SensorDataQoS());
    ts_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist",
                                                                       rclcpp::SensorDataQoS());
    tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  };
  void init_subs_() {
    w_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "command/wrench", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Wrench::SharedPtr msg) {
          auto &state = device_->get_state();
          state.command.device_force_n = {
              msg->force.x,
              msg->force.y,
              msg->force.z,
          };
        });
  };
  void init_device_params_() {
    auto &state = device_->get_state();
    state.max_force = params_.max_force;
    state.command.impedance_stiffness = to_array3_(params_.K);
    state.command.impedance_damping = to_array3_(params_.D);
  };
  void on_update_() {
    // get the current state
    const auto &state = device_->get_state();
    const auto stamp = this->get_clock()->now();

    // Pack transform stamp and translation
    tf_stamped_.header.stamp = stamp;
    tf_stamped_.header.frame_id = FRAME_ID;
    tf_stamped_.child_frame_id = CHILD_FRAME_ID;
    tf_stamped_.transform.translation.x = MM2M * state.transform[12];
    tf_stamped_.transform.translation.y = MM2M * state.transform[13];
    tf_stamped_.transform.translation.z = MM2M * state.transform[14];

    // Convert state transform to rotation matrix, then quaternion, and pack transform
    Eigen::Matrix3d R(3, 3);
    R(0, 0) = state.transform[0];
    R(0, 1) = state.transform[1];
    R(0, 2) = state.transform[2];
    R(1, 0) = state.transform[4];
    R(1, 1) = state.transform[5];
    R(1, 2) = state.transform[6];
    R(2, 0) = state.transform[8];
    R(2, 1) = state.transform[9];
    R(2, 2) = state.transform[10];

    Eigen::Quaterniond quat(R.transpose()); // require transpose otherwise rotation is inverted
    tf_stamped_.transform.rotation.x = quat.x();
    tf_stamped_.transform.rotation.y = quat.y();
    tf_stamped_.transform.rotation.z = quat.z();
    tf_stamped_.transform.rotation.w = quat.w();

    // Send transform
    tf_bc_->sendTransform(tf_stamped_);

    // Pack joint states
    js_.header.stamp = stamp;

    for (uint i = 0; i < 3; ++i) {
      q_(i) = state.joint_angles[i];
      q_(i + 3) = state.gimbal_angles[i];
      js_.effort[i] = state.joint_torque[i];
      js_.effort[i + 3] = state.gimbal_torque[i];
    }

    if (q_prev_set_) {
      dq_ = (q_ - q_prev_) * (1.0 / static_cast<double>(params_.update_rate));
    } else {
      dq_.setZero();
    }

    q_prev_ = q_;
    q_prev_set_ = true;

    for (uint i = 0; i < 6; ++i) {
      js_.position[i] = q_(i);
      js_.velocity[i] = dq_(i);
    }

    // Publish joint states
    js_pub_->publish(js_);

    // Pack twist stamped
    ts_.header.stamp = stamp;
    ts_.header.frame_id = CHILD_FRAME_ID;
    ts_.twist.linear.x = MM2M * state.velocity[0];
    ts_.twist.linear.y = MM2M * state.velocity[1];
    ts_.twist.linear.z = MM2M * state.velocity[2];

    ts_.twist.angular.x = state.angular_velocity[0];
    ts_.twist.angular.y = state.angular_velocity[1];
    ts_.twist.angular.z = state.angular_velocity[2];

    // Publish twist stamped
    ts_pub_->publish(ts_);
  };
};
} // namespace touch_hardware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(touch_hardware::DeviceDriver)
