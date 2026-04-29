#ifndef TOUCH_HARDWARE__DEVICE_HPP_
#define TOUCH_HARDWARE__DEVICE_HPP_

#include "touch_hardware/types.hpp"

#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace touch_hardware
{
class Device
{
public:
  struct State
  {
    HDdouble position[3];
    HDdouble velocity[3];
    HDdouble transform[16];
    HDdouble angular_velocity[3];
    HDdouble joint_angles[3];
    HDdouble gimbal_angles[3];
    HDdouble force[3];
    HDdouble torque[3];
    HDdouble joint_torque[3];
    HDdouble gimbal_torque[3];

    HDdouble commanded_force[3]{0., 0., 0.};
    HDdouble max_force{8.};

    CommandState command;

    std::atomic<std::uint64_t> callback_count{0};
  };

public:
  Device() : is_open_(false) {}
  explicit Device(const std::string & device_name) : is_open_(false) { this->open(device_name); }
  ~Device() { this->close(); }

  inline State & get_state() { return state_; }

protected:
  std::unique_ptr<HHD> hHD_ptr_;
  HDSchedulerHandle scheduler_handle_{0};
  bool is_open_;
  State state_{};

public:
  void open(const std::string & device_name);
  void close();

protected:
  static HDCallbackCode HDCALLBACK on_device_state_(void * data);
};
}  // namespace touch_hardware

#endif  // TOUCH_HARDWARE__DEVICE_HPP_
