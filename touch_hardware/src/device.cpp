#include "touch_hardware/device.hpp"
#include "touch_hardware/impedance_math.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace touch_hardware {
namespace {
constexpr auto DEVICE_OPEN_SETTLE_DELAY = std::chrono::seconds(1);
constexpr auto DEVICE_CLOSE_SETTLE_DELAY = std::chrono::seconds(1);

void clamp_force(HDdouble force[3], HDdouble max_force) {
  const double magnitude =
      std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
  if (magnitude <= max_force || magnitude <= 0.) {
    return;
  }

  const double scale = max_force / magnitude;
  for (int i = 0; i < 3; ++i) {
    force[i] *= scale;
  }
}

RawDeviceState to_raw_device_state(const Device::State &state) {
  RawDeviceState raw_state;
  for (int i = 0; i < 3; ++i) {
    raw_state.velocity_mm_s[i] = state.velocity[i];
    raw_state.arm_joint_angles_rad[i] = state.joint_angles[i];
    raw_state.gimbal_angles_rad[i] = state.gimbal_angles[i];
  }
  for (int i = 0; i < 16; ++i) {
    raw_state.transform[i] = state.transform[i];
  }
  return raw_state;
}

HDCallbackCode HDCALLBACK zero_force_callback(void *) {
  HDdouble zero_force[3] = {0., 0., 0.};
  const HHD current_device = hdGetCurrentDevice();
  hdBeginFrame(current_device);
  hdSetDoublev(HD_CURRENT_FORCE, zero_force);
  hdEndFrame(current_device);
  return HD_CALLBACK_DONE;
}
} // namespace

void Device::open(const std::string &device_name) {
  if (is_open_)
    return;
  if (device_name.empty()) {
    throw std::runtime_error("No emtpy device_name allowed.");
  }
  // The OpenHaptics USB backend can keep stale force-output state briefly after
  // a process exits. A small settle delay makes rapid relaunches deterministic
  // instead of letting hdInitDevice() succeed while the physical force output
  // remains disarmed.
  std::this_thread::sleep_for(DEVICE_OPEN_SETTLE_DELAY);
  // Initialize haptic device
  hHD_ptr_ = std::make_unique<HHD>(hdInitDevice(device_name.c_str()));
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    throw std::runtime_error("Failed to initialize haptic device.");
  }
  // Enable force output, i.e. all motors are turned on.
  if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
    hdEnable(HD_FORCE_OUTPUT);
  } else {
    throw std::runtime_error("Failed to enable force output.");
  }
  // Setup device state
  for (int i = 0; i < 3; ++i) {
    state_.commanded_force[i] = 0.;
  }
  state_.callback_count.store(0, std::memory_order_relaxed);
  // Start state publisher
  scheduler_handle_ =
      hdScheduleAsynchronous(this->on_device_state_, &state_, HD_DEFAULT_SCHEDULER_PRIORITY);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    scheduler_handle_ = 0;
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(*hHD_ptr_);
    hHD_ptr_.reset();
    throw std::runtime_error("Failed to schedule haptic device callback.");
  }
  // Start the scheduler
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hdStopScheduler();
    if (scheduler_handle_ != 0) {
      hdUnschedule(scheduler_handle_);
      scheduler_handle_ = 0;
    }
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(*hHD_ptr_);
    hHD_ptr_.reset();
    throw std::runtime_error("Failed to start haptic scheduler.");
  }
  is_open_ = true;
  return;
}

void Device::close() {
  if (!is_open_)
    return;
  // Commit one final zero-force frame while the scheduler is still running, then
  // stop and unschedule explicitly. This avoids leaving a stale force command in
  // the HDAPI frame state across process restarts.
  hdScheduleSynchronous(zero_force_callback, nullptr, HD_DEFAULT_SCHEDULER_PRIORITY);
  hdStopScheduler();
  if (scheduler_handle_ != 0) {
    hdUnschedule(scheduler_handle_);
    scheduler_handle_ = 0;
  }
  hdDisable(HD_FORCE_OUTPUT);
  hdDisableDevice(*hHD_ptr_);
  hHD_ptr_.reset();
  is_open_ = false;
  // Give PhantomManager/USB teardown time to release the device before a fast
  // Ctrl-C + relaunch sequence tries to initialize it again.
  std::this_thread::sleep_for(DEVICE_CLOSE_SETTLE_DELAY);
}

HDCallbackCode HDCALLBACK Device::on_device_state_(void *data) {
  // Setup
  State *state = (State *)data;
  state->callback_count.fetch_add(1, std::memory_order_relaxed);

  // Begin frame
  hdBeginFrame(hdGetCurrentDevice());

  // Get device state
  hdGetDoublev(HD_CURRENT_POSITION, state->position);
  hdGetDoublev(HD_CURRENT_VELOCITY, state->velocity);
  hdGetDoublev(HD_CURRENT_TRANSFORM, state->transform);
  hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, state->angular_velocity);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state->joint_angles);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, state->gimbal_angles);
  hdGetDoublev(HD_CURRENT_FORCE, state->force);
  hdGetDoublev(HD_CURRENT_TORQUE, state->torque);
  hdGetDoublev(HD_CURRENT_JOINT_TORQUE, state->joint_torque);
  hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, state->gimbal_torque);

  CommandState command = state->command;
  if (!state->impedance_enabled) {
    command.impedance_stiffness = {0., 0., 0.};
    command.impedance_damping = {0., 0., 0.};
  }
  if (!command.target_pose_valid) {
    command.target_position_m = {0., 0., 0.};
  }

  const ForceCommand force_command = compute_impedance_force_device(to_raw_device_state(*state), command);
  for (int i = 0; i < 3; ++i) {
    state->commanded_force[i] = std::isfinite(force_command.device_force_n[i])
                                    ? force_command.device_force_n[i]
                                    : 0.;
  }

  clamp_force(state->commanded_force, state->max_force);
  hdSetDoublev(HD_CURRENT_FORCE, state->commanded_force);

  // End frame
  hdEndFrame(hdGetCurrentDevice());

  // Check for error
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback\n");
    if (hduIsSchedulerError(&error)) {
      return HD_CALLBACK_DONE;
    }
  }

  return HD_CALLBACK_CONTINUE;
}
} // namespace touch_hardware
