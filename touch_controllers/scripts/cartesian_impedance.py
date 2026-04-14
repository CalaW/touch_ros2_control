#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node


class CartesianImpedanceNode(Node):
    def __init__(self) -> None:
        super().__init__("touch_cartesian_impedance")

        self.declare_parameter("control_rate_hz", 1000.0)
        self.declare_parameter("pose_topic", "/touch_pose_broadcaster/pose")
        self.declare_parameter("force_command_topic", "/touch_force_controller/commands")
        self.declare_parameter("force_group_name", "force")
        self.declare_parameter("force_interface_names", ["force.x", "force.y", "force.z"])
        self.declare_parameter("kp", [45.0, 45.0, 45.0])
        self.declare_parameter("kd", [2.5, 2.5, 2.5])
        self.declare_parameter("ka", [0.001, 0.001, 0.001])
        self.declare_parameter("goal_position", [0.0, 0.0, 0.0])
        self.declare_parameter("goal_velocity", [0.0, 0.0, 0.0])
        self.declare_parameter("goal_acceleration", [0.0, 0.0, 0.0])
        self.declare_parameter("hold_current_pose_on_start", True)
        self.declare_parameter("max_force", [8.0, 8.0, 8.0])
        self.declare_parameter("fig8_enable", False)
        self.declare_parameter("fig8_amp_x", 0.02)
        self.declare_parameter("fig8_amp_y", 0.02)
        self.declare_parameter("fig8_freq_x_hz", 0.5)
        self.declare_parameter("fig8_freq_y_hz", 1.0)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / max(self.control_rate_hz, 1.0)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.force_command_topic = str(self.get_parameter("force_command_topic").value)
        self.force_group_name = str(self.get_parameter("force_group_name").value)
        self.force_interface_names = list(self.get_parameter("force_interface_names").value)
        self.hold_current_pose_on_start = bool(
            self.get_parameter("hold_current_pose_on_start").value
        )

        self.kp = np.diag(self._read_vec3_param("kp"))
        self.kd = np.diag(self._read_vec3_param("kd"))
        self.ka = np.diag(self._read_vec3_param("ka"))
        self.goal_position = self._read_vec3_param("goal_position")
        self.goal_velocity = self._read_vec3_param("goal_velocity")
        self.goal_acceleration = self._read_vec3_param("goal_acceleration")
        self.max_force = np.abs(self._read_vec3_param("max_force"))

        self.fig8_enable = bool(self.get_parameter("fig8_enable").value)
        self.fig8_amp_x = float(self.get_parameter("fig8_amp_x").value)
        self.fig8_amp_y = float(self.get_parameter("fig8_amp_y").value)
        self.fig8_freq_x_hz = float(self.get_parameter("fig8_freq_x_hz").value)
        self.fig8_freq_y_hz = float(self.get_parameter("fig8_freq_y_hz").value)

        self.measured_position = np.zeros(3)
        self.measured_velocity = np.zeros(3)
        self.measured_acceleration = np.zeros(3)
        self.prev_position = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.prev_pose_time_sec: Optional[float] = None
        self.pose_received = False
        self.fig8_origin = self.goal_position.copy()
        self.start_time_sec = self._now_sec()

        self.force_pub = self.create_publisher(
            DynamicInterfaceGroupValues,
            self.force_command_topic,
            10,
        )
        self.wrench_debug_pub = self.create_publisher(WrenchStamped, "~/wrench_cmd", 10)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"touch_cartesian_impedance ready: pose_topic='{self.pose_topic}', "
            f"force_command_topic='{self.force_command_topic}'"
        )

    def _read_vec3_param(self, name: str) -> np.ndarray:
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"Parameter '{name}' must be a 3-element array.")
        return np.array([float(value[0]), float(value[1]), float(value[2])], dtype=float)

    def _now_sec(self) -> float:
        now = self.get_clock().now().nanoseconds
        return float(now) * 1e-9

    def _msg_time_sec(self, msg: PoseStamped) -> Optional[float]:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        if sec == 0 and nanosec == 0:
            return None
        return float(sec) + float(nanosec) * 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        p = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=float,
        )

        t_sec = self._msg_time_sec(msg)
        if t_sec is None:
            t_sec = self._now_sec()

        if not self.pose_received:
            self.measured_position = p
            self.prev_position = p
            self.measured_velocity = np.zeros(3)
            self.prev_velocity = np.zeros(3)
            self.measured_acceleration = np.zeros(3)
            self.prev_pose_time_sec = t_sec
            self.pose_received = True

            if self.hold_current_pose_on_start:
                self.goal_position = p.copy()
                self.fig8_origin = p.copy()
            return

        dt_pose = t_sec - float(self.prev_pose_time_sec)
        if dt_pose <= 1e-6:
            return

        velocity = (p - self.prev_position) / dt_pose
        acceleration = (velocity - self.prev_velocity) / dt_pose

        self.measured_position = p
        self.measured_velocity = velocity
        self.measured_acceleration = acceleration
        self.prev_position = p
        self.prev_velocity = velocity
        self.prev_pose_time_sec = t_sec

    def _update_goal(self) -> None:
        if not self.fig8_enable:
            return

        t = self._now_sec() - self.start_time_sec
        wx = 2.0 * math.pi * self.fig8_freq_x_hz
        wy = 2.0 * math.pi * self.fig8_freq_y_hz

        self.goal_position[0] = self.fig8_origin[0] + self.fig8_amp_x * math.sin(wx * t)
        self.goal_position[1] = self.fig8_origin[1] + self.fig8_amp_y * math.sin(wy * t)
        self.goal_velocity[0] = self.fig8_amp_x * wx * math.cos(wx * t)
        self.goal_velocity[1] = self.fig8_amp_y * wy * math.cos(wy * t)
        self.goal_acceleration[0] = -self.fig8_amp_x * wx * wx * math.sin(wx * t)
        self.goal_acceleration[1] = -self.fig8_amp_y * wy * wy * math.sin(wy * t)

    def _on_timer(self) -> None:
        if not self.pose_received:
            return

        self._update_goal()

        force = (
            self.kp @ (self.goal_position - self.measured_position)
            + self.kd @ (self.goal_velocity - self.measured_velocity)
            + self.ka @ (self.goal_acceleration - self.measured_acceleration)
        )
        force = np.clip(force, -self.max_force, self.max_force)
        self._publish_force(force)
        self._publish_debug_wrench(force)

    def _publish_force(self, force: np.ndarray) -> None:
        cmd = DynamicInterfaceGroupValues()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.interface_groups = [self.force_group_name]

        values = InterfaceValue()
        values.interface_names = self.force_interface_names
        values.values = [float(force[0]), float(force[1]), float(force[2])]
        cmd.interface_values = [values]
        self.force_pub.publish(cmd)

    def _publish_debug_wrench(self, force: np.ndarray) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = "touch_base"
        wrench.wrench.force.x = float(force[0])
        wrench.wrench.force.y = float(force[1])
        wrench.wrench.force.z = float(force[2])
        self.wrench_debug_pub.publish(wrench)


def main() -> None:
    rclpy.init()
    node = CartesianImpedanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
