import os
import pytest
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

import launch_testing
import rclpy
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("touch_bringup"), "launch", "touch.launch.py"
            )
        ),
        launch_arguments={"use_mock_hardware": "true"}.items(),
    )
    return LaunchDescription([launch_include, ReadyToTest()])


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("touch_test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_node_start(self, proc_output):
        check_node_running(self.node, "robot_state_publisher")

    def test_controller_running(self, proc_output):
        check_controllers_running(
            self.node,
            ["joint_state_broadcaster", "touch_force_controller", "touch_pose_broadcaster"],
        )

    def test_joint_state_published(self):
        check_if_js_published(
            "/joint_states", ["waist", "shoulder", "elbow", "yaw", "pitch", "roll"]
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
