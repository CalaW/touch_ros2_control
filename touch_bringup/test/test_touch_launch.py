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


pytestmark = pytest.mark.skipif(
    os.environ.get("TOUCH_RUN_HARDWARE_TESTS") != "1",
    reason="real Touch hardware launch test is disabled by default",
)


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("touch_bringup"), "launch", "touch.launch.py"
            )
        )
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

    def test_placeholder(self):
        self.assertTrue(self.node is not None)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
