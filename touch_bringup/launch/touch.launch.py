from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_name = LaunchConfiguration("device_name")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("touch_description"), "urdf", "touch.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    driver_params = PathJoinSubstitution(
        [FindPackageShare("touch_bringup"), "config", "touch_driver.yaml"]
    )

    touch_driver_node = Node(
        package="touch_hardware",
        executable="touch_driver",
        name="touch_driver",
        parameters=[driver_params, {"device_name": device_name}],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="Default Device"),
            touch_driver_node,
            robot_state_publisher_node,
        ]
    )
