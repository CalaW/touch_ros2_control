from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    device_name = LaunchConfiguration("device_name")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("touch_description"), "urdf", "touch.urdf.xacro"]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
            " device_name:=",
            device_name,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("touch_bringup"), "config", "touch_controllers.yaml"]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="both",
    )

    force_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["touch_force_controller"],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("device_name", default_value="left"),
            controller_manager_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            force_controller_spawner,
        ]
    )
