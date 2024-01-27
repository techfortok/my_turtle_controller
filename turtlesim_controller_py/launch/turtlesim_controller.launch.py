from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    num_of_sides_launch_arg = DeclareLaunchArgument(
            "num_of_sides", default_value=TextSubstitution(text="4")
    )
    length_of_side_launch_arg = DeclareLaunchArgument(
            "length_of_side", default_value=TextSubstitution(text="1.0")
    )
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node",
    )
    turtlesim_controller_node = Node(
        package="turtlesim_controller_py",
        executable="turtlesim_controller_node_py",
        name="turtlesim_controller_node_py",
        parameters=[{
            "num_of_sides": LaunchConfiguration("num_of_sides"),
            "length_of_side": LaunchConfiguration("length_of_side"),
            "velocity": 0.5,
            "yawrate": 0.3,
        }],
    )

    return LaunchDescription([
        num_of_sides_launch_arg,
        length_of_side_launch_arg,
        turtlesim_node,
        turtlesim_controller_node,
    ])