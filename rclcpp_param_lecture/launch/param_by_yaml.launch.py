from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rclcpp_param_lecture",
                executable="param_server",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rclcpp_param_lecture"),
                            "config",
                            "param_server.yaml",
                        ]
                    )
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
