from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rclcpp_param_lecture',
            executable='param_server',
            parameters=[{'int_param': 4}],
            output='screen',
            emulate_tty=True,
        ),
    ])
