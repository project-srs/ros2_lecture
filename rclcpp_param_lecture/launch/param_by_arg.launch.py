from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='param', default_value='0', description='int parameter'),
        Node(
            package='rclcpp_param_lecture',
            executable='param_server',
            parameters=[{'int_param': LaunchConfiguration('param')}],
            output='screen',
            emulate_tty=True,
        ),
    ])
