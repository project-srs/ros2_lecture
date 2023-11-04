from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    world_file_name = LaunchConfiguration('world')
    world_full_path = PathJoinSubstitution([
            get_package_share_directory('ignition_lecture'),
            'worlds',
            world_file_name
        ])

    return LaunchDescription([
        DeclareLaunchArgument('world', description='world file name under ignition_lecture/world/'),
        ExecuteProcess(
            cmd=["ign gazebo -r ", world_full_path],
            output='screen',
            shell=True,
        )
    ])
