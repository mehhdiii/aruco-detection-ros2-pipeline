import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the world file
    world_file_name = 'empty.world'
    world_path = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', world_file_name)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'worlds/aruca_test.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])