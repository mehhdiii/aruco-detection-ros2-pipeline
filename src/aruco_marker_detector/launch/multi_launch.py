import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the gazebo.launch.py file
    robot_urdf_launch_dir = os.path.join(get_package_share_directory('robot_urdf'), 'launch')
    gazebo_launch_file = os.path.join(robot_urdf_launch_dir, 'gazebo.launch.py')

    return LaunchDescription([
        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        # Run the aruco_node from the ros2_aruco package
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_aruco', 'aruco_node'],
            output='screen'
        ),

        # Run the talker node from the aruco_marker_detector package
        ExecuteProcess(
            cmd=['ros2', 'run', 'aruco_marker_detector', 'talker'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/Twist',
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
            ],
            output='screen'
        ),
        
    ])