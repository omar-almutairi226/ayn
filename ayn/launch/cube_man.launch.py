from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your custom world file
    pkg_path = get_package_share_directory('ayn')  # Replace with your package name
    world_file = os.path.join(pkg_path, 'worlds', 'empty_world.world')  # Update with your world file path

    # Path to gazebo_ros launch file
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([
        # Launch Gazebo with your custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_file}.items()  # Pass custom world file
        ),

        # Publish robot description to topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(os.path.join(pkg_path, 'urdf', 'cube_man.urdf')).read()}]
        ),

        # Spawn the entity from the /robot_description topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'cube_man'],
            output='screen'
        )
    ])
