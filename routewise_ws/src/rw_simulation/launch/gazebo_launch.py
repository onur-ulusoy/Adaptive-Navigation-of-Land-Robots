from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to necessary files
    rw_description_dir = get_package_share_directory('rw_description')
    urdf_path = os.path.join(rw_description_dir, 'urdf', 'routewise.urdf')
    rw_simulation_dir = get_package_share_directory('rw_simulation')
    world_path = os.path.join(rw_simulation_dir, 'gazebo_maps', 'initial_map.world')

    # Use IncludeLaunchDescription to include Gazebo's launch file
    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'verbose': 'yes',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items(),
    )

    return LaunchDescription([
        gazebo_launch,
        # Spawn the robot into the Gazebo world
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'routewise', '-file', urdf_path],
            output='screen'
        ),
    ])
