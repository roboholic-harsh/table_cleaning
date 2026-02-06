import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    cube_spawner_pkg = get_package_share_directory('cube_spawner')
    
    # --- 1. SIMULATION BRINGUP (Gazebo + MoveIt + Controllers + Block) ---
    # We include the master launch file from the previous step
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cube_spawner_pkg, 'launch', 'bringup.launch.py')
        )
    )

    # --- 2. VISION NODE ---
    # We wait 15 seconds to ensure Gazebo and the Camera are fully loaded
    vision_node = TimerAction(
        period=15.0,
        actions=[Node(
            package='tf_detector',
            executable='start_vision',
            output='screen'
        )]
    )

    return LaunchDescription([
        simulation,
        vision_node
    ])