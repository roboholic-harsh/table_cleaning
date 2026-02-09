import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    franka_config = get_package_share_directory('franka_moveit_config')
    cube_spawner_pkg = get_package_share_directory('cube_spawner')

    # --- 1. GAZEBO LAUNCH ---
    # Launches the simulation world and robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_config, 'launch', 'gazebo_sorting_world.launch.py')
        )
    )



    # --- 3. MOVEIT ---
    # Start MoveIt with Sim Time enabled
    moveit = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(franka_config, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # --- 4. FORCE SIM TIME (The Safety Fix) ---
    # Just in case MoveIt ignores the argument, we force the parameter after it starts
    force_sim_time = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true'],
            output='screen'
        )]
    )

    # --- 5. SPAWN BLOCK ---
    spawn_block = TimerAction(
        period=10.0,
        actions=[Node(
            package='cube_spawner',
            executable='spawn_colored',
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        spawn_block,
        moveit,
        force_sim_time
    ])