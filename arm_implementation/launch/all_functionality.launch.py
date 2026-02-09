import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # -----------------------------------------------------------------
    # 1. PATHS TO EXTERNAL LAUNCH FILES
    # -----------------------------------------------------------------
    tf_detector_dir = get_package_share_directory('tf_detector')
    
    # Path to: ros2 launch tf_detector sorting_system.launch.py
    sorting_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tf_detector_dir, 'launch', 'sorting_system.launch.py')
        )
    )

    # -----------------------------------------------------------------
    # 2. DEFINE NODES (ros2 run commands)
    # -----------------------------------------------------------------
    
    # ros2 run arm_implementation arm_action_server
    arm_action_server_node = Node(
        package='arm_implementation',
        executable='arm_action_server',
        name='arm_action_server',
        output='screen'
    )

    # ros2 run automatic_pnp clear_table
    clear_table_node = Node(
        package='automatic_pnp',
        executable='clear_table',
        name='clear_table_node',
        output='screen'
    )

    # ros2 run automatic_pnp automated_sorting
    automated_sorting_node = Node(
        package='automatic_pnp',
        executable='automated_sorting',
        name='automated_sorting_node',
        output='screen'
    )

    # ros2 run tf_detector start_vision
    start_vision_node = Node(
        package='tf_detector',
        executable='start_vision',
        name='vision_node',
        output='screen'
    )

    # -----------------------------------------------------------------
    # 3. CONSTRUCT LAUNCH DESCRIPTION
    # -----------------------------------------------------------------
    return LaunchDescription([
        # Start the sorting system environment/simulation first
        sorting_system_launch,
        
        # Start the controller
        arm_action_server_node,
        
        # Start the vision processing
        start_vision_node,
        
        # Start the autonomous task nodes
        clear_table_node,
        automated_sorting_node
    ])