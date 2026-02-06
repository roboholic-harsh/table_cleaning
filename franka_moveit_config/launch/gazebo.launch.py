import os
import xacro
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. SETUP PATHS & PROCESS XACRO
    # ========================================================================
    franka_config_path = get_package_share_directory('franka_moveit_config')
    panda_pkg_path = get_package_share_directory('panda_description')
    gazebo_resource_path = os.path.dirname(panda_pkg_path)

    # Process Xacro to XML string
    xacro_file = os.path.join(franka_config_path, 'config', 'panda.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_content = doc.toxml()

    # Write XML to temp file (for Gazebo spawner)
    urdf_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    urdf_file.write(robot_description_content)
    urdf_path = urdf_file.name
    urdf_file.close()

    # ========================================================================
    # 2. CONFIGURE ENVIRONMENT
    # ========================================================================
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''), ':', gazebo_resource_path]
    )
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_MODEL_PATH',
        value=[os.environ.get('IGN_GAZEBO_MODEL_PATH', ''), ':', gazebo_resource_path]
    )

    # ========================================================================
    # 3. ROBOT STATE PUBLISHER (The Missing Piece!)
    # ========================================================================
    # This node publishes the robot description to the topic /robot_description
    # The Gazebo plugin listens to this topic to know about the joints.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ========================================================================
    # 4. START GAZEBO & SPAWN
    # ========================================================================
    # Define path to world file
    world_file = os.path.join(franka_config_path, 'worlds', 'table_world.sdf')

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
        ),
        # We pass the full path to the world file here
        launch_arguments={'ign_args': f'-r {world_file}'}.items()
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'panda_model',
            '-file', urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # ========================================================================
    # 5. LOAD CONTROLLERS
    # ========================================================================
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )


    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
        output="screen",
    )

    # Add this to your simulation.launch.py
    load_hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller"],
        output="screen",
    )

    # Event Handlers (Timing)
    # 1. Start Robot State Publisher immediately
    # 2. Wait 5s, then spawn robot
    delayed_spawn = TimerAction(
        period=5.0, 
        actions=[spawn_robot]
    )

    # 3. Once robot spawns, start joint state broadcaster
    start_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # 4. Once joint state broadcaster is ready, start arm controller
    start_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )
    # ========================================================================
    # 6. BRIDGE: GAZEBO -> ROS 2 (Dual Camera Setup)
    # ========================================================================
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera 1: Overhead (Static)
            '/camera/overhead_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            
            # Camera 2: Eye-in-Hand (Gripper)
            '/camera/hand_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/hand_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        ign_resource_path,
        ign_model_path,
        node_robot_state_publisher, # Don't forget to return this!
        start_gazebo,
        delayed_spawn,
        load_hand_controller,
        start_joint_state_broadcaster,
        start_arm_controller,
        bridge_camera,
    ])