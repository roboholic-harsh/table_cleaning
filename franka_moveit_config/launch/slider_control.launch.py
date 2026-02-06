import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Start the Slider GUI
    # This creates a window with sliders for every joint
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[
            # We must tell it where the URDF is so it knows what sliders to make
            '/home/roboholic_harsh/Desktop/lerobot/src/franka_moveit_config/config/panda.urdf.xacro' 
        ],
        parameters=[{'use_gui': True}],
        remappings=[
            # We remap the output to a specific topic that we can forward to the robot
            ('/joint_states', '/gui_joint_states')
        ]
    )

    # 2. Bridge GUI to Gazebo
    # The GUI publishes /gui_joint_states. 
    # We need a bridge to send these positions to the Gazebo controllers.
    # NOTE: This requires writing a small custom node or using a topic relay.
    # A simpler way for testing is to just run the GUI and visualize it in RViz first.
    
    return LaunchDescription([
        joint_state_publisher_gui
    ])