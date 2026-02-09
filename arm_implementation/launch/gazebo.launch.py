import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Spawn Panda Robot -------------------
    panda_urdf_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_description"),
        "urdf",
        "panda.urdf"
    )

    spawn_panda = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "panda",
            "-file", panda_urdf_path,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        spawn_panda
    ])
