import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Processing our urdf file
    pkg_path = os.path.join(get_package_share_directory('micromouse_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'micromouse.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file)
    # world_path = 'src/micromouse_description/worlds/maze.world'

    # world_path = os.path.join(
    #     get_package_share_directory('micromouse_description'),
    #     'worlds/maze.world'
    # )
    # # Path to the micromouse URDF
    # # urdf_path = 'src/micromouse_description/urdf/micromouse.urdf'

    # urdf_path = os.path.join(
    #     get_package_share_directory('micromouse_description'),
    #     'urdf/micromouse.urdf.xacro'
    # )

    # # # Gazebo launch setup (this includes the world)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': world_path}.items()
    # )

    # Robot state publisher node
    params = {'robot_description': robot_desc.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Spawn micromouse robot
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=['-entity', 'micromouse', '-x', '0', '-y', '0', '-z', '0', '-file', urdf_path],
    # )

    # Return LaunchDescription to launch both processes
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        # gazebo_launch,
        robot_state_publisher,
        # spawn_robot,
    ])
