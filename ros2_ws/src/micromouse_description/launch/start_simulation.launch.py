from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from ament_index_python.packages import PackageNotFoundError

def generate_launch_description():
    # world_path = 'src/micromouse_description/worlds/maze.world'

    world_path = os.path.join(
        get_package_share_directory('micromouse_description'),
        'worlds/maze.world'
    )
    # Path to the micromouse URDF
    # urdf_path = 'src/micromouse_description/urdf/micromouse.urdf'

    urdf_path = os.path.join(
        get_package_share_directory('micromouse_description'),
        'urdf/micromouse.urdf'
    )

    # # Gazebo launch setup (this includes the world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_path],
    )

    # Spawn micromouse robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'micromouse', '-x', '0', '-y', '0', '-z', '0', '-file', urdf_path],
    )

    # Return LaunchDescription to launch both processes
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
    ])
