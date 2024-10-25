import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    package_name='micromouse_description'
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'maze.world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','simple.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
             )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'micromouse', '-x', '1', '-y', '1', '-z', '1'],
                        output='screen')
    
    delayed_spawn = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[spawn_entity]
    )

    # Launch them all
    return LaunchDescription([
        rsp,
        gazebo,
        delayed_spawn,
    ])