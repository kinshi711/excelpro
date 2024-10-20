from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_file = PathJoinSubstitution(
        [FindPackageShare('turtlebot_line_follower'), 'worlds', 'my_world.world'])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={'world': world_file}.items()
    )

    line_follower_node = Node(
        package='turtlebot_line_follower',
        executable='line_follower',
        name='line_follower',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        line_follower_node,
    ])
