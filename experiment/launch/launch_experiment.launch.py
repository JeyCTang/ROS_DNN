from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, Shutdown, DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Set the pkg path
    spawning_pkg_dir = FindPackageShare(package='spawning').find('spawning')
    experiment_pkg_dir = FindPackageShare(package='experiment').find('experiment')
    yolov3_pkg_dir = FindPackageShare(package='recognizer_yolov3').find('recognizer_yolov3')

    launch_gazebo_world_cmd = IncludeLaunchDescription(
        os.path.join(spawning_pkg_dir, 'launch', 'launch_gazebo_empty_world.launch.py')
    )

    item_name = LaunchConfiguration('item_name')
    declare_spawning_item_name_cmd = DeclareLaunchArgument(
        name='item_name',
        default_value='bus',
        description='The item name we are going to spawn in the gazebo world'
    )
    launch_item_in_gazebo_cmd = IncludeLaunchDescription(
        os.path.join(spawning_pkg_dir, 'launch', 'launch_robot_model.launch.py'),
        launch_arguments={'item_name': item_name}.items()
    )