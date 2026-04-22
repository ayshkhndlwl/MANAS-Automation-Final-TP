import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'task2'

    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'robot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
        
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diff_drive_robot'],
        output='screen'
    )

    dead_reckoning_node = Node(
        package=pkg_name,
        executable='dead_reckoning.py',
        name='dead_reckoning',
        output='screen'
    )

    controller_node = Node(
        package=pkg_name,
        executable='controller.py',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        dead_reckoning_node,
        controller_node
    ])