import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('diffdrive'), 'basic_robot.urdf')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            arguments=[urdf]
        ),
        Node(
            package='diffdrive',
            executable='simulator',
            name='simulator',
            output='screen'
        ),
        Node(
            package='diffdrive',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen'
        )
    ])