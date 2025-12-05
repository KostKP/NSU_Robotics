from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'md05ex01'

    # Get package share directory
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)

    # URDF file path
    model_file = os.path.join(pkg_share, 'robot.xacro')

    # Robot description
    robot_description_content = Command(['xacro ', model_file])
    robot_description = {'robot_description': robot_description_content}
   
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
