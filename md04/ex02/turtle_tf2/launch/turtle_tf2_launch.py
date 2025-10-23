from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    radius_arg = DeclareLaunchArgument('radius', default_value='1.0')
    direction_arg = DeclareLaunchArgument('direction_of_rotation', default_value='1')

    return LaunchDescription([
        radius_arg,
        direction_arg,
        Node(
            package='turtle_tf2',
            executable='turtle_tf2_broadcaster',
            name='turtle_tf2_broadcaster',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                '{x: 4.0, y: 2.0, theta: 0.0, name: "turtle2"}'
            ],
            output='screen'
        ),
        Node(
            package='turtle_tf2',
            executable='carrot_tf2_broadcaster',
            name='carrot_tf2_broadcaster',
            output='screen',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
        Node(
            package='turtle_tf2',
            executable='turtle_tf2_listener',
            name='turtle_tf2_listener',
            output='screen'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'install/turtle_tf2/share/turtle_tf2/rviz/carrot.rviz'],
            output='screen'
        ),
    ])
