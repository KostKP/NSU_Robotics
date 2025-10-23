from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    delay_arg = DeclareLaunchArgument('delay', default_value='2.0', description='Time delay in seconds')

    return LaunchDescription([
        delay_arg,
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
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
            package='md04ex03',
            executable='turtle_tf2_broadcaster',
            name='turtle1_broadcaster',
            parameters=[{'turtle': 'turtle1'}]
        ),
        Node(
            package='md04ex03',
            executable='turtle_tf2_broadcaster',
            name='turtle2_broadcaster',
            parameters=[{'turtle': 'turtle2'}]
        ),      
        Node(
            package='md04ex03',
            executable='delayed_tf_listener',
            name='delayed_follower',
            parameters=[{'delay': LaunchConfiguration('delay')}]
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        ),
    ])
