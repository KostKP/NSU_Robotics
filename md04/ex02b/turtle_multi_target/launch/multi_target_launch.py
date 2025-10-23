from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    switch_threshold_arg = DeclareLaunchArgument('switch_threshold', default_value='0.0')
    return LaunchDescription([
        switch_threshold_arg,
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
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                '{x: 1.0, y: 1.0, theta: 0.0, name: "turtle3"}'
            ],
            output='screen'
        ),
        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='turtle_tf2_broadcaster',
            output='screen'
        ),
        Node(
            package='turtle_multi_target',
            executable='target_switcher',
            name='target_switcher',
            output='screen'
        ),
        Node(
            package='turtle_multi_target',
            executable='turtle_controller',
            prefix='xterm -e',
            name='turtle_controller',
            output='screen',
            parameters=[{'switch_threshold': LaunchConfiguration('switch_threshold')}]
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
            arguments=['-d', 'install/turtle_multi_target/share/turtle_multi_target/rviz/carrot.rviz'],
            output='screen'
        ),
    ])
