from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'md05ex05'

    # Get package share directory
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)

    # URDF file path
    model_file = os.path.join(pkg_share, 'robot.gazebo.xacro')

    # Robot description
    robot_description_content = Command(['xacro ', model_file])
    robot_description = {'robot_description': robot_description_content}
   
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}, {'frame_prefix': 'differential_robot/'}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': '-r empty.sdf --render-engine ogre'
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'differential_robot',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5'
            ],
            output='screen'
        ),

	    Node(
	        package='ros_gz_bridge',
	        executable='parameter_bridge',
	        arguments=[
	            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
		        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

		        # Gazebo odom/tf (реальные топики) -> ROS
		        '/model/differential_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
		        '/model/differential_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
		        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
	        ],
	        remappings=[
		        ('/model/differential_robot/odometry', '/odom'),
		        ('/model/differential_robot/tf', '/tf')
	        ],
	        output='screen'
	    ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'robot.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='md05ex05',
            executable='random_movement',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'max_linear_speed': 2.0},
                        {'max_angular_speed ': 5.0},
                        {'goal_tolerance': 0.15},
                        {'spiral_radius': 0.5},
                        {'movement_radius': 10.0},
                        {'rate': 10.0}]
        )
    ])
