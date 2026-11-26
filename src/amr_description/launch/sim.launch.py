import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('amr_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'amr.urdf')
    world_file = os.path.join(package_dir, 'worlds', 'warehouse.sdf')

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/model/amr/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/amr2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/amr2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    spawn_amr = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr',
            '-x', '-2.0',
            '-y', '0.8',
            '-z', '0.1'
        ],
        output='screen'
    )

    spawn_amr2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr2',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr2',
            '-x', '-2.0',
            '-y', '-0.8',
            '-z', '0.1'
        ],
        output='screen'
    )

    spawn_robots = TimerAction(
        period=5.0,
        actions=[
            spawn_amr,
            spawn_amr2
        ]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robots
    ])

