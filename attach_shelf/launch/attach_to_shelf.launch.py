from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('attach_shelf'),
        'config',
        'scene.rviz'
    ])

    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.2',
        description='Distance to obstacle in meters'
    )

    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='90',
        description='Rotation angle in degrees'
    )

    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false',
        description='Whether to perform final approach'
    )

    pre_approach_node = Node(
        package='attach_shelf',
        executable='approach_executable',
        name='pre_approach_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'obstacle': LaunchConfiguration('obstacle'),
                'degrees': LaunchConfiguration('degrees'),
                'final_approach': LaunchConfiguration('final_approach'),
            }
        ],
    )

    service_server_node = Node(
        package='attach_shelf',
        executable='approach_srv_server_executable',
        name='approach_service_server',
        output='screen',
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        service_server_node,
        pre_approach_node,
        rviz_node,
    ])