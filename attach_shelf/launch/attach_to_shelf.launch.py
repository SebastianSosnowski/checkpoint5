from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('attach_shelf'),
        'config',
        'scene.rviz'
    ])

    pre_approach_node = Node(
        package='attach_shelf',
        executable='approach_executable',
        name='pre_approach_node',
        output='screen',
        emulate_tty=True,
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
        service_server_node,
        pre_approach_node,
        rviz_node,
    ])