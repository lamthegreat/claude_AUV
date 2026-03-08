from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'mission_file', default_value='', description='Path to mission YAML file')

    return LaunchDescription([
        mission_arg,
        Node(
            package='auv_navigation',
            executable='mission_node',
            name='mission_node',
            namespace='auv',
            parameters=[{'mission_file': LaunchConfiguration('mission_file')}],
            output='screen',
            emulate_tty=True,
        ),
    ])
