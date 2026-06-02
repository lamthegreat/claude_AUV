from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('auv_controller'), 'config', 'controller_params.yaml'])

    return LaunchDescription([
        Node(
            package='auv_controller',
            executable='controller_node',
            name='controller_node',
            namespace='auv',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
