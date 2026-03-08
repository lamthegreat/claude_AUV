from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('auv_motor_controller'),
        'config',
        'motor_controller_params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='auv_motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            namespace='auv',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
