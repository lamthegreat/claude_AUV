from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('auv_sensor_hub'),
        'config',
        'sensor_hub_params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='auv_sensor_hub',
            executable='sensor_hub_node',
            name='sensor_hub_node',
            namespace='auv',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
