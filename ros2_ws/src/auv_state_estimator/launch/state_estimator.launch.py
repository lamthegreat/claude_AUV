from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('auv_state_estimator'), 'config', 'ekf_params.yaml'])

    return LaunchDescription([
        Node(
            package='auv_state_estimator',
            executable='state_estimator_node',
            name='state_estimator_node',
            namespace='auv',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
