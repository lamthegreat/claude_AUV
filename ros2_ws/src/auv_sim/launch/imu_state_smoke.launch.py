from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    profile = LaunchConfiguration('profile')
    observation_window_s = LaunchConfiguration('observation_window_s')

    state_estimator_config = PathJoinSubstitution([
        FindPackageShare('auv_state_estimator'),
        'config',
        'ekf_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='auv'),
        DeclareLaunchArgument('profile', default_value='attitude_sweep'),
        DeclareLaunchArgument('observation_window_s', default_value='2.0'),
        Node(
            package='auv_sim',
            executable='bno085_imu_sim_node',
            name='bno085_imu_sim_node',
            parameters=[{
                'namespace': namespace,
                'profile': profile,
                'start_fully_calibrated': True,
            }],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='auv_sensor_hub',
            executable='sensor_hub_node',
            name='sensor_hub_node',
            parameters=[{'namespace': namespace}],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='auv_state_estimator',
            executable='state_estimator_node',
            name='state_estimator_node',
            parameters=[
                state_estimator_config,
                {'namespace': namespace},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='auv_sim',
            executable='imu_state_smoke_runner',
            name='imu_state_smoke_runner',
            parameters=[{
                'namespace': namespace,
                'observation_window_s': ParameterValue(observation_window_s, value_type=float),
            }],
            output='screen',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),
    ])
