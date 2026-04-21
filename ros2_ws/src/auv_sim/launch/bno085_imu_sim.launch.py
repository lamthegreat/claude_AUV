from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    profile = LaunchConfiguration('profile')
    imu_rate_hz = LaunchConfiguration('imu_rate_hz')
    mag_rate_hz = LaunchConfiguration('mag_rate_hz')
    start_fully_calibrated = LaunchConfiguration('start_fully_calibrated')
    dropout_probability = LaunchConfiguration('dropout_probability')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='auv'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('profile', default_value='idle'),
        DeclareLaunchArgument('imu_rate_hz', default_value='200.0'),
        DeclareLaunchArgument('mag_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('start_fully_calibrated', default_value='true'),
        DeclareLaunchArgument('dropout_probability', default_value='0.0'),
        Node(
            package='auv_sim',
            executable='bno085_imu_sim_node',
            name='bno085_imu_sim_node',
            parameters=[{
                'namespace': namespace,
                'frame_id': frame_id,
                'profile': profile,
                'imu_rate_hz': ParameterValue(imu_rate_hz, value_type=float),
                'mag_rate_hz': ParameterValue(mag_rate_hz, value_type=float),
                'start_fully_calibrated': ParameterValue(start_fully_calibrated, value_type=bool),
                'dropout_probability': ParameterValue(dropout_probability, value_type=float),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
