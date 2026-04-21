"""
Simulation Bringup (Placeholder)

TODO: Add Gazebo / simulation environment when available.
For now, launches the autonomy stack with the lightweight BNO085 IMU simulator.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def launch(pkg_name, file_name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg_name), 'launch', file_name])))

    return LaunchDescription([
        launch('auv_sim', 'bno085_imu_sim.launch.py'),
        launch('auv_state_estimator', 'state_estimator.launch.py'),
        launch('auv_controller',      'controller.launch.py'),
        launch('auv_safety',          'safety.launch.py'),
        launch('auv_navigation',      'navigation.launch.py'),
    ])
