"""
Simulation Bringup (Placeholder)

TODO: Add Gazebo / simulation environment when available.
For now, launches the autonomy stack with ROS_DOMAIN_ID isolation.
Use rosbag playback to inject simulated sensor data.
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
        # State estimator, controller, navigation only (no hardware bridges in sim)
        launch('auv_state_estimator', 'state_estimator.launch.py'),
        launch('auv_controller',      'controller.launch.py'),
        launch('auv_safety',          'safety.launch.py'),
        launch('auv_navigation',      'navigation.launch.py'),
    ])
