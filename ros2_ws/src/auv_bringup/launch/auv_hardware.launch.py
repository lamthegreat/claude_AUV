"""
Hardware-Only Bringup

Launches sensor hub and motor controller bridge nodes.
Use this during initial hardware bring-up before adding the autonomy stack.
The microROS agents must be started separately before launching this file.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_hub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_sensor_hub'), 'launch', 'sensor_hub.launch.py'])))

    motor_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_motor_controller'), 'launch', 'motor_controller.launch.py'])))

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_safety'), 'launch', 'safety.launch.py'])))

    return LaunchDescription([
        sensor_hub,
        motor_ctrl,
        safety,
    ])
