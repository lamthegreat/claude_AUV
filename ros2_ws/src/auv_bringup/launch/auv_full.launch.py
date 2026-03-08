"""
Full AUV Bringup

Launches all nodes: hardware bridges, state estimator, controller,
safety monitor, and mission planner.

The microROS agents must be started separately before launching this file.
See CLAUDE.md for the agent commands.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'mission_file', default_value='', description='Optional mission YAML to load')

    def pkg(name):
        return FindPackageShare(name)

    def launch(pkg_name, file_name, extra_args=None):
        kwargs = {}
        if extra_args:
            kwargs['launch_arguments'] = extra_args
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg(pkg_name), 'launch', file_name])),
            **kwargs)

    return LaunchDescription([
        mission_arg,
        launch('auv_sensor_hub',      'sensor_hub.launch.py'),
        launch('auv_motor_controller', 'motor_controller.launch.py'),
        launch('auv_state_estimator',  'state_estimator.launch.py'),
        launch('auv_controller',       'controller.launch.py'),
        launch('auv_safety',           'safety.launch.py'),
        launch('auv_navigation',       'navigation.launch.py',
               {'mission_file': LaunchConfiguration('mission_file')}.items()),
    ])
