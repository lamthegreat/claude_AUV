from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    command_timeout_ms = LaunchConfiguration('command_timeout_ms')
    surge_force_n = LaunchConfiguration('surge_force_n')
    record_bag = LaunchConfiguration('record_bag')
    bag_output_dir = LaunchConfiguration('bag_output_dir')

    motor_controller_config = PathJoinSubstitution([
        FindPackageShare('auv_motor_controller'),
        'config',
        'motor_controller_params.yaml',
    ])
    thruster_config = PathJoinSubstitution([
        FindPackageShare('auv_motor_controller'),
        'config',
        'thruster_configs',
        'vectored_6dof.yaml',
    ])

    bag_topics = [
        '/auv/control/wrench_output',
        '/auv/thrusters/commands',
        '/auv/thrusters/armed',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='auv'),
        DeclareLaunchArgument('command_timeout_ms', default_value='200'),
        DeclareLaunchArgument('surge_force_n', default_value='1.5'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument(
            'bag_output_dir',
            default_value='/tmp/auv_allocator_smoke_bag',
        ),
        Node(
            package='auv_motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            namespace=namespace,
            parameters=[
                motor_controller_config,
                {
                    'namespace': namespace,
                    'command_timeout_ms': ParameterValue(command_timeout_ms, value_type=int),
                    'thruster_config_path': thruster_config,
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_output_dir, *bag_topics],
            output='screen',
            condition=IfCondition(record_bag),
        ),
        Node(
            package='auv_sim',
            executable='allocator_smoke_runner',
            name='allocator_smoke_runner',
            parameters=[{
                'namespace': namespace,
                'command_timeout_ms': ParameterValue(command_timeout_ms, value_type=int),
                'surge_force_n': ParameterValue(surge_force_n, value_type=float),
            }],
            output='screen',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),
    ])
