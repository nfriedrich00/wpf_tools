import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition


def raise_key_not_found_error(key, filepath):
    raise ValueError(key + ' not found in config file at ' + filepath)


def start_wpf_lifecycle_node(context, *args, **kwargs):
    waypoints_filepath = LaunchConfiguration('waypoints_filepath').perform(context)
    use_simulation = LaunchConfiguration('use_simulation').perform(context)
    config_filepath = LaunchConfiguration('config_filepath').perform(context)

    if waypoints_filepath == 'placeholder - will raise error if not set':
        raise ValueError('waypoints_filepath not set')

    additional_arguments = '-p waypoints_yaml_filepath:=' + waypoints_filepath

    wpf_lifecycle_node = Node(
        package='claudi_monitoring',
        executable='lifecycle_node',
        name='wpf_lifecycle_node',
        output='screen',
        parameters=[{'use_sim_time' : bool(use_simulation),
                     'waypoint_follower.additional_arguments' : additional_arguments},
                    config_filepath]
    )

    return [wpf_lifecycle_node]



def generate_launch_description():
    this_pkg_share = get_package_share_directory('wpf_tools')

    config_filepath = LaunchConfiguration('config_filepath')
    decl_config_filepath = DeclareLaunchArgument('config_filepath', default_value='')

    start_experiment = Node(
        package='claudi_monitoring',
        executable='lifecycle_launch',
        name='wpf_launch_lifecycle_node',
        output='screen',
        parameters=[{'use_sim_time' : True},
                    config_filepath],
    )

    restart_experiment_manager = Node(
        package='claudi_monitoring',
        executable='manager',
        name='wpf_launch_manager_node',
        parameters=[config_filepath],
    )

    # Autostart experiment
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_autostart',
        output='screen',
        parameters=[{'autostart': True,
                     'bond_timeout': 0.0, # python lifecycle node -> disable bond check
                     'node_names': ['wpf_launch_lifecycle_node']}],
    )


    ld = LaunchDescription()
    ld.add_action(decl_config_filepath)
    ld.add_action(start_experiment)
    ld.add_action(restart_experiment_manager)
    ld.add_action(lifecycle_manager)

    return ld
