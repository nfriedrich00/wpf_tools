import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')
    decl_config_filepath = DeclareLaunchArgument('config_filepath', default_value='')

    log_decision_module_state = Node(
        package='wpf_tools',
        executable='logging_node',
        name='log_decision_module_state',
        parameters=[{'use_sim_time' : True},
                    config_filepath],
    )

    start_experiment = Node(
        package='claudi_monitoring',
        executable='lifecycle_launch_node',
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
    ld.add_action(log_decision_module_state)
    ld.add_action(decl_config_filepath)
    ld.add_action(start_experiment)
    ld.add_action(restart_experiment_manager)
    ld.add_action(lifecycle_manager)

    return ld
