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
        parameters=[{'use_sim_time' : True,
                     'autostart': True,
                    },
                    config_filepath],
    )

    restart_experiment_manager = Node(
        package='claudi_monitoring',
        executable='manager',
        name='wpf_launch_manager_node',
        parameters=[config_filepath],
    )


    ld = LaunchDescription()
    ld.add_action(log_decision_module_state)
    ld.add_action(decl_config_filepath)
    ld.add_action(start_experiment)
    ld.add_action(restart_experiment_manager)

    return ld
