import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_pkg_share = get_package_share_directory('wpf_tools')
    gazebo_config = os.path.join(this_pkg_share,
                                 'config/monitor_node_gazebo.yaml')
    localization_config = os.path.join(this_pkg_share,
                                       'config/monitor_node_localization.yaml')
    navigation_config = os.path.join(this_pkg_share,
                                     'config/monitor_node_navigation.yaml')
    goal_checker_config = os.path.join(this_pkg_share,
                                     'config/monitor_node_goal_checker.yaml')

    start_monitor_gazebo = Node(
        package='wpf_tools',
        executable='start_topic_monitor',
        output='screen',
        parameters=[gazebo_config]
    )

    start_monitor_localization = Node(
        package='wpf_tools',
        executable='start_topic_monitor',
        output='screen',
        parameters=[localization_config]
    )

    start_monitor_navigation = Node(
        package='wpf_tools',
        executable='start_topic_monitor',
        output='screen',
        parameters=[navigation_config]
    )

    start_monitor_goal_checker = Node(
        package='wpf_tools',
        executable='start_topic_monitor',
        output='screen',
        parameters=[goal_checker_config]
    )


    ld = LaunchDescription()
    ld.add_action(start_monitor_gazebo)
    ld.add_action(start_monitor_localization)
    ld.add_action(start_monitor_navigation)
    ld.add_action(start_monitor_goal_checker)


    return ld
