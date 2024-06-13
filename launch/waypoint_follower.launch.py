import os
import yaml
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import NotSubstitution



def generate_launch_description():
    this_pkg_share = get_package_share_directory('wpf_tools')
    config_filepath = os.path.join(this_pkg_share, 'config/waypoint_follower_config.yaml')

    session_start_time = datetime.datetime.now()
    sst = session_start_time
    session_start_time_string = f"{sst.year}{sst.month:02d}{sst.day:02d}{sst.hour:02d}{sst.minute:02d}{sst.second:02d}"
    #waypoints_filepath = LaunchConfiguration('waypoints_filepath')
    #decl_waypoints_filepath = DeclareLaunchArgument('waypoints_filepath', default_value=os.path.join(this_pkg_share, 'config/waypoints.yaml'), description='Path to the waypoints yaml file.')


    run_headless = LaunchConfiguration('run_headless')
    decl_run_headless = DeclareLaunchArgument('run_headless', default_value='False')

    with open(config_filepath, 'r') as config_file:
        config_data = yaml.safe_load(config_file)

    waypoints_filepath = os.path.expanduser("~") + config_data.get("waypoints_filepath")
    gps_error_simulator_config_filepath = os.path.expanduser("~") + config_data.get("gps_error_simulator_config_filepath")
    planner_config_filepath = os.path.expanduser("~") + config_data.get("nav_planner_config_filepath")
    controller_config_filepath = os.path.expanduser("~") + config_data.get("nav_controller_config_filepath")


    start_monitoring = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('wpf_tools'), 'launch', 'monitor.launch.py']))
            )
        ]
    )
    
    start_rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("claudi_viz"), 'launch', 'view_robot.launch.py'])),
                launch_arguments = {'use_sim_time' : 'True'}.items(),
                condition=IfCondition(NotSubstitution(run_headless))
                # try out SetUseSimTime instead
            )

    start_gazebo = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("claudi_gazebo"), 'launch', 'gazebo_sim.launch.py'])),
                launch_arguments = {'run_headless' : run_headless}.items(),
            ),
        ]
    )

    start_ground_truth_publisher = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("wpf_tools"), 'launch', 'ground_truth_publisher.launch.py']))
            ),
        ]
    )

    unpause_simulation = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call',
            ' /world/empty/control',
            ' ros_gz_interfaces/srv/ControlWorld',
            ' "{world_control: {pause: false}}"'
        ]],
        shell=True
    )

    start_localization = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('claudi_navigation'), 'launch','localization_gazebo_sim.launch.py'])),
                launch_arguments = {'gnss_input_topic' : 'emlid/modified'}.items()
            )
        ]
    )

    # The localization starts, as soon as the simulation starts.
    simulation_started_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=unpause_simulation,
            on_exit=[start_localization]
        )
    )

    wait_for_localization = Node(
        package='wpf_tools',
        executable='wait_for_localization',
        output='screen'
    )


    start_navigation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('claudi_navigation'), 'launch', 'navigation.launch.py'])),
                launch_arguments = {'planner_config_filepath' : planner_config_filepath,
                                    'controller_config_filepath' : controller_config_filepath}.items()
            )
        ]
    )

    # The navigation starts, as soon as the localization is ready.
    localization_ready_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_localization,
            on_exit=[start_navigation]
        )
    )

    wait_for_navigation = Node(
        package='wpf_tools',
        executable='wait_for_navigation',
        output='screen'
    )


    start_waypoint_follower = Node(
        package='claudi_navigation',
        executable='waypoint_follower',
        output='screen',
        parameters=[{'waypoints_yaml_file_path' : waypoints_filepath}]
    )


    start_gps_error_simulator = Node(
        package='wpf_tools',
        executable='gps_error_simulator',
        output='screen',
        parameters=[{'use_sim_time' : True},
                    {'session_start_time_string' : session_start_time_string},
                    # use the start time from this launch file so all nodes
                    # have the same time to use as logfile identifier 
                   gps_error_simulator_config_filepath]
    )


    start_path_plotter = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('wpf_tools'), 'launch', 'path_plotter.launch.py']))
            )
        ]
    )

    start_goal_checker = Node(
        package='wpf_tools',
        executable='goal_checker',
        output='screen',
        parameters=[{'use_sim_time' : True},
                    {'session_start_time_string' : session_start_time_string}
                    # use the start time from this launch file so all nodes
                    # have the same time to use as logfile identifier 
                   ]
    )

    start_position_logging = Node(
        package='wpf_tools',
        executable='log_position',
        output='screen',
        parameters=[{'use_sim_time' : True},
                    {'session_start_time_string' : session_start_time_string}
                    # use the start time from this launch file so all nodes
                    # have the same time to use as logfile identifier 
                   ]
    )

    # The waypoint follower starts, as soon as the navigation is ready.
    navigation_ready_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_navigation,
            on_exit=[start_waypoint_follower,
                     start_gps_error_simulator,
                     start_path_plotter,
                     start_goal_checker,
                     start_position_logging]
        )
    )


    ld = LaunchDescription()
    ld.add_action(start_monitoring)
    ld.add_action(decl_run_headless)
    ld.add_action(start_rviz)
    ld.add_action(start_gazebo)
    ld.add_action(start_ground_truth_publisher)
    ld.add_action(unpause_simulation)
    ld.add_action(simulation_started_callback)
    ld.add_action(wait_for_localization)
    ld.add_action(localization_ready_callback)
    ld.add_action(wait_for_navigation)
    ld.add_action(navigation_ready_callback)


    return ld
