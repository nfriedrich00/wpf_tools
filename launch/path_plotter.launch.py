from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    decl_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')

    path_localization_enabled = LaunchConfiguration('path_localization_enabled', default='True')
    decl_path_localization_enabled = DeclareLaunchArgument('path_localization_enabled', default_value='True')
    path_ground_truth_enabled = LaunchConfiguration('path_ground_truth_enabled', default="True")
    decl_path_ground_truth_enabled = DeclareLaunchArgument('path_ground_truth_enabled', default_value='True')

    # Path plotter ground truth (requires ground truth published as PoseStamped on /pose/base_link)
    path_plotter_ground_truth = GroupAction( 
        condition=IfCondition(path_ground_truth_enabled),
        actions=[
            Node(
                package='wpf_tools',
                executable='path_plotter_ground_truth',
                output='screen',
                parameters=[
                        {'use_sim_time': use_sim_time}
                    ]
                )
        ]
    )


    # Path plotter localization (odometry/global)
    path_plotter_localization = GroupAction( 
        condition=IfCondition(path_localization_enabled),
        actions=[
            Node(
                package='wpf_tools',
                executable='path_plotter_localization',
                output='screen',
                parameters=[
                        {'use_sim_time': use_sim_time}
                    ]
                )
        ]
    )

    return LaunchDescription([
        decl_use_sim_time,
        path_plotter_ground_truth,
        path_plotter_localization,
        decl_path_localization_enabled,
        decl_path_ground_truth_enabled
    ])
