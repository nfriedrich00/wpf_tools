from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    decl_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                   ],
        output='screen',
        remappings=[('/world/empty/dynamic_pose/info', '/pose/tf'),
                    ]
    )

    # Publisher
    ground_truth_publisher = Node(
        package='wpf_tools',
        executable='ground_truth_publisher',
        output='screen',
        parameters=[
                {'use_sim_time': use_sim_time}
            ]
    )

    return LaunchDescription([
        decl_use_sim_time,
        bridge,
        ground_truth_publisher
    ])
