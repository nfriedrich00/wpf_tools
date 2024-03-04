from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([
        bridge,
    ])
