from os.path import expanduser, exists
from os import makedirs, mkdir
import os
import rclpy
import yaml
from rclpy.node import Node
import datetime

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPlotterGroundTruth(Node):

    path = Path()
    path.header.frame_id = 'map'
    counter = 0
    pose_stamped = PoseStamped()

    def __init__(self):
        super().__init__('path_plotter_ground_truth')
        time_now = datetime.datetime.now()
        time_now_string = f"{time_now.year}{time_now.month:02d}{time_now.day:02d}{time_now.hour:02d}{time_now.minute:02d}{time_now.second:02d}"
        self.declare_parameter('session_start_time_string', time_now_string).value
        self.session_start_time_string = self.get_parameter('session_start_time_string').value
        self.session_identifier = self.session_start_time_string
        
        
        self.logs_path = expanduser("~") + "/Documents" + "/wpf/logs/position"

        self.declare_parameter('logs_path', self.logs_path)
        if self.get_parameter('logs_path').value is not None:
            self.logs_path = os.path.join(self.get_parameter('logs_path').value, 'position')

        if not exists(self.logs_path):
            makedirs(self.logs_path)
        mkdir(self.logs_path + f'/{self.session_identifier}')

        #Subscriber
        self.subscription_ground_truth = self.create_subscription(
            PoseStamped,
            'pose/base_link',
            self.listener_callback_ground_truth,
            10)
        self.subscription_ground_truth  # prevent unused variable warning

        #Subscriber
        self.subscription_localization = self.create_subscription(
            Odometry,
            'odometry/global',
            self.listener_callback_localization,
            10)
        self.subscription_localization  # prevent unused variable warning

        #Subscriber
        self.subscription_path = self.create_subscription(
            Path,
            'plan',
            self.listener_callback_path,
            10)
        self.subscription_path  # prevent unused variable warning


    def listener_callback_path(self, msg):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        time_message_float = float(f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

        path_data = [{'x': waypoint_pose_stamped.pose.position.x,
                    'y': waypoint_pose_stamped.pose.position.y,
                    'z': waypoint_pose_stamped.pose.position.z,
                    'q0': waypoint_pose_stamped.pose.orientation.x,
                    'q1': waypoint_pose_stamped.pose.orientation.y,
                    'q2': waypoint_pose_stamped.pose.orientation.z,
                    'q3': waypoint_pose_stamped.pose.orientation.w} for waypoint_pose_stamped in msg.poses]
        yaml_data = {time_now_float: {'time': time_message_float,
                                      'waypoints': path_data
                                    }}
        
        with open((self.logs_path + f'/{self.session_identifier}/path.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)

        self.destroy_subscription(self.subscription_path)

    def listener_callback_ground_truth(self, msg):
        time_now = self.get_clock().now().to_msg()
        time_now_ns_to_s = time_now.nanosec/1e9
        time_now_float = time_now.sec + time_now_ns_to_s
        time_message_ns_to_s = msg.header.stamp.nanosec/1e9
        time_message_float = msg.header.stamp.sec + time_message_ns_to_s

        position_data = {  'x': msg.pose.position.x,
                            'y': msg.pose.position.y,
                            'z': msg.pose.position.z,
                            'q0': msg.pose.orientation.x,
                            'q1': msg.pose.orientation.y,
                            'q2': msg.pose.orientation.z,
                            'q3': msg.pose.orientation.w
                        }
        yaml_data = {time_now_float: {'time': time_message_float,
                                      'position': position_data
                                      }}
        
        with open((self.logs_path + f'/{self.session_identifier}/ground_truth.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)

    def listener_callback_localization(self, msg):
        time_now = self.get_clock().now().to_msg()
        time_now_ns_to_s = time_now.nanosec/1e9
        time_now_float = time_now.sec + time_now_ns_to_s
        time_message_ns_to_s = msg.header.stamp.nanosec/1e9
        time_message_float = msg.header.stamp.sec + time_message_ns_to_s

        position_data = {  'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z,
                            'q0': msg.pose.pose.orientation.x,
                            'q1': msg.pose.pose.orientation.y,
                            'q2': msg.pose.pose.orientation.z,
                            'q3': msg.pose.pose.orientation.w
                        }
        yaml_data = {time_now_float: {'time': time_message_float,
                                      'position': position_data
                                      }}
        
        with open((self.logs_path + f'/{self.session_identifier}/localization.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    path_plotter_ground_truth = PathPlotterGroundTruth()
    rclpy.spin(path_plotter_ground_truth)
    path_plotter_ground_truth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
