from os.path import expanduser, exists
from os import makedirs
import rclpy
import yaml
from rclpy.node import Node
import datetime

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PositionLoggingNode(Node):

    def __init__(self):
        super().__init__('position_logging_node')
        time_now = datetime.datetime.now()
        time_now_string = f"{time_now.year}{time_now.month:02d}{time_now.day:02d}{time_now.hour:02d}{time_now.minute:02d}{time_now.second:02d}"
        self.declare_parameter('session_start_time_string', time_now_string).value
        self.session_start_time_string = self.get_parameter('session_start_time_string').value
        self.session_id = self.session_start_time_string

        default_logs_dir = expanduser("~") + "/Documents/wpf/logs/" + self.session_id
        self.declare_parameter('logs_directory', default_logs_dir)
        
        
        self.logs_dir = self.get_parameter('logs_directory').value

        if not exists(self.logs_dir):
            makedirs(self.logs_dir)

        if exists(self.logs_dir + '/ground_truth.yaml'):
            self.get_logger().warn(f'Log file {self.logs_dir}/ground_truth.yaml did already exists. Overwriting it...')
            open(self.logs_dir + '/ground_truth.yaml', 'w').close()
        if exists(self.logs_dir + '/localization.yaml'):
            self.get_logger().warn(f'Log file {self.logs_dir}/localization.yaml did already exists. Overwriting it...')
            open(self.logs_dir + '/localization.yaml', 'w').close()
        if exists(self.logs_dir + '/path.yaml'):
            self.get_logger().warn(f'Log file {self.logs_dir}/path.yaml did already exists. Overwriting it...')
            open(self.logs_dir + '/path.yaml', 'w').close()

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

        with open((self.logs_dir + '/path.yaml'), 'a') as logfile:
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

        with open((self.logs_dir + '/ground_truth.yaml'), 'a') as logfile:
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

        with open((self.logs_dir + '/localization.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    postion_logging_node = PositionLoggingNode()
    rclpy.spin(postion_logging_node)
    postion_logging_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
