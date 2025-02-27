from os.path import expanduser, exists
from os import makedirs
import rclpy
import yaml
from rclpy.node import Node
import datetime
from collections import OrderedDict
from rosidl_runtime_py.utilities import get_interface
from rosidl_runtime_py.convert import message_to_ordereddict

def ordered_to_dict(obj):
    if isinstance(obj, OrderedDict):
        return {k: ordered_to_dict(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [ordered_to_dict(el) for el in obj]
    else:
        return obj


class LoggingNode(Node):

    def __init__(self):
        super().__init__('logging_node')
        time_now = datetime.datetime.now()
        time_now_string = f"{time_now.year}{time_now.month:02d}{time_now.day:02d}{time_now.hour:02d}{time_now.minute:02d}{time_now.second:02d}"
        self.declare_parameter('session_start_time_string', time_now_string).value
        self.session_start_time_string = self.get_parameter('session_start_time_string').value
        self.session_id = self.session_start_time_string

        default_logs_dir = expanduser("~") + "/Documents/wpf/logs/" + self.session_id
        self.declare_parameter('logs_directory', default_logs_dir)
        self.declare_parameter('topic', '')
        self.declare_parameter('message_type', '')
        topic = self.get_parameter('topic').value
        self.declare_parameter('logfile_name', f'logfile-{topic[1:]}.yaml')
        message_type = self.get_parameter('message_type').value
        self.logfile_name = self.get_parameter('logfile_name').value
        self.logs_dir = self.get_parameter('logs_directory').value

        if not topic or not message_type:
            self.get_logger().error('Topic and message_type are required. Aborting.')
            raise SystemExit

        if not self.logfile_name.endswith('.yaml'):
            self.logfile_name += '.yaml'

        self.interface = get_interface(message_type)

        if not exists(self.logs_dir):
            makedirs(self.logs_dir)

        if exists(self.logs_dir + '/' + self.logfile_name):
            self.get_logger().warn(f'Log file {self.logs_dir}/{self.logfile_name} did already exists. Overwriting it...')
            open(self.logs_dir + '/' + self.logfile_name, 'w').close()

        #Subscriber
        self.subscription = self.create_subscription(
            self.interface,
            topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        
        msg_dict = message_to_ordereddict(msg)
        plain_dict = ordered_to_dict(msg_dict)
        yaml_data = {time_now_float: plain_dict}
        with open((self.logs_dir + '/' + self.logfile_name), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    logging_node = LoggingNode()
    rclpy.spin(logging_node)
    logging_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
