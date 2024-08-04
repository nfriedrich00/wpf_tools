import rclpy
from rclpy.executors import MultiThreadedExecutor
from enum import Enum
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

class MonitorStatus(Enum):
    NOT_READY = 0
    OK = 1
    UNRELIABLE = 2
    ERROR = 3


class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        #test = Odometry()
        #print(type(test))

        self.topic = self.declare_parameter('topic_name', 'not_set').value
        self.message_type = self.declare_parameter('message_type', 'not_set').value
        match self.message_type:
            case 'Odometry':
                self.message_type = Odometry
            case 'OccupancyGrid':
                self.message_type = OccupancyGrid
            case _:
                self.message_type = Empty
        self.domain = self.declare_parameter('domain', 'not_set').value
        min_f = self.declare_parameter('min_f', 0.5).value
        timer_period_sec = 1/min_f

        update_f = self.declare_parameter('update_f', 1.0).value
        pub_period_sec = 1/update_f


        self.timer_ = self.create_timer(timer_period_sec, self.timer_callback)
        self.timer_pub_ = self.create_timer(pub_period_sec,
                                            self.timer_callback_pub)
        self.status = MonitorStatus.NOT_READY

        self.subscription_ = self.create_subscription(
            self.message_type,
            self.topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        match self.status:
            case MonitorStatus.NOT_READY:
                self.get_logger().info('First message received on topic' \
                                       f' {self.topic}. {self.domain}' \
                                       ' is ready now.')
                self.status = MonitorStatus.OK
                self.timer_.reset()
            case MonitorStatus.OK:
                self.timer_.reset()
            case MonitorStatus.UNRELIABLE:
                self.timer_.reset()
            case MonitorStatus.ERROR:
                self.get_logger().info(f'Overdue message received on topic' \
                                       f' {self.topic}. {self.domain}' \
                                        ' seems to work unreliable.')
                self.status = MonitorStatus.UNRELIABLE
                self.timer_.reset()
            case _:
                self.get_logger().error(f'Unexpected status error for'\
                                        f' {self.domain}.')
                raise SystemError
    

    def timer_callback(self):
        match self.status:
            case MonitorStatus.OK:
                self.get_logger().info(f'{self.topic} message overdue.' \
                                        ' There seems to be a problem' \
                                       f' with {self.domain}.')
                self.status = MonitorStatus.ERROR
            case _:
                return
    

    def timer_callback_pub(self):
        try:
            self.destroy_publisher(self.publisher)
        except:
            pass

        #match self.status:
        #    case MonitorStatus.NOT_READY:
        #        topic_name = f'status/{self.domain}/not_ready'
        #    case MonitorStatus.OK:
        #        topic_name = f'status/{self.domain}/ok'
        #    case MonitorStatus.UNRELIABLE:
        #        topic_name = f'status/{self.domain}/unreliable'
        #    case MonitorStatus.ERROR:
        #        topic_name = f'status/{self.domain}/error'
        #    case _:
        #        self.get_logger().error(f'Unexpected status error for'\
        #                                f' {self.domain}.')
        #        raise SystemError

        topic_name = f'status/{self.domain}/{self.status.name}'
        self.publisher = self.create_publisher(Empty,
                                               topic_name,
                                               10)
        self.publisher.publish(Empty())

            
        

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
