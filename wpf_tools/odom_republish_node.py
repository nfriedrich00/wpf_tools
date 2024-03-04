import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomRepublish(Node):
    def __init__(self):
        super().__init__('odom_republish')

        #Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            'odom/gazebo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #Publisher
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)


    def listener_callback(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    odom_republish = OdomRepublish()

    rclpy.spin(odom_republish)
    odom_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
