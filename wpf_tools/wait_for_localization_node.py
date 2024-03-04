import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid



class LocalizationListener(Node):
    def __init__(self):
        super().__init__('wait_for_localization_node')

        #Subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info('Localization ready.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    wait_for_localization_node = LocalizationListener()

    rclpy.spin(wait_for_localization_node)
    wait_for_localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
