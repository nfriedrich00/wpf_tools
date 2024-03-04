import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid



class NavigationListener(Node):
    def __init__(self):
        super().__init__('wait_for_navigation_node')

        #Subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info('Navigation ready.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    wait_for_navigation_node = NavigationListener()

    rclpy.spin(wait_for_navigation_node)
    wait_for_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
