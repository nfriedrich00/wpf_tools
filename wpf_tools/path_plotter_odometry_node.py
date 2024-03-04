import rclpy
from rclpy.node import Node

from copy import deepcopy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPlotterOdometry(Node):

    path = Path()
    pose_stamped = PoseStamped()
    counter = 0

    def __init__(self):
        super().__init__('path_plotter_odometry')

        #Subsccriber
        self.subscription = self.create_subscription(
            Odometry,
            'odometry/global',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #Publisher
        self.publisher_ = self.create_publisher(Path, 'path/odometry/global', 10)


    def listener_callback(self, msg):
        self.pose_stamped.header = msg.header
        self.pose_stamped.pose = msg.pose.pose
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = msg.header.frame_id
        self.path.poses.append(deepcopy(self.pose_stamped))
        if self.counter % 10 == 0:
            self.publisher_.publish(self.path)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    path_plotter_odometry = PathPlotterOdometry()
    rclpy.spin(path_plotter_odometry)
    path_plotter_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
