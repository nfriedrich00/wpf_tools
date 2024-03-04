import rclpy
from rclpy.node import Node

from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlotterGroundTruth(Node):

    path = Path()
    path.header.frame_id = 'map'
    counter = 0
    pose_stamped = PoseStamped()

    def __init__(self):
        super().__init__('path_plotter_ground_truth')

        #Subsccriber
        self.subscription = self.create_subscription(
            PoseStamped,
            'pose/base_link',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #Publisher
        self.publisher_ = self.create_publisher(Path, 'path/ground_truth', 10)


    def listener_callback(self, msg):
        self.pose_stamped = msg
        self.pose_stamped.pose.position.z = 0.0 # base link is at z=$(wheel_radius)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(deepcopy(self.pose_stamped))

        if self.counter % 10 == 0:
            self.publisher_.publish(self.path)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    path_plotter_ground_truth = PathPlotterGroundTruth()
    rclpy.spin(path_plotter_ground_truth)
    path_plotter_ground_truth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
