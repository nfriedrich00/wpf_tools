import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

class GroundTruthPublisher(Node):
    base_link_pose = PoseStamped()
    header = Header()
    
    path = Path()
    pose_stamped = PoseStamped()
    waypoints = []
    last_pose_stamped = PoseStamped()
    counter = 0

    def __init__(self):
        super().__init__('ground_truth_publisher')

        # This is probably not necessary, since the node has this parameter per default and it can be set when starting the node. This is just to set the default to true.
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([use_sim_time_param])

        #Subscriber TFMessage
        self.subscription = self.create_subscription(
            TFMessage,
            'pose/tf',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


        #Publisher
        self.publisher_ = self.create_publisher(PoseStamped, 'pose/base_link', 10)


    def listener_callback(self, msg):
        if(len(msg.transforms) > 0):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            
            pose_stamped.pose.position.x = msg.transforms[0].transform.translation.x
            pose_stamped.pose.position.y = msg.transforms[0].transform.translation.y
            pose_stamped.pose.position.z = msg.transforms[0].transform.translation.z

            pose_stamped.pose.orientation = msg.transforms[0].transform.rotation

            self.publisher_.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    ground_truth_publisher = GroundTruthPublisher()
    rclpy.spin(ground_truth_publisher)
    ground_truth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
