import rclpy
from rclpy.node import Node

from waypoint_following_decision_module_msgs.msg import DecisionModuleStateStamped, DecisionModuleState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class DMStateRepublishNode(Node):
    def __init__(self):
        super().__init__('dm_state_republish_node')

        self.marker_offset_x = 2.0 # to counteract delay
        self.marker_offset_y = 2.5
        self.marker_offset_z = 1.0

        self.scale_offset_y = -0.5

        self.max_dwell_time = 12.65 # hardcoded max dwell time from the decision module params file

        #Subscriber state
        self.subscription = self.create_subscription(
            DecisionModuleStateStamped,
            'decision_module_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #Subscriber robot position
        self.subscription = self.create_subscription(
            Odometry,
            'odometry/global',
            self.listener_callback_position,
            10)
        self.subscription  # prevent unused variable warning

        #Publisher
        self.publisher_marker_ = self.create_publisher(Marker, "decision_module_state_marker", 10)
        self.publisher_bar_red_ = self.create_publisher(Marker, "decision_module_state_bar_red", 10)
        self.publisher_bar_green_ = self.create_publisher(Marker, "decision_module_state_bar_green", 10)

        # Text marker
        self.marker_message = Marker()
        self.marker_message.header.frame_id = 'map'
        self.marker_message.type = Marker.TEXT_VIEW_FACING
        self.marker_message.action = Marker.ADD
        self.marker_message.scale.x = 1.0
        self.marker_message.scale.y = 1.0
        self.marker_message.scale.z = 1.0
        self.marker_message.color.a = 1.0
        self.marker_message.color.r = 0.0
        self.marker_message.color.g = 0.0
        self.marker_message.color.b = 0.0
        self.marker_message.ns = 'decision module state'
        self.marker_message.pose.orientation.w = 1.0

        # Red bar (background)
        self.bar_red = Marker()
        self.bar_red.header.frame_id = 'map'
        self.bar_red.type = Marker.CUBE
        self.bar_red.action = Marker.ADD
        self.bar_red.scale.y = 0.19
        self.bar_red.scale.z = 0.19
        self.bar_red.color.a = 1.0
        self.bar_red.color.r = 1.0
        self.bar_red.color.g = 0.0
        self.bar_red.color.b = 0.0
        self.bar_red.ns = 'dwell_time_bar_red'

        # Green bar (foreground)
        self.bar_green = Marker()
        self.bar_green.header.frame_id = 'map'
        self.bar_green.type = Marker.CUBE
        self.bar_green.action = Marker.ADD
        self.bar_green.scale.y = 0.21
        self.bar_green.scale.z = 0.21
        self.bar_green.color.a = 1.0
        self.bar_green.color.r = 0.0
        self.bar_green.color.g = 1.0
        self.bar_green.color.b = 0.0
        self.bar_green.ns = 'dwell_time_bar_green'

        # Fixed length of the bar
        self.max_bar_length = 2.0



    def listener_callback_position(self, msg):
        self.marker_message.pose.position.x = msg.pose.pose.position.x + self.marker_offset_x
        self.marker_message.pose.position.y = msg.pose.pose.position.y + self.marker_offset_y
        self.marker_message.pose.position.z = msg.pose.pose.position.z + self.marker_offset_z

        # keep marker in window
        if self.marker_message.pose.position.x < 1.5:
            self.marker_message.pose.position.x = 1.5
        if self.marker_message.pose.position.x > 37.0:
            self.marker_message.pose.position.x = 37.0

        # Position and size the red bar (always the full length)
        self.bar_red.pose.position.x = self.marker_message.pose.position.x - self.max_bar_length / 2.0  # Centered on x-axis
        self.bar_red.pose.position.y = self.marker_message.pose.position.y - 1.0  # Below the text marker
        self.bar_red.pose.position.z = self.marker_message.pose.position.z
        self.bar_red.scale.x = self.max_bar_length
        # By default, the CUBE marker is centered at its pose.
        # Shifting it so the left edge is at pose.x:
        self.bar_red.pose.position.x += self.bar_red.scale.x / 2.0

        # We'll do a similar approach for the green bar in listener_callback() once we know dwell_scale



    def listener_callback(self, msg):
        self.marker_message.header.stamp = self.get_clock().now().to_msg()
        if msg.state.ha_reason == DecisionModuleState.NO_HA:
            state = 'PERFORMANCE'
        elif msg.state.ha_reason == DecisionModuleState.NO_PLAN:
            state = 'ASSURANCE'
        elif msg.state.ha_reason == DecisionModuleState.OUTSIDE_CONVEX_HULL:
            state = 'ASSURANCE'
        elif msg.state.ha_reason == DecisionModuleState.IN_DWELL_TIME:
            state = 'ASSURANCE'
        else:
            state = 'unknown state'
        self.marker_message.text = state

        if 0 < msg.state.dwell_time_seconds < self.max_dwell_time:
            dwell_scale = msg.state.dwell_time_seconds / self.max_dwell_time
        else:
            dwell_scale = 0.0

        self.publisher_marker_.publish(self.marker_message)

        if state == 'ASSURANCE':
            if 0 < msg.state.dwell_time_seconds < self.max_dwell_time:
                dwell_scale = msg.state.dwell_time_seconds / self.max_dwell_time
            else:
                dwell_scale = 0.0

            self.bar_red.header.stamp = self.get_clock().now().to_msg()
            self.bar_red.action = Marker.ADD
            self.publisher_bar_red_.publish(self.bar_red)

            self.bar_green.header.stamp = self.get_clock().now().to_msg()
            self.bar_green.action = Marker.ADD
            self.bar_green.pose = self.bar_red.pose
            self.bar_green.scale.x = self.max_bar_length * dwell_scale
            self.bar_green.pose.position.x = (
                self.bar_red.pose.position.x
                - (self.bar_red.scale.x / 2.0)
                + (self.bar_green.scale.x / 2.0)
            )
            self.publisher_bar_green_.publish(self.bar_green)

        else:
            # Hide bars in any other state
            self.bar_red.header.stamp = self.get_clock().now().to_msg()
            self.bar_red.action = Marker.DELETE
            self.publisher_bar_red_.publish(self.bar_red)

            self.bar_green.header.stamp = self.get_clock().now().to_msg()
            self.bar_green.action = Marker.DELETE
            self.publisher_bar_green_.publish(self.bar_green)


def main(args=None):
    rclpy.init(args=args)

    dm_state_republish = DMStateRepublishNode()

    rclpy.spin(dm_state_republish)
    dm_state_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
