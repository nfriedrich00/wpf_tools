''' Log time when starting navigation, moving and wait until goal is
reached to log nav finish
'''
import os
from math import sqrt, atan2, asin
import datetime
import yaml
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, Twist


def euler_from_quaternion(orientation):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
    t1 = +1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class GoalChecker(Node):
    goal_pose = Pose()
    xy_tolerance = float()
    yaw_tolerance = float()
    is_path_calculated = False

    def __init__(self):
        super().__init__('goal_checker')
        self.get_logger().info('goal checker started')

        time_now = datetime.datetime.now()
        time_now_string = f"{time_now.year}{time_now.month:02d}{time_now.day:02d}{time_now.hour:02d}{time_now.minute:02d}{time_now.second:02d}"
        self.declare_parameter('session_start_time_string', time_now_string)
        self.session_start_time_string = self.get_parameter('session_start_time_string').value
        session_id = self.get_parameter('session_start_time_string').value

        home_dir = os.path.expanduser('~')
        logs_dir = home_dir + '/Documents/wpf/logs/goal_checker'
        self.logfile = f'{logs_dir}/{session_id}.yaml'

        self.declare_parameter('xy_tolerance', 0.1)
        self.declare_parameter('yaw_tolerance', 3.14)
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        # Create log file folder if not already there
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        # Get time when starting to move
        self.is_moving = False
        self.subscription_twist = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback_twist,
            10
        )
        self.subscription_twist # prevent unused variable warning

        # Get goal postion
        self.subscription_path = self.create_subscription(
            Path,
            'plan',
            self.listener_callback_path,
            10)
        self.subscription_path  # prevent unused variable warning

        # Get current position according to localization
        self.subscription_localization = self.create_subscription(
            Odometry,
            'odometry/global',
            self.listener_callback_localization,
            10
        )
        self.subscription_localization # prevent unused variable warning


    def listener_callback_twist(self, msg):
        ''' This method is called when receiving Twist commands to move
        on /cmd_vel. It is supposed to log the time of the fist message
        in the log file and then destroy listener.
        The movement start time can be used to calculated real speed.
        '''
        if self.is_moving:
            return

        self.is_moving = True
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        # two versions to log data, second is probably better
        data = {time_now_float: {'start moving': time_now_float}}
        data1 = {'start moving': time_now_float}

        with open(self.logfile, 'a', encoding = 'utf-8') as file:
            yaml.dump(data, file, default_flow_style=False)
            yaml.dump(data1, file, default_flow_style=False)

        self.destroy_subscription(self.subscription_twist)


    def log_goal_succeeded(self):
        ''' This method is called once when the goal is reached to log 
        the time in the log file.
        Afterwards the goal checker node gets destroyed.
        '''
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        data = {time_now_float: {'goal succeeded': time_now_float}}
        data1 = {'goal succeeded': time_now_float}

        with open(self.logfile, 'a', encoding = 'utf-8') as file:
            yaml.dump(data, file, default_flow_style=False)
            yaml.dump(data1, file, default_flow_style=False)

        raise SystemExit

    def check_if_goal_succeeded(self, pose):
        ''' This method checks, whether the goal is reached. It is
        called, whenever the localization receives an update.
        It checks, whether the Euclidean distance to the goal pose and
        the difference is between goal orientation and localization
        orientation is within the tolerance.
        '''
        if not self.is_path_calculated:
            return

        xy_distance_to_goal = sqrt((pose.position.x - self.goal_pose.position.x) ** 2 + (pose.position.y - self.goal_pose.position.y) ** 2)
        self.get_logger().debug(f'distance to goal (xy): {xy_distance_to_goal}')

        if not xy_distance_to_goal < self.xy_tolerance:
            return
        else:
            yaw_distance_to_goal = abs(euler_from_quaternion(pose.orientation)[2] - euler_from_quaternion(self.goal_pose.orientation)[2])
            self.get_logger().debug(f'distance to goal (yaw): {yaw_distance_to_goal}')

        if not yaw_distance_to_goal < self.yaw_tolerance:
            return
        else:
            self.get_logger().info('goal reached')
            self.destroy_subscription(self.subscription_localization)
            self.log_goal_succeeded()


    def listener_callback_path(self, msg):
        self.get_logger().info('path received')
        self.goal_pose = msg.poses[-1].pose
        self.get_logger().info(f'goal pose: {self.goal_pose}')
        self.is_path_calculated = True
        self.destroy_subscription(self.subscription_path)

    def listener_callback_localization(self, msg):
        self.check_if_goal_succeeded(pose = msg.pose.pose)

def main(args=None):
    rclpy.init(args=args)
    goal_checker = GoalChecker()
    rclpy.spin(goal_checker)
    goal_checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
