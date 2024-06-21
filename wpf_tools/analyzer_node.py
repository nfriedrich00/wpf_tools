""" Analyze data from claudi simulation to give out and save additional
information like average speed, traveled distance and error.
"""
import os
import glob
from pathlib import Path as pathlibPath
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header
from rclpy.node import Node

from wpf_tools.wpf_utils import (float_to_time, convert_dict_to_list,
                                 get_distance_through_poses, convert_list_type,
                                 get_max_distance, get_max_dist_simpl,
                                 get_avg_distance, get_avg_dist_simpl)


class AnalyzerNode(Node):
    """ Analyzer Node, which analyzes the data logged during any
    claudi simulation. Per default, it analyzes the data of the last
    simulation and generates and logs some additional data,
    like average speed, average/max error and traveled distance.
    """

    def __init__(self):
        super().__init__('analyzer_node')

        # first get session identifier to select files to analyze

        # the following directory contains exactly one file for each session
        self.logs_path = os.path.expanduser("~") + "/Documents/wpf/logs"
        self.declare_parameter('logs_path', None)
        if self.get_parameter('logs_path').value is not None:
            self.logs_path = self.get_parameter('logs_path').value
        goal_checker_dir = self.logs_path + "/goal_checker"
        list_of_goal_checker_log_files = glob.glob(goal_checker_dir + '/*')
        path_of_newest_file = max(
            list_of_goal_checker_log_files,
            key=os.path.getctime)
        name_of_newest_file = pathlibPath(path_of_newest_file).stem

        self.declare_parameter('session_id', name_of_newest_file)
        self.session_id = self.get_parameter('session_id').value

        self.analyze_data()
        raise SystemExit

    def analyze_data(self):
        ''' Start data analysis. Load log files and calculate average
        speed, error and traveled distance.
        '''
        self.get_logger().info('Start loading data ...')
        with open(self.logs_path + '/position/'
                  + self.session_id + '/path.yaml', 'r',
                  encoding='utf-8') as path_file:
            path_data = yaml.safe_load(path_file)

        with open(self.logs_path + '/goal_checker/'
                  + self.session_id + '.yaml', 'r',
                  encoding='utf-8') as goal_checker_file:
            goal_checker_data = yaml.safe_load(goal_checker_file)

        with open(self.logs_path + '/position/'
                  + self.session_id + '/ground_truth.yaml', 'r',
                  encoding='utf-8') as ground_truth_file:
            ground_truth_data = yaml.safe_load(ground_truth_file)

        with open(self.logs_path + '/position/'
                  + self.session_id + '/localization.yaml', 'r',
                  encoding='utf-8') as localization_file:
            localization_data = yaml.safe_load(localization_file)
        self.get_logger().info("Data loaded successfully.")

        # start time when actually starting to move
        start_time = goal_checker_data['start moving']
        finish_time = goal_checker_data['goal succeeded']

        traveled_time = finish_time - start_time

        # Get AVERAGE SPEED
        path_poses = []
        path_key = list(path_data.keys())[0]
        path_stamp = float_to_time(path_key)
        path_header = Header(stamp=path_stamp)
        for waypoint in path_data[path_key]['waypoints']:
            position = Point(x=waypoint['x'],
                             y=waypoint['y'],
                             z=waypoint['z'])
            orientation = Quaternion(x=waypoint['q0'], y=waypoint['q1'],
                                     z=waypoint['q2'], w=waypoint['q3'])
            pose = Pose(position=position, orientation=orientation)
            path_poses.append(PoseStamped(header=path_header,
                                          pose=pose))

        ground_truth_poses = convert_dict_to_list(start_time=start_time,
                                                  end_time=finish_time,
                                                  dic=ground_truth_data)

        localization_poses = convert_dict_to_list(start_time=start_time,
                                                  end_time=finish_time,
                                                  dic=localization_data)

        path_distance = get_distance_through_poses(path_poses)
        ground_truth_distance = get_distance_through_poses(ground_truth_poses)
        localization_distance = get_distance_through_poses(localization_poses)

        average_speed_effective = path_distance / traveled_time
        average_speed_real = ground_truth_distance / traveled_time
        average_speed_assumed = localization_distance / traveled_time

        average_speed = {'average_speed_effective': average_speed_effective,
                         'average_speed_real': average_speed_real,
                         'average_speed_assumed': average_speed_assumed}

        distance_data = {'path': path_distance,
                         'ground_truth': ground_truth_distance,
                         'localization': localization_distance}

        # debug
        print(f'start_time: {start_time}')
        print(f'finish_time: {finish_time}')
        print(f'path distance: {path_distance}')
        print(f'ground truth distance: {ground_truth_distance}')
        print(f'localization distance: {localization_distance}')
        print(f'average_speed_effective: {average_speed["average_speed_effective"]}')

        # GET ERROR
        localization_points = convert_list_type(localization_poses, Point())
        ground_truth_points = convert_list_type(ground_truth_poses, Point())
        path_points = convert_list_type(path_poses, Point())

        # MAX
        # max_error_loc = get_max_distance(self,
        max_error_loc = get_max_distance(self,
                                         inaccurate_poses=localization_poses,
                                         reference_poses=ground_truth_poses)
        max_error_loc_simpl = get_max_dist_simpl(localization_points,
                                                 ground_truth_points)
        avg_error_loc = get_avg_distance(localization_poses,
                                         ground_truth_poses)
        avg_error_loc_simpl = get_avg_dist_simpl(localization_points,
                                                 ground_truth_points)

        max_error_real = get_max_dist_simpl(points=ground_truth_points,
                                            reference_points=path_points)
        avg_error_real = get_avg_dist_simpl(ground_truth_points,
                                            path_points)

        max_error_contr = get_max_dist_simpl(points=localization_points,
                                             reference_points=path_points)
        avg_error_contr = get_avg_dist_simpl(localization_points,
                                             path_points)

        localization_errors = {'max': float(max_error_loc),
                               'max simplified': float(max_error_loc_simpl),
                               'average': float(avg_error_loc),
                               'average simplified': float(avg_error_loc_simpl)}

        real_errors = {'max': float(max_error_real),
                       'average': float(avg_error_real)}

        controller_errors = {'max': float(max_error_contr),
                             'average': float(avg_error_contr)}

        error_data = {'localization': localization_errors,
                      'real (path - ground truth)': real_errors,
                      'controller (path - localization)': controller_errors}

        # debug
        print(f'max loc error0: {max_error_loc}')
        print(f'max loc error1: {max_error_loc_simpl}')
        print(f'avg loc error0: {avg_error_loc}')
        print(f'avg loc error1: {avg_error_loc_simpl}')
        print(f'max nav error:  {max_error_real}')
        print(f'avg nav error:  {avg_error_real}')

        # save it
        results = {'errors': error_data,
                   'speed': average_speed,
                   'distance': distance_data}
        self.log_results(results)

    def log_results(self, results):
        ''' Save the results to a yaml file in the logs path root.
        '''
        self.get_logger().info('Saving data ...')
        logfile = self.logs_path + '/results.yaml'
        with open(logfile, 'w', encoding='utf-8') as file:
            yaml.dump(results, file, default_flow_style=False)


def main(args=None):
    """ Analyze data from claudi simulation to give out and save
    information like average speed, traveled distance and error.
    """
    rclpy.init(args=args)
    analyzer_node = AnalyzerNode()
    rclpy.spin(analyzer_node)
    analyzer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
