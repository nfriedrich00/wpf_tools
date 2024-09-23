""" Analyze data from claudi simulation to give out and save additional
information like average speed, traveled distance and error.
"""
import os
import glob
import numpy as np
from pathlib import Path as pathlibPath
import rclpy
import yaml
from rclpy.node import Node

from wpf_utils import (get_distance_point_line, get_reference_point,
                       filter_points, get_distance_through_points,
                       euclidean_distance_2d)


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
        self.results_path = os.path.expanduser("~") + "/Documents/wpf/results.yaml"

        self.declare_parameter('logs_path', self.logs_path)
        if self.get_parameter('logs_path').value is not None:
            self.logs_path = self.get_parameter('logs_path').value
        
        self.declare_parameter('results_path', self.results_path)
        if self.get_parameter('results_path').value is not None:
            self.results_path = self.get_parameter('results_path').value

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
            pos_data = yaml.safe_load(ground_truth_file)

        with open(self.logs_path + '/position/'
                  + self.session_id + '/localization.yaml', 'r',
                  encoding='utf-8') as localization_file:
            loc_data = yaml.safe_load(localization_file)
        self.get_logger().info("Data loaded successfully.")

        self.get_logger().info("Converting data to numpy array")
        pos_points = np.array([[pos_data[key]['position']['x'],
                                pos_data[key]['position']['y'],
                                pos_data[key]['position']['z'],
                                pos_data[key]['time'],
                                key] for key in pos_data])
        loc_points = np.array([[loc_data[key]['position']['x'],
                                loc_data[key]['position']['y'],
                                loc_data[key]['position']['z'],
                                loc_data[key]['time'],
                                key] for key in loc_data])
        
        loc_points = filter_points(loc_points)
        pos_points = filter_points(pos_points)


        # start time when actually starting to move, if timestamp provided.
        start_time = goal_checker_data.get('start moving', 0.0)

        # finish time is goal reached timestamp, if goal was reached.
        # else, use last timestamp in logs. Should be the same. Unless:
        # If the goal was reached but the time was not recorded,
        # the last timestamp in the logs may be significantly larger.
        # -> traveled_time higher -> avg_speed slower.
        goal_timestamp_key = 'goal succeeded'
        if goal_timestamp_key in goal_checker_data:
            finish_time = goal_checker_data[goal_timestamp_key]
        else:
            pos_timestamps = pos_points[:, -1]
            loc_timestamps = loc_points[:, -1]
            last_pos_time = np.max(pos_timestamps) \
                if pos_timestamps.size > 0 else 0.0
            last_loc_time = np.max(loc_timestamps) \
                if loc_timestamps.size > 0 else 0.0
            #last_pos_time = max(pos_points.keys(), default=0.0)
            #last_loc_time = max(loc_points.keys(), default=0.0)
            finish_time = max(last_pos_time, last_loc_time)

        traveled_time = finish_time - start_time
        if traveled_time <= 0.0:
            error_message = 'Traveled time is zero.' + \
            'This is probably due to incomplete or corrupted logs. Aborting.'
            self.get_logger().error(error_message)
            raise ValueError

        
        # restrict to points within relevant time interval (ignore measurements when standing still)
        # todo add threshould: find timestamp, when robot pos enters relevant part of path and update mask limits
        pos_times = pos_points[:, 3]
        pos_mask = (pos_times >= start_time) & (pos_times <= finish_time)
        pos_points = pos_points[pos_mask]

        loc_times = loc_points[:, 3]
        loc_mask = (loc_times >= start_time) & (loc_times <= finish_time)
        loc_points = loc_points[loc_mask]



        path_key = list(path_data.keys())[0]
        path_points = np.array([[wp['x'],
                                 wp['y'],
                                 wp['z'],
                                 0.0] for wp in path_data[path_key]['waypoints']])
        self.get_logger().info("Conversion done")


        # Get errors
        self.get_logger().info("Getting errors ...")

        # pos error = difference between real is position and path (ideal desired position)
        max_pos_error = 0.0
        avg_pos_error = 0.0
        for point in pos_points:
            distances = np.linalg.norm(path_points[:, :3] - point[:3], axis=1)

            closest_index = np.argmin(distances)
            distances[closest_index] = np.inf
            second_closest_index = np.argmin(distances)
            samepoints = np.array_equal(path_points[closest_index],
                                        path_points[second_closest_index])
            while (samepoints):
                distances[second_closest_index] = np.inf
                second_closest_index = np.argmin(distances)
                samepoints = np.array_equal(path_points[closest_index],
                                            path_points[second_closest_index])


            closest_point = path_points[closest_index]
            second_closest_point = path_points[second_closest_index]
            
            # ignore z axis = Δbase_link-base_footprint: base link is elevated,
            #       loc and path are not)
            distance = get_distance_point_line(point[:2],
                                               closest_point[:2],
                                               second_closest_point[:2])
            
            avg_pos_error += distance / len(pos_points)  
            if distance > max_pos_error:
                max_pos_error = distance
        self.get_logger().info(f"Got max pos error: {max_pos_error}")
        self.get_logger().info(f"Got avg pos error: {avg_pos_error}")


        # loc error = difference between localization and real position
        max_loc_error = 0.0
        avg_loc_error = 0.0
        for point in loc_points:
            time_differences = np.array(abs(pos_points[:, 3] - point[3]))

            closest_index = np.argmin(time_differences)
            time_differences[closest_index] = np.inf
            second_closest_index = np.argmin(time_differences)

            closest_point = pos_points[closest_index]
            second_closest_point = pos_points[second_closest_index]
            
            ref_point = get_reference_point(point[3], (closest_point[:-1],
                                                       second_closest_point[:-1]))
            distance = np.linalg.norm(point[:2] - ref_point[:2])

            avg_loc_error += distance / len(loc_points)            
            if distance > max_loc_error:
                max_loc_error = distance


        self.get_logger().info(f"Got max loc error: {max_loc_error}")
        self.get_logger().info(f"Got avg loc error: {avg_loc_error}")

        localization_errors = {'max': float(max_loc_error),
                               'average': float(avg_loc_error)}
        
        real_errors = {'max': float(max_pos_error),
                       'average': float(avg_pos_error)}

        error_data = {'localization': localization_errors,
                      'real (path - ground truth)': real_errors}


        # Get distances
        self.get_logger().info("Getting distances ...")

        path_distance = get_distance_through_points(path_points)
        ground_truth_distance = get_distance_through_points(pos_points)
        localization_distance = get_distance_through_points(loc_points)
        start_distance_to_goal = euclidean_distance_2d(path_points[0],
                                                       path_points[-1])
        distance_to_goal = euclidean_distance_2d(pos_points[-1],
                                                 path_points[-1])
        self.get_logger().info('Got path distance: ' \
                               + str(path_distance))
        self.get_logger().info('Got ground truth distance: ' \
                               + str(ground_truth_distance))
        self.get_logger().info('Got localization distance: ' \
                               + str(localization_distance))
        self.get_logger().info('Got start distance to goal: ' \
                               + str(start_distance_to_goal))
        self.get_logger().info('Got distance to goal: ' \
                               + str(distance_to_goal))

        distance_data = {'path': float(path_distance),
                         'ground_truth': float(ground_truth_distance),
                         'localization': float(localization_distance),
                         'start_distance_to_goal': float(start_goal_separation),
                         'distance_to_goal': float(distance_to_goal)}
        
        # Get speed
        self.get_logger().info("Getting speed ...")
        avg_speed = ground_truth_distance / traveled_time
        self.get_logger().info(f"Got avg speed: {avg_speed}")
        speed_data = {'average' : float(avg_speed)}


        results = {'errors': error_data,
                   'distance': distance_data,
                   'speed' : speed_data}
        
        self.log_results(results)


    def log_results(self, results):
        ''' Save the results to a yaml file in the logs path root.
        '''
        self.get_logger().info('Saving data ...')
        logfile = self.results_path
        with open(logfile, 'w', encoding = 'utf-8') as file:
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
