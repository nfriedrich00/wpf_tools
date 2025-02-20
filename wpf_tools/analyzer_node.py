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

        default_logs_parent_dir = os.path.expanduser("~") + "/Documents/wpf/logs"

        # temp debugging:
        temp_debugging_path = "/home/nfriedrich/Documents/docker/jazzy_vnc/Documents/wpf/logs"
        default_logs_parent_dir = temp_debugging_path
        # / temp debugging

        self.declare_parameter('logs_directory', '')
        self.logs_dir = self.get_parameter('logs_directory').value
        if self.logs_dir:
            # True if parameter is set
            self.get_logger().info(f'Logs directory provided: {self.logs_dir}')
        else:
            # Parameter not set, using default value
            #default_logs_dir = ... # todo: move here
            self.get_logger().info(f'No logs directory provided. Looking for logs in default directory {default_logs_parent_dir}.')
            if not os.path.exists(default_logs_parent_dir):
                self.get_logger().error(f'Default logs directory {default_logs_parent_dir} does not exist. Aborting.')
                raise FileNotFoundError
            list_of_log_dirs = [f for f in glob.glob(default_logs_parent_dir + '/*') if os.path.basename(f).isdigit()]
            if not list_of_log_dirs:
                self.get_logger().error(f'No logs found in default directory {default_logs_parent_dir}. Aborting.')
                raise FileNotFoundError
            path_of_newest_log = max(
                list_of_log_dirs,
                key=lambda x: int(os.path.basename(x)))
            self.logs_dir = path_of_newest_log
            self.get_logger().info(f'Using logs from latest session: {self.logs_dir}.')


        self.session_id = pathlibPath(self.logs_dir).stem

        self.declare_parameter('overwrite_results', True)
        self.overwrite_results = self.get_parameter('overwrite_results').value
        """
        Parameter to decide if an existing results file should be overwritten. Otherwise, a new file with a number suffix is created.
        """

        self.declare_parameter('start_time', 0.0)
        self.start_time = self.get_parameter('start_time').value
        """
        Parameter to limit the analysis to a specific time interval.
        The analysis will only consider data with a timestamp greater or equal to this value.
        Set to 0.0 to disable.
        """

        self.declare_parameter('end_time', 0.0)
        self.end_time = self.get_parameter('end_time').value
        """
        Parameter to limit the analysis to a specific time interval.
        The analysis will only consider data with a timestamp smaller or equal to this value.
        Set to 0.0 to disable.
        """

        if self.end_time < self.start_time:
            self.get_logger().error('End time is smaller than start time. Aborting.')
            raise ValueError
        
        self.analyze_data() #todo: change this so we can call the function with session_id as an arguments to call it for multiple sessions
        raise SystemExit

    def analyze_data(self):
        ''' Start data analysis. Load log files and calculate average
        speed, error and traveled distance.
        '''
        logs_dir = self.logs_dir
        session_id = self.session_id
        self.get_logger().info('Start looking for data ...')

        # Check if results file already exists
        # todo: maybe move to relevant part of code
        self.results_path = logs_dir + '/results.yaml'
        if os.path.exists(self.results_path) and not self.overwrite_results:
            number = 1
            while os.path.exists(logs_dir + f'/results_{number}.yaml'):
                number += 1
            self.results_path = logs_dir + '/' + session_id + f'/results_{number}.yaml'

        print(f"Results path: {self.results_path}")


        self.get_logger().info('Start loading data ...')

        position_logs = logs_dir + '/ground_truth.yaml'
        if not os.path.exists(position_logs):
            self.get_logger().error(f'No position logs found for session {session_id}. Aborting.')
            raise FileNotFoundError
        else:
            self.get_logger().info(f"Position logs found.")
            with open(logs_dir + '/ground_truth.yaml', 'r',
                    encoding='utf-8') as position_file:
                pos_data = yaml.safe_load(position_file)
            # skip loading this long ass file while debugging

        localization_logs = logs_dir + '/localization.yaml'
        if not os.path.exists(localization_logs):
            self.get_logger().info(f'No localization logs found for session {session_id}.')
            loc_data = {}
        else:
            self.get_logger().info(f"Localization logs found.")
            with open(self.logs_dir + '/localization.yaml', 'r',
                    encoding='utf-8') as localization_file:
                loc_data = yaml.safe_load(localization_file)

        path_logs = logs_dir + '/path.yaml'
        if not os.path.exists(path_logs):
            self.get_logger().info(f'No path logs found for session {session_id}.')
            path_data = {}
        else:
            self.get_logger().info(f"Path logs found.")
            with open(logs_dir + '/path.yaml', 'r',
                    encoding='utf-8') as path_file:
                path_data = yaml.safe_load(path_file)

        # todo: evaluate role of the goal checker
        goal_checker_logs = logs_dir + '/goal_checker.yaml'
        if not os.path.exists(goal_checker_logs):
            self.get_logger().info(f'No goal checker logs found for session {session_id}.')
            goal_checker_data = {}
        else:
            self.get_logger().info(f"Goal checker logs found.")
            with open(logs_dir + '/goal_checker.yaml', 'r',
                    encoding='utf-8') as goal_checker_file:
                goal_checker_data = yaml.safe_load(goal_checker_file)


        self.get_logger().info("Data loaded successfully.")

        # if start_time or end_time is set, use it to limit the analysis
        if not self.start_time and not self.end_time:
            self.start_time = 0.0
            # self.end_time should be the last timestamp in the logs
            # timestamp means: we don't want to use the first key, we use key 'time'
            self.end_time = max([float(pos_data[key]['time']) for key in pos_data])
        else:
            self.get_logger().info('Limiting analysis to time interval' +
                                   f' [{self.start_time}, {self.end_time}].')
            
        # We use the timestampe at the 'time' key and not the first key,
        # because the first key is the logging time and not the time of the measurement.
        pos_data = {k: v for k, v in pos_data.items()
                    if self.start_time <= float(v['time']) <= self.end_time}
        loc_data = {k: v for k, v in loc_data.items()
                    if self.start_time <= float(v['time']) <= self.end_time}


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

        path_key = list(path_data.keys())[0]
        path_points = np.array([[wp['x'],
                                 wp['y'],
                                 wp['z'],
                                 0.0] for wp in path_data[path_key]['waypoints']])
        self.get_logger().info("Conversion done.")


        self.get_logger().info("Start analyzing data ...")

        # We do no longer need to filter the points, BUT
        # todo: we need to check the plausibility of the data
        #loc_points = filter_points(loc_points)
        #pos_points = filter_points(pos_points)

        # These time limits do not necessarily match with the actual robot movement.
        # We would need to check the position data for that
        # Instead, make sure to set the limits to not include the robot standing still,
        # if the average speed is important.
        time_interval = self.end_time - self.start_time
        """
        The length of the time interval of the analyzed data is used to calculate the average speed.
        """


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
                         'start_distance_to_goal': float(start_distance_to_goal),
                         'distance_to_goal': float(distance_to_goal)}
        
        # Get speed
        self.get_logger().info("Getting speed ...")
        avg_speed = ground_truth_distance / time_interval
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
