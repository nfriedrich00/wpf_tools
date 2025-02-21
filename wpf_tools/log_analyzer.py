from os.path import exists
import numpy as np
import yaml
from pathlib import Path as pathlibPath

from wpf_utils import (get_distance_point_line, get_reference_point,
                       get_distance_through_points, euclidean_distance_2d)

class LogAnalyzer:
    def __init__(self, logs_dir: str, overwrite_results: bool,
                 start_time: float, end_time: float):
        self.logs_dir = logs_dir
        self.overwrite_results = overwrite_results
        self.start_time = start_time
        self.end_time = end_time

    def analyze_data(self):
        if not self.load_data():
            return False
        if not self.get_errors():
            return False
        if not self.log_results(self.results):
            return False
        return True

    def load_data(self):
        ''' Start looking for the log files and load the relevant part into memory.
        ''' # todo: try really making it just the relevant part
            # otherwise make a seperate function to restrict the data to the relevant part 
        logs_dir = self.logs_dir
        session_id = pathlibPath(logs_dir).stem


        # Check if results file already exists
        # todo: maybe move to relevant part of code
        self.results_path = logs_dir + '/results.yaml'
        if exists(self.results_path) and not self.overwrite_results:
            number = 1
            while exists(logs_dir + f'/results_{number}.yaml'):
                number += 1
            self.results_path = logs_dir + f'/results_{number}.yaml'


        position_logs = logs_dir + '/ground_truth.yaml'
        if not exists(position_logs):
            return False
        else:
            with open(logs_dir + '/ground_truth.yaml', 'r',
                    encoding='utf-8') as position_file:
                pos_data = yaml.safe_load(position_file)

        localization_logs = logs_dir + '/localization.yaml'
        if not exists(localization_logs):
            loc_data = {}#todo: why do we need an empry dict if there is no data?
        else:
            with open(self.logs_dir + '/localization.yaml', 'r',
                    encoding='utf-8') as localization_file:
                loc_data = yaml.safe_load(localization_file)

        path_logs = logs_dir + '/path.yaml'
        if not exists(path_logs):
            path_data = {}
        else:
            with open(logs_dir + '/path.yaml', 'r',
                    encoding='utf-8') as path_file:
                path_data = yaml.safe_load(path_file)

        # todo: evaluate role of the goal checker
        goal_checker_logs = logs_dir + '/goal_checker.yaml'
        if not exists(goal_checker_logs):
            goal_checker_data = {}
        else:
            with open(logs_dir + '/goal_checker.yaml', 'r',
                    encoding='utf-8') as goal_checker_file:
                goal_checker_data = yaml.safe_load(goal_checker_file)

        ###### restrict data to relevant part

        # if start_time or end_time is set, use it to limit the analysis
        if not self.start_time and not self.end_time:
            # none are set, make it manually
            self.start_time = 0.0
            # self.end_time should be the last timestamp in the logs
            # timestamp means: we don't want to use the first key, we use key 'time'
            # do not set it to inf, because we want to know the lenght of the time interval
            self.end_time = max([float(pos_data[key]['time']) for key in pos_data])
        else:
            # at least one is set, make sure values are valid
            if self.start_time < 0.0:
                self.start_time = 0.0
            if self.end_time < 0.0:
                self.end_time = 0.0

            # start_time has to be smaller than end_time, but
            # one exception: only start_time is set, then end_time is the last timestamp
            if self.start_time and not self.end_time:
                self.end_time = max([float(pos_data[key]['time']) for key in pos_data])
            elif self.start_time > self.end_time:
                # make no assumptions, this is not a valid request, so return false
                return False

            
        # We use the timestampe at the 'time' key and not the first key,
        # because the first key is the logging time and not the time of the measurement.
        pos_data = {k: v for k, v in pos_data.items()
                    if self.start_time <= float(v['time']) <= self.end_time}
        loc_data = {k: v for k, v in loc_data.items()
                    if self.start_time <= float(v['time']) <= self.end_time}

        self.pos_points = np.array([[pos_data[key]['position']['x'],
                                pos_data[key]['position']['y'],
                                pos_data[key]['position']['z'],
                                pos_data[key]['time'],
                                key] for key in pos_data])
        self.loc_points = np.array([[loc_data[key]['position']['x'],
                                loc_data[key]['position']['y'],
                                loc_data[key]['position']['z'],
                                loc_data[key]['time'],
                                key] for key in loc_data])

        path_key = list(path_data.keys())[0]
        self.path_points = np.array([[wp['x'],
                                 wp['y'],
                                 wp['z'],
                                 0.0] for wp in path_data[path_key]['waypoints']])
        return True

    def get_errors(self):  
        time_interval = self.end_time - self.start_time
        """
        The length of the time interval of the analyzed data is used to calculate the average speed.
        """
        # Get errors

        # pos error = difference between real is position and path (ideal desired position)
        max_pos_error = 0.0
        avg_pos_error = 0.0
        for point in self.pos_points:
            distances = np.linalg.norm(self.path_points[:, :3] - point[:3], axis=1)

            closest_index = np.argmin(distances)
            distances[closest_index] = np.inf
            second_closest_index = np.argmin(distances)
            samepoints = np.array_equal(self.path_points[closest_index],
                                        self.path_points[second_closest_index])
            while (samepoints):
                distances[second_closest_index] = np.inf
                second_closest_index = np.argmin(distances)
                samepoints = np.array_equal(self.path_points[closest_index],
                                            self.path_points[second_closest_index])


            closest_point = self.path_points[closest_index]
            second_closest_point = self.path_points[second_closest_index]
            
            # ignore z axis = Δbase_link-base_footprint: base link is elevated,
            #       loc and path are not)
            distance = get_distance_point_line(point[:2],
                                               closest_point[:2],
                                               second_closest_point[:2])
            
            avg_pos_error += distance / len(self.pos_points)  
            if distance > max_pos_error:
                max_pos_error = distance


        # loc error = difference between localization and real position
        max_loc_error = 0.0
        avg_loc_error = 0.0
        for point in self.loc_points:
            time_differences = np.array(abs(self.pos_points[:, 3] - point[3]))

            closest_index = np.argmin(time_differences)
            time_differences[closest_index] = np.inf
            second_closest_index = np.argmin(time_differences)

            closest_point = self.pos_points[closest_index]
            second_closest_point = self.pos_points[second_closest_index]
            
            ref_point = get_reference_point(point[3], (closest_point[:-1],
                                                       second_closest_point[:-1]))
            distance = np.linalg.norm(point[:2] - ref_point[:2])

            avg_loc_error += distance / len(self.loc_points)            
            if distance > max_loc_error:
                max_loc_error = distance


        localization_errors = {'max': float(max_loc_error),
                               'average': float(avg_loc_error)}
        
        real_errors = {'max': float(max_pos_error),
                       'average': float(avg_pos_error)}

        error_data = {'localization': localization_errors,
                      'real (path - ground truth)': real_errors}


        # Get distances

        path_distance = get_distance_through_points(self.path_points)
        ground_truth_distance = get_distance_through_points(self.pos_points)
        localization_distance = get_distance_through_points(self.loc_points)
        start_distance_to_goal = euclidean_distance_2d(self.path_points[0],
                                                       self.path_points[-1])
        distance_to_goal = euclidean_distance_2d(self.pos_points[-1],
                                                 self.path_points[-1])


        distance_data = {'path': float(path_distance),
                         'ground_truth': float(ground_truth_distance),
                         'localization': float(localization_distance),
                         'start_distance_to_goal': float(start_distance_to_goal),
                         'distance_to_goal': float(distance_to_goal)}
        
        # Get speed
        avg_speed = ground_truth_distance / time_interval
        speed_data = {'average' : float(avg_speed)}


        self.results = {'errors': error_data,
                   'distance': distance_data,
                   'speed' : speed_data}

        return True

    def log_results(self, results):
        ''' Save the results to a yaml file in the logs path root.
        '''
        logfile = self.results_path
        with open(logfile, 'w', encoding = 'utf-8') as file:
            pass
        with open(logfile, 'w', encoding = 'utf-8') as file:
            yaml.dump(results, file, default_flow_style=False)
        return True
