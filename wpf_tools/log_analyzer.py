from os.path import exists
import numpy as np
import yaml
from pathlib import Path as pathlibPath

from wpf_utils import (get_distance_point_line, get_reference_point,
                       get_distance_through_points, euclidean_distance_2d,
                       get_first_timestamp_after_distance)

class LogAnalyzer:
    def __init__(self, logs_dir: str, overwrite_results: bool,
                 start_time: float, end_time: float,
                 start_position: float, end_position: float):
        self.logs_dir = logs_dir
        self.overwrite_results = overwrite_results
        self.start_time = start_time
        self.end_time = end_time
        self.start_position = start_position
        self.end_position = end_position

    def analyze_data(self):
        if not self.load_data():
            return False
        if not self.limit_data():
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
                self.pos_data = yaml.safe_load(position_file)

        localization_logs = logs_dir + '/localization.yaml'
        if not exists(localization_logs):
            self.loc_data = {}#todo: why do we need an empry dict if there is no data?
        else:
            with open(self.logs_dir + '/localization.yaml', 'r',
                    encoding='utf-8') as localization_file:
                self.loc_data = yaml.safe_load(localization_file)

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

        path_key = list(path_data.keys())[0]
        self.path_points = np.array([[wp['x'],
                                      wp['y'],
                                      wp['z'],
                                      0.0] for wp in path_data[path_key]['waypoints']])
        return True

    def limit_data(self):
        ''' Limit the data to the relevant part.
        '''
        self.pos_points = np.array([[self.pos_data[key]['position']['x'],
                                     self.pos_data[key]['position']['y'],
                                     self.pos_data[key]['position']['z'],
                                     self.pos_data[key]['time'],
                                     key] for key in self.pos_data if 'position' in self.pos_data[key] and 'time' in self.pos_data[key]])
        self.loc_points = np.array([[self.loc_data[key]['position']['x'],
                                     self.loc_data[key]['position']['y'],
                                     self.loc_data[key]['position']['z'],
                                     self.loc_data[key]['time'],
                                     key] for key in self.loc_data if 'position' in self.loc_data[key] and 'time' in self.loc_data[key]])
        self.total_traveled_distance = get_distance_through_points(self.pos_points)
        # limits withouth restrictions
        min_time = min([float(self.pos_data[key]['time']) for key in self.pos_data if 'time' in self.pos_data[key]])
        max_time = max([float(self.pos_data[key]['time']) for key in self.pos_data if 'time' in self.pos_data[key]])

        # time restrictions
        if not self.start_time:
            start_time = 0.0
        elif min_time < self.start_time <= max_time:
            start_time = self.start_time
        elif self.start_time < 0.0:
            start_time = max_time - self.start_time
        else:
            return False

        if not self.end_time:
            end_time = 0.0
        elif min_time < self.end_time <= max_time:
            end_time = self.end_time
        elif self.end_time < 0.0:
            end_time = max_time + self.end_time
            # + because end_time is negative!
        else:
            return False

        # path length restrictions
        if self.start_position:
            if 0.0 <= self.start_position <= self.total_traveled_distance:
                start_position = self.start_position
            elif -self.total_traveled_distance < self.start_position < 0.0:
                start_position = self.total_traveled_distance - self.start_position
            else:
                return False

            start_time = max(start_time, get_first_timestamp_after_distance(self.pos_points, start_position))

        if self.end_position:
            if 0.0 <= self.end_position <= self.total_traveled_distance:
                end_position = self.end_position
            elif -self.total_traveled_distance < self.end_position < 0.0:
                end_position = self.total_traveled_distance + self.end_position
                # + because end_position is negative!
            else:
                return False
            
            if end_time:
                end_time = min(end_time, get_first_timestamp_after_distance(self.pos_points, end_position))
            else:
                end_time = get_first_timestamp_after_distance(self.pos_points, end_position)

        # start_time has to be smaller than end_time, but
        # one exception: only start_time is set, then end_time is the last timestamp
        if start_time and not end_time:
            end_time = max([float(self.pos_data[key]['time']) for key in self.pos_data])
        elif start_time > end_time:
            # make no assumptions, this is not a valid request, so return false
            return False

        if not start_time:
            self.start_time = min_time
        else:
            self.start_time = start_time
        if not end_time:
            self.end_time = max_time
        else:
            self.end_time = end_time
        self.pos_points = self.pos_points[(self.pos_points[:, 3] >= self.start_time) &
                                           (self.pos_points[:, 3] <= self.end_time)]
        self.pos_points = self.pos_points[np.argsort(self.pos_points[:, 3])]
        self.loc_points = self.loc_points[(self.loc_points[:, 3] >= self.start_time) &
                                          (self.loc_points[:, 3] <= self.end_time)]
        self.loc_points = self.loc_points[np.argsort(self.loc_points[:, 3])]
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
                         'ground_truth_total': float(self.total_traveled_distance),
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
