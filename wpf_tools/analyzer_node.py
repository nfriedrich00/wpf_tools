""" Analyze data from claudi simulation to give out and save additional
information like average speed, traveled distance and error.
"""
import os
import glob
import numpy as np
from pathlib import Path as pathlibPath
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header
from rclpy.node import Node
from math import isnan

from wpf_utils import (float_to_time, convert_dict_to_list,
                                 get_distance_through_poses, convert_list_type,
                                 get_max_distance, get_max_dist_simpl,
                                 get_avg_distance, get_avg_dist_simpl)

def get_distance_point_line_np(point: np.array,
                               line_point0: np.array,
                               line_point1: np.array):
    ''' NumPy version. Take in three points. Returns the Euclidean distance between the
    first point and a line, which is defined by the other two points.
    '''
    line_vector = line_point1 - line_point0
    point_vector = point - line_point0
    projection = np.dot(point_vector, line_vector) / np.dot(line_vector,
                                                            line_vector)
    perpendicular_vector = point_vector - projection * line_vector
    distance = np.linalg.norm(perpendicular_vector)

    if isnan(distance):
        print(point, line_point0, line_point1)

    return distance

def get_reference_point_np(time_reference: float, points: tuple[np.array]):
    ''' Takes in a Time and a tuple of two arrays. Last array entry is assumed
    to be a timestamp. Returns an array with interpolated values to match
    given time.
    '''
    time_start = points[0][-1]
    time_end = points[1][-1]
    time_separation = time_end - time_start
    if round(time_separation, 3) == 0.0:
        return points[0]
    multiplier = (time_reference - time_start) / time_separation
    return points[0] + multiplier * (points[1] - points[0])


def filter_points(points: np.array):
    ''' Filter corrupted measurements from logs:
        1. Sort by Euclidean distance to previous point (in R3: x-y-timestamp).
        2. Remove if time increment is negative
            (moving forward in place but backward in time)
        3. Then sort by timestamp
        4. Remove points that cannot be reached with maximum speed.
    This seems to removes all wrong points and none of the correct points.
    '''      
    filtered_points = []
    removed_points = []

    max_speed = 1.0
    speed_multiplier = 2.0
    
    def euclidean_distance_2d(a, b):
        return np.linalg.norm(a[:2] - b[:2])
    
    def sort_points(points):
        n = len(points)
        if n == 0:
            return points
        
        start_index = np.argmin(np.linalg.norm(points[:, :3] - points[0][:3], axis=1))
        sorted_points = [points[start_index]]
        remaining_points = np.delete(points, start_index, axis=0)
        
        while len(remaining_points) > 0:
            last_point = sorted_points[-1]
            distances = np.linalg.norm(remaining_points[:, :3] - last_point[:3], axis=1)
            next_index = np.argmin(distances)
            sorted_points.append(remaining_points[next_index])
            remaining_points = np.delete(remaining_points, next_index, axis=0)
        
        return np.array(sorted_points)

    sorted_points = sort_points(points)
    n = len(sorted_points)
    
    removed = 1
    while removed > 0:
        filtered_points.clear()
        removed = 0
        n = len(sorted_points)
        for i in range(n-1):
            curr_point = sorted_points[i]
            next_point = sorted_points[i+1]

            if curr_point[3] > next_point[3]:
                # timestamp decreases while moving forwards -> remove point
                removed_points.append(curr_point)
                removed += 1
            else:
                filtered_points.append(curr_point)
        sorted_points = np.array(filtered_points)

    # round two: plausibility
    # sort filtered points by timestamp
    filtered_points = np.array(filtered_points)
    timestamps = filtered_points[:, 3]
    sorted_indices = np.argsort(timestamps)
    sorted_points = filtered_points[sorted_indices]
    n = len(sorted_points)
    filtered_points = []

    for i in range(n-1):
        curr_point = sorted_points[i]
        next_point = sorted_points[i+1]

        distance = euclidean_distance_2d(curr_point, next_point)
        time_sep = next_point[3] - curr_point[3]
        # this is always positive due to previous step
        if time_sep == 0.0:
            speed = 999.9
        else:
            speed = distance/time_sep

        if speed > max_speed * speed_multiplier:
            # use multiplier to prevent filtering of points with 1.1 * max_speed
        # implausible movement: current point could not be reached, unless
        # previous point was wrong
            removed_points.append(curr_point)

        else:
            filtered_points.append(curr_point)

    filtered_points = np.array(filtered_points)

    return filtered_points

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
        default_logs_path = os.path.expanduser("~") + "/Documents/wpf/logs"
        self.declare_parameter('logs_path', default_logs_path)
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
            pos_data = yaml.safe_load(ground_truth_file)

        with open(self.logs_path + '/position/'
                  + self.session_id + '/localization.yaml', 'r',
                  encoding='utf-8') as localization_file:
            loc_data = yaml.safe_load(localization_file)
        self.get_logger().info("Data loaded successfully.")

        # start time when actually starting to move
        start_time = goal_checker_data['start moving']
        finish_time = goal_checker_data['goal succeeded']
        traveled_time = finish_time - start_time

        self.get_logger().info("Convert data to numpy array")
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
        
        # restrict to points within relevant time interval
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

        self.get_logger().info("Get max position error")
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
            distance = get_distance_point_line_np(point[:2],
                                                  closest_point[:2],
                                                  second_closest_point[:2])
            
            avg_pos_error += distance / len(pos_points)  
            if distance > max_pos_error:
                max_pos_error = distance
        print("got max pos error: " + str(max_pos_error))
        print("got avg pos error: " + str(avg_pos_error))


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
            
            ref_point = get_reference_point_np(point[3], (closest_point,
                                                          second_closest_point))
            distance = np.linalg.norm(point[:2] - ref_point[:2])

            avg_loc_error += distance / len(loc_points)            
            if distance > max_loc_error:
                max_loc_error = distance
                p_is = point
                p1 = closest_point
                p2 = second_closest_point
                ref = ref_point


        #print(f'is: {p_is}, p1: {p1}, p2: {p2}, ref: {ref}')
        print("got max loc error: " + str(max_loc_error))
        print("got avg loc error: " + str(avg_loc_error))

        localization_errors = {'max': float(max_loc_error),
                               'average': float(avg_loc_error)}
        
        real_errors = {'max': float(max_pos_error),
                       'average': float(avg_pos_error)}

        error_data = {'localization': localization_errors,
                'real (path - ground truth)': real_errors}


        results = {'errors': error_data}
        self.log_results(results)

        raise SystemExit
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
                                                  dic=pos_data)

        localization_poses = convert_dict_to_list(start_time=start_time,
                                                  end_time=finish_time,
                                                  dic=loc_data)

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
