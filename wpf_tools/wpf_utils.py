""" Useful functions for the waypoint follower analyzer node.
"""
from math import isnan
import numpy as np

def euclidean_distance_2d(a, b):
    ''' Take in two points as numpy array.
    Return the Euclidean distance in 2d.
    This means, only the first two array entries are taken into account.
    '''
    return np.linalg.norm(a[:2] - b[:2])

def get_distance_point_line(point: np.array,
                            line_point0: np.array,
                            line_point1: np.array):
    ''' Take in three points as numpy array.
    Return the Euclidean distance between the first point
    and a line, which is defined by the other two points.
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

def get_reference_point(time_reference: float, points: tuple[np.array]):
    ''' Takes in a time as float and a two points as tuple of numpy arrays.
    Last array entry is assumed to be a timestamp. Returns a point as numpy
    array with interpolated values to match given time.
    '''
    time_start = points[0][-1]
    time_end = points[1][-1]
    time_separation = time_end - time_start
    if round(time_separation, 3) == 0.0:
        return points[0]
    multiplier = (time_reference - time_start) / time_separation
    return points[0] + multiplier * (points[1] - points[0])


def get_distance_through_points(points: np.array):
    """ Return the accumulated distance through multiple points. """
    distance = 0.0
    for i in range (1, len(points)):
        distance += euclidean_distance_2d(points[i-1], points[i])
    return distance


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
