""" Useful functions for the waypoint follower analyzer node.
"""
from math import sqrt, floor
import numpy as np
from geometry_msgs.msg import (Point, Pose, Quaternion, PoseStamped,
                               PointStamped)
from builtin_interfaces.msg._time import Time
from std_msgs.msg import Header


def time_to_float(time: Time):
    ''' Takes in a Time message (e.g. a stamp)
    and converts it to a float.
    '''
    return float(f'{time.sec}.{time.nanosec}')


def float_to_time(time_float):
    ''' Takes in a float
    and converts it to a Time message with sec and nanosec.
    '''
    return Time(sec = floor(time_float),
                nanosec = round(((time_float - floor(time_float)) * 1e9)))


def get_distance_between_points(point0: Point, point1: Point):
    """ Return Euclidean distance between two points in ℝ³. """
    return sqrt((point0.x - point1.x) ** 2
                + (point0.y - point1.y) ** 2
                + (point0.z - point1.z) ** 2)


def get_distance_through_points(points: list[Point]):
    """ Return the accumulated distance through multiple points. """
    distance = 0.0
    if points == []:
        return distance
    for i in range(1, len(points)):
        distance += get_distance_between_points(points[i], points[i-1])
    return distance


def get_distance_through_poses(poses: list[PoseStamped]):
    """ Return the distance through multiple poses.
    Takes in a list of geometry_msgs.msg.PoseStamped and returns a float
    for the accumulated Euclidean distance between them.
    """
    distance = 0.0
    for i in range(1, len(poses)):
        distance += get_distance_between_points(poses[i].pose.position,
                                                poses[i-1].pose.position)
    return distance


def convert_dict_to_list(start_time, end_time, dic: dict):
    ''' Takes in a dictionary and converts it to a list of PoseStamped,
    which contains all PoseStamped in a certain time period.
    '''
    poses = []
    for key in dic.keys():
        entry_time = dic[key]['time']
        if start_time <= entry_time <= end_time:
            position = Point(x = dic[key]['position']['x'],
                             y = dic[key]['position']['y'],
                             z = 0.0)
#                             z = dic[key]['position']['z'])
            pose = Quaternion(x = dic[key]['position']['q0'],
                              y = dic[key]['position']['q1'],
                              z = dic[key]['position']['q2'],
                              w = dic[key]['position']['q3'])
            header = Header(stamp = float_to_time(entry_time))

            poses.append(PoseStamped(header = header,
                                     pose = Pose(position = position,
                                                 orientation = pose)))

    # Remove suspicious entries
    remove_suspicious_entires = True
    if remove_suspicious_entires:
        counter = 1
        while True:
            for i in range(0, len(poses) - 1):
                if time_to_float(poses[i].header.stamp) > time_to_float(poses[i+1].header.stamp):
                    counter += 1
                    del(poses[i])
                    # break and restart to prevent IndexError
                    break
            if counter == 0:
                # no entries removed this iteration, finished
                break
            counter = 0
    return poses


def find_nearest_poses(time: Time, reference_poses: list[PoseStamped]):
    ''' Nearest time. Returns a tuple with two PoseStamped from
    a reference list of PoseStamped, which have the smallest time
    difference to a given PoseStamped.
    '''
    if len(reference_poses) < 2:
        return (PoseStamped(), PoseStamped())

    if (abs(time_to_float(reference_poses[0].header.stamp)
            - time_to_float(time))
        > abs(time_to_float(reference_poses[1].header.stamp)
              - time_to_float(time))):
        poses = [reference_poses[0], reference_poses[1]]
    else:
        poses = [reference_poses[1], reference_poses[0]]

    for pose_stamped in reference_poses[2:]:
        if (abs(time_to_float(pose_stamped.header.stamp)
                - time_to_float(time))
            < abs(time_to_float(poses[0].header.stamp)
                - time_to_float(time))):
            poses.append(pose_stamped)
            if (abs(time_to_float(poses[0].header.stamp) - time_to_float(time))
                > abs(time_to_float(poses[1].header.stamp) - time_to_float(time))):
                del(poses[0])
            else:
                del(poses[1])
    #print(f'nearest poses to {time_to_float(time)}: {time_to_float(poses[0].header.stamp)}, {time_to_float(poses[1].header.stamp)}')
    return (poses[0], poses[1])


def find_nearest_points(point: Point, reference_points: list[Point]):
    ''' Nearest distance. Returns a tuple with two different adjacent 
    points from a reference list of points, which have the smallest
    Euclidean distance to a given Point.
    '''
    if len(reference_points) < 2:
        return (Point(), Point())
    
    # any point to start search
    point0 = reference_points[0]

    # find the nearest point (point0)
    for r_point in reference_points[1:]:
        if get_distance_between_points(point, r_point) < get_distance_between_points(point, point0):
            point0 = r_point

    point1 = None
    # Second point should be different object but also at a different position
    for r_point in reference_points:
        if get_distance_between_points(point0, r_point) < 0.00001:
            # then these points are practically the same
            continue
        point1 = r_point
    
    if point1 is None:
        # then all reference points are extremely close to the nearest one
        print('The reference points are too similar.')
        raise RuntimeError

    # find the optimal second point, the second smallest distance
    for r_point in reference_points:
        if (get_distance_between_points(r_point, point0) < 0.00001
            or get_distance_between_points(r_point, point1) < 0.00001):
            continue
        if (get_distance_between_points(point, r_point)
            < get_distance_between_points(point, point1)):
            # then this r_point is closer to the point as the current point1
            point1 = r_point

    return (point0, point1)


def find_surrounding_points(point_stamped: PointStamped,
                            reference_points: list[PointStamped]):
    ''' Returns a tuple with two points surrounding a reference point:
    The first element is the latest point before the reference point,
    the second element is the earliest point after the reference point.
    If there are no such points, returns the two points with smallest
    time difference.
    '''
    if len(reference_points) < 2:
        print('The reference list has not enough entries.'
               + ' Result may be inaccurate!')
        return (PointStamped(), PointStamped())
    
    # Note that the list is not necessarily sorted.
    points = [None, None]
    for point_s in reference_points:
        if time_to_float(point_s.header.stamp) <= time_to_float(point_stamped.header.stamp):
            # Make sure there is any predecessor at all
            if points[0] is None:
                points[0] = point_s
                continue
            # If there is a predecessor, check if current point is later
            if time_to_float(points[0].header.stamp) > time_to_float(point_s.header.stamp):
                points[0] = point_s
            continue

        if points[1] is None:
            # Make sure there is any successor at all
            points[1] = point_s
            continue
        # If there is already a successor, check if current point is earlier
        if time_to_float(points[1].header.stamp) < time_to_float(point_s.header.stamp):
            points[1] = point_s

    # Now there must be at least one surrounding point but maybe the input
    # point is from before or after all reference points.
    if points[0] is not None and points[1] is not None:
        return (points[0], points[1])



    if points[0] is None:
        # then input point is earlier than all points from list and point[1] is
        # the earliest point from the list

        # Set initial points[0] but make sure it's not the same as points[1]
        if points[1] is not reference_points[0]:
            points[0] = reference_points[0]
        else:
            points[0] = reference_points[1]

        for point_s in reference_points:
            if point_s is points[1]:
                continue
            if time_to_float(point_s.header.stamp) < time_to_float(points[0].header.stamp):
                points[0] = point_s

        return (points[1], points[0])



    # else points[1] is None and points[0] is latest point from list
    if points[0] is not reference_points[0]:
        points[1] = reference_points[0]
    else:
        points[1] = reference_points[1]

    for point_s in reference_points:
        if point_s is points[0]:
            continue
        if time_to_float(point_s.header.stamp) > time_to_float(points[1].header.stamp):
            points[1] = point_s

    return (points[1], points[0])


def get_distance_point_line(point: Point,
                            line_point0: Point,
                            line_point1: Point):
    ''' Take in three points. Returns the Euclidean distance between the
    first point and a line, which is defined by the other two points.
    '''
    #print(f'calc distance from {point.x}/{point.y}/{point.z} to a line through {line_point0.x}/{line_point0.y}/{line_point0.z} and {line_point1.x}/{line_point1.y}/{line_point1.z}')
    point = np.array((point.x, point.y, point.z))
    line_point0 = np.array((line_point0.x, line_point0.y, line_point0.z))
    line_point1 = np.array((line_point1.x, line_point1.y, line_point1.z))

    line_vector = line_point1 - line_point0
    point_vector = point - line_point0
    projection = np.dot(point_vector, line_vector) / np.dot(line_vector,
                                                            line_vector)
    perpendicular_vector = point_vector - projection * line_vector
    distance = np.linalg.norm(perpendicular_vector)

    #if isnan(distance):
    #    print(f'error: {point[0]}/{point[1]}/{point[2]} to line through {line_point0[0]}/{line_point0[1]}/{line_point0[2]} and {line_point1[0]}/{line_point1[1]}/{line_point1[2]}')
    return distance


def find_reference_point(time: Time, points: tuple[PointStamped]):
    ''' Takes in a Time and a tuple of two PointStamped.
    Returns a PointStamped with the input time as timestamp
    and a position on the line through the input points.
    '''
    time_input = time_to_float(time)
    time_point0 = time_to_float(points[0].header.stamp)
    time_point1 = time_to_float(points[1].header.stamp)

    #return points[0]
    #print(f'point separation; time: {abs(time_point0 - time_point1)}, m: {get_distance_between_points(points[0].point, points[1].point)}')

    time_separation = time_point1 - time_point0
    if round(time_separation, 3) == 0.0:
        print('here')
        return points[0]
    multiplier = (time_input - time_point0) / time_separation
    #print(multiplier)

    point_stamped = PointStamped(header = Header(stamp = time))
    point = Point()

    point.x = points[0].point.x + multiplier*(points[1].point.x
                                              - points[0].point.x)
    for attribute in ['x', 'y', 'z']:
        setattr(point, attribute,
                getattr(points[0].point, attribute)
                + multiplier*(getattr(points[1].point, attribute)
                              - getattr(points[0].point, attribute)))

    point_stamped.point = point

    return point_stamped


def get_avg_distance(inaccurate_poses: list[PoseStamped],
                     reference_poses: list[PoseStamped]):
    ''' Takes in two lists of PoseStamped. Compares every PoseStamped in
    the first list to a path through the poses of the reference poses
    and return a float of the average distance.
    '''
    average_error = 0.0
    for pose_stamped in inaccurate_poses:
        nearest_poses = find_nearest_poses(pose_stamped.header.stamp,
                                           reference_poses)
        nearest_points = (PointStamped(header = nearest_poses[0].header,
                                       point = nearest_poses[0].pose.position),
                          PointStamped(header = nearest_poses[1].header,
                                       point = nearest_poses[1].pose.position))
        reference_point = find_reference_point(pose_stamped.header.stamp,
                                               nearest_points)
        distance = get_distance_between_points(pose_stamped.pose.position,
                                               reference_point.point)
        
        average_error += distance / len(inaccurate_poses)

    return average_error


def get_avg_dist_simpl(inaccurate_points: list[Point],
                       reference_points: list[Point]):
    ''' todo
    '''
    average_error = 0.0
    for point in inaccurate_points:
        nearest_points = find_nearest_points(point = point,
                                             reference_points = reference_points)
        distance = get_distance_point_line(point,
                                             nearest_points[0],
                                             nearest_points[1])
        
        average_error += distance / len(inaccurate_points)

    return average_error


def get_max_distance(self,
                     inaccurate_poses: list[PoseStamped],
                     reference_poses: list[PoseStamped]):
    ''' Takes in two lists of PoseStamped. Compares every PoseStamped in
    the first list to a path through the poses of the reference poses
    and return a float of the maxiumum distance.
    '''
    self.get_logger().info('Start error calculation.')
    max_distance = 0.0
    for pose_stamped in inaccurate_poses:
        nearest_poses = find_nearest_poses(pose_stamped.header.stamp,
                                           reference_poses)
        nearest_points = (PointStamped(header = nearest_poses[0].header,
                                       point = nearest_poses[0].pose.position),
                          PointStamped(header = nearest_poses[1].header,
                                       point = nearest_poses[1].pose.position))
        reference_point = find_reference_point(pose_stamped.header.stamp,
                                               nearest_points)
        
        distance = get_distance_between_points(pose_stamped.pose.position,
                                               reference_point.point)

        if distance > max_distance:
            #print(f'found new max distance: {distance}')
            max_distance = distance
    return max_distance


def get_max_dist_simpl(points: list[Point],
                       reference_points: list[Point]):
    ''' Takes in two lists of Points. Returns float
    that is the distance between the Point that is furthest away 
    (Euclidean) from a path through all reference points with a straight
    line between neighboring points.
    '''
    max_distance = 0.0
    point: PointStamped
    for point in points:
        nearest_points = find_nearest_points(point = point,
                                             reference_points = reference_points)
        #print(f'This point: {point.x}/{point.y}')
        #print(f'is between: {nearest_points[0].x}/{nearest_points[0].y}' \
        #      + f' and {nearest_points[1].x}/{nearest_points[1].y}')
        separation = get_distance_point_line(point,
                                             nearest_points[0],
                                             nearest_points[1])
        if separation > max_distance:
            max_distance = separation

        #if separation > 0.12:
        #    print(f'Is: {point.x}/{point.y}, between: {nearest_points[0].x}/{nearest_points[0].y} and {nearest_points[1].x}/{nearest_points[1].y}')

    return max_distance


def convert_list_type(input_list, output_type = Point()):
    ''' Convert a list to a list of different types for example remove
    the stamps and Convert a list of PoseStamed to Pose.
    '''
    list_out = []
    if isinstance(input_list[0], PoseStamped):
        if isinstance(output_type, PointStamped):
            #print('Convert list of type PoseStamped to list of type PointStamped')
            pose_s: PoseStamped
            for pose_s in input_list:
                list_out.append(PointStamped(header = pose_s.header,
                                             point = pose_s.pose.position))


        elif isinstance(output_type, Point):
            print('Convert list of type PoseStamped to list of type Point')
            pose_s: PoseStamped
            for pose_s in input_list:
                list_out.append(pose_s.pose.position)


        else:
            print(f'Tried to convert list of type {type(input_list[0])} to list of type {type(output_type)}')
            raise NotImplementedError


    elif isinstance(input_list[0], PointStamped):
        if isinstance(output_type, Point):
            print('Convert list of type PointStamped to list of type Point')
            point_s: PointStamped
            for point_s in input_list:
                list_out.append(point_s.point)

    else:
        print(f'Tried to convert list of type {type(input_list[0])} to list of type {type(output_type)}')
        raise NotImplementedError

    return list_out
