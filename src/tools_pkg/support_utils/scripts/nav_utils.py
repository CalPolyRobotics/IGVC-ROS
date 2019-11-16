import math
import numpy as np
from geometry_msgs.msg import Point, Vector3
from igvc_msgs.msg import Path, PathPoint, Trajectory, TrajectoryPoint

def get_closest_point(path, point):
    """
    Find the next closest point in a path from a given location.

    Args:
        path: An array-like type (list, numpy array, Path, Trajectory)
        point: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)

    Returns:
        closest_point: next closest point
        closest_idx: index of next closest point
    """
    np_path = convert_path_type(path)  # modify path to be a numpy array
    np_point = convert_point_type(point)  # modify point to be a [x,y,z] numpy array

    # compute the distance from current location to every point in path and find index of the min distance
    distances = ((np_path[:,0] - np_point[0])**2 + (np_path[:,1] - np_point[1])**2)**0.5
    closest_idx = np.argmin(distances)

    if closest_idx != len(np_path) - 1:  # check if this point is behind current location, if so use index+1
        closest_point = np_path[closest_idx]
        next_closest_point = np_path[closest_idx+1]

        # create vectors between the three points
        path_vector = next_closest_point - closest_point
        current_vector = np_point - closest_point

        # compute dot product to figure out whether location is behind or in front of closest_point
        dot_prod = np.dot(path_vector, current_vector)

        if dot_prod >= 0:  # closest point is behind current location
            closest_idx += 1

    closest_point = path[closest_idx]  # retrieve point from original `path` argument for type consistency

    return closest_point, closest_idx

def calculate_distance(point1, point2):
    """
    Calculate the distance between two points.

    Args:
        point1: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)
        point2: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)

    Returns:
        distance: the distance between two points
    """
    # modify points to be a [x,y,z] numpy array
    np_point_1 = convert_point_type(point1)
    np_point_2 = convert_point_type(point2)

    distance = ((np_point_1[0] - np_point_2[0])**2 + (np_point_1[1] - np_point_2[1])**2)**0.5

    return distance

def calculate_yaw_error(current_yaw, desired_yaw):
    """
    Calculate the yaw error with [-pi, pi] bounds.

    Args:
        current_yaw: a current orientation
        desired_yaw: a desired orientation

    Returns:
        yaw_error: the angle difference in orientations
    """
    yaw_error = current_yaw - desired_yaw

    # ensure within [-pi, pi] bounds
    if yaw_error < -np.pi:
        yaw_error += 2*np.pi
    elif yaw_error > np.pi:
        yaw_error -= 2*np.pi

    return yaw_error

def convert_path_type(path):
    """
    Convert any array-like type to a numpy array.

    Args:
        path: An array-like type (list, numpy array, Path, Trajectory)

    Returns:
        np_path: a numpy array
    """
    if isinstance(path, Path):
        np_path = path_to_array(path)[:,0]
    elif isinstance(path, Trajectory):
        np_path = trajectory_to_array(path)[:,0]
    elif isinstance(path, (list, np.array)):
        np_path = np.array(path)
    else:
        raise ValueError("Invalid type for `path` argument. Must be an array-like type.")

    return np_path

def convert_point_type(point):
    """
    Convert any point-like type to a numpy array.

    Args:
        point: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)

    Returns:
        np_point: a numpy array
    """
    if isinstance(point, (PathPoint, TrajectoryPoint)):
        np_point = np.array([point.point.x, point.point.y, point.point.z])
    elif isinstance(point, (list, np.array)):
        np_point = np.array(point)
    else:
        raise ValueError("Invalid type for `point` argument. Must be a point-like type.")

    return np_point

def path_to_array(path):
    """
    Convert a Path msg into a numpy array.

    Args:
        path: a Path msg

    Returns:
        np_path: a numpy array
    """
    p = []

    for point in path.points:
        p.append([[point.point.x, point.point.y, point.point.z],
                  [point.orientation.x, point.orientation.y, point.orientation.z]])

    np_path = np.array(p)

    return np_path

def trajectory_to_array(trajectory):
    """
    Convert a Trajectory msg into a numpy array.

    Args:
        trajectory: a Trajectory msg

    Returns:
        np_trajectory: a numpy array
    """
    traj = []

    for point in trajectory.points:
        traj.append([[point.point.x, point.point.y, point.point.z],
                     [point.velocity.x, point.velocity.y, point.velocity.z],
                     [point.orientation.x, point.orientation.y, point.orientation.z]])

    np_trajectory = np.array(traj)

    return np_trajectory

def array_to_path(np_array):
    """
    Convert a numpy array into a Path msg.

    Args:
        np_array: a numpy array

    Returns:
        path: a Path msg
    """
    path = Path()

    for point in np_array:
        location = Point(point[0,0], point[0,1], point[0,2])
        orientation = Vector3(point[1,0], point[1,1], point[1,2])

        path.points.append(PathPoint(location, orientation))

    return path

def array_to_trajectory(np_array):
    """
    Convert a numpy array into a Trajectory msg.

    Args:
        np_array: a numpy array

    Returns:
        trajectory: a Trajectory msg
    """
    trajectory = Trajectory()

    for point in np_array:
        location = Point(point[0,0], point[0,1], point[0,2])
        velocity = Vector3(point[1,0], point[1,1], point[1,2])
        orientation = Vector3(point[2,0], point[2,1], point[2,2])

        trajectory.points.append(TrajectoryPoint(location, velocity, orientation))

    return trajectory
