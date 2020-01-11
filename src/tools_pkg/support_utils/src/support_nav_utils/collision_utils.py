import nav_utils
import numpy as np

def check_intersection(point1, point2, point3, point4):
    """
    Check if two line segments intersect each other.

    Algorithm taken and converted from python to C++:
    https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

    Args:
        point1: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)
        point2: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)
        point3: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)
        point4: A point-like type (PathPoint, TrajectoryPoint, [x,y,z] list or array)

    Returns
        intersects: a boolean denoting whether two line segments intersect
    """
    intersects = False

    # modify points to be [x,y,z] numpy arrays
    p1 = nav_utils.convert_point_type(point1)
    p2 = nav_utils.convert_point_type(point2)
    p3 = nav_utils.convert_point_type(point3)
    p4 = nav_utils.convert_point_type(point4)

    # calculate orientations of ordered triplet of points
    o1 = calculate_orientation(p1, p2, p3)
    o2 = calculate_orientation(p1, p2, p4)
    o3 = calculate_orientation(p3, p4, p1)
    o4 = calculate_orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:  # general case: if orientations don't match then lines intersect
        intersects = True
    else:  # special case: check for overlap of line segments when they are collinear
        if (o1 == 0 and check_on_segment(p1, p2, p3)) or (o2 == 0 and check_on_segment(p1, p2, p4)) or \
           (o3 == 0 and check_on_segment(p3, p4, p1)) or (o4 == 0 and check_on_segment(p3, p4, p2)):
            intersects = True

    return intersects

def calculate_orientation(p1, p2, p3):
    """
    Calculate the orientation of an ordered triplet of points.

    Args:
        p1: A [x,y,z] Numpy array
        p2: A [x,y,z] Numpy array
        p3: A [x,y,z] Numpy array

    Returns:
        orientation: 0 = collinear, 1 = clockwise, 2 = counterclockwise
    """
    val = (p2[1] - p1[1])*(p3[0] - p2[0]) - (p3[1] - p2[1])*(p2[0] - p1[0])

    if val == 0:  # collinear
        orientation = 0
    elif val > 0:  # clockwise
        orientation = 1
    else:  # counterclockwise
        orientation = 2

    return orientation

def check_on_segment(p1, p2, p3):
    """
    Given collinear points check if p2 lies on line segment p1 to p3.

    Args:
        p1: A [x,y,z] Numpy array
        p2: A [x,y,z] Numpy array
        p3: A [x,y,z] Numpy array

    Returns:
        on_segment: a boolean denoting whether p2 lies on line segment p1 to p3
    """
    min_x = min(p1[0], p3[0])
    max_x = max(p1[0], p3[0])
    min_y = min(p1[1], p3[1])
    max_y = max(p1[1], p3[1])

    # check if p2 lies between p1 and p3
    if (min_x <= p2[0] and p2[0] <= max_x) and (min_y <= p2[1] and p2[1] <= max_y):
        on_segment = True
    else:
        on_segment = False

    return on_segment
