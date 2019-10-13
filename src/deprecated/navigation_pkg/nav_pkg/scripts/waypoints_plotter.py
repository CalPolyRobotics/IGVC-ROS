#!/usr/bin/env python
""" 
Description:
    This module gets GPS data points from the /fix topic and plots them to view in RViz.

Node Name:
    waypoints_plotter

Publishers:
    /visualization_marker

Subscribers:
    /fix

Services:
    None

"""
#import sys
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Polygon
#from nav_msgs.msg import Odometry
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

import rviz_tools_py as rviz_tools

DEBUG = 1

origin_color = 'white'
gps_waypoints_color = 'blue'
set_waypoints_color = 'red'
diameter = 0.3


def mark_waypoints(markers, gps_waypoints_list, set_waypoints_list):
    # path, color, diameter, lifetime=None (default)
    markers.publishSpheres(
        gps_waypoints_list, gps_waypoints_color, diameter, lifetime=None)
    markers.publishSpheres(
        set_waypoints_list, set_waypoints_color, diameter, lifetime=None)

    '''
    # Publish a set of spheres using a list of ROS Point Msgs
    points = []
    points.append(Point(-3, 4, 0))
    points.append(Point(-2, 4, 0))
    points.append(Point(-1, 4, 0))
    points.append(Point(0, 4, 0))
    diameter = 0.3
    '''
    '''
    # Publish a line between two ROS Point Msgs
    point1 = Point(-2, 1, 0)
    point2 = Point(2, 1, 0)
    width = 0.05
    # point1, point2, color, width, lifetime=None (default)
    markers.publishLine(point1, point2, 'green', width, lifetime=999)
    '''


def create_origin(latitude, longitude, altitude):
    ''' 
    Create the origin waypoint.
    @param latitude - latitude from GPS
    @param longitude - longitude from GPS
    '''
    global origin_offset

    origin_offset = (latitude, longitude, altitude)
    origin_point = Point(0, 0, 0)

    if DEBUG:
        print("-------------------------------")
        print("waypoints_plotter: origin_offset:")
        print("\tLatitude: {} Longitude: {} Altitude: {}".format(
            latitude, longitude, altitude))

    # path, color, diameter, lifetime=None (default)
    markers.publishSphere(origin_point, origin_color, diameter, lifetime=None)


def create_gps_waypoint(latitude, longitude, altitude):
    ''' 
    Create a new waypoint using GPS data.
    @param latitude - latitude from GPS
    @param longitude - longitude from GPS
    '''
    global origin_offset, gps_waypoints_list

    adjusted_latitude = latitude - origin_offset[0]
    adjusted_longitude = longitude - origin_offset[1]
    adjusted_altitude = altitude - origin_offset[2]

    if DEBUG:
        print("-------------------------------")
        print("waypoints_plotter: adding adjusted values:")
        print("\tLatitude: {} Longitude: {} Altitude: {}".format(
            adjusted_latitude, adjusted_longitude, adjusted_altitude))

    gps_waypoints_list.append(
        Point(adjusted_latitude, adjusted_longitude, adjusted_altitude))


def create_set_waypoints():
    '''
    Creates statically defined waypoints
    '''
    global set_waypoints_list

    set_waypoints_list.append(Point(-3, 4, 0))
    set_waypoints_list.append(Point(-2, 4, 0))
    set_waypoints_list.append(Point(-1, 4, 0))
    set_waypoints_list.append(Point(0, 4, 0))


def gps_callback(data):
    ''' Callback for new GPS wapoint '''
    global gps_waypoints_list, origin_offset

    # TODO: Extract latitude, longitude from NavSatFix message
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude

    if DEBUG:
        print("-------------------------------")
        print("waypoints_plotter: NEW Values:")
        print("\tLatitude: {} Longitude: {} Altitude: {}".format(
            latitude, longitude, altitude))

    print(len(gps_waypoints_list))

    if(origin_offset == None):
        create_origin(latitude, longitude, altitude)
    else:
        create_gps_waypoint(latitude, longitude, altitude)


def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()


def main():
    '''
    Main loop of node
    '''
    global gps_waypoints_list, set_waypoints_list

    # Create the statically defined waypoints
    create_set_waypoints()
    #----------Continuously update waypoint list-----------------------------------------
    while not rospy.is_shutdown():
        mark_waypoints(markers, gps_waypoints_list, set_waypoints_list)
        node_rate.sleep()


if __name__ == '__main__':
    #----------Initialize ROS things--------------------------------------------
    # Create gps_waypoint_plotter node
    rospy.init_node('waypoints_plotter', anonymous=False,
                    log_level=rospy.INFO, disable_signals=False)
    rospy.on_shutdown(cleanup_node)  # Cleanup waypoints on close
    node_rate = rospy.Rate(1)  # Run node at 1 Hz

    #----------Create Publishers/Subscribers------------------------------------
    # Create RVizMarkers object with base_frame, and marker_topic with its own publisher
    markers = rviz_tools.RvizMarkers('/odom', 'visualization_marker')
    # Create GPS data subscriber
    gps_sub = rospy.Subscriber(
        "fix", NavSatFix, gps_callback, queue_size=10)

    #----------Create a list to hold waypoints----------------------------------
    gps_waypoints_list = []
    set_waypoints_list = []
    # Tuple to be defined to hold initial (lat., long.) position
    origin_offset = None

    main()

    #rospy.spin()  # To make sure python doesn't exit
