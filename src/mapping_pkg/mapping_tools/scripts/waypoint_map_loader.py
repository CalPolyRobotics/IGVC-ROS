#!/usr/bin/env python
import csv
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from igvc_msgs.msg import Map, MapPoint


class WaypointMapLoader:
    """
    Load a waypoint map into a Map msg and publish to ROS topic.
    """
    def __init__(self, waypoint_map_name):
        self.waypoint_map_name = waypoint_map_name

        # create publisher for publishing Map msg
        map_pub = rospy.Publisher('waypoint_map', Map, queue_size=1, latch=True)

    def load_map(self):
        """
        Load map from file and publish to ROS topic.
        """
        point_map = Map()

        with open(self.waypoint_map_name) as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',')

            # build map from file
            for row in csvreader:
                point_id = int(row[0])
                point = Point(float(row[2]), float(row[3]), 0.)
                on_road = Bool(row[1] == 'True')
                visited = Bool(False)
                edges = [int(edge) for edge in row[4].strip('[]').split(', ')]

                point_map.points.append(Point(point_id, point, on_road, False, edges))

        map_pub.publish(point_map)  # publish map

if __name__ == '__main__':
    rospy.init_node('waypoint_map_loader') 

    try:
        # get map file name off the parameter server
        waypoint_map_name = rospy.get_param('waypoint_map_name')

        waypoint_map_loader = WaypointMapLoader(waypoint_map_name)
        waypoint_map_loader.load_map()

        while not rospy.is_shutdown():
            continue
    except rospy.ROSInterruptException:
        pass
